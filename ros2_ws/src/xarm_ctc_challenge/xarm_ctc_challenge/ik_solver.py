"""
Weighted Resolved-Rate IK for xArm Lite 6.

Reference: Challenge instructions Section 5.

Method:
  W  = diag(1, 1, wz)          — Z-priority weight matrix (task space)
  Jw = W @ J                   — weighted Jacobian
  J# = Jw^T @ (Jw @ Jw^T + λ²I)^{-1}  — DLS pseudo-inverse
  q̇_null = (I - J# @ Jw) @ (-k_null*(q - q_home) + limit_push)
  q̇_des  = J# @ (ṗ_des + k_task*(p_des - p)) + q̇_null

Reference signal generation (for CTC feedforward):
  q_des   : integrated from q̇_des
  q̈_des  : finite difference of q̇_des  (lightweight Ĵ estimate)

Notes on the velocity-control interface scaling
-----------------------------------------------
MoveIt Servo applies each JointJog velocity for exactly one servo publish
period before waiting for the next command.  With a servo publish period of
~2 ms and our command period of 10 ms (100 Hz), the effective Cartesian
velocity is ~20-22% of what we command.  K_TASK and MAX_TASK_VEL are set
to compensate: a higher K_TASK raises the correction velocity so even at 22%
scale the position error is corrected quickly, and a larger MAX_TASK_VEL
prevents that correction from being clipped too early.

What did NOT work: joint-space cost weighting (W_Q) with high penalties on
J5/J6.  Values of 10× froze the wrist joints and forced J3 to swing 99°,
causing the robot to diverge 455 mm from the intended trajectory.  The 0.22
servo scaling is NOT caused by wrist singularity; it is a fixed property of
the JointJog interface and cannot be fixed by changing the IK structure.
"""

import numpy as np
from .kinematics import forward_kinematics, position_jacobian

# ------------------------------------------------------------------
# Default IK parameters
# ------------------------------------------------------------------
WZ           = 2.5     # Z-axis task-space weight   (range 2.0–3.5)
LAMBDA       = 1.5e-2  # DLS damping                (range 0.01–0.02)
K_TASK       = 20.0    # task-space error gain — raised from 14 so that at the
                       # 0.22 effective servo scale the position loop is still
                       # adequately aggressive (effective gain ≈ 4.4)
K_NULL       = 5.0     # null-space posture gain — raised from 1.5; prevents
                       # joint drift that caused J4 to hit its ±175° limit
MAX_TASK_VEL = 0.25    # m/s — raised from 0.15; allows faster error correction
                       # while keeping joint velocity commands within ±2 rad/s
IK_VEL_LIMIT = 2.0     # rad/s — per-joint clip on q_hat integration
MAX_QDD      = 50.0    # rad/s² — hard clamp on IK-level qdd_des finite difference

# xArm Lite 6 joint limits [rad] — from official URDF (non-limited mode).
# Joint3 is asymmetric: lower=-0.061087 rad (~-3.5°), upper=5.235988 rad (~300°).
# Joint5 physical limit is ±2.1642 rad (~124°); joint2 is ±2.61799 rad (~150°).
JOINT_LIMIT_UPPER = np.array([6.283, 2.618, 5.236, 3.054, 2.164, 6.283])
JOINT_LIMIT_LOWER = np.array([-6.283, -2.618, -0.061, -3.054, -2.164, -6.283])
LIMIT_MARGIN = 0.35    # rad (~20°) — avoidance kicks in this far from limit
K_LIMIT      = 20.0    # rad/s per rad of penetration into the margin


class WeightedIKSolver:
    """
    Online weighted resolved-rate IK integrator.

    Maintains internal IK state (q_hat) which is the reference joint
    trajectory.  Call step() at each control tick.
    """

    def __init__(
        self,
        q_home: np.ndarray,
        dt: float,
        wz: float = WZ,
        lam: float = LAMBDA,
        k_task: float = K_TASK,
        k_null: float = K_NULL,
    ):
        self.q_home  = q_home.copy()
        self.dt      = dt
        self.wz      = wz
        self.lam     = lam
        self.k_task  = k_task
        self.k_null  = k_null
        self.max_tv  = MAX_TASK_VEL
        self.ik_vel_lim = IK_VEL_LIMIT
        self.max_qdd    = MAX_QDD

        # Task-space weight matrix (3×3) — z-axis priority
        self._W  = np.diag([1.0, 1.0, wz])
        self._I6 = np.eye(6)
        self._I3 = np.eye(3)

        # IK state
        self.q_hat    = q_home.copy()   # current IK joint estimate
        self._qd_prev = np.zeros(6)     # previous q̇_des (for q̈_des FD)

    def reset(self, q_init: np.ndarray):
        self.q_hat    = q_init.copy()
        self.q_home   = q_init.copy()   # null-space reference = actual start config
        self._qd_prev = np.zeros(6)

    def step(
        self,
        p_des:    np.ndarray,
        pd_des:   np.ndarray,
        pdd_des:  np.ndarray,
        p_actual: np.ndarray = None,
        q_actual: np.ndarray = None,
    ):
        """
        Run one IK step and advance q_hat.

        Args:
            p_des:    (3,) desired Cartesian position [m]
            pd_des:   (3,) desired Cartesian velocity [m/s]
            pdd_des:  (3,) desired Cartesian acceleration [m/s²]
            p_actual: (3,) optional actual EE position from TF2.
            q_actual: (6,) optional actual joint angles from /joint_states.

        Returns:
            q_des:   (6,) desired joint position (integrated)
            qd_des:  (6,) desired joint velocity
            qdd_des: (6,) desired joint acceleration (FD estimate)
        """
        # Pure CLIK: anchor every computation to actual joint angles so that
        # servo saturation and latency do not cause q_hat to drift unboundedly.
        q = q_actual if q_actual is not None else self.q_hat

        # Jacobian and FK at actual (or best-known) configuration
        J = position_jacobian(q)
        if p_actual is not None:
            p = p_actual
        else:
            p, _ = forward_kinematics(q)

        # Task-space weighted DLS pseudo-inverse
        Jw    = self._W @ J                                    # (3,6)
        JwJwT = Jw @ Jw.T + (self.lam ** 2) * self._I3       # (3,3)
        J_sharp = Jw.T @ np.linalg.inv(JwJwT)                 # (6,3)

        # Null-space: posture term (pull joints toward home config)
        posture = -self.k_null * (q - self.q_home)

        # Joint-limit avoidance: repulsive push when within LIMIT_MARGIN of a
        # hardware limit.  Projected through the null-space so it does not
        # fight the Cartesian task when joints are far from limits.
        limit_push = np.zeros(6)
        for j in range(6):
            dist_upper = JOINT_LIMIT_UPPER[j] - q[j]
            dist_lower = q[j] - JOINT_LIMIT_LOWER[j]
            if dist_upper < LIMIT_MARGIN:
                limit_push[j] = -K_LIMIT * (LIMIT_MARGIN - dist_upper)
            elif dist_lower < LIMIT_MARGIN:
                limit_push[j] =  K_LIMIT * (LIMIT_MARGIN - dist_lower)

        q_null_dot = (self._I6 - J_sharp @ Jw) @ (posture + limit_push)

        # Task-space error feedback + trajectory feedforward
        err_raw  = self.k_task * (p_des - p)
        spd      = np.linalg.norm(err_raw)
        err_term = err_raw if spd <= self.max_tv else err_raw * (self.max_tv / spd)
        task_vel = pd_des + err_term                           # (3,)
        qd_des   = J_sharp @ task_vel + q_null_dot            # (6,)

        # Desired acceleration via finite difference — computed on CLIPPED velocity
        # so that FD does not spike when qd_des jumps after a perturbation.
        qd_des_clipped = np.clip(qd_des, -self.ik_vel_lim, self.ik_vel_lim)
        qdd_des = (qd_des_clipped - self._qd_prev) / self.dt
        qdd_des = np.clip(qdd_des, -self.max_qdd, self.max_qdd)

        # One-step-ahead reference, anchored to actual → q_hat never diverges
        self.q_hat = q + qd_des_clipped * self.dt
        self._qd_prev = qd_des_clipped.copy()   # store CLIPPED to avoid next-tick spike

        return self.q_hat.copy(), qd_des_clipped, qdd_des
