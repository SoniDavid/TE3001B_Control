#!/usr/bin/env python3
"""
ChallengeController — xArm Lite 6  (Challenge 4.1)
CTC vs PD/PID Under Perturbations — PCB Component Placement Task

Reference generation (both modes):
  On the first /joint_states message the actual EE position is computed via FK
  and used as the trajectory centre.  PCBTrajectory then generates Cartesian
  references p_des(t), ṗ_des(t), p̈_des(t) as relative offsets from that
  centre — mirroring the approach used in the working xarm_perturbations
  package and avoiding IK cold-start divergence.

  CTC: WeightedIKSolver runs online at every control tick to convert the
       Cartesian reference to joint-space (q_des, q̇_des, q̈_des).
       Commands: JointJog → /servo_server/delta_joint_cmds

  PD/PID: tracks the Cartesian reference directly in task-space.
          Commands: TwistStamped → /servo_server/delta_twist_cmds

Safety:
  - Joint velocity saturation: ±2.0 rad/s
  - Torque saturation: ±10.0 Nm (CTC only)
  - Emergency stop: publish True on /challenge_stop
"""

import csv
import json
import os
import threading
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TwistStamped
from std_msgs.msg import ColorRGBA, Bool
from tf2_ros import Buffer, TransformListener

from .kinematics import forward_kinematics, position_jacobian, JOINT_NAMES
from .dynamics import get_dynamics
from .ik_solver import WeightedIKSolver
from .trajectory import PCBTrajectory

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

CONTROL_HZ   = 100
DT           = 1.0 / CONTROL_HZ
VEL_LIMIT    = 2.0
TORQUE_LIMIT = 10.0

KP_PD  = np.array([2.5, 2.5, 2.5])
KD_PD  = np.array([0.6, 0.6, 0.6])
KI_PD  = np.array([0.1, 0.1, 0.1])
I_LIM  = 0.05
MAX_SPEED_PD = 0.12
LPF_FC_HZ    = 2.0     # first-order LPF cutoff (Hz) — matches perturbations package
DEADBAND_M   = 0.002   # per-axis deadband (m) — zero command when within 2 mm

KP_CTC = np.diag([20.0, 20.0, 20.0, 15.0, 10.0, 8.0])
KD_CTC = np.diag([8.0,  8.0,  8.0,  6.0,  4.0,  3.0])

GAMMA    = 2.5
K_ROBUST = 5.5
SIGN_EPS = 2e-3

Q_HOME_JOINTS = np.array([-1.14, 0.30, 1.70, 0.50, 0.5, 0.0])


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class ChallengeController(Node):

    def __init__(self):
        super().__init__('challenge_controller')

        ctrl_type = str(self.declare_parameter('controller_type', '').value).upper()
        self._use_ctc  = (ctrl_type == 'CTC') or bool(self.declare_parameter('use_ctc', False).value)
        self._use_pid  = (ctrl_type == 'PID') or bool(self.declare_parameter('use_pid', False).value)
        self._save_csv = bool(self.declare_parameter('save_csv', True).value)
        self._pert_en  = bool(self.declare_parameter('perturbation_enabled', False).value)
        self._loop     = bool(self.declare_parameter('loop_trajectory', False).value)
        self._csv_dir  = str(self.declare_parameter('csv_dir', 'data').value)

        self._pert_mode = str(self.declare_parameter('pert_mode', 'gaussian').value)
        self._pert_std  = float(self.declare_parameter('pert_std_linear', 0.5).value)
        self._pert_axis = str(self.declare_parameter('pert_gauss_axis', 'x').value)

        kp = list(self.declare_parameter('kp', []).value)
        kd = list(self.declare_parameter('kd', []).value)
        if self._use_ctc:
            if len(kp) == 6: np.fill_diagonal(KP_CTC, kp)
            if len(kd) == 6: np.fill_diagonal(KD_CTC, kd)
        else:
            if len(kp) == 3: KP_PD[:] = kp
            if len(kd) == 3: KD_PD[:] = kd

        self.add_on_set_parameters_callback(self._on_param_change)

        self._state_lock = threading.Lock()
        self._q   = Q_HOME_JOINTS.copy()
        self._qd  = np.zeros(6)
        self._ready      = False
        self._stopped    = False
        self._traj_ready = False
        self._t_start    = None

        self._e_int      = np.zeros(3)
        self._qd_cmd_ctc = np.zeros(6)
        self._prev_e_pd  = np.zeros(3)
        self._prev_vel_pd = np.zeros(3)

        self._pert_prev    = self._pert_en
        self._pert_start_t = None

        self._last_log_t  = None   # for periodic console diagnostics

        # Trajectory is built in _control_cb from TF2 actual EE position.
        self._cart_traj: PCBTrajectory = None
        self._ik = WeightedIKSolver(Q_HOME_JOINTS, DT)

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._jog_pub = self.create_publisher(
            JointJog, '/servo_server/delta_joint_cmds', reliable_qos)
        self._twist_pub = self.create_publisher(
            TwistStamped, '/servo_server/delta_twist_cmds', reliable_qos)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/challenge_markers', 10)

        self._js_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, sensor_qos)
        self._stop_sub = self.create_subscription(
            Bool, '/challenge_stop', self._estop_cb, 10)

        self._csv_file    = None
        self._csv_writer  = None
        self._csv_row_buf = []
        if self._save_csv:
            self._init_csv()

        self._ctrl_timer = self.create_timer(DT, self._control_cb)

        mode_str = "CTC" if self._use_ctc else ("PID" if self._use_pid else "PD")
        self.get_logger().info(
            f"ChallengeController started — {mode_str}  "
            f"pert={'ON' if self._pert_en else 'off'}  "
            f"Waiting for /joint_states to build trajectory...")

    # ── Parameter callback ────────────────────────────────────────────────────

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'perturbation_enabled':
                self._pert_en = bool(p.value)
                self.get_logger().info(f'perturbation_enabled → {self._pert_en}')
        return SetParametersResult(successful=True)

    # ── Emergency stop ────────────────────────────────────────────────────────

    def _estop_cb(self, msg: Bool):
        if msg.data and not self._stopped:
            self._stopped = True
            self._publish_jog(np.zeros(6))
            self._flush_csv()
            self.get_logger().warn('EMERGENCY STOP via /challenge_stop')

    # ── Joint state callback ──────────────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        q  = np.zeros(6)
        qd = np.zeros(6)
        for j, jname in enumerate(JOINT_NAMES):
            if jname in name_to_idx:
                idx = name_to_idx[jname]
                q[j]  = msg.position[idx]
                qd[j] = msg.velocity[idx] if msg.velocity else 0.0

        with self._state_lock:
            self._q  = q
            self._qd = qd
            if not self._ready:
                self._ready = True
                self._qd_cmd_ctc = qd.copy()
                self._ik.reset(q)
                self.get_logger().info(
                    f"Joint states received — q={np.round(q,3)}\n"
                    f"  Waiting for TF2 to build trajectory centre..."
                )

    # ── TF2 EE position ───────────────────────────────────────────────────────

    def _read_pose_tf(self):
        """Return actual EE position from TF2, or None if unavailable."""
        try:
            trans = self._tf_buffer.lookup_transform(
                'link_base', 'link_eef', rclpy.time.Time())
            t = trans.transform.translation
            return np.array([t.x, t.y, t.z], dtype=float)
        except Exception:
            return None

    # ── Main control callback ─────────────────────────────────────────────────

    def _control_cb(self):
        if self._stopped:
            return

        # ── Build trajectory once TF2 is available (matches perturbations pkg) ──
        if self._cart_traj is None:
            if not self._ready:
                return
            p_centre = self._read_pose_tf()
            if p_centre is None:
                return   # TF2 not ready yet — try next tick
            with self._state_lock:
                q_init = self._q.copy()
            self._cart_traj = PCBTrajectory(centre=p_centre)
            self._ik.reset(q_init)
            self._t_start    = self.get_clock().now()
            self._traj_ready = True
            self._last_log_t = self.get_clock().now()
            mode_str = "CTC" if self._use_ctc else ("PID" if self._use_pid else "PD")
            self.get_logger().info(
                f"TF2 centre: {np.round(p_centre, 4)}\n"
                f"  {mode_str} running — total trajectory: "
                f"{self._cart_traj.total_time:.1f} s"
            )
            self._publish_waypoint_markers()
            self._write_metadata()
            return

        if not self._traj_ready:
            return

        with self._state_lock:
            q  = self._q.copy()
            qd = self._qd.copy()

        t_rel = (self.get_clock().now() - self._t_start).nanoseconds * 1e-9

        if t_rel > self._cart_traj.total_time:
            if self._loop:
                self._t_start = self.get_clock().now()
                self._e_int[:]       = 0.0
                self._qd_cmd_ctc[:]  = 0.0
                self._prev_e_pd[:]   = 0.0
                self._prev_vel_pd[:] = 0.0
                self._ik.reset(q)
                t_rel = 0.0
            else:
                if self._use_ctc:
                    self._publish_jog(np.zeros(6))
                else:
                    self._publish_twist(np.zeros(3))
                self._flush_csv()
                return

        ctp = self._cart_traj.at(t_rel)

        if self._use_ctc:
            # Use TF2 for position feedback so the IK sees real robot state,
            # not a potentially inaccurate FK estimate.
            p_tf2_ctc = self._read_pose_tf()
            q_des, qd_des, qdd_des = self._ik.step(
                ctp.p, ctp.pd, ctp.pdd, p_actual=p_tf2_ctc, q_actual=q)

            # --- Temporary IK sanity check ---
            p_ik_check = forward_kinematics(q_des)[0]
            ik_err_mm = np.linalg.norm(ctp.p - p_ik_check) * 1000
            if ik_err_mm > 5.0:
                self.get_logger().warn(
                    f'IK diverged: {ik_err_mm:.1f}mm  '
                    f'q_des={np.round(q_des,3)}  q={np.round(q,3)}')

            qd_cmd, sat_flags = self._ctc(q, qd, q_des, qd_des, qdd_des)
            qd_sat = np.clip(qd_cmd, -VEL_LIMIT, VEL_LIMIT)
            sat_flags |= (np.abs(qd_cmd) > VEL_LIMIT)
            self._publish_jog(qd_sat)
            p_act = p_tf2_ctc if p_tf2_ctc is not None else forward_kinematics(q)[0]
            p_des     = ctp.p
            cmd_log   = qd_sat.tolist()
            sat_log   = sat_flags.astype(int).tolist()
        else:
            # Use TF2 for actual EE position (same as perturbations package)
            p_act_tf = self._read_pose_tf()
            p_act = p_act_tf if p_act_tf is not None else forward_kinematics(q)[0]
            v_cmd, sat_flags = self._pd_pid_cart(p_act, ctp, q)
            self._publish_twist(v_cmd)
            p_des     = ctp.p
            q_des, qd_des, qdd_des = q.copy(), np.zeros(6), np.zeros(6)
            cmd_log   = list(v_cmd) + [0.0, 0.0, 0.0]
            sat_log   = list(sat_flags.astype(int)) + [0, 0, 0]

        wp_idx, label, phase = ctp.wp_idx, ctp.label, ctp.phase

        abs_t = self.get_clock().now().nanoseconds * 1e-9
        pert_changed = int(self._pert_en != self._pert_prev)
        if pert_changed:
            self.get_logger().info(
                f'Perturbation {"ENABLED" if self._pert_en else "DISABLED"} '
                f't={t_rel:.3f}s')
            self._pert_prev = self._pert_en

        if self._save_csv:
            row = (
                [abs_t, t_rel] +
                q.tolist() + qd.tolist() +
                q_des.tolist() + qd_des.tolist() + qdd_des.tolist() +
                p_act.tolist() + p_des.tolist() +
                cmd_log + sat_log +
                [int(self._pert_en), pert_changed, wp_idx, label, phase]
            )
            self._csv_row_buf.append(row)
            if len(self._csv_row_buf) >= 100:
                self._flush_csv()

        # ── Periodic console diagnostic (every 2 s) ───────────────────────────
        now_t = self.get_clock().now()
        if self._last_log_t is None or \
                (now_t - self._last_log_t).nanoseconds > 2e9:
            err   = p_des - p_act
            # Jacobian condition number — high value means servo singularity scaling
            J_diag = position_jacobian(q)
            sigma  = np.linalg.svd(J_diag, compute_uv=False)
            cond   = sigma[0] / max(sigma[-1], 1e-6)
            ctrl_extra = (
                f"  σ_min={sigma[-1]:.4f}  cond={cond:.1f}"
                if self._use_ctc else ""
            )
            self.get_logger().info(
                f"t={t_rel:.1f}/{self._cart_traj.total_time:.1f}s  "
                f"wp={label}({phase})  "
                f"p_act=[{p_act[0]:.3f},{p_act[1]:.3f},{p_act[2]:.3f}]  "
                f"p_des=[{p_des[0]:.3f},{p_des[1]:.3f},{p_des[2]:.3f}]  "
                f"|err|={np.linalg.norm(err)*1000:.1f}mm"
                + ctrl_extra
            )
            if self._use_ctc:
                self.get_logger().info(
                    f"  q=[{','.join(f'{np.degrees(v):.1f}' for v in q)}]°  "
                    f"qd_cmd=[{','.join(f'{np.degrees(v):.1f}' for v in (qd_sat if self._use_ctc else np.zeros(6)))}]°/s"
                )
            self._last_log_t = now_t

    # ── PD / PID  (identical approach to xarm_perturbations Lab 4.2) ─────────

    # Singularity avoidance thresholds for the PD Cartesian path.
    # MoveIt Servo's internal IK has a hard-stop condition-number threshold.
    # By scaling our velocity command near singularity we keep the servo from
    # halting while still tracking the trajectory (just more slowly).
    _SIGMA_SOFT = 0.06   # m/rad — begin scaling velocity below this
    _SIGMA_HARD = 0.01   # m/rad — command zero below this

    def _pd_pid_cart(self, p_act: np.ndarray, ctp, q: np.ndarray):
        """
        Cartesian PD/PID step — mirrors TrajectoryAndController._servo_to().

        Uses:
          - error FD for derivative (not Jacobian × qd)
          - per-axis deadband (DEADBAND_M)
          - exponential first-order LPF on commanded velocity
          - magnitude-norm saturation
          - Jacobian minimum-singular-value scaling to survive near-singularity
        """
        import math
        e      = ctp.p - p_act
        d_error = (e - self._prev_e_pd) / DT

        active_mask = np.abs(e) > DEADBAND_M

        if self._use_pid:
            self._e_int = np.clip(
                self._e_int + np.where(active_mask, e, 0.0) * DT,
                -I_LIM, I_LIM,
            )
            v = KP_PD * e + KD_PD * d_error + KI_PD * self._e_int
        else:
            v = KP_PD * e + KD_PD * d_error

        v = np.where(active_mask, v, 0.0)

        # Exponential LPF (same formula as perturbations package)
        alpha = math.exp(-2.0 * math.pi * LPF_FC_HZ * DT)
        self._prev_vel_pd = alpha * self._prev_vel_pd + (1.0 - alpha) * v
        v = self._prev_vel_pd

        # Magnitude-norm saturation (preserves direction)
        v_norm = float(np.linalg.norm(v))
        sat = v_norm > MAX_SPEED_PD
        if sat:
            v = v * (MAX_SPEED_PD / max(v_norm, 1e-9))

        # Singularity avoidance — scale Cartesian velocity by σ_min of J so
        # that we naturally slow down as the robot approaches a singular config,
        # preventing MoveIt Servo's internal hard-stop from triggering.
        J = position_jacobian(q)
        sigma_min = float(np.linalg.svd(J, compute_uv=False)[-1])
        if sigma_min < self._SIGMA_SOFT:
            sing_scale = max(
                0.0,
                (sigma_min - self._SIGMA_HARD) / (self._SIGMA_SOFT - self._SIGMA_HARD)
            )
            v = v * sing_scale
            self.get_logger().warn(
                f'Near singularity: σ_min={sigma_min:.4f} → scale={sing_scale:.2f}',
                throttle_duration_sec=1.0,
            )

        self._prev_e_pd = e.copy()
        return v, np.array([sat] * 3)

    # ── CTC ───────────────────────────────────────────────────────────────────

    def _ctc(self, q, qd, q_des, qd_des, qdd_des):
        e    = q_des - q
        edot = qd_des - qd

        # --- CTC acceleration reference ---
        # v is the desired joint acceleration that gives PD-stable error dynamics
        # when the dynamics are perfectly cancelled.
        v = qdd_des + KP_CTC @ e + KD_CTC @ edot

        # --- Dynamics feedforward (model-based cancellation) ---
        M, Cqd, G, F = get_dynamics(q, qd)
        tau  = M @ v + Cqd + G + F
        tau  = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)
        sat_flags = np.abs(tau) >= TORQUE_LIMIT

        # --- Velocity command for velocity-controlled interface ---
        # Recover the (possibly torque-saturated) acceleration, then integrate
        # one step from the desired state to get the velocity command.
        # If tau is unsaturated: qdd_cmd = M^{-1}(M@v + Cqd + G + F - Cqd - G - F) = v
        # If saturated: qdd_cmd is reduced accordingly, limiting aggressive moves.
        qdd_cmd = np.linalg.solve(M, tau - Cqd - G - F)
        qd_cmd  = qd_des + qdd_cmd * DT
        qd_cmd  = np.clip(qd_cmd, -VEL_LIMIT, VEL_LIMIT)
        return qd_cmd, sat_flags

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_jog(self, qd_cmd: np.ndarray):
        msg = JointJog()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_base'
        msg.joint_names     = JOINT_NAMES
        msg.velocities      = qd_cmd.tolist()
        msg.duration        = DT
        self._jog_pub.publish(msg)

    def _publish_twist(self, v_cmd: np.ndarray):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_base'
        msg.twist.linear.x  = float(v_cmd[0])
        msg.twist.linear.y  = float(v_cmd[1])
        msg.twist.linear.z  = float(v_cmd[2])
        self._twist_pub.publish(msg)

    # ── RViz markers ──────────────────────────────────────────────────────────

    def _publish_waypoint_markers(self):
        if self._cart_traj is None:
            return
        arr = MarkerArray()
        home = self._cart_traj.home
        wps  = self._cart_traj.waypoints

        for i, wp in enumerate(wps):
            m = Marker()
            m.header.frame_id = 'link_base'
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns, m.id        = 'waypoints', i
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = wp[0], wp[1], wp[2]
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.015
            is_place = 'low' in wp[4]
            m.color = (ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0) if is_place
                       else ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.7))
            arr.markers.append(m)

        line = Marker()
        line.header.frame_id = 'link_base'
        line.header.stamp    = self.get_clock().now().to_msg()
        line.ns, line.id     = 'path', 100
        line.type            = Marker.LINE_STRIP
        line.action          = Marker.ADD
        line.scale.x         = 0.003
        line.color           = ColorRGBA(r=0.0, g=0.9, b=0.0, a=0.6)
        pt = Point(); pt.x, pt.y, pt.z = home; line.points.append(pt)
        for wp in wps:
            pt = Point(); pt.x, pt.y, pt.z = wp[0], wp[1], wp[2]; line.points.append(pt)
        pt = Point(); pt.x, pt.y, pt.z = home; line.points.append(pt)
        arr.markers.append(line)
        self._marker_pub.publish(arr)

    # ── Metadata ──────────────────────────────────────────────────────────────

    def _write_metadata(self):
        if not self._save_csv or self._cart_traj is None:
            return
        ctrl = "ctc" if self._use_ctc else "pdpid"
        pert = "pert" if self._pert_en else "nopert"
        ts   = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        fname = os.path.join(self._csv_dir, f"trial_{ctrl}_{pert}_{ts}_metadata.json")
        meta = {
            "trial_id":         f"trial_{ctrl}_{pert}_{ts}",
            "controller":       "CTC" if self._use_ctc else ("PID" if self._use_pid else "PD"),
            "trajectory_centre": self._cart_traj.home.tolist(),
            "ik_solver":        ("WeightedIKSolver wz=2.5 λ=0.015 k_task=14 k_null=1.5"
                                 if self._use_ctc else "N/A"),
            "control_rate_hz":  CONTROL_HZ,
            "initial_config_rad": Q_HOME_JOINTS.tolist(),
            "gains": {
                "KP": np.diag(KP_CTC).tolist() if self._use_ctc else KP_PD.tolist(),
                "KD": np.diag(KD_CTC).tolist() if self._use_ctc else KD_PD.tolist(),
                "KI_PD": KI_PD.tolist() if self._use_pid else None,
                "GAMMA": GAMMA, "K_ROBUST": K_ROBUST, "SIGN_EPS": SIGN_EPS,
            },
            "perturbation": {
                "enabled": self._pert_en, "mode": self._pert_mode,
                "std_linear": self._pert_std, "axis": self._pert_axis,
            },
            "waypoints": [
                {"id": i+1, "label": wp[4], "x": wp[0], "y": wp[1],
                 "z": wp[2], "layer": "low" if "low" in wp[4] else "high",
                 "dwell_s": wp[3]}
                for i, wp in enumerate(self._cart_traj.waypoints)
            ],
            "total_traj_time_s": self._cart_traj.total_time,
            "success_threshold_m": 0.005,
        }
        os.makedirs(self._csv_dir, exist_ok=True)
        abs_path = os.path.abspath(fname)
        with open(abs_path, 'w') as f:
            json.dump(meta, f, indent=2)
        self.get_logger().info(f'Metadata → {abs_path}')

    # ── CSV ───────────────────────────────────────────────────────────────────

    def _init_csv(self):
        os.makedirs(self._csv_dir, exist_ok=True)
        ctrl = "ctc" if self._use_ctc else "pdpid"
        pert = "pert" if self._pert_en else "nopert"
        ts   = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        fname = os.path.join(self._csv_dir, f"trial_{ctrl}_{pert}_{ts}.csv")
        abs_path = os.path.abspath(fname)
        self._csv_file   = open(abs_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self.get_logger().info(f'CSV log → {abs_path}')
        self._csv_writer.writerow(
            ['time', 'time_rel'] +
            [f'q_{i}' for i in range(6)] + [f'qd_{i}' for i in range(6)] +
            [f'q_des_{i}' for i in range(6)] + [f'qd_des_{i}' for i in range(6)] +
            [f'qdd_des_{i}' for i in range(6)] +
            ['p_act_x','p_act_y','p_act_z'] + ['p_des_x','p_des_y','p_des_z'] +
            [f'qd_cmd_{i}' for i in range(6)] + [f'sat_{i}' for i in range(6)] +
            ['pert_enabled','pert_changed','wp_idx','wp_label','phase']
        )
        self.get_logger().info(f'CSV → {fname}')

    def _flush_csv(self):
        if self._csv_writer and self._csv_row_buf:
            self._csv_writer.writerows(self._csv_row_buf)
            self._csv_file.flush()
            self._csv_row_buf.clear()

    def destroy_node(self):
        self._flush_csv()
        if self._csv_file:
            self._csv_file.close()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = ChallengeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
