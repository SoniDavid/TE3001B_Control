#!/usr/bin/env python3
"""
ChallengeController - xArm Lite 6  (Challenge 4.1)
CTC vs PD/PID Under Perturbations - PCB Component Placement Task
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

# KP_CTC = np.diag([20.0, 20.0, 20.0, 15.0, 10.0, 8.0])
# KD_CTC = np.diag([8.0,  8.0,  8.0,  6.0,  4.0,  3.0])

KP_CTC = np.diag([20.0, 20.0, 20.0, 2.0, 1.0, 0.5])
KD_CTC = np.diag([8.0,  8.0,  8.0, 0.8, 0.4, 0.2])

GAMMA    = 2.5
K_ROBUST = 5.5
SIGN_EPS = 2e-3

# CTC signal filters
# qdd_des spikes up to ±114 rad/s² at every 9-Hz state update.
# Without clamping and smoothing, the CTC feedforward τ = M·v explodes,
# driving joints 3 and 5 into velocity saturation (18% of all ticks).
# Three cascaded filters bring the signal chain under control:
#   1. Hard clamp on qdd_des        — prevents τ blow-up before filtering settles
#   2. LPF on qdd_des  (fc=5 Hz)   — removes state-update spikes
#   3. LPF on qd_des   (fc=10 Hz)  — smooths IK velocity chatter → quieter ė term
#   4. LPF on qd_cmd   (fc=20 Hz)  — smooths servo input → less abrupt joint moves
MAX_QDD_DES  = 10.0   # rad/s²  — hard clamp on desired joint acceleration
FILT_FC_QDD  = 5.0    # Hz      — LPF cutoff for qdd_des
FILT_FC_QD   = 10.0   # Hz      — LPF cutoff for qd_des
FILT_FC_CMD  = 20.0   # Hz      — LPF cutoff for qd_cmd output

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
        # joint_states_topic: override to '/ufactory/joint_states' (ros2_control, ~150 Hz)
        # instead of '/joint_states' (joint_state_publisher aggregator, ~10 Hz)
        self._js_topic  = str(self.declare_parameter('joint_states_topic', '/joint_states').value)

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

        # --- State predictor (CTC) ---
        # The hardware publishes /joint_states at ~10 Hz via joint_state_publisher,
        # while the CTC control loop runs at 100 Hz.  Between hardware updates the
        # actual q/qd are stale, causing M/C/G feedforward errors and qdd_des spikes
        # in the IK finite-difference.  We keep a predicted state that integrates
        # the last commanded velocity every tick, reset to the real measurement
        # whenever a fresh /joint_states message arrives.
        self._state_updated = False            # set True by every new joint_state msg
        self._q_pred  = Q_HOME_JOINTS.copy()   # predicted joint position
        self._qd_pred = np.zeros(6)            # predicted joint velocity
        self._last_qd_cmd = np.zeros(6)        # last CTC velocity command (for prediction)

        # --- CTC signal filters (tunable via ROS2 parameters) ---
        self._max_qdd  = float(self.declare_parameter('max_qdd_des',   MAX_QDD_DES).value)
        self._fc_qdd   = float(self.declare_parameter('filter_fc_qdd', FILT_FC_QDD).value)
        self._fc_qd    = float(self.declare_parameter('filter_fc_qd',  FILT_FC_QD).value)
        self._fc_cmd   = float(self.declare_parameter('filter_fc_cmd', FILT_FC_CMD).value)
        # Precompute first-order IIR coefficients  α = exp(-2π·fc·dt)
        self._a_qdd = float(np.exp(-2 * np.pi * self._fc_qdd * DT))
        self._a_qd  = float(np.exp(-2 * np.pi * self._fc_qd  * DT))
        self._a_cmd = float(np.exp(-2 * np.pi * self._fc_cmd  * DT))
        # Filter states
        self._qdd_des_filt = np.zeros(6)
        self._qd_des_filt  = np.zeros(6)
        self._qd_cmd_filt  = np.zeros(6)

        self._pert_prev    = self._pert_en
        self._pert_start_t = None

        self._last_log_t  = None   # for periodic console diagnostics

        # Waypoint reach tracking
        self._wp_reached     = {}   # {wp_label: bool} — set once per trial when within threshold
        self._wp_reach_thr_m = 0.005  # 5 mm success threshold

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
            JointState, self._js_topic, self._joint_state_cb, sensor_qos)
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
            f"joint_states_topic='{self._js_topic}'  "
            f"Waiting for joint states to build trajectory...")

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
            self._publish_twist(np.zeros(3))
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
            self._state_updated = True
            if not self._ready:
                self._ready = True
                self._qd_cmd_ctc = qd.copy()
                self._q_pred  = q.copy()
                self._qd_pred = qd.copy()
                self._ik.reset(q)
                self.get_logger().info(
                    f"Joint states received on '{self._js_topic}' — q={np.round(q,3)}\n"
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
            self._q_pred  = q_init.copy()
            self._qd_pred = np.zeros(6)
            self._last_qd_cmd[:] = 0.0
            self._qdd_des_filt[:] = 0.0
            self._qd_des_filt[:]  = 0.0
            self._qd_cmd_filt[:]  = 0.0
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
            state_fresh = self._state_updated
            self._state_updated = False

        # --- State predictor update (CTC only) ---
        # If a new hardware measurement just arrived, reset the predictor to the
        # real state.  Otherwise advance it by integrating the last command so
        # that CTC dynamics and IK always see a smooth, non-frozen q/qd.
        if self._use_ctc:
            if state_fresh:
                self._q_pred  = q.copy()
                self._qd_pred = qd.copy()
            else:
                self._q_pred  = self._q_pred + self._last_qd_cmd * DT
                self._qd_pred = self._last_qd_cmd.copy()

        t_rel = (self.get_clock().now() - self._t_start).nanoseconds * 1e-9

        if t_rel > self._cart_traj.total_time:
            if self._loop:
                self._t_start = self.get_clock().now()
                self._e_int[:]       = 0.0
                self._qd_cmd_ctc[:]  = 0.0
                self._prev_e_pd[:]   = 0.0
                self._prev_vel_pd[:] = 0.0
                self._last_qd_cmd[:] = 0.0
                self._q_pred  = q.copy()
                self._qd_pred = np.zeros(6)
                self._qdd_des_filt[:] = 0.0
                self._qd_des_filt[:]  = 0.0
                self._qd_cmd_filt[:]  = 0.0
                self._ik.reset(q)
                self._wp_reached = {}   # reset reach flags for the new lap
                t_rel = 0.0
            else:
                self._publish_twist(np.zeros(3))
                self._flush_csv()
                return

        ctp = self._cart_traj.at(t_rel)

        if self._use_ctc:
            # Use TF2 for Cartesian position (updates faster than /joint_states)
            # and the state predictor for joint-space (smooth between hw updates).
            p_tf2_ctc = self._read_pose_tf()
            # --- Diagnostic: compare analytical FK against TF2 actual EE pose ---
            if p_tf2_ctc is not None:
                p_fk_q = forward_kinematics(q)[0]
                fk_tf_err_mm = np.linalg.norm(p_tf2_ctc - p_fk_q) * 1000.0
                self.get_logger().info(
                    f"FK(q)-vs-TF error = {fk_tf_err_mm:.1f} mm  "
                    f"TF={np.round(p_tf2_ctc,4)}  FK={np.round(p_fk_q,4)}",
                    throttle_duration_sec=1.0,
                )

                p_fk_pred = forward_kinematics(self._q_pred)[0]
                fk_pred_tf_err_mm = np.linalg.norm(p_tf2_ctc - p_fk_pred) * 1000.0
                self.get_logger().info(
                    f"FK(q_pred)-vs-TF error = {fk_pred_tf_err_mm:.1f} mm",
                    throttle_duration_sec=1.0,
                )
                if fk_pred_tf_err_mm > 30.0:
                    self.get_logger().warn(
                        f'q_pred drifted {fk_pred_tf_err_mm:.1f} mm — resetting IK state',
                        throttle_duration_sec=0.5)
                    self._ik._qd_prev = np.zeros(6)
            q_des, qd_des, qdd_des = self._ik.step(
                ctp.p, ctp.pd, ctp.pdd,
                p_actual=p_tf2_ctc,
                q_actual=q)  # always anchor Jacobian to actual hardware state

            # --- IK sanity check (MUST run before wrist override) ---
            # Checks FK(q_des_ik) vs p_des using the full IK solution (all 6 joints).
            # After wrist pins are applied q_des[3:] changes, making FK irrelevant.
            p_ik_check = forward_kinematics(q_des)[0]
            ik_err_mm = np.linalg.norm(ctp.p - p_ik_check) * 1000
            if ik_err_mm > 5.0:
                self.get_logger().warn(
                    f'IK diverged: {ik_err_mm:.1f}mm  '
                    f'q_des={np.round(q_des,3)}  q_pred={np.round(self._q_pred,3)}',
                    throttle_duration_sec=0.5)

            # Pin wrist joints (4-6) to actual hardware config — task-space IK
            # only needs to move joints 1-3 for 3-DOF position control.
            q_des[3:] = q[3:]
            qd_des[3:] = 0.0
            qdd_des[3:] = 0.0

            # --- Filter cascade (CTC-specific) ---
            # 1. Hard clamp qdd_des: prevents M·v blow-up from spike accelerations
            #    (raw max observed: ±114 rad/s² every ~10 rows at 9-Hz state updates)
            qdd_raw = qdd_des.copy()   # preserve pre-clamp value for logging
            qdd_des = np.clip(qdd_des, -self._max_qdd, self._max_qdd)

            # 2. LPF on qdd_des (fc=5 Hz): removes residual noise after clamp
            self._qdd_des_filt = (self._a_qdd * self._qdd_des_filt
                                  + (1.0 - self._a_qdd) * qdd_des)

            # 3. LPF on qd_des (fc=10 Hz): smooths IK velocity → quieter ė = qd_des − qd
            self._qd_des_filt = (self._a_qd * self._qd_des_filt
                                 + (1.0 - self._a_qd) * qd_des)

            # CTC uses actual hardware state so that e = q_des - q is always small
            # and correct (predictor drifts when servo ignores JointJog commands)
            qd_cmd, sat_flags = self._ctc(
                q, qd,
                q_des, self._qd_des_filt, self._qdd_des_filt)

            qd_sat = np.clip(qd_cmd, -VEL_LIMIT, VEL_LIMIT)
            sat_flags |= (np.abs(qd_cmd) > VEL_LIMIT)

            # 4. LPF on qd_cmd output (fc=20 Hz): smooths servo input so the
            #    trajectory controller sees gradual velocity changes, not steps
            self._qd_cmd_filt = (self._a_cmd * self._qd_cmd_filt
                                 + (1.0 - self._a_cmd) * qd_sat)
            qd_sat = np.clip(self._qd_cmd_filt, -VEL_LIMIT, VEL_LIMIT)

            self._last_qd_cmd = qd_sat.copy()  # store for next-tick prediction
            # Convert joint velocity → Cartesian velocity so CTC shares the same
            # servo channel (/servo_server/delta_twist_cmds) as the perturbation
            # injector.  MoveIt Servo alternates command-type when it receives both
            # JointJog and TwistStamped simultaneously, silently dropping ~half of
            # the CTC joint commands and causing divergence.  Using one channel
            # avoids that conflict while keeping all CTC dynamics math unchanged.
            J_q = position_jacobian(q)
            pd_cmd = J_q @ qd_sat
            # Wrist singularity: scale Cartesian command when joint5 (index 4) is
            # near 0.  The hard_stop threshold in the Servo config is raised to
            # 50000 so Servo no longer emergency-stops, but we also limit the
            # commanded velocity here to reduce IK amplification through the
            # near-singular 6×6 Jacobian.  At |q5| ≥ 0.20 rad the scale is 1.0
            # (full speed); at |q5| = 0 it floors at 0.05 (5 % speed, never zero
            # so Servo stays active).
            wrist_scale = float(np.clip(abs(q[4]) / 0.20, 0.05, 1.0))
            if wrist_scale < 1.0:
                pd_cmd = pd_cmd * wrist_scale
                self.get_logger().warn(
                    f'Wrist near singular: q5={np.degrees(q[4]):.1f}° '
                    f'→ vel scale={wrist_scale:.2f}',
                    throttle_duration_sec=1.0)
            self._publish_twist(pd_cmd)
            p_act = p_tf2_ctc if p_tf2_ctc is not None else forward_kinematics(q)[0]
            p_des      = ctp.p
            cmd_log    = qd_sat.tolist()
            sat_log    = sat_flags.astype(int).tolist()
            # Extended log fields (CTC-specific)
            q_pred_log    = self._q_pred.tolist()
            qdd_raw_log   = qdd_raw.tolist()
            qdd_filt_log  = self._qdd_des_filt.tolist()
            qd_filt_log   = self._qd_des_filt.tolist()
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
            # Extended log fields (PD: no predictor/filters — use neutral values)
            q_pred_log   = q.tolist()
            qdd_raw_log  = [0.0] * 6
            qdd_filt_log = [0.0] * 6
            qd_filt_log  = [0.0] * 6

        wp_idx, label, phase = ctp.wp_idx, ctp.label, ctp.phase

        # ── Waypoint reach detection ─────────────────────────────────────────
        # During dwell phases the desired position is held constant — this is
        # where we can meaningfully judge whether the EE reached the target.
        if phase == 'dwell' and label not in self._wp_reached:
            err_wp = np.linalg.norm(p_act - p_des)
            if err_wp <= self._wp_reach_thr_m:
                self._wp_reached[label] = True
                self.get_logger().info(
                    f'WAYPOINT REACHED  {label}  |err|={err_wp*1000:.1f} mm  '
                    f't={t_rel:.2f}s')
            else:
                self.get_logger().warn(
                    f'Waypoint NOT yet reached  {label}  '
                    f'|err|={err_wp*1000:.1f} mm (thr={self._wp_reach_thr_m*1000:.0f} mm)',
                    throttle_duration_sec=1.0)

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
                [int(self._pert_en), pert_changed, wp_idx, label, phase] +
                # Extended diagnostics
                [int(state_fresh)] +       # 1 = hardware updated this tick (~9 Hz)
                q_pred_log +               # CTC: predicted q;  PD: actual q
                qdd_raw_log +              # CTC: raw IK qdd before clamp; PD: zeros
                qdd_filt_log +             # CTC: filtered qdd entering τ;  PD: zeros
                qd_filt_log                # CTC: filtered qd_des entering ė; PD: zeros
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
            # Actual hardware state (updates ~9 Hz)
            [f'q_{i}'  for i in range(6)] + [f'qd_{i}' for i in range(6)] +
            # Desired trajectory (from IK for CTC, from traj for PD)
            [f'q_des_{i}' for i in range(6)] + [f'qd_des_{i}' for i in range(6)] +
            # qdd_des: CTC=clamped IK acc; PD=zeros
            [f'qdd_des_{i}' for i in range(6)] +
            # Cartesian feedback
            ['p_act_x','p_act_y','p_act_z'] + ['p_des_x','p_des_y','p_des_z'] +
            # Commands sent to servo: CTC=filtered qd; PD=v_cmd(x,y,z)+0s
            [f'qd_cmd_{i}' for i in range(6)] + [f'sat_{i}' for i in range(6)] +
            ['pert_enabled','pert_changed','wp_idx','wp_label','phase'] +
            # ── Extended diagnostics ───────────────────────────────────────────
            # state_fresh=1 when /joint_states had a new message this tick (~9 Hz)
            ['state_fresh'] +
            # CTC: predicted joint pos (integrates qd_cmd between hw updates); PD: =q
            [f'q_pred_{i}' for i in range(6)] +
            # CTC: raw IK qdd_des BEFORE clamp (reveals 9-Hz spikes); PD: zeros
            [f'qdd_raw_{i}' for i in range(6)] +
            # CTC: qdd_des AFTER clamp+LPF (what enters τ=Mv); PD: zeros
            [f'qdd_filt_{i}' for i in range(6)] +
            # CTC: qd_des AFTER LPF (what enters ė=qd_des−qd); PD: zeros
            [f'qd_filt_{i}' for i in range(6)]
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
