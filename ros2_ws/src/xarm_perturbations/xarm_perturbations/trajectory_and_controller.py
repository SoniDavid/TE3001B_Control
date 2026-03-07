#!/usr/bin/env python3
"""
trajectory_and_controller.py

MoveIt Servo Cartesian PD controller for xArm Lite 6:
- Generates a waypoint-based CMM inspection trajectory visiting 8 discrete
  points across 3 feature regions on a simulated machined part.
- Quintic-spline interpolation between waypoints (zero velocity & acceleration
  at every endpoint — no velocity or acceleration jumps).
- Configurable dwell periods at inspection waypoints.
- Lateral moves at low Z between adjacent inspection points within the same
  region (probe stays at measurement height without ascending).
- Tracks using PD in task-space (TwistStamped -> /servo_server/delta_twist_cmds)
- Publishes RViz markers:
    /traj_marker   -> LINE_STRIP of the complete waypoint path
    /target_marker -> SPHERE at the current interpolated target
    /wp_markers    -> SPHERE per waypoint (orange=inspection, blue=transit)
- Publishes per-cycle metrics:
    /cycle_metrics -> Float64MultiArray [rmse_x, rmse_y, rmse_z, rmse_total, sat_ratio]

Waypoints (relative to center set at startup, see WAYPOINTS_REL):
  WP1  transit_A     (-0.09, +0.07,  0.00)  high  — approach region A
  WP2  inspect_F1    (-0.09, +0.07, -0.10)  low   — probe bore F1       [dwell]
  WP3  inspect_F2    (+0.01, +0.09, -0.10)  low   — probe bore F2       [dwell, lateral]
  WP4  transit_B     (+0.10,  0.00,  0.00)  high  — ascend + transit B
  WP5  inspect_F3    (+0.10,  0.00, -0.10)  low   — probe bore F3       [dwell]
  WP6  inspect_F4    (+0.06, -0.08, -0.10)  low   — probe bore F4       [dwell, lateral]
  WP7  transit_C     (-0.06, -0.05,  0.00)  high  — ascend + transit C
  WP8  inspect_F5    (-0.06, -0.05, -0.10)  low   — surface scan point  [dwell]
  WP9  home          ( 0.00,  0.00,  0.00)  high  — return to center

Run example:
  ros2 run xarm_perturbations trajectory_and_controller --ros-args \
    -p segment_sec:=2.0 -p dwell_sec:=1.5 \
    -p kp:="[2.665,2.665,2.665]" -p kd:="[0.64,0.64,0.64]" \
    -p max_speed:=0.12 -p deadband:=0.002 \
    -p save_csv:=true -p csv_filename:=inspection_baseline.csv
"""

import math
import csv
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import TwistStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener


# ---------------------------------------------------------------------------
# Inspection trajectory — 8 waypoints + return home
# (dx, dy, dz relative to center)  |  dwell_sec (0 = no dwell)  |  label
# ---------------------------------------------------------------------------
WAYPOINTS_REL = [
    (-0.09, +0.07,  0.00, 0.0, "transit_A"),    # WP1 — transit to region A
    (-0.09, +0.07, -0.10, 1.5, "inspect_F1"),   # WP2 — probe bore F1          [dwell]
    (+0.01, +0.09, -0.10, 1.5, "inspect_F2"),   # WP3 — probe bore F2 lateral  [dwell]
    (+0.10,  0.00,  0.00, 0.0, "transit_B"),    # WP4 — ascend + transit B
    (+0.10,  0.00, -0.10, 1.5, "inspect_F3"),   # WP5 — probe bore F3          [dwell]
    (+0.06, -0.08, -0.10, 1.5, "inspect_F4"),   # WP6 — probe bore F4 lateral  [dwell]
    (-0.06, -0.05,  0.00, 0.0, "transit_C"),    # WP7 — ascend + transit C
    (-0.06, -0.05, -0.10, 1.5, "inspect_F5"),   # WP8 — surface scan point     [dwell]
    ( 0.00,  0.00,  0.00, 0.0, "home"),         # WP9 — return to center
]

N_WP = len(WAYPOINTS_REL)  # 9


# ---------------------------------------------------------------------------
# Quintic blend helpers
# ---------------------------------------------------------------------------
def _quintic(tau: float) -> float:
    """Normalised quintic polynomial: [0,1]->[0,1], zero vel & accel at endpoints."""
    tau = max(0.0, min(1.0, tau))
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


# ---------------------------------------------------------------------------
# State enums
# ---------------------------------------------------------------------------
class SegState:
    IN_SEGMENT = 0
    IN_DWELL   = 1
    COMPLETE   = 2


class RobotState:
    RUNNING = 1
    PAUSED  = 2
    HOME    = 3


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------
class TrajectoryAndController(Node):

    def __init__(self):
        super().__init__("trajectory_and_controller")

        # ── Trajectory parameters ──────────────────────────────────────────
        self.segment_sec = float(self.declare_parameter("segment_sec", 2.0).value)
        self.dwell_sec   = float(self.declare_parameter("dwell_sec",   1.5).value)
        self.loop_traj   = bool(self.declare_parameter("loop_traj",   True).value)

        # ── Controller parameters ──────────────────────────────────────────
        self.declare_parameter("kp", [2.5, 2.5, 2.5])
        self.declare_parameter("kd", [0.6, 0.6, 0.6])
        self.declare_parameter("max_speed", 0.12)
        self.declare_parameter("deadband", 0.002)
        self.declare_parameter("enable_keyboard", False)

        self.kp = np.array(
            self.get_parameter("kp").get_parameter_value().double_array_value,
            dtype=float)
        self.kd = np.array(
            self.get_parameter("kd").get_parameter_value().double_array_value,
            dtype=float)
        self.max_speed = float(self.get_parameter("max_speed").value)
        db = float(self.get_parameter("deadband").value)
        self.epsilon = np.array([db, db, db], dtype=float)
        self.enable_keyboard = bool(self.get_parameter("enable_keyboard").value)

        # ── PID optional integral term + anti-windup ───────────────────────
        # Default: use_pid=False → pure PD (no change to existing behaviour).
        # Set use_pid:=true to activate the integral term with clamped
        # anti-windup.  i_limit is the per-axis saturation bound (m) applied
        # to the accumulated integral before multiplying by ki.
        self.use_pid = bool(self.declare_parameter("use_pid", False).value)
        self.ki = np.array(
            self.declare_parameter("ki", [0.1, 0.1, 0.1])
                .get_parameter_value().double_array_value,
            dtype=float)
        self.i_limit = float(self.declare_parameter("i_limit", 0.05).value)
        self.error_integral = np.zeros(3, dtype=float)

        # ── LPF parameters ─────────────────────────────────────────────────
        self.use_lpf   = bool(self.declare_parameter("use_lpf",    True).value)
        self.lpf_fc_hz = float(self.declare_parameter("lpf_fc_hz", 2.0).value)
        self.v_filt    = np.zeros(3, dtype=float)

        # ── CSV logging ────────────────────────────────────────────────────
        self.declare_parameter("save_csv",      False)
        self.declare_parameter("csv_filename",  "tracking_data.csv")
        self.save_csv     = bool(self.get_parameter("save_csv").value)
        self.csv_filename = str(self.get_parameter("csv_filename").value)
        self.csv_file     = None
        self.csv_writer   = None

        if self.save_csv:
            os.makedirs("data", exist_ok=True)
            self.csv_path = os.path.join("data", self.csv_filename)
            self.csv_file = open(self.csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "time", "wp_idx", "wp_label", "phase",
                "x_des", "y_des", "z_des",
                "x_act", "y_act", "z_act",
                "ex", "ey", "ez",
                "vx_cmd", "vy_cmd", "vz_cmd",
                "v_norm", "is_saturated",
            ])
            self.get_logger().info(f"Saving CSV to {self.csv_path}")

        # ── Publishers ─────────────────────────────────────────────────────
        self.servo_pub         = self.create_publisher(
            TwistStamped,      "/servo_server/delta_twist_cmds", 10)
        self.traj_marker_pub   = self.create_publisher(
            Marker,            "/traj_marker",   10)
        self.target_marker_pub = self.create_publisher(
            Marker,            "/target_marker", 10)
        self.wp_marker_pub     = self.create_publisher(
            Marker,            "/wp_markers",    10)
        self.metrics_pub       = self.create_publisher(
            Float64MultiArray, "/cycle_metrics", 10)

        # ── TF ─────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Robot state machine ────────────────────────────────────────────
        self.robot_state   = RobotState.RUNNING
        self.home_position = np.array([0.215, 0.00, 0.468], dtype=float)
        self.center        = None
        self.waypoints     = None   # built once center is known

        # ── Trajectory sequencer state ─────────────────────────────────────
        self.seg_state        = SegState.IN_SEGMENT
        self.current_wp_idx   = 0
        self.seg_start_time   = None
        self.dwell_start_time = None
        self.prev_wp_pos      = None  # absolute position of previous waypoint

        # ── PD derivative memory ───────────────────────────────────────────
        self.prev_time  = self.get_clock().now()
        self.prev_error = np.zeros(3)

        # ── Metrics accumulators ───────────────────────────────────────────
        self.cumulative_sq_error = np.zeros(3)
        self.sample_count = 0
        self.sat_count    = 0
        self.total_count  = 0
        self.cycle_count  = 0

        # ── Throttle timestamps ────────────────────────────────────────────
        self.last_info_time   = self.get_clock().now()
        self.last_marker_time = self.get_clock().now()
        self.start_time       = self.get_clock().now()

        # ── Optional keyboard ──────────────────────────────────────────────
        if self.enable_keyboard:
            self._start_keyboard()

        # ── Live gain update callback ──────────────────────────────────────
        self.add_on_set_parameters_callback(self._on_param_change)

        # ── 50 Hz control loop ─────────────────────────────────────────────
        self.timer = self.create_timer(0.02, self._loop)

        self.get_logger().info(
            "TrajectoryAndController (CMM inspection trajectory) started.\n"
            f"  segment_sec={self.segment_sec}  dwell_sec={self.dwell_sec}  "
            f"loop={self.loop_traj}\n"
            f"  kp={self.kp.tolist()}  kd={self.kd.tolist()}  "
            f"max_speed={self.max_speed}"
        )

    # ──────────────────────────────────────────────────────────────────────
    # Waypoint setup
    # ──────────────────────────────────────────────────────────────────────
    def _build_waypoints(self):
        """Convert relative waypoints to absolute positions using current center."""
        cx, cy, cz = self.center
        self.waypoints = []
        for dx, dy, dz, dwell_flag, label in WAYPOINTS_REL:
            self.waypoints.append({
                "pos":   np.array([cx + dx, cy + dy, cz + dz], dtype=float),
                "dwell": self.dwell_sec if dwell_flag > 0.0 else 0.0,
                "label": label,
            })

    # ──────────────────────────────────────────────────────────────────────
    # Quintic interpolation
    # ──────────────────────────────────────────────────────────────────────
    def _interpolate(self, p0: np.ndarray, p1: np.ndarray, elapsed: float) -> np.ndarray:
        tau = elapsed / max(self.segment_sec, 1e-6)
        return p0 + (p1 - p0) * _quintic(tau)

    # ──────────────────────────────────────────────────────────────────────
    # Current desired target
    # ──────────────────────────────────────────────────────────────────────
    def _get_target(self) -> np.ndarray:
        wp = self.waypoints[self.current_wp_idx]
        if self.seg_state == SegState.IN_DWELL:
            return wp["pos"].copy()
        elapsed = (self.get_clock().now() - self.seg_start_time).nanoseconds / 1e9
        return self._interpolate(self.prev_wp_pos, wp["pos"], elapsed)

    # ──────────────────────────────────────────────────────────────────────
    # Sequencer advance (called every control tick)
    # ──────────────────────────────────────────────────────────────────────
    def _advance_sequencer(self):
        now = self.get_clock().now()

        if self.seg_state == SegState.IN_SEGMENT:
            elapsed = (now - self.seg_start_time).nanoseconds / 1e9
            if elapsed < self.segment_sec:
                return
            wp = self.waypoints[self.current_wp_idx]
            if wp["dwell"] > 0.0:
                self.seg_state        = SegState.IN_DWELL
                self.dwell_start_time = now
                self.get_logger().info(
                    f"[WP{self.current_wp_idx + 1}] {wp['label']} — "
                    f"dwell {wp['dwell']:.1f} s"
                )
            else:
                self._start_next_segment(now)

        elif self.seg_state == SegState.IN_DWELL:
            elapsed = (now - self.dwell_start_time).nanoseconds / 1e9
            if elapsed < self.waypoints[self.current_wp_idx]["dwell"]:
                return
            self._start_next_segment(now)

    def _start_next_segment(self, now):
        wp = self.waypoints[self.current_wp_idx]
        self.prev_wp_pos  = wp["pos"].copy()
        self.current_wp_idx += 1

        if self.current_wp_idx >= N_WP:
            self.cycle_count += 1
            self._publish_cycle_metrics()
            self.get_logger().info(f"Cycle {self.cycle_count} complete.")
            if self.loop_traj:
                self.current_wp_idx = 0
                self.seg_state      = SegState.IN_SEGMENT
                self.seg_start_time = now
            else:
                self.seg_state = SegState.COMPLETE
            return

        self.seg_state      = SegState.IN_SEGMENT
        self.seg_start_time = now
        self.get_logger().info(
            f"Moving to WP{self.current_wp_idx + 1} "
            f"[{self.waypoints[self.current_wp_idx]['label']}]"
        )

    # ──────────────────────────────────────────────────────────────────────
    # TF read
    # ──────────────────────────────────────────────────────────────────────
    def _read_position(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                "link_base", "link_eef", rclpy.time.Time())
            return np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
            ], dtype=float)
        except Exception as e:
            now = self.get_clock().now()
            if (now - self.last_info_time).nanoseconds > 2e9:
                self.get_logger().warn(f"TF not ready: {e}")
                self.last_info_time = now
            return None

    # ──────────────────────────────────────────────────────────────────────
    # Publish helpers
    # ──────────────────────────────────────────────────────────────────────
    def _publish_twist(self, v_xyz: np.ndarray):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = "link_base"
        cmd.twist.linear.x  = float(v_xyz[0])
        cmd.twist.linear.y  = float(v_xyz[1])
        cmd.twist.linear.z  = float(v_xyz[2])
        cmd.twist.angular.x = cmd.twist.angular.y = cmd.twist.angular.z = 0.0
        self.servo_pub.publish(cmd)

    def _publish_zero(self):
        self._publish_twist(np.zeros(3, dtype=float))

    # ──────────────────────────────────────────────────────────────────────
    # PD control step  (identical to Lab 4.2)
    # ──────────────────────────────────────────────────────────────────────
    def _servo_to(self, target_pos: np.ndarray, t_sec: float):
        current = self._read_position()
        if current is None:
            return

        now = self.get_clock().now()
        dt  = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1e-6

        error   = target_pos - current
        d_error = (error - self.prev_error) / dt

        active_mask = np.abs(error) > self.epsilon

        # PID mode: accumulate integral only on active axes, then clamp
        # (integral clamping = anti-windup: prevents integrator wind-up
        #  when the actuator is saturated or the error is outside deadband).
        if self.use_pid:
            self.error_integral = np.clip(
                self.error_integral + np.where(active_mask, error, 0.0) * dt,
                -self.i_limit, self.i_limit
            )
            v = (self.kp * error
                 + self.kd * d_error
                 + self.ki * self.error_integral)
        else:
            v = self.kp * error + self.kd * d_error

        v = np.where(active_mask, v, 0.0)

        # Optional first-order LPF on commanded velocity
        if self.use_lpf:
            fc    = max(self.lpf_fc_hz, 1e-3)
            alpha = math.exp(-2.0 * math.pi * fc * dt)
            self.v_filt = alpha * self.v_filt + (1.0 - alpha) * v
            v = self.v_filt

        # Norm saturation (preserves direction)
        v_norm = float(np.linalg.norm(v))
        was_saturated = False
        if v_norm > self.max_speed:
            v = v * (self.max_speed / max(v_norm, 1e-9))
            was_saturated = True
            v_norm = float(np.linalg.norm(v))

        # Metrics accumulation
        self.cumulative_sq_error += error ** 2
        self.sample_count += 1
        self.total_count  += 1
        if was_saturated:
            self.sat_count += 1

        # CSV logging
        if self.save_csv and self.csv_writer is not None:
            wp    = self.waypoints[self.current_wp_idx]
            phase = "dwell" if self.seg_state == SegState.IN_DWELL else "segment"
            self.csv_writer.writerow([
                float(t_sec),
                self.current_wp_idx, wp["label"], phase,
                float(target_pos[0]), float(target_pos[1]), float(target_pos[2]),
                float(current[0]),    float(current[1]),    float(current[2]),
                float(error[0]),      float(error[1]),      float(error[2]),
                float(v[0]),          float(v[1]),          float(v[2]),
                float(v_norm),        1.0 if was_saturated else 0.0,
            ])
            if (self.total_count % 50) == 0 and self.csv_file is not None:
                self.csv_file.flush()

        self._publish_twist(v)
        self.prev_error = error
        self.prev_time  = now

    # ──────────────────────────────────────────────────────────────────────
    # Cycle metrics  (published at end of every full pass)
    # ──────────────────────────────────────────────────────────────────────
    def _publish_cycle_metrics(self):
        if self.sample_count == 0:
            return
        rmse_xyz   = np.sqrt(self.cumulative_sq_error / self.sample_count)
        rmse_total = float(np.linalg.norm(rmse_xyz))
        sat_ratio  = float(self.sat_count / max(self.total_count, 1))

        msg = Float64MultiArray()
        msg.data = [
            float(rmse_xyz[0]), float(rmse_xyz[1]), float(rmse_xyz[2]),
            rmse_total, sat_ratio,
        ]
        self.metrics_pub.publish(msg)
        self.get_logger().info(
            f"Cycle {self.cycle_count} | "
            f"RMSE xyz=({rmse_xyz[0]:.5f},{rmse_xyz[1]:.5f},{rmse_xyz[2]:.5f}) "
            f"total={rmse_total:.5f} sat={sat_ratio:.3f}"
        )

        # Reset accumulators
        self.cumulative_sq_error[:] = 0.0
        self.sample_count = self.sat_count = self.total_count = 0

    # ──────────────────────────────────────────────────────────────────────
    # RViz markers
    # ──────────────────────────────────────────────────────────────────────
    def _publish_markers(self, target: np.ndarray):
        now = self.get_clock().now()
        if (now - self.last_marker_time).nanoseconds < 1e8:
            return
        self.last_marker_time = now

        if self.waypoints is None:
            return

        # -- PATH: LINE_STRIP connecting center -> all waypoints
        m = Marker()
        m.header.frame_id    = "link_base"
        m.header.stamp       = now.to_msg()
        m.ns, m.id           = "traj", 0
        m.type               = Marker.LINE_STRIP
        m.action             = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x            = 0.004
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0

        # start at center (robot home for this trajectory)
        p0 = Point()
        p0.x, p0.y, p0.z = float(self.center[0]), float(self.center[1]), float(self.center[2])
        m.points.append(p0)
        for wp in self.waypoints:
            p = Point()
            p.x, p.y, p.z = float(wp["pos"][0]), float(wp["pos"][1]), float(wp["pos"][2])
            m.points.append(p)
        self.traj_marker_pub.publish(m)

        # -- WAYPOINT SPHERES: orange = inspection (has dwell), blue = transit
        for i, wp in enumerate(self.waypoints):
            s = Marker()
            s.header.frame_id    = "link_base"
            s.header.stamp       = now.to_msg()
            s.ns, s.id           = "waypoints", i
            s.type               = Marker.SPHERE
            s.action             = Marker.ADD
            s.pose.position.x    = float(wp["pos"][0])
            s.pose.position.y    = float(wp["pos"][1])
            s.pose.position.z    = float(wp["pos"][2])
            s.pose.orientation.w = 1.0
            s.scale.x = s.scale.y = s.scale.z = 0.015
            if wp["dwell"] > 0.0:
                s.color.r, s.color.g, s.color.b, s.color.a = 1.0, 0.5, 0.0, 0.9  # orange
            else:
                s.color.r, s.color.g, s.color.b, s.color.a = 0.0, 0.5, 1.0, 0.7  # blue
            self.wp_marker_pub.publish(s)

        # -- CURRENT TARGET: red sphere
        t = Marker()
        t.header.frame_id    = "link_base"
        t.header.stamp       = now.to_msg()
        t.ns, t.id           = "target", 0
        t.type               = Marker.SPHERE
        t.action             = Marker.ADD
        t.pose.position.x    = float(target[0])
        t.pose.position.y    = float(target[1])
        t.pose.position.z    = float(target[2])
        t.pose.orientation.w = 1.0
        t.scale.x = t.scale.y = t.scale.z = 0.02
        t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 0.0, 0.0, 1.0
        self.target_marker_pub.publish(t)

    # ──────────────────────────────────────────────────────────────────────
    # Main loop (50 Hz)
    # ──────────────────────────────────────────────────────────────────────
    def _loop(self):
        # ── Initialize center & waypoints once ────────────────────────────
        if self.center is None:
            p = self._read_position()
            if p is None:
                return
            self.center = p.copy()
            self._build_waypoints()
            self.prev_wp_pos    = self.center.copy()
            self.current_wp_idx = 0
            self.seg_state      = SegState.IN_SEGMENT
            self.seg_start_time = self.get_clock().now()
            self.start_time     = self.get_clock().now()
            self.prev_time       = self.get_clock().now()
            self.prev_error      = np.zeros(3)
            self.error_integral  = np.zeros(3)
            self.get_logger().info(
                f"Center set to {self.center.round(3)}. "
                "Starting inspection trajectory."
            )
            return

        # ── PAUSED ────────────────────────────────────────────────────────
        if self.robot_state == RobotState.PAUSED:
            return

        # ── HOME ──────────────────────────────────────────────────────────
        if self.robot_state == RobotState.HOME:
            current = self._read_position()
            if current is None:
                return
            error = self.home_position - current
            if float(np.linalg.norm(error)) < 0.005:
                self._publish_zero()
                self.robot_state = RobotState.RUNNING
                self.center      = current.copy()
                self._build_waypoints()
                self.prev_wp_pos    = self.center.copy()
                self.current_wp_idx = 0
                self.seg_state      = SegState.IN_SEGMENT
                self.seg_start_time = self.get_clock().now()
                self.prev_time      = self.get_clock().now()
                self.prev_error     = np.zeros(3)
                self.get_logger().info("Home reached. Re-centered, restarting trajectory.")
                return
            direction = np.where(np.abs(error) > 1e-4, np.sign(error), 0.0)
            v = direction * 0.08
            v_norm = float(np.linalg.norm(v))
            if v_norm > self.max_speed:
                v = v * (self.max_speed / max(v_norm, 1e-9))
            self._publish_twist(v)
            return

        # ── COMPLETE (loop_traj=False, all waypoints visited) ─────────────
        if self.seg_state == SegState.COMPLETE:
            self._publish_zero()
            return

        # ── RUNNING ───────────────────────────────────────────────────────
        t_sec = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        self._advance_sequencer()

        if self.seg_state == SegState.COMPLETE:
            self._publish_zero()
            return

        target = self._get_target()
        self._publish_markers(target)
        self._servo_to(target, t_sec)

    # ──────────────────────────────────────────────────────────────────────
    # Live gain update  (compatible with auto_tuner SetParameters service)
    # ──────────────────────────────────────────────────────────────────────
    def _on_param_change(self, params):
        kp_new = kd_new = None
        for p in params:
            if p.name == "kp" and p.type_ == Parameter.Type.DOUBLE_ARRAY:
                kp_new = np.array(p.value, dtype=float)
            if p.name == "kd" and p.type_ == Parameter.Type.DOUBLE_ARRAY:
                kd_new = np.array(p.value, dtype=float)
        if kp_new is not None:
            if kp_new.shape != (3,):
                return SetParametersResult(successful=False, reason="kp must have length 3")
            self.kp = kp_new
        if kd_new is not None:
            if kd_new.shape != (3,):
                return SetParametersResult(successful=False, reason="kd must have length 3")
            self.kd = kd_new
        self.prev_error      = np.zeros(3)
        self.error_integral  = np.zeros(3)
        self.prev_time       = self.get_clock().now()
        self.get_logger().info(f"Gains updated: kp={self.kp.tolist()} kd={self.kd.tolist()}")
        return SetParametersResult(successful=True)

    # ──────────────────────────────────────────────────────────────────────
    # Optional keyboard control
    # ──────────────────────────────────────────────────────────────────────
    def _start_keyboard(self):
        try:
            from pynput import keyboard
        except Exception as e:
            self.get_logger().warn(f"Keyboard disabled (pynput not available): {e}")
            self.enable_keyboard = False
            return

        def on_press(key):
            if hasattr(key, "char") and key.char == "p":
                if self.robot_state == RobotState.RUNNING:
                    self.robot_state = RobotState.PAUSED
                    self._publish_zero()
                    self.get_logger().warn("Paused.")
                else:
                    self.robot_state = RobotState.RUNNING
                    self.prev_time   = self.get_clock().now()
                    self.prev_error  = np.zeros(3)
                    self.get_logger().info("Resumed.")
            if hasattr(key, "char") and key.char == "h":
                self.robot_state = RobotState.HOME
                self.get_logger().info("Going HOME...")

        self.keyboard_listener = keyboard.Listener(on_press=on_press)
        self.keyboard_listener.start()
        self.get_logger().info("Keyboard: 'p'=pause/resume  'h'=home")

    # ──────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        if self.save_csv and self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryAndController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
