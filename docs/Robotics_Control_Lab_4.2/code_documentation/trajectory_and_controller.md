# trajectory_and_controller.py

**Package**: `xarm_perturbations`
**Node name**: `trajectory_and_controller`
**Entry point**: `trajectory_and_controller = xarm_perturbations.trajectory_and_controller:main`

---

## Purpose

This is the **main node** of Lab 4.2. It:

1. Executes a **waypoint-based CMM inspection trajectory** — 8 discrete points across 3 feature regions on a simulated machined part, with quintic-spline interpolation and dwell periods
2. Reads the current end-effector position via **TF2**
3. Computes a **Cartesian PD control** velocity to track the trajectory
4. Publishes velocity commands via **MoveIt Servo** (`TwistStamped`)
5. Optionally logs all data to a **CSV file**
6. Publishes **RViz markers** (trajectory path, waypoint spheres, current target)
7. Publishes **per-cycle RMSE** metrics at the end of every full inspection pass

For the trajectory design rationale and waypoint table, see [inspection_trajectory.md](inspection_trajectory.md).

---

## How the Robot Moves

This node does **not** command joint positions or torques directly. It uses **MoveIt Servo's Cartesian velocity interface**:

```
v_cmd (TwistStamped, linear only) -> /servo_server/delta_twist_cmds
```

MoveIt Servo converts the continuous stream of Cartesian velocity commands into joint velocities using the robot's Jacobian at the current configuration. The node must keep publishing at **50 Hz** — if it stops, MoveIt Servo times out and the robot stops.

The `frame_id` is `"link_base"`: velocity components (x, y, z) are in the robot base frame.

---

## Trajectory: Waypoint Sequencer

The trajectory is not a continuous formula. It is a **finite state machine** that steps through a list of waypoints.

```
WAYPOINTS_REL (9 entries, defined at module level)
        │
        ▼ _build_waypoints()  — called once, after center is set from TF
WAYPOINTS (absolute positions in link_base)
        │
        ▼ _loop() at 50 Hz
_advance_sequencer()
  ├── IN_SEGMENT: quintic blend from prev_wp to current_wp over segment_sec
  │       └── on arrival → IN_DWELL (if dwell > 0) or next segment
  └── IN_DWELL:  hold at waypoint for dwell_sec
          └── on expiry → next segment (or COMPLETE if last waypoint)
```

### Quintic blend

Transitions between waypoints use a **quintic polynomial** in normalised time τ = t/T:

```
h(τ) = 10τ³ − 15τ⁴ + 6τ⁵
p(t) = p₀ + (p₁ − p₀) · h(τ)
```

This guarantees **zero velocity and zero acceleration at both endpoints** — no step changes in the commanded velocity that could cause jerk at the robot joints. It is equivalent to the "minimum-jerk" polynomial widely used in robot motion planning.

---

## Control Loop (50 Hz)

```
_loop() every 20 ms
    │
    ├── Initialize center (first TF read) → _build_waypoints()
    ├── [PAUSED]   → do nothing
    ├── [HOME]     → proportional homing at 0.08 m/s, re-center on arrival
    └── [RUNNING]
            ├── _advance_sequencer()   (update which waypoint/phase we're in)
            ├── target = _get_target() (quintic interp or dwell hold)
            ├── _publish_markers()     (throttled to ~10 Hz)
            └── _servo_to(target)      (PD + LPF + saturation + publish)
```

---

## Classes and Constants

### `WAYPOINTS_REL`

Module-level list of 9 tuples `(dx, dy, dz, dwell_flag, label)` defining the inspection path relative to `center`. See [inspection_trajectory.md](inspection_trajectory.md) for the full table.

### `SegState`

Integer constants for the trajectory sequencer:

| State | Value | Meaning |
|---|---|---|
| `IN_SEGMENT` | 0 | Moving between waypoints via quintic spline |
| `IN_DWELL` | 1 | Holding at an inspection waypoint |
| `COMPLETE` | 2 | All waypoints visited (only when `loop_traj=False`) |

### `RobotState`

| State | Value | Meaning |
|---|---|---|
| `RUNNING` | 1 | Actively tracking the trajectory |
| `PAUSED` | 2 | Controller halted, zero velocity sent |
| `HOME` | 3 | Moving back to `home_position` |

---

## Key Methods

### `_build_waypoints()`

Called once after `self.center` is set. Converts relative offsets in `WAYPOINTS_REL` to absolute positions in `link_base`. Also applies the `dwell_sec` parameter to all waypoints that have a non-zero dwell flag.

### `_get_target() -> np.ndarray`

Returns the desired 3D position for the current tick:
- During `IN_SEGMENT`: calls `_interpolate(prev_wp, current_wp, elapsed)` using the quintic blend
- During `IN_DWELL`: returns the current waypoint position unchanged

### `_advance_sequencer()`

Called every tick. Checks elapsed time and transitions between states:
- Segment → Dwell (if waypoint has dwell)
- Segment → Next segment (if no dwell)
- Dwell → Next segment (after dwell expires)
- Last waypoint → COMPLETE or loop back to WP1

Logs a line to the console at each transition.

### `_servo_to(target_pos, t_sec)`

**The core PD control step**, identical to Lab 4.2:

```
error     = target_pos − current_pos
d_error   = (error − prev_error) / dt

Step 1 — Deadband:   zero axes where |error| < epsilon
Step 2 — PD law:     v = Kp·error + Kd·d_error
Step 3 — LPF:        v = α·v_prev + (1−α)·v   (optional, α from lpf_fc_hz)
Step 4 — Saturation: if ||v|| > max_speed, scale v down preserving direction
Step 5 — CSV log
Step 6 — Publish TwistStamped
```

### `_publish_cycle_metrics()`

Triggered at the end of every full cycle (all 9 waypoints visited). Publishes `Float64MultiArray([rmse_x, rmse_y, rmse_z, rmse_total, sat_ratio])` to `/cycle_metrics` and resets all accumulators.

### `_publish_markers(target)`

Throttled to ~10 Hz. Publishes three sets of RViz markers:
- `/traj_marker` — `LINE_STRIP` connecting center through all 9 waypoints (green)
- `/wp_markers` — one `SPHERE` per waypoint: **orange** for inspection (has dwell), **blue** for transit
- `/target_marker` — `SPHERE` at the current interpolated target (red)

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `segment_sec` | 2.0 s | Duration of each interpolated transition |
| `dwell_sec` | 1.5 s | Duration of each inspection hold |
| `loop_traj` | `true` | Repeat trajectory continuously after completing a cycle |
| `kp` | [2.5, 2.5, 2.5] | Proportional gains [x, y, z] |
| `kd` | [0.6, 0.6, 0.6] | Derivative gains [x, y, z] |
| `max_speed` | 0.12 m/s | Velocity norm saturation limit |
| `deadband` | 0.002 m | Per-axis deadband threshold |
| `use_lpf` | `true` | Enable first-order LPF on v_cmd |
| `lpf_fc_hz` | 2.0 Hz | LPF cutoff frequency |
| `save_csv` | `false` | Enable CSV logging |
| `csv_filename` | `tracking_data.csv` | Output filename (written to `data/`) |
| `enable_keyboard` | `false` | Enable pynput keyboard listener (`p`=pause, `h`=home) |

---

## CSV Output Format

Saved to `data/<csv_filename>`.

| Column | Description |
|---|---|
| `time` | Elapsed seconds since trajectory start |
| `wp_idx` | Index of current waypoint being targeted (0-based) |
| `wp_label` | Label string (e.g. `inspect_F1`, `transit_B`) |
| `phase` | `"segment"` or `"dwell"` |
| `x_des`, `y_des`, `z_des` | Desired position (m) |
| `x_act`, `y_act`, `z_act` | Actual EE position from TF2 (m) |
| `ex`, `ey`, `ez` | Position error = desired − actual (m) |
| `vx_cmd`, `vy_cmd`, `vz_cmd` | Commanded velocity after all filters (m/s) |
| `v_norm` | Velocity vector magnitude (m/s) |
| `is_saturated` | 1.0 if velocity was clipped this step, else 0.0 |

The `phase` and `wp_label` columns allow post-processing to separate **segment tracking error** from **dwell steady-state error** per inspection point.

---

## Run Example

```bash
ros2 run xarm_perturbations trajectory_and_controller --ros-args \
  -p segment_sec:=2.0 \
  -p dwell_sec:=1.5 \
  -p kp:="[2.665,2.665,2.665]" \
  -p kd:="[0.64,0.64,0.64]" \
  -p max_speed:=0.12 \
  -p deadband:=0.002 \
  -p save_csv:=true \
  -p csv_filename:=inspection_baseline.csv
```

---

## Related Files

- [inspection_trajectory.md](inspection_trajectory.md) — full trajectory design, waypoint table, geometry, timing
- [perturbation_injector.md](perturbation_injector.md) — injects noise on the same topic
- [auto_tuner.md](auto_tuner.md) — consumes `/cycle_metrics` and calls `SetParameters`
- [analysis_evaluation.md](analysis_evaluation.md) — processes the CSV output
