# xarm_ctc_challenge — Challenge 4.1: CTC vs Cartesian PD Under Perturbations

ROS 2 Humble package that implements and compares Computed Torque Control (CTC)
against Cartesian PD control on the UFACTORY xArm Lite 6, with and without
Gaussian velocity perturbations injected at the MoveIt Servo command interface.
The task is defined in Cartesian space; joint references are generated online
via a weighted resolved-rate inverse kinematics solver.

---

## Table of Contents

1. [Package Overview](#1-package-overview)
2. [Prerequisites](#2-prerequisites)
3. [Package Structure](#3-package-structure)
4. [Module Descriptions](#4-module-descriptions)
   - [kinematics.py](#kinematicspy)
   - [dynamics.py](#dynamicspy)
   - [trajectory.py](#trajectorypy)
   - [ik_solver.py](#ik_solverpy)
   - [controller_node.py](#controller_nodepy)
   - [analysis.py](#analysispy)
   - [go_home.py](#go_homepy)
   - [joint_state_logger.py](#joint_state_loggerpy)
5. [ROS 2 Interface](#5-ros-2-interface)
   - [Topics Published](#topics-published)
   - [Topics Subscribed](#topics-subscribed)
   - [TF2 Frames Used](#tf2-frames-used)
   - [Launch Arguments](#launch-arguments)
6. [Control Algorithms](#6-control-algorithms)
   - [CTC — Computed Torque Control](#ctc--computed-torque-control)
   - [Cartesian PD](#cartesian-pd)
   - [Weighted Resolved-Rate IK](#weighted-resolved-rate-ik)
   - [Signal Filter Cascade](#signal-filter-cascade)
7. [Kinematic and Dynamic Model](#7-kinematic-and-dynamic-model)
   - [DH Parameters](#dh-parameters)
   - [Link Dynamics Parameters](#link-dynamics-parameters)
8. [Trajectory Description](#8-trajectory-description)
9. [Running the Four Required Trials](#9-running-the-four-required-trials)
10. [Emergency Stop](#10-emergency-stop)
11. [Running the Analysis](#11-running-the-analysis)
12. [CSV Output Format](#12-csv-output-format)
13. [File Naming Convention](#13-file-naming-convention)
14. [Controller Gains Reference](#14-controller-gains-reference)
15. [Safety Limits](#15-safety-limits)
16. [Known Issues and Troubleshooting](#16-known-issues-and-troubleshooting)

---

## 1. Package Overview

Challenge 4.1 requires:
- A Cartesian-space task (PCB component placement, 2x2 grid, 8 waypoints)
- Two controllers: model-based CTC and model-free Cartesian PD
- Four experimental trials: each controller run with and without perturbation
- Quantitative comparison via RMSE, max error, and waypoint success rate

**Control rate:** 100 Hz

**Robot interface:** MoveIt Servo, `TwistStamped` on `/servo_server/delta_twist_cmds`

**Effective Cartesian velocity scaling:** approximately 22% of commanded value,
because MoveIt Servo internally republishes at 2 ms while commands arrive at 10 ms.
Gains and limits are tuned to compensate for this scaling.

---

## 2. Prerequisites

**Build the workspace (skip simulation packages):**

```bash
cd ros2_ws
colcon build --packages-skip gazebo_mujoco_bridge mbot_demo
source install/setup.bash
```

If the build fails with `ModuleNotFoundError: No module named 'catkin_pkg'`
(conda intercepting CMake's Python):

```bash
pip install catkin_pkg   # once, inside the active conda environment
```

**Start MoveIt Servo before any trial:**

```bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py \
  robot_ip:=192.168.1.154
```

---

## 3. Package Structure

```
xarm_ctc_challenge/
|-- xarm_ctc_challenge/
|   |-- __init__.py
|   |-- kinematics.py         FK + position Jacobian (product-of-transforms, URDF-direct)
|   |-- dynamics.py           Nominal M(q), C(q,qd), G(q), F(qd) for xArm Lite 6
|   |-- trajectory.py         PCBTrajectory: 8-waypoint quintic spline, C2 continuous
|   |-- ik_solver.py          WeightedIKSolver: DLS resolved-rate IK with null-space
|   |-- controller_node.py    ChallengeController: CTC or PD, 100 Hz, CSV logging
|   |-- analysis.py           Offline analysis: metrics computation and 7 plot types
|   |-- go_home.py            Move arm to home configuration (FollowJointTrajectory)
|   `-- joint_state_logger.py Standalone CSV logger for /joint_states
|-- launch/
|   `-- challenge.launch.py   Launch file with controller_type, perturbation, csv_dir
|-- package.xml
|-- setup.py
`-- README.md
```

---

## 4. Module Descriptions

### kinematics.py

Implements forward kinematics and the position Jacobian for xArm Lite 6 using the
product-of-homogeneous-transforms convention derived directly from the URDF joint
parameters (modified DH, Craig 1989).

**Public functions:**

| Function | Signature | Returns |
|----------|-----------|---------|
| `forward_kinematics` | `(q: array[6])` | `(p: array[3], T: array[4,4])` — EE position and transform |
| `position_jacobian` | `(q: array[6])` | `J: array[3,6]` — geometric position Jacobian |
| `com_position` | `(q, link_idx, com_local)` | `p_com: array[3]` — CoM of link in world frame |
| `com_jacobian` | `(q, link_idx, com_local)` | `J_com: array[3,6]` — Jacobian for link CoM |

**Validation:** FK matches TF2 transform `link_base -> link_eef` to less than 0.2 mm
across all tested configurations.

---

### dynamics.py

Computes the nominal rigid-body dynamics of xArm Lite 6 using approximate link
parameters derived from the URDF. Provides the four terms of the standard
manipulator equation:

```
M(q) * q_ddot + C(q, q_dot) * q_dot + G(q) + F(q_dot) = tau
```

**Public class:** `Lite6NominalDynamics`

| Method | Returns |
|--------|---------|
| `mass_matrix(q)` | `M: array[6,6]` — joint-space inertia matrix |
| `coriolis_torques(q, qd)` | `array[6]` — Coriolis and centripetal torques (C * qd) |
| `gravity_torques(q)` | `array[6]` — gravity compensation torques |
| `friction_torques(qd)` | `array[6]` — viscous friction torques |
| `get_dynamics(q, qd)` | `(M, C*qd, G, F)` — all four terms at once |

**Note:** These are nominal parameters. Model uncertainty is compensated by the
robust sliding-mode term in CTC (see Section 6).

---

### trajectory.py

Generates the PCB component-placement Cartesian reference trajectory. The trajectory
is computed relative to the actual end-effector position at startup to avoid
IK cold-start divergence.

**Public class:** `PCBTrajectory(centre: array[3])`

| Method | Returns |
|--------|---------|
| `at(t: float)` | `TrajPoint` namedtuple with fields: `p, pd, pdd, wp_idx, label, phase` |
| `total_time` | `float` — total trajectory duration in seconds (53.0 s) |

**Waypoint layout:**

```
Site A (front-left)   Site B (front-right)
Site C (back-left)    Site D (back-right)

DX = +/-0.040 m  (forward/backward)
DY = +/-0.045 m  (left/right)
DZ =  0.090 m    (vertical drop, satisfies >= 0.08 m requirement)
```

**Sequence per site:** approach (HIGH, 1.5 s dwell) -> place (LOW, 2.0 s dwell)
-> depart (HIGH, no dwell). Returns to home after all four sites. Total: 53.0 s.

**Interpolation:** Piecewise quintic polynomial with zero velocity and zero
acceleration boundary conditions at every waypoint (C2 continuous).
Profile function: `h(s) = 10s^3 - 15s^4 + 6s^5`, `s = t / segment_duration`.

---

### ik_solver.py

Online resolved-rate inverse kinematics using Damped Least Squares (DLS) pseudoinverse
with Z-axis task-space weighting and null-space posture control.

**Public class:** `WeightedIKSolver(q_home, dt, wz, lam, k_task, k_null)`

| Method | Returns |
|--------|---------|
| `reset(q_init)` | — reinitialize IK state to known configuration |
| `step(p_des, pd_des, pdd_des, p_actual, q_actual)` | `(q_des, qd_des, qdd_des)` |

**IK formulation:**

```
W   = diag(1, 1, wz)                                   Z-priority task-space weight
Jw  = W @ J                                             weighted Jacobian (3x6)
J#  = Jw^T @ inv(Jw @ Jw^T + lambda^2 * I)             DLS pseudoinverse
q_dot_null = (I - J# @ Jw) @ (-k_null*(q - q_home) + limit_push)
q_dot_des  = J# @ (pd_des + k_task*(p_des - p)) + q_dot_null
q_des      = integral of q_dot_des dt
q_ddot_des = finite difference of q_dot_des
```

**Default parameters:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| `wz` | 2.5 | Z-axis task-space weight |
| `lambda` | 0.015 | DLS damping coefficient |
| `k_task` | 20.0 | Task-space error gain (m/s per m) |
| `k_null` | 5.0 | Null-space posture gain |
| `MAX_TASK_VEL` | 0.25 m/s | Cartesian velocity limit in IK |
| `IK_VEL_LIMIT` | 2.0 rad/s | Per-joint velocity limit from IK |
| `MAX_QDD` | 50.0 rad/s^2 | Hard clamp on finite-difference acceleration |

**Null-space control purpose:** Prevents joint drift that would otherwise cause
joint 4 to reach the +/-175 degree hardware limit during the full 53 s trajectory.

---

### controller_node.py

Main ROS 2 node implementing both controllers. This is the largest and most complex
file (approximately 822 lines).

**Class:** `ChallengeController(Node)`, timer at 100 Hz.

**Initialization sequence:**
1. Wait for `/joint_states` to establish actual robot configuration.
2. Wait for TF2 transform `link_base -> link_eef` to establish actual EE position.
3. Build `PCBTrajectory` centred at the actual EE position (avoids IK cold start).
4. Initialize `WeightedIKSolver` and `Lite6NominalDynamics`.
5. Start 100 Hz control timer.

**CTC control loop (per tick):**
1. Lookup TF2 for actual EE position.
2. Query trajectory `at(t)` for desired Cartesian state.
3. Run IK solver `step()` to get `q_des, qd_des, qdd_des`.
4. Apply state predictor to get best estimate of `q, qd` (see below).
5. Compute model terms `M, C*qd, G, F` from `Lite6NominalDynamics`.
6. Compute sliding surface `S = gamma*e + e_dot`.
7. Compute CTC torque: `tau = M*v + C*qd + G + F + k_robust*tanh(S/epsilon)`.
8. Compute joint acceleration: `q_ddot_cmd = M_inv @ tau`.
9. Integrate to joint velocity: `qd_cmd = qd_des + q_ddot_cmd * dt`.
10. Apply 4-stage filter cascade (see Section 6).
11. Saturate to joint velocity limits.
12. Publish `TwistStamped` (converts qd_cmd to Cartesian via Jacobian).
13. Log 55 columns to CSV buffer; flush every 100 rows.

**PD control loop (per tick):**
1. Lookup TF2 for actual EE position.
2. Query trajectory `at(t)` for desired Cartesian position.
3. Compute error: `e = p_des - p_act`.
4. Compute derivative error via LPF-filtered velocity.
5. Compute Cartesian velocity: `v_c = Kp*e + Kd*e_dot`.
6. Apply deadband (2 mm), LPF (2 Hz), magnitude saturation (0.12 m/s).
7. Publish `TwistStamped`.
8. Log to CSV.

**State predictor:** Because `/joint_states` arrives at approximately 10 Hz but the
control loop runs at 100 Hz, the node maintains a first-order predictor:
`q_pred = q_last + qd_last * (t_now - t_last)`. Fresh measurements reset the predictor.

---

### analysis.py

Offline analysis script (approximately 529 lines). Loads all four trial CSV files,
computes metrics, and generates plots.

**Usage:**

```bash
# Using miniconda Python to avoid ROS 2 environment package conflicts:
PYTHONPATH=ros2_ws/src/xarm_ctc_challenge \
  ~/miniconda3/bin/python3 \
  ros2_ws/src/xarm_ctc_challenge/xarm_ctc_challenge/analysis.py \
  /path/to/data/directory/
```

**CSV discovery:** Automatically finds files matching
`trial_{ctc,pdpid}_{nopert,pert}_*.csv` in the specified directory.

**Metrics computed:**

| Metric | Definition |
|--------|-----------|
| Joint RMSE | sqrt(mean(e_j^2)) for each joint j |
| Joint max error | max(|e_j|) for each joint j |
| Dwell mean error | mean(|e_j|) during dwell phases only |
| EE RMSE | sqrt(mean(||p_err||^2)), where p_err = p_act - p_des |
| EE max error | max(||p_err||) |
| Waypoint success | Fraction of dwells where mean error in final 0.5 s is < 5 mm |

**Plot outputs per trial:**

| Filename | Content |
|----------|---------|
| `joints_{trial}.png` | 6-subplot joint angle tracking (desired vs measured), dwell windows shaded |
| `taskspace_{trial}.png` | X/Y/Z tracking, 3D path, EE error norm over time |
| `phase_{trial}.png` | Phase portraits e_j vs e_dot_j for all 6 joints |

**Comparison and summary plots:**

| Filename | Content |
|----------|---------|
| `comparison_nopert.png` | CTC vs PD overlay, no perturbation |
| `comparison_pert.png` | CTC vs PD overlay, with perturbation |
| `summary_table.png` | All metrics for all 4 trials as a formatted table |
| `metrics_summary.csv` | Metrics in CSV format for further processing |

---

### go_home.py

Utility node that commands the arm to a predefined home joint configuration
using the `FollowJointTrajectory` action server. Safe to run before or after trials.

```bash
ros2 run xarm_ctc_challenge go_home
```

---

### joint_state_logger.py

Standalone CSV logger that subscribes to `/joint_states` and records joint
positions and velocities over time. Useful for debugging and comparing raw
hardware data without running the full controller.

```bash
ros2 run xarm_ctc_challenge joint_state_logger
```

---

## 5. ROS 2 Interface

### Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/servo_server/delta_twist_cmds` | `geometry_msgs/TwistStamped` | Cartesian velocity command to MoveIt Servo |

### Topics Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Measured joint positions and velocities (~10 Hz from hardware) |
| `/challenge_stop` | `std_msgs/Bool` | Emergency stop signal; `data: true` halts the controller |

### TF2 Frames Used

| Parent Frame | Child Frame | Purpose |
|-------------|-------------|---------|
| `link_base` | `link_eef` | Actual end-effector position for PD error and trajectory initialization |

### Launch Arguments

| Argument | Default | Values | Description |
|----------|---------|--------|-------------|
| `controller_type` | `CTC` | `CTC`, `PD` | Select control algorithm |
| `perturbation_enabled` | `false` | `true`, `false` | Log perturbation state in CSV |
| `csv_dir` | `data` | any path | Directory for CSV and metadata output |

---

## 6. Control Algorithms

### CTC — Computed Torque Control

The CTC law linearizes the nonlinear manipulator dynamics in joint space:

```
v   = q_ddot_des + Kp*(q_des - q) + Kd*(qd_des - qd)
S   = gamma*e + e_dot                    sliding surface (gamma = 2.5)
tau = M(q)*v + C(q,qd)*qd + G(q) + F(qd) + k_robust*tanh(S/epsilon)

q_ddot_cmd = M(q)^{-1} * tau
qd_cmd     = qd_des + q_ddot_cmd * dt   (Euler integration)
```

The robust term `k_robust * tanh(S/epsilon)` compensates for model uncertainty
and external perturbations without introducing the chattering of a sign function.

**Gains:**

| Gain | Value |
|------|-------|
| Kp | diag(20, 20, 20, 2, 1, 0.5) |
| Kd | diag(8, 8, 8, 0.8, 0.4, 0.2) |
| gamma (sliding gain) | 2.5 |
| k_robust | 5.5 Nm |
| epsilon | 0.002 rad/s |

### Cartesian PD

Model-free control on Cartesian position error obtained from TF2:

```
e     = p_des - p_act    (Cartesian position error, meters)
e_dot = (e - e_prev) / dt filtered by LPF at fc = 2 Hz
v_c   = Kp*e + Kd*e_dot
```

**Gains:**

| Gain | Value |
|------|-------|
| Kp | diag(2.5, 2.5, 2.5) |
| Kd | diag(0.6, 0.6, 0.6) |
| LPF cutoff | 2 Hz |
| Deadband | 2 mm |
| Max Cartesian speed | 0.12 m/s |

### Weighted Resolved-Rate IK

Used by both controllers to convert Cartesian trajectory references to joint
references (see Section 4, ik_solver.py for full detail).

### Signal Filter Cascade

Applied in CTC to prevent model-based torque blow-up from high-frequency spikes
in the finite-difference acceleration `q_ddot_des` (spikes reach ~114 rad/s^2
every 9-10 ticks without filtering; saturation rate drops from 18% to <1%
after filtering):

```
Stage 1: Hard clamp    q_ddot_des <- clip(q_ddot_des, -10, +10)  rad/s^2
Stage 2: LPF (5 Hz)    q_ddot_des <- alpha*q_ddot_prev + (1-alpha)*q_ddot_des
Stage 3: LPF (10 Hz)   qd_des     <- alpha*qd_prev    + (1-alpha)*qd_des
Stage 4: LPF (20 Hz)   qd_cmd     <- alpha*qd_cmd_prev + (1-alpha)*qd_cmd
```

Where `alpha = exp(-2*pi*fc*dt)` for each respective cutoff `fc`.

---

## 7. Kinematic and Dynamic Model

### DH Parameters

Modified DH convention (Craig 1989), derived from xArm Lite 6 URDF joint offsets.

| Joint | alpha_{i-1} (rad) | a_{i-1} (m) | d_i (m) | theta_offset_i (rad) |
|-------|-------------------|-------------|---------|----------------------|
| 1 | 0 | 0 | 0.2433 | 0 |
| 2 | -pi/2 | 0 | 0 | -pi/2 |
| 3 | 0 | 0.2 | 0 | 0 |
| 4 | 0 | 0.087 | 0.2276 | pi/2 |
| 5 | -pi/2 | 0 | 0 | 0 |
| 6 | pi/2 | 0 | 0.0615 | 0 |

### Link Dynamics Parameters

Nominal values used in `dynamics.py`. These are approximate and the CTC robust
term compensates for the resulting model uncertainty.

| Link | Mass (kg) | Viscous Friction (Nm/(rad/s)) | Diagonal Inertia (kg*m^2) |
|------|-----------|-------------------------------|---------------------------|
| 1 | 0.84 | 0.40 | 0.080 |
| 2 | 1.02 | 0.40 | 0.080 |
| 3 | 0.72 | 0.30 | 0.050 |
| 4 | 0.62 | 0.20 | 0.030 |
| 5 | 0.16 | 0.10 | 0.010 |
| 6 | 0.14 | 0.10 | 0.005 |

Total arm mass (links only): approximately 3.5 kg.

---

## 8. Trajectory Description

PCB component-placement task simulating automated assembly of four components
on a 2x2 grid.

**Grid layout (offsets relative to actual EE position at startup):**

```
+--------+--------+
| Site A | Site B |   DX = +0.040 m (forward)
+--------+--------+   DX = -0.040 m (backward)
| Site C | Site D |   DY = +0.045 m (left)
+--------+--------+   DY = -0.045 m (right)
```

**Height layers:**
- HIGH (approach/depart): Z offset = 0 m from start (relative)
- LOW (placement): Z offset = -0.090 m from HIGH (meets >= 0.08 m requirement)

**Waypoint sequence and timing:**

| Step | Location | Dwell | Duration |
|------|----------|-------|---------|
| A HIGH | approach Site A | 1.5 s | 3 s segment + 1.5 s dwell |
| A LOW | place at Site A | 2.0 s | 3 s segment + 2.0 s dwell |
| A HIGH | depart Site A | — | 3 s segment |
| B HIGH | approach Site B | 1.5 s | 3 s segment + 1.5 s dwell |
| ... | ... | ... | ... |
| HOME | return | — | 3 s segment |

**Total:** 53.0 seconds per trial.

**Interpolation:** Piecewise quintic polynomial, C2 continuous. All boundary
velocities and accelerations are zero (smooth stop-and-go motion).

---

## 9. Running the Four Required Trials

**Start MoveIt Servo first (keep running):**

```bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py \
  robot_ip:=192.168.1.154
```

**Trial 1 — CTC, no perturbation:**

```bash
ros2 launch xarm_ctc_challenge challenge.launch.py \
  controller_type:=CTC \
  perturbation_enabled:=false \
  csv_dir:=/path/to/data
```

**Trial 2 — CTC, with perturbation:**

In one terminal, start the perturbation injector:

```bash
ros2 run xarm_perturbations perturbation_injector --ros-args \
  -p output_topic:=/servo_server/delta_twist_cmds \
  -p enabled:=true \
  -p mode:=gaussian \
  -p gauss_std_linear:=0.5 \
  -p gauss_axis:=x
```

In another terminal, launch the controller:

```bash
ros2 launch xarm_ctc_challenge challenge.launch.py \
  controller_type:=CTC \
  perturbation_enabled:=true \
  csv_dir:=/path/to/data
```

**Trial 3 — PD, no perturbation:**

```bash
ros2 launch xarm_ctc_challenge challenge.launch.py \
  controller_type:=PD \
  perturbation_enabled:=false \
  csv_dir:=/path/to/data
```

**Trial 4 — PD, with perturbation:**

Start perturbation injector as in Trial 2, then:

```bash
ros2 launch xarm_ctc_challenge challenge.launch.py \
  controller_type:=PD \
  perturbation_enabled:=true \
  csv_dir:=/path/to/data
```

---

## 10. Emergency Stop

To halt the controller at any time without killing the process:

```bash
ros2 topic pub /challenge_stop std_msgs/Bool "data: true" --once
```

The controller will zero its output command and stop publishing. MoveIt Servo
will coast to a stop. To recover, relaunch the controller node.

For hardware emergency, also use the physical E-stop button on the robot base.

---

## 11. Running the Analysis

After all four trials are complete and CSV files are in the data directory:

```bash
# Using miniconda Python to avoid ROS 2 numpy/pandas version conflicts:
PYTHONPATH=ros2_ws/src/xarm_ctc_challenge \
  ~/miniconda3/bin/python3 \
  ros2_ws/src/xarm_ctc_challenge/xarm_ctc_challenge/analysis.py \
  /path/to/data/
```

The script auto-discovers trial files by naming convention. All 16 output files
(12 per-trial plots + 4 comparison/summary files) are written to the same
data directory.

Expected output files:

```
data/
|-- joints_ctc_nopert.png
|-- joints_ctc_pert.png
|-- joints_pdpid_nopert.png
|-- joints_pdpid_pert.png
|-- taskspace_ctc_nopert.png
|-- taskspace_ctc_pert.png
|-- taskspace_pdpid_nopert.png
|-- taskspace_pdpid_pert.png
|-- phase_ctc_nopert.png
|-- phase_ctc_pert.png
|-- phase_pdpid_nopert.png
|-- phase_pdpid_pert.png
|-- comparison_nopert.png
|-- comparison_pert.png
|-- summary_table.png
`-- metrics_summary.csv
```

---

## 12. CSV Output Format

Each trial produces one CSV file at 100 Hz (approximately 5,300 rows for the
53 s trajectory) with 55 columns, plus a companion `_metadata.json`.

**Column layout:**

| Columns | Contents |
|---------|----------|
| 1-2 | `time` (Unix timestamp), `time_rel` (seconds since trial start) |
| 3-8 | `q[0:6]` — measured joint angles (rad) |
| 9-14 | `qd[0:6]` — measured joint velocities (rad/s) |
| 15-20 | `q_des[0:6]` — desired joint angles from IK (rad) |
| 21-26 | `qd_des[0:6]` — desired joint velocities (rad/s) |
| 27-32 | `qdd_des[0:6]` — desired accelerations, finite difference (rad/s^2) |
| 33-35 | `p_act_x, p_act_y, p_act_z` — actual EE position from TF2 (m) |
| 36-38 | `p_des_x, p_des_y, p_des_z` — desired EE position from trajectory (m) |
| 39-44 | `qd_cmd[0:6]` — commanded joint velocity after saturation (rad/s) |
| 45-50 | `sat[0:6]` — saturation flags (1 if joint j was clamped, 0 otherwise) |
| 51 | `pert_enabled` — perturbation state flag (bool) |
| 52 | `pert_changed` — 1 if perturbation state changed this tick |
| 53 | `wp_idx` — current waypoint index (0-7) |
| 54 | `wp_label` — waypoint name string (e.g. `site_A_high`) |
| 55 | `phase` — `segment` or `dwell` |

**CTC-specific extended columns (appended after column 55):**

| Columns | Contents |
|---------|----------|
| 56 | `state_fresh` — 1 if `/joint_states` updated on this tick |
| 57-62 | `q_pred[0:6]` — state predictor estimate (rad) |
| 63-68 | `qdd_raw[0:6]` — pre-filter desired acceleration (rad/s^2) |
| 69-74 | `qdd_filt[0:6]` — post-filter desired acceleration (rad/s^2) |
| 75-80 | `qd_filt[0:6]` — filtered desired velocity (rad/s) |

**Metadata JSON fields:** controller type, all gains, IK parameters, waypoint
coordinates, perturbation configuration, trial start time, robot IP.

The CSV buffer flushes to disk every 100 rows to minimize I/O overhead while
keeping data loss bounded to 1 second at 100 Hz.

---

## 13. File Naming Convention

Trial CSV files follow this pattern:

```
trial_{controller}_{perturbation}_{YYYY-MM-DD}_{HH-MM-SS}.csv
trial_{controller}_{perturbation}_{YYYY-MM-DD}_{HH-MM-SS}_metadata.json
```

Where `controller` is `ctc` or `pdpid`, and `perturbation` is `nopert` or `pert`.

Examples:

```
trial_ctc_nopert_2026-03-01_14-23-05.csv
trial_ctc_nopert_2026-03-01_14-23-05_metadata.json
trial_pdpid_pert_2026-03-01_15-01-47.csv
trial_pdpid_pert_2026-03-01_15-01-47_metadata.json
```

---

## 14. Controller Gains Reference

### CTC

| Parameter | Value | Units |
|-----------|-------|-------|
| Kp | diag(20, 20, 20, 2, 1, 0.5) | rad/s per rad |
| Kd | diag(8, 8, 8, 0.8, 0.4, 0.2) | 1/s |
| gamma (sliding surface) | 2.5 | 1/s |
| k_robust | 5.5 | Nm |
| epsilon | 0.002 | rad/s |
| State predictor order | 1 (linear extrapolation) | — |

### Cartesian PD

| Parameter | Value | Units |
|-----------|-------|-------|
| Kp | diag(2.5, 2.5, 2.5) | (m/s)/m |
| Kd | diag(0.6, 0.6, 0.6) | 1/s |
| LPF cutoff (derivative) | 2 | Hz |
| Deadband | 0.002 | m |
| Max Cartesian speed | 0.12 | m/s |

### IK Solver

| Parameter | Value | Units |
|-----------|-------|-------|
| wz (Z weight) | 2.5 | — |
| lambda (DLS damping) | 0.015 | — |
| k_task | 20.0 | 1/s |
| k_null | 5.0 | 1/s |
| Max task velocity | 0.25 | m/s |
| IK velocity limit | 2.0 | rad/s |
| Max q_ddot (clamp) | 50.0 | rad/s^2 |

---

## 15. Safety Limits

| Limit | Value | Where Enforced |
|-------|-------|----------------|
| Joint velocity (hardware) | +/- 2.0 rad/s | controller_node.py, IK solver |
| Cartesian velocity (PD) | 0.12 m/s (magnitude) | controller_node.py |
| CTC torque | +/- 10.0 Nm | controller_node.py |
| Deadband (PD) | 2 mm | controller_node.py |
| IK acceleration clamp | +/- 50 rad/s^2 | ik_solver.py |
| CTC q_ddot clamp (stage 1) | +/- 10 rad/s^2 | controller_node.py (filter cascade) |
| Wrist singularity scaling | 5%-100% velocity | controller_node.py (|q5| < 0.20 rad) |
| Joint limit margin | 0.35 rad (~20 deg) | ik_solver.py (null-space limit push) |

**Singularity handling:** When joint 5 approaches zero (wrist singularity),
the output Cartesian velocity is scaled by `clip(|q5| / 0.20, 0.05, 1.0)`.
This prevents command blow-up while keeping MoveIt Servo active (minimum 5%
velocity prevents timeout).

---

## 16. Known Issues and Troubleshooting

**TF2 lookup fails at startup:**
The controller waits up to 10 s for the `link_base -> link_eef` transform.
Ensure MoveIt Servo is fully running and the robot is connected before launching
the controller.

**FK vs TF2 mismatch warning:**
If the node prints a warning about FK error exceeding 30 mm, the DH parameters
may not match the current URDF. Verify that `xarm_ros2` submodule is up to date
and that `kinematics.py` constants match the URDF joint origins.

**IK divergence (joint 4 hitting limits):**
If null-space gains are too low, joint 4 drifts toward the +/-175 degree limit
over the 53 s trajectory. Increase `k_null` (default 5.0) if this occurs.

**CTC saturating frequently:**
Check `sat[0:6]` columns in the CSV. If saturation exceeds 5% of ticks on
joints 3 or 5, reduce the CTC filter stage 1 clamp from +/-10 to +/-8 rad/s^2
or lower the stage 2 LPF cutoff from 5 Hz to 3 Hz.

**Analysis script cannot find trial files:**
The script expects exact naming: `trial_{ctc,pdpid}_{nopert,pert}_*.csv`.
If files were renamed, the auto-discovery will fail. Either rename files to
match the convention or pass explicit file paths.

**numpy/pandas version conflict in ROS 2 environment:**
Run the analysis script with a separate Python installation (e.g. miniconda)
using the `PYTHONPATH` trick shown in Section 11.
