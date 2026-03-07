# xarm_ctc_challenge — Challenge 4.1: CTC vs PD/PID

ROS 2 package for comparing Computed Torque Control (CTC) against PD/PID control on the xArm Lite 6, under nominal conditions and with external perturbations.

## Overview

The controller executes a CMM-style inspection trajectory (8 Cartesian waypoints) using MoveIt `/compute_ik` for inverse kinematics and MoveIt Servo for joint-velocity execution. Four trials are run:

| Trial | Controller | Perturbation |
|-------|-----------|-------------|
| 1     | CTC        | No          |
| 2     | PD/PID     | No          |
| 3     | CTC        | Yes         |
| 4     | PD/PID     | Yes         |

## Package Structure

```
xarm_ctc_challenge/
├── kinematics.py        # Modified DH FK + Jacobian (Craig 1989)
├── dynamics.py          # Nominal M, C, G, F matrices (Lite 6)
├── trajectory.py        # 8-waypoint quintic spline + JointRefTrajectory
├── ik_solver.py         # Legacy weighted DLS IK (unused in challenge)
├── controller_node.py   # Main controller: CTC or PD/PID via JointJog
└── analysis.py          # Offline analysis and comparison plots
```

## Trajectory

8 Cartesian waypoints forming a surface inspection pattern (base frame):

- **High layer** (z = 0.400 m): safe transit height
- **Low layer** (z = 0.280 m): inspection/probing height
- Vertical separation: 0.120 m (> 0.08 m requirement)
- Segment time: 3.0 s (quintic polynomial, zero vel/acc at endpoints)
- Dwell at inspection points: 2.0 s; transit points: 1.0 s

IK solutions are computed once via MoveIt at startup and saved to `data/ik_refs.npy` to ensure all 4 trials use identical joint-space references (§14.5).

## Controllers

### CTC (Computed Torque Control)

```
τ = M₀(q)·v + C₀(q,q̇)·q̇ + G₀(q) + F₀(q̇) + k_robust·tanh(S/ε)
v = q̈_des + Kp·e + Kd·ė
S = ė + γ·e                     (sliding surface)
q̈_cmd = M₀⁻¹·τ  →  integrate to q̇_cmd
```

### PD/PID

```
q̇_cmd = q̇_des + Kp·e + Kd·ė [+ Ki·∫e  (with anti-windup, active during dwell)]
```

## Running

### Prerequisites

```bash
# Start xArm hardware + MoveIt Servo
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123

# (Optional) Start perturbation injector
ros2 run xarm_perturbations perturbation_injector
```

### Run a trial

```bash
# CTC, no perturbation
ros2 run xarm_ctc_challenge challenge_controller \
    --ros-args -p controller_type:=ctc -p enable_perturbation:=false

# PD/PID, with perturbation
ros2 run xarm_ctc_challenge challenge_controller \
    --ros-args -p controller_type:=pd -p enable_perturbation:=true
```

### Emergency stop

```bash
ros2 topic pub /challenge_stop std_msgs/Bool "data: true" --once
```

## Analysis

```bash
# Run offline analysis on a directory of trial CSVs
python3 src/xarm_ctc_challenge/xarm_ctc_challenge/analysis.py data/
```

Generates per-trial:
- `joints_<key>.png` — Joint angle tracking (6 subplots, dwell windows highlighted)
- `taskspace_<key>.png` — X/Y/Z tracking, 3D path with waypoint markers, EE error norm
- `phase_<key>.png` — Phase portraits (e_j vs ė_j)

And comparison plots:
- `comparison_nopert.png` — CTC vs PD/PID, no perturbation
- `comparison_pert.png` — CTC vs PD/PID, with perturbation

## CSV Output

Each trial saves a CSV with 56 columns including joint states, desired trajectory, task-space positions, control commands, saturation flags, perturbation status, and waypoint metadata.

## Build

```bash
cd ros2_ws && colcon build --packages-select xarm_ctc_challenge
```
