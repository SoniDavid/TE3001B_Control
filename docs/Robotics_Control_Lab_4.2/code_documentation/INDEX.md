# Source Code Documentation Index — xarm_perturbations

Complete documentation for all source files in the `xarm_perturbations` ROS 2 package (Lab 4.2).

## Quick Navigation

### Trajectory Design
1. **[inspection_trajectory.md](inspection_trajectory.md)** - CMM inspection trajectory: real-world application, waypoint table, path geometry, timing

### ROS 2 Nodes
2. **[trajectory_and_controller.md](trajectory_and_controller.md)** - Main node: inspection trajectory sequencer + Cartesian PD controller + CSV logging
3. **[perturbation_injector.md](perturbation_injector.md)** - Standalone perturbation publisher (sine / Gaussian / off)
4. **[auto_tuner.md](auto_tuner.md)** - Automated grid-search Kp/Kd tuner using live `/cycle_metrics`
5. **[circle_maker.md](circle_maker.md)** - Simple circle trajectory prototype (hardcoded gains)

### Offline Analysis Scripts
6. **[analysis_evaluation.md](analysis_evaluation.md)** - Multi-experiment segmented analysis from one large CSV
7. **[analysis_evaluation_only.md](analysis_evaluation_only.md)** - Single-experiment analysis from one CSV

---

## File Overview Table

| File | Type | Lines | Key Class/Function | Status |
|---|---|---|---|---|
| `trajectory_and_controller.py` | ROS 2 Node | ~571 | `TrajectoryAndController` | Current - main entry point |
| `perturbation_injector.py` | ROS 2 Node | ~126 | `PerturbationGenerator` | Current |
| `auto_tuner.py` | ROS 2 Node | ~212 | `GridSearchTuner` | Current |
| `circle_maker.py` | ROS 2 Node | ~349 | `CircleServoXArmLite6` | Prototype (use trajectory_and_controller instead) |
| `analysis_evaluation.py` | Script | ~114 | `main()` | Current - multi-experiment |
| `analysis_evaluation_only.py` | Script | ~88 | `main()` | Current - single experiment |

---

## Architecture Diagram

```
[Physical Robot: xArm Lite 6]
        ^
        | joint velocities (MoveIt Servo internal)
        |
[MoveIt Servo: lite6_moveit_servo_realmove.launch.py]
        ^
        | TwistStamped -> /servo_server/delta_twist_cmds
        |
[trajectory_and_controller node]
        |-- reads TF2: link_base -> link_eef (current EE position)
        |-- publishes /traj_marker, /target_marker (RViz)
        |-- publishes /cycle_metrics (RMSE per cycle)
        |-- writes CSV (optional)
        ^
        | (optional) TwistStamped -> /servo_server/delta_twist_cmds
        |
[perturbation_injector node]     [auto_tuner node]
  (adds noise on top)              (sets Kp/Kd via SetParameters service)
```

---

## By Feature

### Trajectory Generation
- **[trajectory_and_controller.md](trajectory_and_controller.md)** — `_target()`: Lissajous 1:2 figure-eight with soft-start ramp and XY yaw rotation
- **[circle_maker.md](circle_maker.md)** — `_circle_target()`: simple circular trajectory

### Cartesian PD Control
- **[trajectory_and_controller.md](trajectory_and_controller.md)** — `_servo_to()`: error computation, deadband, LPF, norm saturation
- **[circle_maker.md](circle_maker.md)** — `_servo_to()`: simpler per-axis clip version

### Robot Interface (MoveIt Servo)
- Both nodes publish `TwistStamped` to `/servo_server/delta_twist_cmds`
- Both read EE position from TF2: `link_base` -> `link_eef`

### Perturbation Injection
- **[perturbation_injector.md](perturbation_injector.md)** — external node, three modes: off / sine / gaussian

### Automated Gain Tuning
- **[auto_tuner.md](auto_tuner.md)** — grid search over Kp x Kd, scores by `RMSE + lambda * sat_ratio`

### Metrics & Logging
- **[trajectory_and_controller.md](trajectory_and_controller.md)** — per-cycle RMSE on `/cycle_metrics`, optional CSV
- **[analysis_evaluation.md](analysis_evaluation.md)** — offline RMSE + max error + plots (3 experiments from 1 CSV)
- **[analysis_evaluation_only.md](analysis_evaluation_only.md)** — offline analysis for a single CSV

---

## By Use Case

### "I want to run the basic trajectory tracking experiment"
-> **[trajectory_and_controller.md](trajectory_and_controller.md)**

### "I want to inject sinusoidal or Gaussian disturbances"
-> **[perturbation_injector.md](perturbation_injector.md)**

### "I want to find the best Kp/Kd gains automatically"
-> **[auto_tuner.md](auto_tuner.md)**

### "I want to analyze my CSV data and compute RMSE"
-> **[analysis_evaluation.md](analysis_evaluation.md)** (multiple experiments)
-> **[analysis_evaluation_only.md](analysis_evaluation_only.md)** (single experiment)

### "I want to understand how the robot is moved (velocity control)"
-> **[README.md](README.md)** system overview section
-> **[trajectory_and_controller.md](trajectory_and_controller.md)** `_publish_twist()` and `_servo_to()` sections

---

## Code Dependencies

```
trajectory_and_controller.py
    ├── rclpy, geometry_msgs, visualization_msgs, std_msgs
    ├── tf2_ros (reads link_base -> link_eef)
    ├── numpy
    └── pynput (optional keyboard)

perturbation_injector.py
    ├── rclpy, geometry_msgs
    └── numpy

auto_tuner.py
    ├── rclpy, rcl_interfaces (SetParameters service)
    └── std_msgs (reads /cycle_metrics)

circle_maker.py
    ├── rclpy, geometry_msgs, visualization_msgs
    ├── tf2_ros
    ├── numpy
    └── pynput

analysis_evaluation.py / analysis_evaluation_only.py
    ├── pandas
    ├── numpy
    └── matplotlib
```

---

## Parameter Reference (trajectory_and_controller)

| Parameter | Default | Description |
|---|---|---|
| `radius` | 0.03 m | Half-amplitude of the figure-eight |
| `frequency` | 0.07 Hz | Oscillation frequency |
| `plane` | `"xy"` | Active plane: `xy`, `xz`, or `yz` |
| `hold_z` | `true` | Keep Z fixed (only applies to xz/yz planes) |
| `yaw_deg` | 0.0 | Rotation of the figure-eight inside the XY plane |
| `ramp_seconds` | 2.0 | Soft-start duration; metrics ignored during ramp |
| `kp` | [2.5, 2.5, 2.5] | Proportional gains [x, y, z] |
| `kd` | [0.6, 0.6, 0.6] | Derivative gains [x, y, z] |
| `max_speed` | 0.12 m/s | Velocity norm saturation limit |
| `deadband` | 0.002 m | Per-axis deadband threshold |
| `use_lpf` | `true` | Enable first-order low-pass filter on v_cmd |
| `lpf_fc_hz` | 2.0 Hz | LPF cutoff frequency |
| `save_csv` | `false` | Enable CSV logging |
| `csv_filename` | `tracking_data.csv` | Output CSV filename (saved in `data/`) |
| `enable_keyboard` | `false` | Enable pynput keyboard listener |

---

## Last Updated

Documentation created: 2026-03-05

**Total Documentation Pages**: 7
