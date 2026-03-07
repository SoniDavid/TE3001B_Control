# xArm Lite 6 Cartesian PD Control with Perturbations - Lab 4.2

This documentation covers the `xarm_perturbations` ROS 2 package, which implements a **task-space (Cartesian) PD controller** for the UFACTORY xArm Lite 6 manipulator. The controller tracks a figure-eight (Lissajous) trajectory while supporting external perturbation injection and automated gain tuning.

## System Overview

The package communicates with the physical robot through **MoveIt Servo**, sending `TwistStamped` velocity commands to the `/servo_server/delta_twist_cmds` topic. MoveIt Servo internally converts these Cartesian velocities into safe joint velocities using the robot's Jacobian.

```
TrajectoryAndController node
        |
        | TwistStamped -> /servo_server/delta_twist_cmds
        v
   MoveIt Servo (xarm_moveit_servo)
        |
        | joint velocities
        v
   xArm Lite 6 hardware (192.168.1.123)
```

The end-effector position is read in real time via **TF2** (`link_base` -> `link_eef`), which MoveIt Servo keeps updated from the robot's joint state feedback.

## Package Structure

```
xarm_perturbations/
├── xarm_perturbations/
│   ├── __init__.py
│   ├── trajectory_and_controller.py   # Main node: trajectory + PD control
│   ├── perturbation_injector.py       # Standalone perturbation publisher
│   ├── auto_tuner.py                  # Grid-search Kp/Kd tuner
│   ├── circle_maker.py                # Simple circle trajectory (prototype)
│   ├── analysis_evaluation.py         # Multi-experiment offline analysis
│   ├── analysis_evaluation_only.py    # Single-experiment offline analysis
│   └── README.md                      # Package-level readme
├── package.xml
├── setup.py
└── setup.cfg
```

## Entry Points (console_scripts)

| Command | Module | Description |
|---|---|---|
| `trajectory_and_controller` | `trajectory_and_controller:main` | Main controller node |
| `perturbation_injector` | `perturbation_injector:main` | External perturbation injector |
| `auto_tuner` | `auto_tuner:main` | Grid-search gain tuner |
| `circle_maker` | `circle_maker:main` | Circle trajectory prototype |

## Quick Start

### 1. Build

```bash
cd ~/ros2_ws
colcon build --packages-select xarm_perturbations
source install/setup.bash
```

### 2. Launch MoveIt Servo (required for all experiments)

```bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123
```

> The robot must be in Mode 1 and State 0 before sending velocity commands.

### 3. Run the main controller

```bash
ros2 run xarm_perturbations trajectory_and_controller --ros-args \
  -p radius:=0.15 \
  -p frequency:=0.035 \
  -p plane:=xy \
  -p yaw_deg:=90.0 \
  -p kp:="[2.665,2.665,2.665]" \
  -p kd:="[0.64,0.64,0.64]" \
  -p max_speed:=0.12 \
  -p deadband:=0.002 \
  -p save_csv:=true \
  -p csv_filename:=baseline_experiment.csv
```

### 4. (Optional) Inject perturbations in a second terminal

```bash
# Sinusoidal perturbation on X axis
ros2 run xarm_perturbations perturbation_injector --ros-args \
  -p mode:=sine -p sine_axis:=x -p sine_amp_linear:=0.02 -p sine_freq_hz:=0.5

# Gaussian noise
ros2 run xarm_perturbations perturbation_injector --ros-args \
  -p mode:=gaussian -p noise_std_linear:=0.01
```

### 5. (Optional) Run the auto-tuner

With the controller already running in another terminal:

```bash
ros2 run xarm_perturbations auto_tuner
```

### 6. Offline analysis

After collecting CSV data:

```bash
python3 analysis_evaluation.py       # multi-experiment (edits time segments inside)
python3 analysis_evaluation_only.py  # single CSV
```

## ROS Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/servo_server/delta_twist_cmds` | `geometry_msgs/TwistStamped` | Published | Cartesian velocity commands to MoveIt Servo |
| `/traj_marker` | `visualization_msgs/Marker` | Published | LINE_STRIP of the desired figure-eight in RViz |
| `/target_marker` | `visualization_msgs/Marker` | Published | SPHERE showing current target point in RViz |
| `/cycle_metrics` | `std_msgs/Float64MultiArray` | Published | Per-cycle RMSE and saturation ratio |

## Dependencies

| Package | Type | Use |
|---|---|---|
| `rclpy` | ROS 2 | Node lifecycle |
| `geometry_msgs` | ROS 2 | TwistStamped messages |
| `visualization_msgs` | ROS 2 | RViz markers |
| `std_msgs` | ROS 2 | Float64MultiArray for metrics |
| `tf2_ros` | ROS 2 | Read EE transform from TF tree |
| `numpy` | Python | Numerical computation |
| `pynput` | Python | Optional keyboard control |
| `pandas`, `matplotlib` | Python | Offline analysis scripts only |

## Control Law

The PD controller computes a Cartesian velocity command:

```
v_cmd = Kp * e + Kd * de/dt
```

where `e = p_desired - p_actual` is the 3D Cartesian position error read from TF2.

Safety layers applied in order:
1. **Deadband**: components where `|e| < epsilon` are zeroed (avoids motor chatter near target)
2. **Low-Pass Filter** (optional): first-order digital LPF on `v_cmd` to suppress derivative noise
3. **Norm saturation**: if `||v_cmd|| > max_speed`, the vector is scaled down preserving direction

## Documented Files

1. [trajectory_and_controller.md](trajectory_and_controller.md) - Main node
2. [perturbation_injector.md](perturbation_injector.md) - Perturbation injector
3. [auto_tuner.md](auto_tuner.md) - Grid-search auto-tuner
4. [circle_maker.md](circle_maker.md) - Circle prototype node
5. [analysis_evaluation.md](analysis_evaluation.md) - Multi-experiment analysis
6. [analysis_evaluation_only.md](analysis_evaluation_only.md) - Single-experiment analysis
7. [INDEX.md](INDEX.md) - Full index

## Experimental Results Summary

Fixed gains (Kp=2.665, Kd=0.64) were evaluated across three conditions:

| Condition | Description | Total RMSE | Max Abs Error |
|---|---|---|---|
| Baseline | No injected noise | ~0.032 m | low |
| Sinusoidal | Low-frequency deterministic perturbation | moderate | moderate |
| Gaussian | High-frequency stochastic white noise | highest | ~0.115 m |

The derivative term makes the controller sensitive to high-frequency Gaussian noise, producing the largest errors in that condition.
