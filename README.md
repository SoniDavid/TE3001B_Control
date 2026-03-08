# TE3001B Control — xArm Lite 6 Robotic Control Labs

ROS 2 Humble workspace for the TE3001B Fundamentacion de Robotica laboratory series.
This repository contains two independent ROS 2 packages that implement and compare
different control strategies on the UFACTORY xArm Lite 6 six-degree-of-freedom
manipulator, using MoveIt Servo as the hardware interface.

**Course:** TE3001B.101 — Fundamentacion de Robotica (Gpo 101)
**Institution:** Tecnologico de Monterrey
**Team:** Team Funcionaba Ayer
- David Alejandro Soni Cuevas (A01571777)
- Abraham de Jesus Maldonado Mata (A00838581)
- David Gilberto Lomeli Leal (A01571193)
- Irvin David Ornelas Garcia (A00839065)

**Professor:** Nezih Nieto Gutierrez

---

## Table of Contents

1. [Repository Overview](#1-repository-overview)
2. [Hardware](#2-hardware)
3. [Prerequisites and Dependencies](#3-prerequisites-and-dependencies)
4. [Repository Structure](#4-repository-structure)
5. [Build Instructions](#5-build-instructions)
6. [Packages](#6-packages)
   - [xarm_ctc_challenge — Challenge 4.1](#xarm_ctc_challenge--challenge-41)
   - [xarm_perturbations — Lab 4.2](#xarm_perturbations--lab-42)
7. [Common Launch Sequence](#7-common-launch-sequence)
8. [Data and Results](#8-data-and-results)
9. [Documentation](#9-documentation)
10. [Known Issues and Workarounds](#10-known-issues-and-workarounds)

---

## 1. Repository Overview

This repository implements and evaluates two control paradigms on the xArm Lite 6:

| Package | Lab / Challenge | Control Strategy | Task |
|---------|-----------------|------------------|------|
| `xarm_ctc_challenge` | Challenge 4.1 | CTC (model-based) vs Cartesian PD (model-free) | PCB component placement — 2x2 grid, quintic spline trajectory |
| `xarm_perturbations` | Lab 4.2 | Cartesian PD/PID | Lissajous / inspection trajectory under baseline, sinusoidal, and Gaussian perturbations |

Both packages interface with the robot through MoveIt Servo's velocity command
interface, publishing `TwistStamped` or `JointJog` messages at 100 Hz.

---

## 2. Hardware

- **Robot:** UFACTORY xArm Lite 6 (6-DOF collaborative manipulator)
- **Connection:** Ethernet, static IP `192.168.1.154`
- **Control PC:** Ubuntu 22.04, ROS 2 Humble
- **Robot mode before running:** Mode 1, State 0 (velocity servo mode)

Verify connectivity before any experiment:

```bash
ping 192.168.1.154
```

---

## 3. Prerequisites and Dependencies

**System:**
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill (full desktop install)
- Python 3.10+

**Python packages (inside your environment):**
```bash
pip install numpy pandas matplotlib scipy
```

**ROS 2 packages (sourced from workspace):**
- `moveit_ros_planning_interface`
- `moveit_servo`
- `tf2_ros`, `tf2_geometry_msgs`
- `control_msgs`, `sensor_msgs`, `geometry_msgs`
- `xarm_ros2` (included as a git submodule at `ros2_ws/src/xarm_ros2/`)

---

## 4. Repository Structure

```
TE3001B_Control/
|-- README.md                        This file
|-- LICENSE
|-- .gitmodules                      Submodule: xarm_ros2
|
|-- docs/
|   |-- Robotics_Control_Challenge_4.1/
|   |   |-- Instructions/
|   |       |-- Robotics Control Challenge 4.1-1.pdf
|   |       |-- CTCvsPID.ipynb
|   |       `-- TE300XB-5.pdf
|   |-- Robotics_Control_Lab_4.1/
|   |   `-- code_documentation/      Auto-generated per-module docs
|   `-- Robotics_Control_Lab_4.2/
|       |-- code_documentation/      Auto-generated per-module docs
|       `-- Report/
|           `-- Robotics Control Lab 4.2.pdf
|
|-- data/                            Trial results (CSV, PNG, JSON)
|   |-- trial_ctc_nopert_*.csv
|   |-- trial_ctc_pert_*.csv
|   |-- trial_pdpid_nopert_*.csv
|   |-- trial_pdpid_pert_*.csv
|   |-- *_metadata.json
|   |-- joints_*.png
|   |-- taskspace_*.png
|   |-- phase_*.png
|   |-- comparison_*.png
|   |-- summary_table.png
|   `-- metrics_summary.csv
|
`-- ros2_ws/                         ROS 2 Humble colcon workspace
    |-- README.md                    Workspace build notes
    `-- src/
        |-- xarm_ctc_challenge/      Challenge 4.1 — see package README
        |-- xarm_perturbations/      Lab 4.2 — see package README
        |-- xarm_ros2/               xArm driver (git submodule)
        `-- gazebo_mujoco_bridge/    Simulation bridge (skip in build)
```

---

## 5. Build Instructions

### Initialize submodule

If cloning for the first time, initialize the xarm_ros2 submodule:

```bash
git submodule update --init --recursive
```

### Build all non-simulation packages

```bash
cd ros2_ws
colcon build --packages-skip gazebo_mujoco_bridge mbot_demo
```

### Conda environment workaround

If the build fails with `ModuleNotFoundError: No module named 'catkin_pkg'`, conda
is intercepting CMake's Python lookup. Fix by installing catkin_pkg in the active
conda environment:

```bash
pip install catkin_pkg
```

Alternatively, deactivate conda before building and reactivate afterwards:

```bash
conda deactivate
colcon build --packages-skip gazebo_mujoco_bridge mbot_demo
conda activate <your_env>
```

### Source the workspace

Run this in every terminal that needs to use the ROS 2 packages:

```bash
source ros2_ws/install/setup.bash
```

---

## 6. Packages

### xarm_ctc_challenge — Challenge 4.1

**Full documentation:** [`ros2_ws/src/xarm_ctc_challenge/README.md`](ros2_ws/src/xarm_ctc_challenge/README.md)

Implements and compares two joint-space controllers for a PCB component-placement task:

- **CTC (Computed Torque Control):** Model-based feedforward (M, C, G, F) with PD
  error correction and a robust sliding-mode term. Nominal link dynamics are derived
  from xArm Lite 6 URDF parameters.
- **Cartesian PD:** Model-free proportional-derivative control on TF2 end-effector
  position error. Simple, stable, and used as the reference baseline.

Both controllers use an online weighted resolved-rate IK solver (DLS pseudoinverse
with null-space posture control) to convert Cartesian waypoints to joint references.

Four required trials:
1. CTC, no perturbation
2. CTC, Gaussian perturbation (sigma = 0.5 m/s on x-axis)
3. PD, no perturbation
4. PD, Gaussian perturbation

### xarm_perturbations — Lab 4.2

**Full documentation:** [`ros2_ws/src/xarm_perturbations/README.md`](ros2_ws/src/xarm_perturbations/README.md)

Implements Cartesian PD/PID control on Lissajous and inspection trajectories.
Evaluates controller robustness under three perturbation conditions:

- Baseline (no perturbation)
- Sinusoidal (8 Hz deterministic disturbance)
- Gaussian white noise (sigma = 0.01 m/s)

Includes a perturbation injector node, a grid-search auto-tuner for Kp/Kd gains,
and offline analysis scripts for RMSE computation and plot generation.

---

## 7. Common Launch Sequence

Every experiment requires MoveIt Servo to be running first. Open terminals in order:

**Terminal 1 — MoveIt Servo (keep running for all experiments):**

```bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py \
  robot_ip:=192.168.1.154
```

Wait for the servo server to print `Ready to accept commands`. Then proceed with
the package-specific launch commands documented in each package's README.

**Emergency stop (any terminal):**

```bash
ros2 topic pub /challenge_stop std_msgs/Bool "data: true" --once
```

---

## 8. Data and Results

Experimental data from Challenge 4.1 is stored in `data/` and archived in `data.zip`.

**Summary of results (Challenge 4.1):**

| Trial | RMSE EE | Max EE Error | Waypoint Success Rate |
|-------|---------|--------------|-----------------------|
| CTC, no perturbation | 29.4 mm | — | — |
| CTC, with perturbation | ~35.3 mm | — | — |
| PD, no perturbation | 29.2 mm | — | — |
| PD, with perturbation | ~38.9 mm | — | — |

CTC shows approximately 20% error increase under perturbation versus approximately
33% for Cartesian PD, demonstrating the robustness advantage of the model-based
approach.

Phase portrait analysis shows asymptotically stable spiral convergence in joint
error space, with estimated damping ratio of approximately 0.89 for both controllers
under nominal conditions.

---

## 9. Documentation

Additional documentation is available under `docs/`:

| Path | Contents |
|------|----------|
| `docs/Robotics_Control_Challenge_4.1/Instructions/` | Challenge specification PDF and reference notebook |
| `docs/Robotics_Control_Lab_4.1/code_documentation/` | Auto-generated per-module documentation for Lab 4.1 |
| `docs/Robotics_Control_Lab_4.2/code_documentation/` | Auto-generated per-module documentation for Lab 4.2 |
| `docs/Robotics_Control_Lab_4.2/Report/` | Final lab report PDF |

---

## 10. Known Issues and Workarounds

**catkin_pkg not found during colcon build:**
Conda environments intercept CMake's Python executable, causing `catkin_pkg` import
failures. Fix: `pip install catkin_pkg` inside the active conda environment.

**Joint state updates at ~10 Hz instead of 100 Hz:**
The xArm hardware driver publishes `/joint_states` at approximately 10 Hz. The CTC
controller node uses a first-order state predictor to extrapolate joint positions
between updates, maintaining smooth 100 Hz control output.

**MoveIt Servo velocity scaling (~22% effective):**
MoveIt Servo internally publishes joint commands at 2 ms intervals, while the
control loop runs at 10 ms. Effective Cartesian velocity is approximately 22% of
the commanded value. IK task-space gains (K_TASK = 20) and velocity limits are
set to compensate for this scaling.

**Wrist singularity (joint 5 near 0 rad):**
The position Jacobian becomes near-singular when joint 5 approaches zero. The
controller scales Cartesian velocity commands by `clip(|q5| / 0.20, 0.05, 1.0)`
to prevent command blow-up, reaching minimum 5% velocity at the singular configuration.

**Analysis script Python environment:**
The analysis script requires numpy/pandas/matplotlib. If the ROS 2 Python
environment has version conflicts, run with miniconda Python directly:
```bash
PYTHONPATH=ros2_ws/src/xarm_ctc_challenge \
  ~/miniconda3/bin/python3 \
  ros2_ws/src/xarm_ctc_challenge/xarm_ctc_challenge/analysis.py \
  data/
```
