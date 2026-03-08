# xarm_perturbations — Lab 4.2: Cartesian PD Control with Perturbation Analysis

ROS 2 Humble package implementing Cartesian PD/PID control on the UFACTORY xArm
Lite 6, with perturbation injection and offline performance analysis. The primary
objective is to characterize how the controller responds to three distinct
disturbance conditions: baseline (no perturbation), sinusoidal (deterministic),
and Gaussian white noise (stochastic).

The package uses MoveIt Servo's `TwistStamped` interface to command Cartesian
velocities in real time at 100 Hz.

---

## Table of Contents

1. [Package Overview](#1-package-overview)
2. [Prerequisites](#2-prerequisites)
3. [Package Structure](#3-package-structure)
4. [Module Descriptions](#4-module-descriptions)
   - [trajectory_and_controller.py](#trajectory_and_controllerpy)
   - [perturbation_injector.py](#perturbation_injectorpy)
   - [auto_tuner.py](#auto_tunerpy)
   - [circle_maker.py](#circle_makerpy)
   - [analysis_evaluation.py](#analysis_evaluationpy)
   - [analysis_evaluation_only.py](#analysis_evaluation_onlypy)
5. [ROS 2 Interface](#5-ros-2-interface)
   - [Topics Published](#topics-published)
   - [Topics Subscribed](#topics-subscribed)
   - [TF2 Frames Used](#tf2-frames-used)
6. [Control Algorithm](#6-control-algorithm)
7. [Perturbation Modes](#7-perturbation-modes)
8. [Running the Experiments](#8-running-the-experiments)
   - [Step 1: Launch MoveIt Servo](#step-1-launch-moveit-servo)
   - [Step 2: Run Baseline Experiment](#step-2-run-baseline-experiment)
   - [Step 3: Run Sinusoidal Perturbation](#step-3-run-sinusoidal-perturbation)
   - [Step 4: Run Gaussian Perturbation](#step-4-run-gaussian-perturbation)
9. [Auto-Tuner (Optional)](#9-auto-tuner-optional)
10. [Analysis](#10-analysis)
11. [Controller Parameters Reference](#11-controller-parameters-reference)
12. [Experimental Results Summary](#12-experimental-results-summary)
13. [Known Issues and Troubleshooting](#13-known-issues-and-troubleshooting)

---

## 1. Package Overview

The control system is structured as independent, composable ROS 2 nodes:

- **Trajectory and controller node:** Generates the reference trajectory and
  computes Cartesian PD correction commands at 100 Hz.
- **Perturbation injector node:** Publishes additional velocity disturbances on the
  same Servo topic, superimposed on the controller's output.
- **Auto-tuner:** Grid-searches Kp/Kd combinations and evaluates cost (RMSE +
  saturation penalty) to assist manual gain selection.
- **Circle maker:** Generates an alternative circular Cartesian trajectory.
- **Analysis scripts:** Offline RMSE computation and plot generation from CSV logs.

**Control rate:** 100 Hz

**Robot interface:** MoveIt Servo, `TwistStamped` on `/servo_server/delta_twist_cmds`

---

## 2. Prerequisites

**Hardware:**
- xArm Lite 6, connected via Ethernet, static IP `192.168.1.154`
- Robot in Mode 1, State 0 before sending velocity commands

**Build:**

```bash
cd ros2_ws
colcon build --packages-select xarm_perturbations
source install/setup.bash
```

Or build the full workspace, skipping simulation packages:

```bash
colcon build --packages-skip gazebo_mujoco_bridge mbot_demo
source install/setup.bash
```

**Python dependencies** (not installed by ROS 2 automatically):

```bash
pip install numpy pandas matplotlib scipy
```

**pynput** is required for keyboard interaction in the controller node:

```bash
pip install pynput
```

---

## 3. Package Structure

```
xarm_perturbations/
|-- xarm_perturbations/
|   |-- __init__.py
|   |-- trajectory_and_controller.py   Lissajous/inspection trajectory + Cartesian PD
|   |-- perturbation_injector.py       Gaussian or sinusoidal disturbance publisher
|   |-- auto_tuner.py                  Grid-search Kp/Kd gain tuner
|   |-- circle_maker.py                Alternative circular Cartesian trajectory
|   |-- analysis_evaluation.py         Full offline analysis with plots
|   `-- analysis_evaluation_only.py    Lightweight metrics-only analysis (no plots)
|-- test/
|   |-- test_copyright.py
|   |-- test_flake8.py
|   `-- test_pep257.py
|-- package.xml
|-- setup.py
|-- setup.cfg
`-- README.md
```

---

## 4. Module Descriptions

### trajectory_and_controller.py

The main experiment node (approximately 666 lines). Combines trajectory generation
and the Cartesian PD controller in a single ROS 2 node running at 100 Hz.

**Trajectory options:**

| Option | Description |
|--------|-------------|
| Lissajous figure-eight | Continuous curve in the Cartesian XY plane with a soft-start ramp. Parameterized by radius, frequency, and plane. |
| Inspection waypoints | 9-waypoint path (8 inspection positions + home) with quintic-spline interpolation. |

**Controller:**

Cartesian PD/PID on position error obtained from TF2 (`link_base -> link_eef`):

```
e     = p_des - p_act                              Cartesian position error (m)
e_dot = LPF(de/dt, fc)                             Filtered velocity (fc default 2 Hz)
v_c   = Kp*e + Kd*e_dot [+ Ki*integral(e)]        PD or PID velocity command
```

Safety processing applied in order:
1. Per-axis deadband: zero command within 2 mm
2. LPF on derivative term (configurable cutoff, default 2 Hz)
3. Magnitude saturation: clamp ||v_c|| to max_speed (default 0.12 m/s)

**ROS 2 parameters (all settable at launch or runtime):**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `radius` | 0.15 | Trajectory radius (m) |
| `frequency` | 0.035 | Lissajous frequency (Hz) |
| `plane` | `xy` | Motion plane |
| `hold_z` | `false` | Lock Z axis during motion |
| `yaw_deg` | 90.0 | Yaw orientation offset (deg) |
| `kp` | [2.665, 2.665, 2.665] | Proportional gains |
| `kd` | [0.64, 0.64, 0.64] | Derivative gains |
| `ki` | [0.0, 0.0, 0.0] | Integral gains (set > 0 for PID) |
| `max_speed` | 0.12 | Max Cartesian speed (m/s) |
| `deadband` | 0.002 | Per-axis deadband (m) |
| `lpf_cutoff` | 2.0 | LPF cutoff for derivative (Hz) |
| `save_csv` | `true` | Enable CSV logging |
| `csv_filename` | `experiment.csv` | Output CSV filename |

**CSV logging:** Records time, desired position/velocity, actual position, error,
control output, and saturation flags. Flushed continuously.

---

### perturbation_injector.py

Standalone ROS 2 node (approximately 145 lines) that injects velocity disturbances
by publishing additional `TwistStamped` messages on the Servo command topic,
superimposed on the controller output.

**Perturbation modes:**

| Mode | Behavior |
|------|---------|
| `off` | No disturbance published |
| `gaussian` | White noise drawn from N(0, sigma^2) at each tick (seed = 7 for reproducibility) |
| `sine` | Deterministic sinusoidal disturbance at specified frequency and amplitude |

**ROS 2 parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `output_topic` | `/servo_server/delta_twist_cmds` | Topic to publish disturbances on |
| `enabled` | `true` | Master enable switch |
| `mode` | `gaussian` | Perturbation mode: `off`, `gaussian`, `sine` |
| `max_lin` | 0.20 | Maximum linear disturbance amplitude (m/s) |
| `noise_std_linear` | 0.5 | Gaussian standard deviation for linear axes (m/s) |
| `gauss_axis` | `x` | Axis to apply Gaussian noise on (`x`, `y`, `z`, `all`) |
| `sine_freq_hz` | 8.0 | Sinusoidal perturbation frequency (Hz) |
| `sine_amp_linear` | 0.02 | Sinusoidal perturbation amplitude (m/s) |
| `sine_axis` | `x` | Axis for sinusoidal disturbance |

All parameters can be changed at runtime using `ros2 param set`.

---

### auto_tuner.py

Grid-search gain tuner (approximately 211 lines) that evaluates a cost function
over a user-defined Kp/Kd grid:

```
cost = RMSE(position_error) + alpha * saturation_fraction
```

Run this node while the main controller is paused. The tuner commands short test
motions, records the response, and prints ranked gain combinations.

**Entry point:** `auto_tuner`

```bash
ros2 run xarm_perturbations auto_tuner
```

---

### circle_maker.py

Generates a circular Cartesian trajectory (approximately 348 lines) as an
alternative to the Lissajous figure-eight. Parameterized by radius, centre
position, plane, and angular velocity.

**Entry point:** `circle_maker`

```bash
ros2 run xarm_perturbations circle_maker --ros-args \
  -p radius:=0.10 \
  -p angular_velocity:=0.2
```

---

### analysis_evaluation.py

Offline analysis script (approximately 251 lines). Reads CSV files produced by the
controller node and computes:

- Root Mean Square Error (RMSE) per axis and combined 3D error
- Maximum absolute error per axis
- Saturation fraction (percentage of ticks at velocity limit)
- Plots: position tracking overlay, error over time, 3D trajectory

**Usage:**

```bash
python3 ros2_ws/src/xarm_perturbations/xarm_perturbations/analysis_evaluation.py
```

The script searches for CSV files in the current directory by default. Edit the
`FILE_MAP` dictionary at the top of the script to point to specific files.

---

### analysis_evaluation_only.py

Lightweight version of the analysis script (approximately 209 lines) that computes
metrics without generating plots. Useful for quick comparison of multiple runs.

**Usage:**

```bash
python3 ros2_ws/src/xarm_perturbations/xarm_perturbations/analysis_evaluation_only.py
```

---

## 5. ROS 2 Interface

### Topics Published

| Node | Topic | Type | Description |
|------|-------|------|-------------|
| trajectory_and_controller | `/servo_server/delta_twist_cmds` | `geometry_msgs/TwistStamped` | Cartesian PD velocity command |
| perturbation_injector | `/servo_server/delta_twist_cmds` | `geometry_msgs/TwistStamped` | Injected velocity disturbance |

Both nodes publish to the same topic. MoveIt Servo sums the commands internally.

### Topics Subscribed

| Node | Topic | Type | Description |
|------|-------|------|-------------|
| trajectory_and_controller | `/joint_states` | `sensor_msgs/JointState` | Measured joint state (used for singularity check and limit avoidance) |

### TF2 Frames Used

| Parent Frame | Child Frame | Purpose |
|-------------|-------------|---------|
| `link_base` | `link_eef` | Actual end-effector position for PD error computation |

---

## 6. Control Algorithm

The Cartesian PD controller computes a velocity command in the task space:

```
e[k]     = p_des[k] - p_act[k]                  position error (m)
e_dot[k] = alpha * e_dot[k-1] + (1-alpha) * (e[k] - e[k-1]) / dt
                                                  LPF-filtered derivative
v_c[k]   = Kp * e[k] + Kd * e_dot[k]            PD command (m/s)
```

Where `alpha = exp(-2*pi*fc*dt)` is the LPF coefficient for cutoff frequency `fc`.

**Applied safety processing:**

1. Per-axis deadband: if `|e_i| < deadband`, set `v_c_i = 0`.
2. Magnitude saturation: if `||v_c|| > max_speed`, scale uniformly:
   `v_c = v_c * max_speed / ||v_c||`.

The resulting `v_c` is published as the linear component of a `TwistStamped`
message in the `link_base` frame.

**Why Cartesian PD (model-free):**
This baseline controller requires no kinematic or dynamic model of the robot.
It relies entirely on MoveIt Servo to translate Cartesian velocity commands into
safe joint velocity commands, handling joint limits and singularity avoidance
internally through servo's own safety layer.

---

## 7. Perturbation Modes

Three experimental conditions are evaluated to characterize controller robustness:

**Baseline (no perturbation):**
The perturbation injector is not running. Controller tracks the nominal trajectory
under hardware noise and actuator non-idealities only.

**Sinusoidal (deterministic, 8 Hz):**
A periodic velocity disturbance of fixed frequency and amplitude is injected.
This tests the controller's ability to reject a known, structured disturbance.
The derivative term of PD is expected to provide partial rejection.

```
v_pert(t) = sine_amp * sin(2*pi*sine_freq*t)   on the specified axis
```

**Gaussian (stochastic, white noise):**
High-frequency white noise drawn from N(0, sigma^2) is injected independently
at each timestep. This tests the worst-case scenario for the derivative term,
since Gaussian noise excites all frequencies including those above the LPF cutoff.

```
v_pert(t) ~ N(0, noise_std_linear^2)           on the specified axis
```

The random seed is fixed at 7 for reproducibility across runs.

---

## 8. Running the Experiments

### Step 1: Launch MoveIt Servo

This must remain running for all experiments. Start it first and wait for
`Ready to accept commands` in the terminal output:

```bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py \
  robot_ip:=192.168.1.154
```

Ensure the robot is powered on, in Mode 1, State 0, and within its joint limits
before running this command.

### Step 2: Run Baseline Experiment

```bash
ros2 run xarm_perturbations trajectory_and_controller --ros-args \
  -p radius:=0.15 \
  -p frequency:=0.035 \
  -p plane:=xy \
  -p hold_z:=false \
  -p yaw_deg:=90.0 \
  -p kp:="[2.665,2.665,2.665]" \
  -p kd:="[0.64,0.64,0.64]" \
  -p max_speed:=0.12 \
  -p deadband:=0.002 \
  -p save_csv:=true \
  -p csv_filename:=baseline_experiment.csv
```

### Step 3: Run Sinusoidal Perturbation

In one terminal, start the perturbation injector:

```bash
ros2 run xarm_perturbations perturbation_injector --ros-args \
  -p enabled:=true \
  -p mode:=sine \
  -p sine_freq_hz:=8.0 \
  -p sine_amp_linear:=0.02 \
  -p sine_axis:=x
```

In another terminal, run the controller:

```bash
ros2 run xarm_perturbations trajectory_and_controller --ros-args \
  -p kp:="[2.665,2.665,2.665]" \
  -p kd:="[0.64,0.64,0.64]" \
  -p save_csv:=true \
  -p csv_filename:=sine_experiment.csv
```

### Step 4: Run Gaussian Perturbation

In one terminal, start the perturbation injector:

```bash
ros2 run xarm_perturbations perturbation_injector --ros-args \
  -p enabled:=true \
  -p mode:=gaussian \
  -p noise_std_linear:=0.5 \
  -p gauss_axis:=x
```

In another terminal, run the controller:

```bash
ros2 run xarm_perturbations trajectory_and_controller --ros-args \
  -p kp:="[2.665,2.665,2.665]" \
  -p kd:="[0.64,0.64,0.64]" \
  -p save_csv:=true \
  -p csv_filename:=gaussian_experiment.csv
```

---

## 9. Auto-Tuner (Optional)

The auto-tuner performs a grid search over Kp and Kd ranges to minimize a combined
cost function of tracking RMSE and saturation fraction. Run it before selecting
final gains for the experiments.

```bash
ros2 run xarm_perturbations auto_tuner
```

The tuner will print a ranked table of gain combinations. Select gains that balance
low RMSE with low saturation fraction (high saturation indicates the controller is
commanding at its velocity limit, which degrades tracking quality).

The empirically selected gains for this lab were `Kp = 2.665`, `Kd = 0.64` (isotropic).

---

## 10. Analysis

After experiments are complete, run the analysis script to compute metrics and
generate plots:

```bash
python3 ros2_ws/src/xarm_perturbations/xarm_perturbations/analysis_evaluation.py
```

Edit `FILE_MAP` at the top of the script to point to the CSV files for each
experimental condition.

**Metrics output:**

| Metric | Baseline | Sinusoidal | Gaussian |
|--------|----------|------------|---------|
| Total RMSE (m) | ~0.032 | moderate | ~0.032 |
| Max absolute error (m) | low | moderate | ~0.115 |
| Saturation fraction | low | low-moderate | high |

**Interpretation:**
- Baseline and Gaussian have similar average RMSE because Gaussian noise is
  zero-mean. However, Gaussian has far higher instantaneous peaks.
- The derivative term amplifies high-frequency Gaussian noise, causing
  intermittent large errors.
- Sinusoidal perturbation at 8 Hz is partially rejected by the LPF (2 Hz cutoff)
  on the derivative term, resulting in moderate performance degradation.

---

## 11. Controller Parameters Reference

### Cartesian PD Gains (tuned for xArm Lite 6 at 100 Hz)

| Parameter | Value | Units |
|-----------|-------|-------|
| Kp (proportional) | 2.665 (isotropic) | (m/s)/m |
| Kd (derivative) | 0.64 (isotropic) | 1/s |
| LPF cutoff (derivative) | 2.0 | Hz |
| Deadband | 0.002 | m |
| Max Cartesian speed | 0.12 | m/s |

### Lissajous Trajectory Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Radius | 0.15 m | Amplitude of each Lissajous axis |
| Frequency | 0.035 Hz | Base oscillation frequency |
| Soft-start ramp | 5 s | Gradual ramp from zero to full amplitude at start |

### Perturbation Injector (Gaussian configuration)

| Parameter | Value | Units |
|-----------|-------|-------|
| Mode | gaussian | — |
| Standard deviation | 0.5 | m/s |
| Random seed | 7 | — |
| Applied axis | x | — |

### Perturbation Injector (Sinusoidal configuration)

| Parameter | Value | Units |
|-----------|-------|-------|
| Mode | sine | — |
| Frequency | 8.0 | Hz |
| Amplitude | 0.02 | m/s |
| Applied axis | x | — |

---

## 12. Experimental Results Summary

Results obtained with xArm Lite 6, gains Kp = 2.665, Kd = 0.64, Lissajous
trajectory (radius 0.15 m, frequency 0.035 Hz):

| Condition | Total RMSE | Max Absolute Error | Observation |
|-----------|-----------|-------------------|-------------|
| Baseline | ~0.032 m | low | Stable tracking with hardware noise only |
| Sinusoidal (8 Hz) | moderate | moderate | LPF partially rejects structured disturbance |
| Gaussian (sigma=0.5 m/s) | ~0.032 m | ~0.115 m | High instantaneous peaks from noise amplification |

**Key finding:** Gaussian noise produces the highest maximum absolute error (~115 mm)
despite similar average RMSE to baseline, because the derivative term amplifies
high-frequency noise components that exceed the 2 Hz LPF cutoff. This motivates
model-based control (CTC) for improved robustness under stochastic disturbances.

---

## 13. Known Issues and Troubleshooting

**pynput import error at startup:**
Install the package: `pip install pynput`. The controller node uses it for optional
keyboard interaction (pause, stop).

**TF2 lookup fails:**
Ensure MoveIt Servo is fully running before starting the controller. The node
waits up to 5 s for the `link_base -> link_eef` transform. If startup fails,
check that the robot is connected and the servo server reports ready.

**Perturbation injector commands conflict with controller:**
Both nodes publish on the same topic. MoveIt Servo processes all incoming messages,
not just the latest. Start the perturbation injector before the controller to
ensure both are synchronously active during the experiment.

**High saturation rate with Gaussian perturbation:**
This is expected behavior. The `max_speed` limit prevents runaway commands but
means the controller is frequently clamped during high-noise periods. This is
visible in the analysis as flat-topped velocity profiles in the CSV data.

**Analysis script cannot find CSV files:**
Edit `FILE_MAP` at the top of `analysis_evaluation.py` with the correct file paths
for each experimental condition. The script does not auto-discover files by pattern.

**Conda environment numpy conflict:**
If analysis fails with numpy API errors, run the script with a separate Python
installation: `~/miniconda3/bin/python3 analysis_evaluation.py`.
