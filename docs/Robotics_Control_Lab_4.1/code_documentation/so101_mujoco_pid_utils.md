# so101_mujoco_pid_utils.py

## Overview

High-level motion utilities implementing PID-based control sequences for the SO101 robot. Provides ready-to-use functions for moving to poses and holding positions with perturbation injection and real-time plotting support.

**File Path**: `simulation_code/so101_mujoco_pid_utils.py`

## Purpose

Bridges the gap between low-level control algorithms (`so101_control.py`) and simulation scripts by providing:
- **Motion primitives**: `move_to_pose_pid()`, `hold_position_pid()`
- **Default configurations**: Pre-tuned PID gains and perturbation settings
- **Plotting integration**: Optional real-time data visualization
- **Convenience functions**: Pose interpolation, simulation stepping

## Module Contents

```python
so101_mujoco_pid_utils.py
├── Constants
│   └── DEFAULT_JOINTS
├── Helper Functions
│   ├── lerp_pose()
│   ├── step_sim()
│   ├── build_default_pid()
│   └── build_default_perturbations()
└── Motion Functions
    ├── move_to_pose_pid()
    └── hold_position_pid()
```

---

## Constants

### DEFAULT_JOINTS
```python
DEFAULT_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll"
]
```

**Note**: Excludes "gripper" by default (can be added if needed).

---

## Helper Functions

### `lerp_pose()`

Linear interpolation between two poses.

```python
lerp_pose(
    p0: dict[str, float],      # Start pose
    p1: dict[str, float],      # End pose
    s: float                   # Interpolation parameter [0, 1]
) -> dict[str, float]          # Returns: interpolated pose
```

**Formula**:
```python
result[joint] = (1 - s) * p0[joint] + s * p1[joint]
```

**Example**:
```python
start = {"shoulder_pan": 0.0, "shoulder_lift": 0.0}
end = {"shoulder_pan": 90.0, "shoulder_lift": -45.0}

mid = lerp_pose(start, end, 0.5)
# mid = {"shoulder_pan": 45.0, "shoulder_lift": -22.5}
```

**Usage in motion**:
```python
for i in range(steps):
    s = i / steps  # 0.0 to 1.0
    target = lerp_pose(start_pose, end_pose, s)
```

---

### `step_sim()`

Performs one simulation step with optional viewer update, real-time pacing, and plotting.

```python
step_sim(
    m,                         # MuJoCo model
    d,                         # MuJoCo data
    viewer,                    # Viewer instance (or None)
    realtime: bool,            # Real-time pacing flag
    plotter: Optional[_PlotterProto] = None  # Optional plotter
)
```

**Sequence**:
1. `mujoco.mj_step(m, d)` - Advance physics
2. `plotter.sample(m, d)` - Record state (if plotter provided)
3. `viewer.sync()` - Update display (if viewer provided)
4. `time.sleep(...)` - Real-time pacing (if realtime=True)

**Example**:
```python
for _ in range(1000):
    # ... set torques ...
    step_sim(m, d, viewer, realtime=True, plotter=plotter)
```

---

### `build_default_pid()`

Creates a PID controller with conservative default gains.

```python
build_default_pid(joint_names=DEFAULT_JOINTS) -> JointPID
```

**Default Gains**:
```python
{
    "shoulder_pan":  PIDGains(kp=0.0, ki=0.0, kd=0.0, i_limit=2.0, tau_limit=8.0),
    "shoulder_lift": PIDGains(kp=0.0, ki=0.0, kd=0.0, i_limit=2.0, tau_limit=18.0),
    "elbow_flex":    PIDGains(kp=0.0, ki=0.0, kd=0.5, i_limit=2.0, tau_limit=15.0),
    "wrist_flex":    PIDGains(kp=0.0, ki=0.0, kd=0.5, i_limit=2.0, tau_limit=6.0),
    "wrist_roll":    PIDGains(kp=0.0, ki=0.0, kd=0.8, i_limit=2.0, tau_limit=3.0),
}
```

**Note**: Most gains are **zero** - you must tune them!

**Fallback**: For any joint not in the dictionary:
```python
PIDGains(kp=25.0, ki=0.3, kd=1.0, i_limit=2.0, tau_limit=6.0)
```

**Example**:
```python
pid = build_default_pid()
# Tune gains
pid.gains["shoulder_pan"].kp = 50.0
pid.gains["shoulder_pan"].kd = 2.0
```

---

### `build_default_perturbations()`

Creates a perturbation model with moderate disturbances.

```python
build_default_perturbations(joint_names=DEFAULT_JOINTS) -> PerturbationModel
```

**Default Configuration**:
```python
PerturbationConfig(
    sinus_amp=0.8,              # Moderate sinusoid
    sinus_freq_hz=0.5,          # 0.5 Hz oscillation
    noise_std=0.25,             # Low noise
    noise_tau=0.25,             # Fast correlation
    impulse_prob_per_s=0.12,    # 12% chance per second
    impulse_mag=2.0,            # 2 N·m impulses
    impulse_dur=0.05,           # 50ms duration
    meas_q_std=0.0,             # No measurement noise
    meas_qd_std=0.0,
    seed=7
)
```

**Example**:
```python
perturb = build_default_perturbations()
# Or create custom:
from so101_control import PerturbationConfig, PerturbationModel
cfg = PerturbationConfig(sinus_amp=2.0, noise_std=1.0)
perturb = PerturbationModel(DEFAULT_JOINTS, cfg)
```

---

## Motion Functions

### `move_to_pose_pid()`

Moves robot from current pose to target using PID torque control with optional perturbations and plotting.

```python
move_to_pose_pid(
    m,                                  # MuJoCo model
    d,                                  # MuJoCo data
    viewer,                             # Viewer instance
    target_pose_deg: dict[str, float],  # Target (degrees)
    duration: float = 2.0,              # Motion duration (s)
    realtime: bool = True,              # Real-time pacing
    joint_names = DEFAULT_JOINTS,       # Joints to control
    pid: JointPID | None = None,        # Custom PID (or default)
    perturb: PerturbationModel | None = None,  # Custom perturbations
    plotter: Optional[_PlotterProto] = None    # Optional plotter
)
```

#### Algorithm

```python
1. Read current joint positions (q0)
2. Convert target from degrees to radians (qT)
3. For each timestep over duration:
   a. Compute interpolation factor: s = (t - t0) / duration
   b. Compute target: q_des = lerp(q0, qT, s)
   c. Get current state: q, qd = get_q_qd_dict(m, d, joints)
   d. Add measurement noise: q_meas, qd_meas = perturb.noisy_measurement(q, qd)
   e. Compute PID torque: tau_pid = pid.compute(q_meas, qd_meas, q_des, dt)
   f. Compute disturbances: tau_dist = perturb.apply_joint_torques(t, dt)
   g. Total torque: tau_total = tau_pid + tau_dist
   h. Apply torques and step simulation
```

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `m` | MjModel | Required | MuJoCo model |
| `d` | MjData | Required | MuJoCo data |
| `viewer` | Viewer | Required | Viewer instance |
| `target_pose_deg` | dict | Required | Target in degrees |
| `duration` | float | 2.0 | Motion time (seconds) |
| `realtime` | bool | True | Real-time pacing |
| `joint_names` | list | DEFAULT_JOINTS | Joints to control |
| `pid` | JointPID | None | Custom controller |
| `perturb` | PerturbationModel | None | Custom disturbances |
| `plotter` | Plotter | None | Optional plotting |

#### Example

```python
from so101_mujoco_pid_utils import move_to_pose_pid, build_default_pid

pid = build_default_pid()
pid.gains["shoulder_pan"].kp = 50.0
pid.gains["shoulder_pan"].kd = 2.0

target = {
    "shoulder_pan": 45.0,
    "shoulder_lift": -30.0,
    "elbow_flex": 90.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
}

move_to_pose_pid(m, d, viewer, target, duration=3.0, pid=pid, plotter=plotter)
```

#### Behavior

- **Smooth Trajectory**: Linear interpolation from current to target
- **Continuous Control**: PID recomputed every timestep
- **Disturbances**: Injected if perturbation model provided
- **Data Logging**: Recorded if plotter provided
- **Auto-reset**: PID integrator reset at start

---

### `hold_position_pid()`

Maintains a fixed target pose using PID control, actively rejecting disturbances.

```python
hold_position_pid(
    m,
    d,
    viewer,
    hold_pose_deg: dict[str, float],    # Pose to hold (degrees)
    duration: float = 2.0,              # Hold duration (s)
    realtime: bool = True,
    joint_names = DEFAULT_JOINTS,
    pid: JointPID | None = None,
    perturb: PerturbationModel | None = None,
    plotter: Optional[_PlotterProto] = None
)
```

#### Algorithm

```python
1. Convert hold_pose from degrees to radians (q_des)
2. For each timestep over duration:
   a. Get current state: q, qd
   b. Add measurement noise (if configured)
   c. Compute PID torque: tau_pid = pid.compute(q, qd, q_des, dt)
   d. Compute disturbances: tau_dist
   e. Total torque: tau_total = tau_pid + tau_dist
   f. Apply and step
```

#### Difference from `move_to_pose_pid()`

| Feature | move_to_pose_pid | hold_position_pid |
|---------|------------------|-------------------|
| Target | Interpolated trajectory | Fixed position |
| Use Case | Motion | Disturbance rejection |
| Integrator | Reset at start | Reset at start |

#### Example

```python
target = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, ...}

# Move to target
move_to_pose_pid(m, d, viewer, target, duration=2.0, pid=pid)

# Hold against disturbances
hold_position_pid(m, d, viewer, target, duration=5.0, pid=pid)
```

#### Use Cases

1. **Testing disturbance rejection**: See how well PID fights perturbations
2. **Regulation**: Maintain pose indefinitely
3. **Stability analysis**: Observe steady-state behavior
4. **Between motions**: Hold position before next move

---

## Typical Usage Pattern

### Complete Example

```python
import mujoco
import mujoco.viewer
from so101_mujoco_utils2 import set_initial_pose, RealtimeJointPlotter
from so101_mujoco_pid_utils import (
    move_to_pose_pid,
    hold_position_pid,
    build_default_pid,
    build_default_perturbations
)

# Load model
m = mujoco.MjModel.from_xml_path("model/scene_urdf.xml")
d = mujoco.MjData(m)

# Define poses
start = {"shoulder_pan": 0.0, ...}
waypoint1 = {"shoulder_pan": 45.0, ...}
waypoint2 = {"shoulder_pan": -45.0, ...}

# Setup
set_initial_pose(m, d, start)

# Create controllers
pid = build_default_pid()
pid.gains["shoulder_pan"].kp = 50.0
pid.gains["shoulder_pan"].kd = 2.0
# ... tune other joints ...

perturb = build_default_perturbations()

# Start plotter
plotter = RealtimeJointPlotter(max_points=4000)
plotter.start(host="127.0.0.1", port=8050)

# Run sequence
with mujoco.viewer.launch_passive(m, d) as viewer:
    # Move to waypoint 1
    move_to_pose_pid(m, d, viewer, waypoint1, duration=2.0,
                     pid=pid, perturb=perturb, plotter=plotter)

    # Hold
    hold_position_pid(m, d, viewer, waypoint1, duration=1.0,
                      pid=pid, perturb=perturb, plotter=plotter)

    # Move to waypoint 2
    move_to_pose_pid(m, d, viewer, waypoint2, duration=2.0,
                     pid=pid, perturb=perturb, plotter=plotter)

    # Return to start
    move_to_pose_pid(m, d, viewer, start, duration=2.0,
                     pid=pid, perturb=perturb, plotter=plotter)
```

---

## Customization Examples

### Example 1: No Perturbations

```python
from so101_control import PerturbationConfig, PerturbationModel

quiet_cfg = PerturbationConfig(
    sinus_amp=0.0,
    noise_std=0.0,
    impulse_prob_per_s=0.0
)
quiet_perturb = PerturbationModel(DEFAULT_JOINTS, quiet_cfg)

move_to_pose_pid(m, d, viewer, target, perturb=quiet_perturb)
```

### Example 2: High Disturbances

```python
noisy_cfg = PerturbationConfig(
    sinus_amp=3.0,
    noise_std=2.0,
    impulse_prob_per_s=0.5,
    impulse_mag=5.0
)
noisy_perturb = PerturbationModel(DEFAULT_JOINTS, noisy_cfg)

hold_position_pid(m, d, viewer, target, duration=10.0, perturb=noisy_perturb)
```

### Example 3: Custom Joints

```python
# Control only shoulder and elbow
joints = ["shoulder_pan", "shoulder_lift", "elbow_flex"]

pid = build_default_pid(joints)
perturb = build_default_perturbations(joints)

target = {"shoulder_pan": 30.0, "shoulder_lift": -20.0, "elbow_flex": 90.0}
move_to_pose_pid(m, d, viewer, target, joint_names=joints, pid=pid, perturb=perturb)
```

### Example 4: No Real-time (Fast Simulation)

```python
# Run as fast as possible
move_to_pose_pid(m, d, None, target, realtime=False, plotter=None)
```

---

## Protocol: `_PlotterProto`

```python
class _PlotterProto(Protocol):
    def sample(self, m, d, now: float | None = None) -> None: ...
```

Any object with a `sample(m, d)` method can be passed as `plotter`.

**Compatible**:
- `RealtimeJointPlotter` from `so101_mujoco_utils2`
- Custom data loggers

---

## Design Decisions

### Why Linear Interpolation?

**Pros**:
- Simple, predictable
- Minimal computation
- Velocity-continuous (important for PID)

**Cons**:
- Not acceleration-continuous (jerk at start/end)
- Not time-optimal

**Alternative**: Could use polynomial/spline trajectories for smoother motion.

### Why Reset PID Each Motion?

```python
pid.reset()  # Called at start of each move_to_pose_pid
```

**Reason**: Prevents integral windup from previous motion affecting new motion.

### Why Degrees in API?

```python
target_pose_deg: dict[str, float]  # User provides degrees
# Internally converted to radians
```

**Reason**: Degrees are more intuitive for users. Radians used internally for math.

---

## Common Patterns

### Sequential Motions

```python
poses = [pose1, pose2, pose3, pose4]
for target in poses:
    move_to_pose_pid(m, d, viewer, target, duration=2.0, pid=pid)
    hold_position_pid(m, d, viewer, target, duration=0.5, pid=pid)
```

### Cyclic Motion

```python
while viewer.is_running():
    move_to_pose_pid(m, d, viewer, pose_a, duration=1.0, pid=pid)
    move_to_pose_pid(m, d, viewer, pose_b, duration=1.0, pid=pid)
```

### Conditional Hold

```python
move_to_pose_pid(m, d, viewer, target, duration=2.0, pid=pid)
if some_condition:
    hold_position_pid(m, d, viewer, target, duration=3.0, pid=pid)
```

---

## Debugging Tips

### Check PID Gains

```python
print(f"Kp = {pid.gains['shoulder_pan'].kp}")
print(f"Kd = {pid.gains['shoulder_pan'].kd}")
```

### Monitor Torques

```python
tau = pid.compute(q, qd, q_des, dt)
print(f"Torques: {tau}")
```

### Disable Perturbations

```python
move_to_pose_pid(..., perturb=build_default_perturbations())  # With
move_to_pose_pid(..., perturb=None)  # Default (will create quiet one)
```

### Check Trajectory

```python
# After motion, inspect plot at http://127.0.0.1:8050
# Look for: overshoot, oscillation, tracking error
```

---

## Performance Considerations

### Timestep

Default MuJoCo timestep: `0.002s` (500 Hz)
- **Too large**: PID instability, poor control
- **Too small**: Unnecessary computation

### Duration vs. Steps

```python
steps = int(duration / m.opt.timestep)
# duration=2.0, timestep=0.002 → 1000 steps
```

### Real-time Pacing

```python
realtime=True   # Sleeps to match wall clock (slower, visual)
realtime=False  # Runs as fast as possible (faster, headless)
```

---

## Related Files

- **so101_control.py**: Low-level PID and perturbation classes
- **so101_mujoco_utils2.py**: Complementary utilities (plotting, conversions)
- **run_mujoco_simulation2.py**: Example usage

---

## API Quick Reference

### Functions
```python
lerp_pose(p0, p1, s) -> pose
step_sim(m, d, viewer, realtime, plotter)
build_default_pid(joint_names) -> JointPID
build_default_perturbations(joint_names) -> PerturbationModel

move_to_pose_pid(m, d, viewer, target_pose_deg, duration, ...)
hold_position_pid(m, d, viewer, hold_pose_deg, duration, ...)
```

### Default Configurations
```python
DEFAULT_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# PID: Mostly zero gains (requires tuning)
# Perturbations: Moderate (sinus_amp=0.8, noise_std=0.25, impulse=0.12/s)
```
