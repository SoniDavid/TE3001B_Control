# run_mujoco_simulation2.py

## Overview

Advanced simulation demonstrating PID torque control with perturbation injection and real-time plotting. This script showcases realistic control challenges including disturbances, noise, and impulse perturbations.

**File Path**: `simulation_code/run_mujoco_simulation2.py`

## Purpose

This script demonstrates:
- **PID Control**: Joint-space torque control with tunable gains
- **Perturbation Modeling**: Realistic disturbances (sinusoidal, noise, impulses)
- **Real-time Plotting**: Live visualization of joint trajectories
- **Control Testing**: Platform for testing and tuning control algorithms

## Dependencies

```python
import mujoco
import mujoco.viewer
from so101_mujoco_utils2 import set_initial_pose, RealtimeJointPlotter
from so101_mujoco_pid_utils import move_to_pose_pid, hold_position_pid
```

## Key Differences from run_mujoco_simulation.py

| Feature | run_mujoco_simulation.py | run_mujoco_simulation2.py |
|---------|--------------------------|----------------------------|
| Control Method | Position actuators | PID torque control |
| Disturbances | None | Sinusoid + noise + impulses |
| Plotting | None | Real-time web-based plots |
| Tuning | Fixed gains | Customizable PID gains |
| Realism | Servo-like | Research-grade |

## Script Structure

### 1. Model Setup
```python
MODEL_PATH = "model/scene_urdf.xml"
m = mujoco.MjModel.from_xml_path(MODEL_PATH)
d = mujoco.MjData(m)
```

### 2. Pose Definitions
```python
starting_position = {
    "shoulder_pan":  -4.4003158666,
    "shoulder_lift": -92.2462050161,
    "elbow_flex":     89.9543738355,
    "wrist_flex":     55.1185398916,
    "wrist_roll":      0.0,
    "gripper":         0.0,
}

desired_zero = {
    "shoulder_pan":  0.0,
    "shoulder_lift": 0.0,
    "elbow_flex":    0.0,
    "wrist_flex":    0.0,
    "wrist_roll":    0.0,
    "gripper":       0.0,
}
```

### 3. Plotter Initialization
```python
plotter = RealtimeJointPlotter(max_points=4000)
plotter.start(host="127.0.0.1", port=8050, update_ms=100)
```
- **Buffer**: Stores up to 4000 data points
- **Update Rate**: Refreshes plot every 100ms
- **Access**: Open http://127.0.0.1:8050 in browser

### 4. Initial Configuration
```python
set_initial_pose(m, d, starting_position)
```

### 5. PID Control Sequence
```python
with mujoco.viewer.launch_passive(m, d) as viewer:
    move_to_pose_pid(m, d, viewer, desired_zero, duration=2.0, realtime=True, plotter=plotter)
    hold_position_pid(m, d, viewer, desired_zero, duration=2.0, realtime=True, plotter=plotter)

    move_to_pose_pid(m, d, viewer, starting_position, duration=2.0, realtime=True, plotter=plotter)
    hold_position_pid(m, d, viewer, starting_position, duration=2.0, realtime=True, plotter=plotter)
```

## PID Control Functions

### `move_to_pose_pid()`

Moves robot from current pose to target using PID torque control with perturbations.

**Parameters**:
```python
move_to_pose_pid(
    m, d, viewer,
    target_pose_deg: dict,           # Target joint positions (degrees)
    duration: float = 2.0,           # Motion duration (seconds)
    realtime: bool = True,           # Real-time pacing
    joint_names = DEFAULT_JOINTS,    # Joints to control
    pid: JointPID | None = None,     # Custom PID controller
    perturb: PerturbationModel | None = None,  # Custom perturbations
    plotter: RealtimeJointPlotter | None = None  # Plotting instance
)
```

**Behavior**:
1. Interpolates from current to target pose
2. Computes PID torques: `τ = Kp*e + Ki*∫e + Kd*ė`
3. Adds perturbation torques
4. Applies total torque via `d.qfrc_applied`
5. Records data for plotting
6. Steps simulation

### `hold_position_pid()`

Maintains a fixed pose using PID control against disturbances.

**Parameters**: Similar to `move_to_pose_pid()` but with `hold_pose_deg` instead of trajectory.

**Behavior**:
1. Sets constant target position
2. Continuously computes PID correction torques
3. Fights against perturbations
4. Demonstrates disturbance rejection capability

## PID Controller Details

### Default Gains
```python
gains = {
    "shoulder_pan":  PIDGains(kp=0.0, ki=0.0, kd=0.0, i_limit=2.0, tau_limit=8.0),
    "shoulder_lift": PIDGains(kp=0.0, ki=0.0, kd=0.0, i_limit=2.0, tau_limit=18.0),
    "elbow_flex":    PIDGains(kp=0.0, ki=0.0, kd=0.5, i_limit=2.0, tau_limit=15.0),
    "wrist_flex":    PIDGains(kp=0.0, ki=0.0, kd=0.5, i_limit=2.0, tau_limit=6.0),
    "wrist_roll":    PIDGains(kp=0.0, ki=0.0, kd=0.8, i_limit=2.0, tau_limit=3.0),
}
```

**Note**: Default gains are **intentionally conservative** (mostly zero). You need to tune them!

### PID Equation
For each joint:
```
e(t) = target - actual            (position error)
τ = Kp*e + Ki*∫e dt + Kd*ė        (control torque)
τ = clip(τ, -tau_limit, tau_limit)  (saturation)
```

### Anti-windup
```python
integral = clip(integral, -i_limit, i_limit)
```
Prevents integral term from growing unbounded during saturation.

## Perturbation Model

### Default Configuration
```python
cfg = PerturbationConfig(
    sinus_amp=0.8,              # Sinusoid amplitude (N·m)
    sinus_freq_hz=0.5,          # Sinusoid frequency (Hz)
    noise_std=0.25,             # Noise standard deviation (N·m)
    noise_tau=0.25,             # Noise time constant (s)
    impulse_prob_per_s=0.12,    # Impulse rate (per second)
    impulse_mag=2.0,            # Impulse magnitude (N·m)
    impulse_dur=0.05,           # Impulse duration (s)
    meas_q_std=0.0,             # Position measurement noise
    meas_qd_std=0.0,            # Velocity measurement noise
    seed=7                      # Random seed
)
```

### Disturbance Components

1. **Sinusoidal Torque**
   ```python
   τ_sin = amp * sin(2π * freq * t + phase)
   ```
   - Simulates periodic disturbances (e.g., vibration)
   - Each joint has different phase offset

2. **Colored Noise**
   ```python
   noise[t] = (1-α)*noise[t-1] + α*white_noise
   α = dt / (tau + dt)
   ```
   - Low-pass filtered Gaussian noise
   - More realistic than white noise
   - Adjustable correlation time (`noise_tau`)

3. **Random Impulses**
   - Randomly triggered on 1-3 joints
   - Short duration (50ms default)
   - Simulates impacts, contact events
   - Probability: 12% per second

### Total Disturbance Torque
```python
τ_disturb = τ_sin + τ_noise + τ_impulse
τ_disturb = clip(τ_disturb, -6.0, 6.0)  # Safety limit
```

## Real-time Plotting

### Setup
```python
plotter = RealtimeJointPlotter(max_points=4000)
plotter.start(host="127.0.0.1", port=8050, update_ms=100)
```

### Data Recording
```python
plotter.sample(m, d)  # Called every simulation step
```

### Web Interface
- **URL**: http://127.0.0.1:8050
- **Features**:
  - Interactive zoom/pan
  - Legend for each joint
  - Hover for exact values
  - Auto-scrolling time window

### Plot Display
- **X-axis**: Time (seconds)
- **Y-axis**: Joint position (degrees) or gripper (0-100)
- **Traces**: One line per joint, color-coded
- **Update**: ~10 Hz refresh rate

## Usage

### Running the Script
```bash
cd simulation_code
python run_mujoco_simulation2.py
```

### Observing Behavior

1. **3D Viewer**: Shows robot motion and perturbation effects
2. **Web Browser**: Navigate to http://127.0.0.1:8050 for plots
3. **Motion Sequence**:
   - 0-2s: Move to zero (with disturbances)
   - 2-4s: Hold zero (fighting perturbations)
   - 4-6s: Return to start (with disturbances)
   - 6-8s: Hold start (fighting perturbations)

### What to Observe

- **Tracking Error**: How well robot follows desired trajectory
- **Disturbance Rejection**: Response to perturbations during hold
- **Smoothness**: Jittery motion indicates poor gains or high noise
- **Overshoot**: Oscillations indicate high Kp or low Kd

## Tuning PID Gains

### Step 1: Start with Defaults
The default gains are mostly zero - robot will fall!

### Step 2: Add Proportional Gain (Kp)
```python
from so101_mujoco_pid_utils import build_default_pid

pid = build_default_pid()
pid.gains["shoulder_pan"].kp = 25.0
pid.gains["shoulder_lift"].kp = 30.0
pid.gains["elbow_flex"].kp = 20.0

move_to_pose_pid(m, d, viewer, target, duration=2.0, pid=pid, plotter=plotter)
```

**Effect**: Robot will track position but may oscillate.

### Step 3: Add Derivative Gain (Kd)
```python
pid.gains["shoulder_pan"].kd = 2.0
pid.gains["shoulder_lift"].kd = 2.5
```

**Effect**: Reduces oscillations, adds damping.

### Step 4: Add Integral Gain (Ki) if Needed
```python
pid.gains["shoulder_pan"].ki = 0.5
```

**Effect**: Eliminates steady-state error, but can cause instability if too high.

### Tuning Guidelines

| Symptom | Solution |
|---------|----------|
| Robot falls | Increase Kp |
| Oscillations | Increase Kd, decrease Kp |
| Steady-state error | Increase Ki (carefully) |
| Overshoot | Decrease Kp, increase Kd |
| Slow response | Increase Kp |
| Jerky motion | Decrease Kd |

## Modifications

### Disable Perturbations
```python
from so101_control import PerturbationConfig, PerturbationModel

# Create "quiet" perturbation model
quiet_cfg = PerturbationConfig(
    sinus_amp=0.0,
    noise_std=0.0,
    impulse_prob_per_s=0.0
)
perturb = PerturbationModel(DEFAULT_JOINTS, quiet_cfg)

move_to_pose_pid(m, d, viewer, target, perturb=perturb, plotter=plotter)
```

### Increase Disturbances
```python
noisy_cfg = PerturbationConfig(
    sinus_amp=2.0,        # Stronger sinusoid
    noise_std=1.0,        # More noise
    impulse_prob_per_s=0.5,  # More frequent impulses
    impulse_mag=5.0       # Stronger impulses
)
```

### Change Plotting Settings
```python
plotter = RealtimeJointPlotter(max_points=10000)  # Longer history
plotter.start(port=9000, update_ms=50)  # Different port, faster updates
```

### Add Measurement Noise
```python
noisy_cfg.meas_q_std = 0.01   # Position noise (rad)
noisy_cfg.meas_qd_std = 0.05  # Velocity noise (rad/s)
```

## Advanced Usage

### Custom PID Controller
```python
from so101_control import JointPID, PIDGains

gains = {
    "shoulder_pan": PIDGains(kp=50.0, ki=1.0, kd=3.0, i_limit=5.0, tau_limit=10.0),
    "shoulder_lift": PIDGains(kp=60.0, ki=1.2, kd=4.0, i_limit=5.0, tau_limit=20.0),
    # ... configure all joints
}

pid = JointPID(DEFAULT_JOINTS, gains)
move_to_pose_pid(m, d, viewer, target, pid=pid, plotter=plotter)
```

### Save Plot Data
Modify `RealtimeJointPlotter` to export data:
```python
# In your code after simulation:
import json
with open('trajectory_data.json', 'w') as f:
    json.dump({
        'time': list(plotter._t),
        'joints': {jn: list(plotter._y[jn]) for jn in plotter.joint_names}
    }, f)
```

## Troubleshooting

### Robot falls immediately
- **Cause**: PID gains too low (or zero)
- **Solution**: Increase Kp to at least 20-30

### Violent oscillations
- **Cause**: Kp too high, Kd too low
- **Solution**: Decrease Kp, increase Kd

### Plot shows flat lines
- **Cause**: `plotter.sample()` not being called
- **Solution**: Verify it's in the simulation loop

### Browser shows "No samples yet"
- **Cause**: Simulation hasn't started or crashed
- **Solution**: Check console for errors, verify simulation is running

### Unstable during hold_position
- **Cause**: Perturbations too strong or gains too weak
- **Solution**: Tune gains higher or reduce perturbation magnitudes

## Comparison with Other Scripts

### vs. run_mujoco_simulation.py
- **Pros**: More realistic, tunable, testable
- **Cons**: Requires gain tuning, more complex
- **Use when**: Testing controllers, research, learning PID

### vs. run_mujoco_simulation_startingpose.py
- **Difference**: This uses PID, that uses position actuators
- **This script**: Better for control experimentation
- **Other script**: Better for simple demos

## Related Files

- **so101_mujoco_pid_utils.py**: PID motion function implementations
- **so101_control.py**: Core PID and perturbation classes
- **so101_mujoco_utils2.py**: Plotting and utility functions

## Learning Objectives

1. Understanding PID control in robotics
2. Effects of proportional, integral, derivative terms
3. Disturbance rejection strategies
4. Gain tuning methodology
5. Real-time data visualization
6. Realistic simulation practices

## Research Applications

This script is suitable for:
- **Control Algorithm Testing**: Compare PID, LQR, MPC, etc.
- **Robustness Analysis**: Evaluate performance under disturbances
- **Gain Optimization**: Automated tuning experiments
- **Trajectory Planning**: Test different interpolation methods
- **Observer Design**: Add state estimation with noisy measurements

## Next Steps

1. **Tune Gains**: Experiment with different PID values
2. **Modify Perturbations**: Try different disturbance profiles
3. **Add Waypoints**: Create complex trajectories
4. **Implement Feedforward**: Add gravity compensation
5. **Study Code**: Read `so101_control.py` for implementation details
