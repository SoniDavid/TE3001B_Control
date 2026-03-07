# so101_control.py

## Overview

Core control library implementing PID controllers and perturbation models for realistic robot control simulation. This module provides the fundamental algorithms used by the simulation scripts.

**File Path**: `simulation_code/so101_control.py`

## Purpose

Provides:
- **Joint-space PID controller** with anti-windup
- **Multi-component perturbation model** (sinusoid, noise, impulses)
- **MuJoCo helper functions** for state queries and torque application
- **Configurable control parameters** via dataclasses

## Module Structure

```python
so101_control.py
├── PID Control
│   ├── PIDGains (dataclass)
│   └── JointPID (class)
├── Perturbations
│   ├── PerturbationConfig (dataclass)
│   └── PerturbationModel (class)
└── MuJoCo Helpers
    ├── get_q_qd_dict()
    └── apply_joint_torques_qfrc()
```

---

## PID Control

### PIDGains (Dataclass)

```python
@dataclass
class PIDGains:
    kp: float                    # Proportional gain
    ki: float                    # Integral gain
    kd: float                    # Derivative gain
    i_limit: float = 5.0         # Integral anti-windup limit
    tau_limit: float = 10.0      # Torque output limit
```

**Purpose**: Encapsulates PID parameters for one joint.

**Example**:
```python
shoulder_gains = PIDGains(
    kp=50.0,
    ki=1.0,
    kd=3.0,
    i_limit=2.0,
    tau_limit=15.0
)
```

### JointPID (Class)

Full-featured joint-space PID controller.

#### Constructor
```python
JointPID(joint_names: list[str], gains: dict[str, PIDGains])
```

**Parameters**:
- `joint_names`: List of joint names to control
- `gains`: Dictionary mapping joint name → PIDGains

**Example**:
```python
gains = {
    "shoulder_pan": PIDGains(kp=50.0, ki=0.5, kd=2.0),
    "shoulder_lift": PIDGains(kp=60.0, ki=0.6, kd=2.5),
}
pid = JointPID(["shoulder_pan", "shoulder_lift"], gains)
```

#### Methods

##### `reset()`
```python
pid.reset()
```
Resets integrator and derivative state to zero. Call between motions.

##### `compute()`
```python
tau = pid.compute(
    q: dict[str, float],          # Current positions (rad)
    qd: dict[str, float],         # Current velocities (rad/s)
    q_des: dict[str, float],      # Desired positions (rad)
    dt: float                     # Time step (s)
) -> dict[str, float]             # Returns: torques (N·m)
```

**Algorithm** (per joint):
```
e = q_des - q                              # Position error
de = (e - e_prev) / dt                     # Error derivative
integral += e * dt                         # Accumulate integral
integral = clip(integral, -i_limit, i_limit)  # Anti-windup

tau = kp*e + ki*integral + kd*de
tau = clip(tau, -tau_limit, tau_limit)    # Saturation

e_prev = e                                 # Store for next iteration
```

**Returns**: Dictionary of torques for each joint.

**Example**:
```python
q = {"shoulder_pan": 0.1, "shoulder_lift": 0.2}  # Current state
qd = {"shoulder_pan": 0.0, "shoulder_lift": 0.0}
q_des = {"shoulder_pan": 0.0, "shoulder_lift": 0.0}  # Target

tau = pid.compute(q, qd, q_des, dt=0.002)
# tau = {"shoulder_pan": -5.0, "shoulder_lift": -10.0}
```

#### Internal State

```python
self.i_state = {jn: 0.0 for jn in joint_names}  # Integral accumulator
self.prev_e = {jn: 0.0 for jn in joint_names}   # Previous error
```

**Note**: Integrator windup is prevented by clamping `i_state`.

---

## Perturbation Model

### PerturbationConfig (Dataclass)

```python
@dataclass
class PerturbationConfig:
    # Sinusoidal disturbance
    sinus_amp: float = 3.75              # Amplitude (N·m)
    sinus_freq_hz: float = 0.6           # Frequency (Hz)

    # Colored noise
    noise_std: float = 3.25              # Std deviation (N·m)
    noise_tau: float = 3.3               # Correlation time (s)

    # Impulse perturbations
    impulse_prob_per_s: float = 0.15     # Trigger rate (Hz)
    impulse_mag: float = 2.5             # Magnitude (N·m)
    impulse_dur: float = 0.06            # Duration (s)

    # Limits
    disturb_tau_limit: float = 6.0       # Total disturbance limit (N·m)

    # Measurement noise
    meas_q_std: float = 0.0              # Position noise (rad)
    meas_qd_std: float = 0.0             # Velocity noise (rad/s)

    # Randomization
    seed: int = 7
```

**Purpose**: Configuration for realistic disturbances.

### PerturbationModel (Class)

Multi-component disturbance generator.

#### Constructor
```python
PerturbationModel(joint_names: list[str], cfg: PerturbationConfig)
```

**Example**:
```python
cfg = PerturbationConfig(
    sinus_amp=1.0,
    noise_std=0.5,
    impulse_prob_per_s=0.2
)
perturb = PerturbationModel(["shoulder_pan", "shoulder_lift"], cfg)
```

#### Methods

##### `reset()`
```python
perturb.reset()
```
Clears noise state and active impulses.

##### `apply_joint_torques()`
```python
tau_disturb = perturb.apply_joint_torques(
    t: float,      # Current time (s)
    dt: float      # Time step (s)
) -> dict[str, float]  # Returns: disturbance torques (N·m)
```

**Components**:

1. **Sinusoidal**:
   ```python
   tau_sin[j] = amp * sin(2π * freq * t + 0.7*j)
   ```
   Each joint has phase offset of `0.7*index`.

2. **Colored Noise**:
   ```python
   alpha = dt / (tau + dt)
   noise[j] = (1-alpha)*noise[j] + alpha*white_noise
   ```
   Low-pass filtered Gaussian noise.

3. **Impulses**:
   - Random trigger: probability `impulse_prob_per_s * dt` per step
   - Affects 1-3 joints randomly
   - Duration: `impulse_dur` seconds
   - Magnitude: `±impulse_mag` (random sign)

4. **Total**:
   ```python
   tau_total = tau_sin + tau_noise + tau_impulse
   tau_total = clip(tau_total, -disturb_tau_limit, disturb_tau_limit)
   ```

**Example**:
```python
t = 1.5  # Current time
dt = 0.002  # Timestep
tau_disturb = perturb.apply_joint_torques(t, dt)
# tau_disturb = {"shoulder_pan": 1.23, "shoulder_lift": -0.87}
```

##### `noisy_measurement()`
```python
q_noisy, qd_noisy = perturb.noisy_measurement(
    q: dict[str, float],      # True positions (rad)
    qd: dict[str, float]      # True velocities (rad/s)
) -> tuple[dict, dict]        # Returns: noisy measurements
```

Adds Gaussian noise to state measurements:
```python
q_noisy[j] = q[j] + N(0, meas_q_std)
qd_noisy[j] = qd[j] + N(0, meas_qd_std)
```

**Example**:
```python
q_true = {"shoulder_pan": 0.0}
qd_true = {"shoulder_pan": 0.0}
q_meas, qd_meas = perturb.noisy_measurement(q_true, qd_true)
# q_meas = {"shoulder_pan": 0.0023}  (with noise)
```

---

## MuJoCo Helper Functions

### `get_q_qd_dict()`

```python
q, qd = get_q_qd_dict(
    m,                        # MuJoCo model
    d,                        # MuJoCo data
    joint_names: list[str]    # Joint names to query
) -> tuple[dict[str, float], dict[str, float]]
```

**Purpose**: Extracts joint positions and velocities from MuJoCo state.

**Returns**:
- `q`: Joint positions in radians
- `qd`: Joint velocities in rad/s

**Example**:
```python
import mujoco
m = mujoco.MjModel.from_xml_path("model/scene_urdf.xml")
d = mujoco.MjData(m)

joints = ["shoulder_pan", "shoulder_lift"]
q, qd = get_q_qd_dict(m, d, joints)
# q = {"shoulder_pan": 0.0, "shoulder_lift": 0.1}
# qd = {"shoulder_pan": 0.0, "shoulder_lift": 0.05}
```

**Implementation**:
```python
for jn in joint_names:
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
    qadr = int(m.jnt_qposadr[jid])  # Position index
    dadr = int(m.jnt_dofadr[jid])   # Velocity index
    q[jn] = float(d.qpos[qadr])
    qd[jn] = float(d.qvel[dadr])
```

### `apply_joint_torques_qfrc()`

```python
apply_joint_torques_qfrc(
    m,                        # MuJoCo model
    d,                        # MuJoCo data
    joint_names: list[str],   # Joint names
    tau: dict[str, float]     # Torques (N·m)
)
```

**Purpose**: Applies torques to joints via `d.qfrc_applied`.

**Example**:
```python
tau = {"shoulder_pan": 5.0, "shoulder_lift": -3.0}
apply_joint_torques_qfrc(m, d, ["shoulder_pan", "shoulder_lift"], tau)
mujoco.mj_step(m, d)  # Torques applied during this step
```

**Implementation**:
```python
d.qfrc_applied[:] = 0.0  # Clear previous
for jn in joint_names:
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
    dof = int(m.jnt_dofadr[jid])
    d.qfrc_applied[dof] += float(tau[jn])
```

**Note**: Torques persist for only one `mj_step()`. Must be reapplied each frame.

---

## Usage Examples

### Example 1: Basic PID Control

```python
from so101_control import JointPID, PIDGains, get_q_qd_dict, apply_joint_torques_qfrc
import mujoco

# Setup
m = mujoco.MjModel.from_xml_path("model/scene_urdf.xml")
d = mujoco.MjData(m)
joints = ["shoulder_pan", "shoulder_lift"]

# Create PID controller
gains = {
    "shoulder_pan": PIDGains(kp=50.0, ki=0.5, kd=2.0),
    "shoulder_lift": PIDGains(kp=60.0, ki=0.6, kd=2.5),
}
pid = JointPID(joints, gains)

# Target position
q_des = {"shoulder_pan": 0.5, "shoulder_lift": -0.3}

# Control loop
for _ in range(1000):
    q, qd = get_q_qd_dict(m, d, joints)
    tau = pid.compute(q, qd, q_des, m.opt.timestep)
    apply_joint_torques_qfrc(m, d, joints, tau)
    mujoco.mj_step(m, d)
```

### Example 2: PID with Perturbations

```python
from so101_control import PerturbationConfig, PerturbationModel

# Create perturbation model
cfg = PerturbationConfig(
    sinus_amp=1.0,
    noise_std=0.5,
    impulse_prob_per_s=0.1
)
perturb = PerturbationModel(joints, cfg)

# Control loop
for i in range(1000):
    t = i * m.opt.timestep

    # Get state (with measurement noise)
    q, qd = get_q_qd_dict(m, d, joints)
    q_meas, qd_meas = perturb.noisy_measurement(q, qd)

    # Compute control torque
    tau_control = pid.compute(q_meas, qd_meas, q_des, m.opt.timestep)

    # Add disturbances
    tau_disturb = perturb.apply_joint_torques(t, m.opt.timestep)

    # Total torque
    tau_total = {jn: tau_control[jn] + tau_disturb[jn] for jn in joints}

    # Apply and step
    apply_joint_torques_qfrc(m, d, joints, tau_total)
    mujoco.mj_step(m, d)
```

### Example 3: Tuning Gains

```python
# Start conservative
gains = {jn: PIDGains(kp=10.0, ki=0.0, kd=1.0) for jn in joints}
pid = JointPID(joints, gains)

# Increase Kp until oscillation
pid.gains["shoulder_pan"].kp = 50.0

# Add damping
pid.gains["shoulder_pan"].kd = 3.0

# Add integral if steady-state error exists
pid.gains["shoulder_pan"].ki = 0.5
```

---

## Design Patterns

### Dataclass Configuration
```python
@dataclass
class Config:
    param1: float = default_value
    param2: float = default_value
```
- **Advantage**: Clear defaults, easy to modify
- **Usage**: Create variants without changing base code

### Dictionary-Based State
```python
q = {"joint1": value1, "joint2": value2}
```
- **Advantage**: Named access, flexible joint sets
- **Drawback**: No type checking (could use TypedDict)

### Stateful Controllers
```python
class Controller:
    def __init__(self, ...):
        self.state = ...
    def reset(self):
        self.state = ...
    def compute(self, ...):
        # Uses and updates self.state
```
- **Advantage**: Maintains history (integral, prev_error)
- **Usage**: Call `reset()` between independent motions

---

## Mathematical Background

### PID Control Law

```
e(t) = x_desired(t) - x(t)              [Position error]
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·ė(t)   [Control signal]
```

**Roles**:
- **Kp (Proportional)**: Immediate response to error
- **Ki (Integral)**: Eliminates steady-state error
- **Kd (Derivative)**: Damping, reduces overshoot

### Colored Noise

First-order low-pass filter applied to white noise:

```
α = Δt / (τ + Δt)
x[n] = (1-α)·x[n-1] + α·w[n]
```

where `w[n] ~ N(0, σ²)` and `τ` is correlation time.

---

## Best Practices

### Gain Tuning

1. **Start**: All gains at zero or very small
2. **Kp First**: Increase until oscillation, then back off 50%
3. **Kd Next**: Add damping to reduce oscillation
4. **Ki Last**: Only if steady-state error persists (use sparingly)
5. **Limits**: Set `tau_limit` based on actuator specs

### Perturbation Configuration

1. **Start Mild**: Use small amplitudes initially
2. **Test Separately**: Enable one disturbance type at a time
3. **Realistic**: Match noise levels to sensor specs
4. **Seed**: Use fixed seed for reproducibility

### Anti-Windup

Always limit integral:
```python
integral = clip(integral, -i_limit, i_limit)
```
Prevents integrator from accumulating during saturation.

---

## Common Issues

### Oscillation
- **Cause**: Kp too high, Kd too low
- **Solution**: Decrease Kp, increase Kd

### Steady-State Error
- **Cause**: No integral term
- **Solution**: Add small Ki (0.1 - 1.0)

### Instability
- **Cause**: Ki too high, timestep too large
- **Solution**: Reduce Ki, check timestep (should be ~0.002s)

### Integrator Windup
- **Cause**: i_limit too high or not set
- **Solution**: Set i_limit to 2-5x typical error

---

## Related Files

- **so101_mujoco_pid_utils.py**: Higher-level motion functions using this module
- **run_mujoco_simulation2.py**: Example usage in simulation
- **so101_mujoco_utils2.py**: Complementary utilities (plotting, conversions)

---

## API Summary

### Classes
```python
JointPID(joint_names, gains)
    .reset()
    .compute(q, qd, q_des, dt) -> tau

PerturbationModel(joint_names, cfg)
    .reset()
    .apply_joint_torques(t, dt) -> tau
    .noisy_measurement(q, qd) -> (q_noisy, qd_noisy)
```

### Functions
```python
get_q_qd_dict(m, d, joint_names) -> (q, qd)
apply_joint_torques_qfrc(m, d, joint_names, tau)
```

### Dataclasses
```python
PIDGains(kp, ki, kd, i_limit, tau_limit)
PerturbationConfig(sinus_amp, noise_std, impulse_mag, ...)
```

---

## Further Reading

- **PID Control**: Åström & Murray, "Feedback Systems"
- **MuJoCo Docs**: https://mujoco.readthedocs.io
- **Anti-windup**: "Antiwindup for Exponential Setpoint Tracking" (IEEE Transactions)
