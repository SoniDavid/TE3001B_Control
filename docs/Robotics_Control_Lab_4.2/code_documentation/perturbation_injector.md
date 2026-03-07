# perturbation_injector.py

**Package**: `xarm_perturbations`
**Node name**: `xarm_perturbation_injector`
**Entry point**: `perturbation_injector = xarm_perturbations.perturbation_injector:main`

---

## Purpose

A **standalone ROS 2 node** that publishes `TwistStamped` velocity commands to `/servo_server/delta_twist_cmds`, acting as an **external disturbance source** layered on top of whatever the main controller is already sending.

Since MoveIt Servo processes incoming `TwistStamped` messages from its topic, any publisher on that topic contributes to the commanded velocity. Running the perturbation injector simultaneously with `trajectory_and_controller` **adds** perturbation velocities on top of the tracking commands.

---

## Important Note on Concurrent Publishing

MoveIt Servo uses the **last received** `TwistStamped` at its control rate. When both nodes publish to the same topic, they compete for the latest message slot. In practice this is used as an injection mechanism to test controller robustness — the controller and the injector alternate publishing, creating an effective disturbance.

For a clean additive overlay, the two nodes should be running at different rates or a multiplexer/mixer should be used. As implemented, the injector is primarily useful as a **direct disturbance test** (e.g. run injector alone to move robot, or interleave for perturbation testing).

---

## Modes

| Mode | Description |
|---|---|
| `off` | Publishes zero velocity (can be used to "silence" the output) |
| `sine` | Sinusoidal signal on a single axis: `v_axis = amp * sin(2*pi*freq*t)` |
| `gaussian` | Independent Gaussian noise on each axis: `v ~ N(0, std)` |

---

## Class: `PerturbationGenerator(Node)`

### Constructor

Sets up all parameters, creates the publisher with configurable QoS, and starts the 50 Hz timer.

**QoS**: The publisher reliability can be set to `"reliable"` (default) or `"best_effort"` via the `pub_reliability` parameter. Use `"reliable"` to satisfy MoveIt Servo's RELIABLE subscriber QoS requirement.

---

### `_dp() -> np.ndarray`

Returns the 3D perturbation velocity vector `[dx, dy, dz]` for the current timestep.

```python
# off mode
return [0, 0, 0]

# gaussian mode
return rng.normal(0.0, noise_std_linear, size=3)

# sine mode
s = sin(2*pi*sine_freq_hz * (time.time() - t0))
dp[axis] = sine_amp_linear * s
return dp
```

The sine uses **wall-clock time** (`time.time()`) rather than ROS time for simplicity.

The Gaussian RNG is seeded with `7` for reproducibility (`np.random.default_rng(7)`).

---

### `tick()`

Called at `publish_period_s` (default 0.02 s = 50 Hz).

```python
if not enabled:
    publish zero
    return

v = clip(base_linear + _dp(), -max_lin, max_lin)
publish(v)
```

The `base_linear` parameter allows adding a constant DC bias (default `[0, 0, 0]`).

---

### `_publish(v_xyz, note)`

Wraps the velocity into a `TwistStamped` and publishes. Angular components are always zero. Prints a debug log at `debug_period_s` intervals if `debug=true`.

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `output_topic` | `/servo_server/delta_twist_cmds` | Topic to publish to |
| `enabled` | `true` | Master on/off switch |
| `mode` | `"sine"` | Perturbation mode: `off`, `sine`, `gaussian` |
| `publish_period_s` | 0.02 s | Timer period (50 Hz) |
| `max_linear_speed` | 0.20 m/s | Hard clip on output velocity |
| `sine_freq_hz` | 8.0 Hz | Sine frequency |
| `sine_amp_linear` | 0.02 m/s | Sine amplitude |
| `sine_axis` | `"x"` | Axis to inject sine on: `x`, `y`, or `z` |
| `noise_std_linear` | 0.01 m/s | Standard deviation of Gaussian noise |
| `base_linear` | [0, 0, 0] | Constant DC offset added to perturbation |
| `pub_reliability` | `"reliable"` | QoS reliability: `reliable` or `best_effort` |
| `debug` | `true` | Enable debug logging |
| `debug_period_s` | 1.0 s | Interval for debug log output |

---

## Run Examples

```bash
# Sinusoidal perturbation on Y axis at 0.5 Hz
ros2 run xarm_perturbations perturbation_injector --ros-args \
  -p mode:=sine \
  -p sine_axis:=y \
  -p sine_amp_linear:=0.015 \
  -p sine_freq_hz:=0.5

# Gaussian noise on all axes
ros2 run xarm_perturbations perturbation_injector --ros-args \
  -p mode:=gaussian \
  -p noise_std_linear:=0.01

# Disabled (publishes zeros, useful for testing topic connectivity)
ros2 run xarm_perturbations perturbation_injector --ros-args \
  -p mode:=off
```

---

## Related Files

- [trajectory_and_controller.md](trajectory_and_controller.md) — the main controller running on the same topic
- [analysis_evaluation.md](analysis_evaluation.md) — offline analysis of data collected under perturbations
