# circle_maker.py

**Package**: `xarm_perturbations`
**Node name**: `circle_servo_xarm_lite6`
**Entry point**: `circle_maker = xarm_perturbations.circle_maker:main`

---

## Purpose

A **prototype/exploratory node** that makes the xArm Lite 6 end-effector follow a **circular trajectory** in Cartesian space. This was the initial implementation before the more complete `trajectory_and_controller.py` was developed.

> For experiments, prefer `trajectory_and_controller.py` which provides the figure-eight trajectory, CSV logging, per-cycle metrics, configurable parameters, and LPF support. `circle_maker.py` uses hardcoded gains and no CSV output.

---

## Differences from `trajectory_and_controller.py`

| Feature | `circle_maker` | `trajectory_and_controller` |
|---|---|---|
| Trajectory shape | Circle (parametric) | Figure-eight (Lissajous 1:2) |
| Parameters | Hardcoded in `__init__` | ROS 2 parameters (runtime configurable) |
| CSV logging | No | Yes |
| Cycle metrics | No | Yes (`/cycle_metrics`) |
| LPF on velocity | No | Yes (optional) |
| Velocity saturation | Per-axis clip | Norm-based scaling |
| RViz markers | Circle LINE_STRIP + target SPHERE | Same |
| Keyboard | Always enabled (pynput required) | Optional |

---

## Class: `CircleServoXArmLite6(Node)`

### Constructor

Hardcoded defaults:
```python
self.radius    = 0.20       # 20 cm circle radius
self.frequency = 0.10       # 0.10 Hz (~10 s per revolution)
self.plane     = "xy"
self.hold_z    = True
self.kp        = [2.5, 2.5, 2.5]
self.kd        = [0.6, 0.6, 0.6]
self.epsilon   = [0.002, 0.002, 0.002]  # 2 mm deadband
self.max_speed = 0.12       # m/s per axis
self.home_position = [0.227, 0.00, 0.468]
```

The center is initialized from the robot's **actual EE position at startup** (no offset added), so the circle is centered exactly where the robot is when the node first gets a TF reading.

---

### `_circle_target(t_sec) -> np.ndarray`

Parametric circle:
```python
w = 2*pi*frequency
ramp = clamp(t/2.0, 0, 1)   # 2-second soft start
a = ramp * radius * cos(w*t)
b = ramp * radius * sin(w*t)
```

For plane `"xy"`: `x = cx + a`, `y = cy + b`, `z = cz`

Note: unlike the figure-eight, this traces a standard circle (1:1 Lissajous). The yaw rotation feature from `trajectory_and_controller` is not present here.

---

### `_servo_to(target_pos)`

PD control step, simpler than the main controller:

```python
v = kp * error + kd * d_error
v = where(|error| > epsilon, v, 0.0)   # deadband
v = clip(v, -max_speed, max_speed)     # per-axis clip (not norm-based)
```

Key difference: uses **per-axis clipping** (`np.clip`) instead of norm-based saturation. This can distort the velocity direction when multiple axes saturate simultaneously.

Logs position/target/velocity at 1 Hz to the console.

---

### `_publish_circle_marker()`

Publishes a `LINE_STRIP` marker (`/circle_marker`) showing the full circle at the current ramp scale. 100 evenly spaced points.

### `_publish_target_marker(target)`

Publishes a red sphere at the current target position (also to `/circle_marker`, id=1).

---

### State Machine

Same three states as `trajectory_and_controller`:

| State | Behavior |
|---|---|
| `RUNNING` | Track circular target |
| `PAUSED` | Publish zero, wait for `p` key |
| `HOME` | Move toward `home_position` at 0.08 m/s, re-center circle on arrival |

Keyboard: `p` = pause/resume, `h` = home. **pynput is required** (no optional flag here — the import is at the top of the file and will fail if not installed).

---

## Run Example

```bash
ros2 run xarm_perturbations circle_maker
```

No runtime parameters. To change radius/frequency/gains, edit the constructor directly.

---

## Related Files

- [trajectory_and_controller.md](trajectory_and_controller.md) — the production version with full features
