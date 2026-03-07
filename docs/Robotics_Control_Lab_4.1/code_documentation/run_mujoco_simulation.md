# run_mujoco_simulation.py

## Overview

Demonstrates position-based control of the SO101 robot arm using MuJoCo's built-in position actuators. The robot moves between predefined poses with smooth linear interpolation.

**File Path**: `simulation_code/run_mujoco_simulation.py`

## Purpose

This script showcases:
- Setting initial robot configuration
- Position-controlled motion using actuators
- Smooth trajectory generation via linear interpolation
- Holding positions with active control

## Dependencies

```python
import mujoco
import mujoco.viewer
from so101_mujoco_utils2 import set_initial_pose, move_to_pose, hold_position
```

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

**Note**:
- All joint angles are in **degrees**
- Gripper is in **0-100 range** (0 = closed, 100 = open)

### 3. Initial Configuration
```python
set_initial_pose(m, d, starting_position)
```
- Sets robot to starting configuration before viewer opens
- Prevents sudden jumps when simulation starts

### 4. Motion Sequence
```python
with mujoco.viewer.launch_passive(m, d) as viewer:
    # Move to zero configuration
    move_to_pose(m, d, viewer, desired_zero, duration=2.0, realtime=True)

    # Hold zero position
    hold_position(m, d, viewer, duration=2.0, realtime=True)

    # Return to starting position
    move_to_pose(m, d, viewer, starting_position, duration=2.0, realtime=True)

    # Hold starting position
    hold_position(m, d, viewer, duration=2.0, realtime=True)
```

## Motion Functions

### `move_to_pose()`
Smoothly interpolates robot from current pose to target pose.

**Parameters**:
- `m`: MuJoCo model
- `d`: MuJoCo data
- `viewer`: Viewer instance
- `desired_position`: Target joint positions (dict)
- `duration`: Motion time in seconds (float)
- `realtime`: Whether to pace execution in real-time (bool)

**Behavior**:
- Uses linear interpolation (LERP) between current and target
- Commands position actuators via `d.ctrl`
- Maintains smooth trajectory over specified duration

### `hold_position()`
Maintains current robot configuration against external forces.

**Parameters**: Same as `move_to_pose()` except no target position needed

**Behavior**:
- Reads current position at start
- Commands actuators to maintain that position
- Actively resists gravity and perturbations

## Control Method

### Position Actuators

The robot uses MuJoCo's `<position>` actuators defined in the URDF:

```xml
<position class="sts3215" name="shoulder_pan" joint="shoulder_pan"
          forcerange="-3.35 3.35" ctrlrange="-1.91986 1.91986"/>
```

**Characteristics**:
- Built-in PD controller: `tau = kp*(target - actual) + kv*velocity`
- Default gains: `kp=998.22`, `kv=2.731`
- Force limits: ±3.35 N·m per joint
- Position limits enforced automatically

### Control Flow

```
1. Read current joint positions
2. Compute target position at current time
3. Set d.ctrl[actuator_id] = target (in radians)
4. MuJoCo's actuator computes torque
5. Physics step applies torques
6. Repeat
```

## Usage

### Running the Script
```bash
cd simulation_code
python run_mujoco_simulation.py
```

### Expected Behavior

| Time | Action | Description |
|------|--------|-------------|
| 0-2s | Move to zero | Robot moves from starting pose to all joints at 0° |
| 2-4s | Hold zero | Robot maintains zero configuration |
| 4-6s | Return | Robot moves back to starting configuration |
| 6-8s | Hold start | Robot holds starting pose |

**Total Duration**: ~8 seconds

## Key Concepts

### 1. Pose Dictionary Format
```python
pose = {
    "shoulder_pan": -45.0,      # degrees
    "shoulder_lift": 30.0,      # degrees
    "elbow_flex": 90.0,         # degrees
    "wrist_flex": 45.0,         # degrees
    "wrist_roll": 0.0,          # degrees
    "gripper": 50.0,            # 0-100 range
}
```

### 2. Linear Interpolation
For each joint at time `t`:
```python
alpha = t / duration  # 0.0 to 1.0
current = (1 - alpha) * start + alpha * target
```

### 3. Coordinate Conversion
Internally, the utilities handle:
- Degrees → Radians for joint angles
- 0-100 → Radians for gripper (assuming 0-π range)

## Modifications

### Add Custom Poses
```python
custom_pose = {
    "shoulder_pan":  45.0,
    "shoulder_lift": -60.0,
    "elbow_flex":    120.0,
    "wrist_flex":    30.0,
    "wrist_roll":    90.0,
    "gripper":       75.0,
}

move_to_pose(m, d, viewer, custom_pose, duration=3.0, realtime=True)
```

### Change Motion Duration
```python
move_to_pose(m, d, viewer, desired_zero, duration=5.0, realtime=True)  # Slower
```

### Run Without Real-time Pacing
```python
move_to_pose(m, d, viewer, desired_zero, duration=2.0, realtime=False)  # As fast as possible
```

### Add More Waypoints
```python
waypoint1 = {"shoulder_pan": 45.0, ...}
waypoint2 = {"shoulder_pan": -45.0, ...}

move_to_pose(m, d, viewer, waypoint1, duration=2.0, realtime=True)
hold_position(m, d, viewer, duration=1.0, realtime=True)
move_to_pose(m, d, viewer, waypoint2, duration=2.0, realtime=True)
```

## Advantages of Position Control

1. **Simple**: Just specify target positions
2. **Stable**: Built-in PD controller handles stabilization
3. **Smooth**: Linear interpolation ensures continuous motion
4. **Force-limited**: Actuators respect torque constraints

## Limitations

1. **Fixed Gains**: Cannot tune PD gains per motion
2. **No Perturbation Handling**: Cannot simulate external disturbances
3. **Limited Control**: Cannot apply arbitrary torques
4. **Trajectory Type**: Only linear interpolation (not polynomial, spline, etc.)

## Comparison with PID Control

| Feature | Position Control | PID Control (run_mujoco_simulation2.py) |
|---------|------------------|------------------------------------------|
| Control Type | Built-in actuators | Custom torque commands |
| Tuning | Fixed gains | Per-joint tunable gains |
| Disturbances | Not modeled | Can inject perturbations |
| Complexity | Simple | More complex |
| Realism | Servo-like | Research-grade |

## Troubleshooting

### Robot doesn't move smoothly
- Increase `duration` parameter
- Check that `realtime=True` for realistic motion
- Verify pose values are within joint limits

### Robot oscillates
- Position actuator gains may be too high
- Reduce kp/kv in URDF model if needed

### Viewer freezes
- Ensure `viewer.sync()` is called in motion loop
- Check for infinite loops

## Related Files

- **so101_mujoco_utils2.py**: Implementation of `move_to_pose()` and `hold_position()`
- **run_mujoco_simulation2.py**: PID control variant with perturbations
- **model/scene_urdf.xml**: Model with position actuators

## Learning Objectives

1. Understanding position-based control
2. Working with pose dictionaries
3. Linear trajectory generation
4. MuJoCo actuator system
5. Real-time simulation pacing

## Next Steps

- Try **run_mujoco_simulation2.py** for PID control with disturbances
- Modify poses to create custom motion sequences
- Experiment with different durations
- Read **so101_mujoco_utils2.py** to understand implementation
