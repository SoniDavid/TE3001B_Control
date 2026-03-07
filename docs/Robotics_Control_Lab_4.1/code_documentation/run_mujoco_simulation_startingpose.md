# run_mujoco_simulation_startingpose.py

## Overview

Simple demonstration script showing basic position control from a custom starting pose. This is the most basic example using position actuators without any advanced features.

**File Path**: `simulation_code/run_mujoco_simulation_startingpose.py`

## Purpose

This script demonstrates:
- Setting an initial robot configuration
- Using position actuators for basic control
- Running a timed simulation
- Minimal MuJoCo simulation setup

This is an ideal **first script** for beginners to understand MuJoCo basics.

## Dependencies

```python
import time
import mujoco
import mujoco.viewer
from so101_mujoco_utils import set_initial_pose, send_position_command
```

**Note**: Uses `so101_mujoco_utils` (legacy version), not `so101_mujoco_utils2`.

## Complete Script Breakdown

### 1. Model Loading
```python
m = mujoco.MjModel.from_xml_path('model/scene_urdf.xml')
d = mujoco.MjData(m)
```
- Loads robot model and environment
- Creates data structure for simulation state

### 2. Starting Position Definition
```python
starting_position = {
    'shoulder_pan': 0.0,    # degrees
    'shoulder_lift': 0.0,
    'elbow_flex': 0.0,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 0.0          # 0-100 range
}
```
- All joints at zero (neutral) position
- Simple, safe starting configuration

### 3. Set Initial Pose
```python
set_initial_pose(d, starting_position)
```
- Directly sets `d.qpos` to starting position
- Prevents robot from falling on startup

### 4. Simulation Loop
```python
with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()

        send_position_command(d, starting_position)
        mujoco.mj_step(m, d)
        viewer.sync()

        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
```

**Loop Components**:
- **Duration**: 30 seconds
- **Command**: Continuously send starting position to actuators
- **Physics**: Step simulation forward
- **Display**: Update viewer
- **Timing**: Maintain real-time pacing

## Key Functions

### `set_initial_pose(d, position_dict)`
```python
def set_initial_pose(d, position_dict):
    pos = convert_to_list(position_dict)
    d.qpos = pos
```
- Converts dictionary (degrees) to list (radians)
- Directly sets joint positions in simulation state
- **Note**: Older version takes `d` only, newer takes `(m, d)`

### `send_position_command(d, position_dict)`
```python
def send_position_command(d, position_dict):
    pos = convert_to_list(position_dict)
    d.ctrl = pos
```
- Converts dictionary to radians
- Sets actuator control signals
- Position actuators compute torques automatically

## Usage

### Running the Script
```bash
cd simulation_code
python run_mujoco_simulation_startingpose.py
```

### Expected Behavior
1. **Viewer opens** showing SO101 robot
2. **Robot is at zero position** (all joints straight)
3. **Robot holds position** actively for 30 seconds
4. **Viewer closes** automatically after timeout

### What You'll See
- Robot maintaining neutral pose
- Slight oscillations due to PD control
- Gravity being compensated by actuators

## Modifications

### Change Starting Pose
```python
starting_position = {
    'shoulder_pan': 45.0,     # Rotate base
    'shoulder_lift': -30.0,   # Tilt shoulder down
    'elbow_flex': 90.0,       # Bend elbow
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 50.0,          # Half-open gripper
}
```

### Change Duration
```python
while viewer.is_running() and time.time() - start < 60:  # 60 seconds
```

### Remove Time Limit
```python
while viewer.is_running():  # Run until manually closed
```

### Add Motion
```python
# Define multiple poses
pose1 = {'shoulder_pan': 0.0, ...}
pose2 = {'shoulder_pan': 90.0, ...}

# Switch between poses
while viewer.is_running():
    if time.time() % 4 < 2:
        send_position_command(d, pose1)
    else:
        send_position_command(d, pose2)
    # ... rest of loop
```

## Code Flow

```
┌─────────────────────────┐
│ Load model & data       │
└──────────┬──────────────┘
           │
┌──────────▼──────────────┐
│ Define starting pose    │
└──────────┬──────────────┘
           │
┌──────────▼──────────────┐
│ Set initial qpos        │
└──────────┬──────────────┘
           │
┌──────────▼──────────────┐
│ Launch viewer           │
└──────────┬──────────────┘
           │
┌──────────▼──────────────┐
│ For 30 seconds:         │
│  1. Send position cmd   │
│  2. Step physics        │
│  3. Update viewer       │
│  4. Sleep (realtime)    │
└──────────┬──────────────┘
           │
┌──────────▼──────────────┐
│ Close and exit          │
└─────────────────────────┘
```

## Legacy vs. Modern Utilities

### This Script (Legacy)
```python
from so101_mujoco_utils import set_initial_pose, send_position_command

set_initial_pose(d, pose)  # Takes only 'd'
send_position_command(d, pose)  # Takes only 'd'
```

### Modern Alternative
```python
from so101_mujoco_utils2 import set_initial_pose, send_position_command

set_initial_pose(m, d, pose)  # Takes 'm, d'
send_position_command(m, d, pose)  # Takes 'm, d'
```

**Difference**: Modern version is more robust (uses joint name lookup).

## Why This Script Exists

### Educational Purpose
1. **Minimal code**: Easy to understand for beginners
2. **No advanced features**: Focus on basics
3. **Self-contained**: All you need in one file
4. **Safe pose**: Zero configuration won't damage anything

### Stepping Stone
After understanding this script, progress to:
1. **run_mujoco_simulation.py**: Motion between poses
2. **run_mujoco_simulation2.py**: PID control with disturbances
3. **mujoco_simulation.py**: Real-time plotting

## Common Issues

### Robot falls through floor
**Cause**: `set_initial_pose()` not called before viewer launch
**Solution**: Always set initial pose before opening viewer

### Viewer opens but robot is gone
**Cause**: Invalid joint positions (NaN or out of bounds)
**Solution**: Verify pose dictionary values are reasonable

### Script exits immediately
**Cause**: XML file not found
**Solution**: Ensure you're running from `simulation_code/` directory

### Robot shakes/oscillates
**Cause**: Position actuator PD gains too high
**Solution**: Adjust gains in URDF or accept slight oscillation

## Learning Objectives

1. Basic MuJoCo model loading
2. Setting initial configurations
3. Position actuator control
4. Simulation loop structure
5. Real-time timing
6. Viewer interaction

## Comparison with Other Scripts

| Feature | This Script | run_mujoco_simulation.py | run_mujoco_simulation2.py |
|---------|-------------|--------------------------|----------------------------|
| Complexity | Simplest | Medium | Advanced |
| Motion | Static hold | Move between poses | PID with perturbations |
| Plotting | None | None | Real-time web plots |
| Control | Position | Position | Torque (PID) |
| Best For | Learning basics | Simple demos | Research/tuning |

## Extending This Script

### Example 1: Cyclic Motion
```python
t = 0
while viewer.is_running() and time.time() - start < 30:
    # Sinusoidal motion
    pose = {
        'shoulder_pan': 45 * np.sin(2*np.pi*0.5*t),  # ±45° @ 0.5 Hz
        'shoulder_lift': 0.0,
        'elbow_flex': 0.0,
        'wrist_flex': 0.0,
        'wrist_roll': 0.0,
        'gripper': 0.0,
    }
    send_position_command(d, pose)
    mujoco.mj_step(m, d)
    viewer.sync()
    t += m.opt.timestep
```

### Example 2: Keyboard Control
```python
current_pan = 0.0

while viewer.is_running():
    # Check for keyboard input (pseudocode)
    if left_arrow_pressed:
        current_pan -= 5.0
    if right_arrow_pressed:
        current_pan += 5.0

    pose = {'shoulder_pan': current_pan, ...}
    send_position_command(d, pose)
    # ... rest of loop
```

## Related Files

- **so101_mujoco_utils.py**: Utility functions used by this script
- **model/scene_urdf.xml**: Robot and environment model
- **run_mujoco_simulation.py**: Next step (motion between poses)

## Quick Reference

### Position Dictionary Format
```python
{
    'shoulder_pan': float,   # -110 to 110 degrees
    'shoulder_lift': float,  # -100 to 100 degrees
    'elbow_flex': float,     # -97 to 97 degrees
    'wrist_flex': float,     # -95 to 95 degrees
    'wrist_roll': float,     # -157 to 163 degrees
    'gripper': float,        # 0 (closed) to 100 (open)
}
```

### Viewer Shortcuts
- **Left Mouse Drag**: Rotate view
- **Right Mouse Drag**: Zoom
- **Middle Mouse Drag**: Pan
- **Space**: Pause/resume simulation
- **Ctrl+Right Drag**: Apply force to bodies
- **Esc**: Close viewer

## Tips for Beginners

1. **Start Here**: This is the perfect first script
2. **Experiment**: Change the starting pose values
3. **Observe**: Watch how actuators fight gravity
4. **Understand**: Read through line by line
5. **Modify**: Add your own features incrementally

## Next Steps

1. Run this script successfully
2. Modify the starting pose
3. Try adding simple motion
4. Move to `run_mujoco_simulation.py`
5. Explore position control concepts
6. Eventually try PID control in `run_mujoco_simulation2.py`
