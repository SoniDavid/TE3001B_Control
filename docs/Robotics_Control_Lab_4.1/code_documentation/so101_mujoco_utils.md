# so101_mujoco_utils.py

## Overview

**Legacy** utility module providing basic robot control functions for the SO101 arm. This is the original version used by `run_mujoco_simulation_startingpose.py`.

**File Path**: `simulation_code/so101_mujoco_utils.py`

**Status**: ⚠️ **Legacy Code** - Consider using `so101_mujoco_utils2.py` for new projects.

## Purpose

Provides minimal utilities for:
- Converting between degrees/radians and dictionary/list formats
- Setting initial robot poses
- Sending position commands to actuators

## Why This File Exists

### Historical Context
- **Original version**: Created for simple demonstrations
- **Simplified API**: Takes only `d` (data), not `m` (model)
- **Assumption-based**: Assumes fixed joint ordering in `d.qpos`

### When to Use
- **Legacy scripts**: `run_mujoco_simulation_startingpose.py` uses this
- **Simple demos**: If you don't need advanced features
- **Compatibility**: When maintaining old code

### Prefer `so101_mujoco_utils2.py` For
- New projects
- Robust joint name lookup
- Real-time plotting
- Motion primitives

---

## Functions

### `convert_to_dictionary()`

Converts joint positions array to dictionary format.

```python
convert_to_dictionary(qpos) -> dict[str, float]
```

**Parameters**:
- `qpos`: Array-like of 6 joint positions (radians)

**Returns**: Dictionary with joint angles in degrees, gripper in 0-100

**Implementation**:
```python
{
    'shoulder_pan': qpos[0] * 180.0 / 3.14159,
    'shoulder_lift': qpos[1] * 180.0 / 3.14159,
    'elbow_flex': qpos[2] * 180.0 / 3.14159,
    'wrist_flex': qpos[3] * 180.0 / 3.14159,
    'wrist_roll': qpos[4] * 180.0 / 3.14159,
    'gripper': qpos[5] * 100.0 / 3.14159  # rad → 0-100
}
```

**Example**:
```python
import numpy as np
qpos = np.array([0.0, 0.5, -0.5, 0.2, 0.0, 1.57])
pose_dict = convert_to_dictionary(qpos)
# pose_dict = {
#     'shoulder_pan': 0.0,
#     'shoulder_lift': 28.65,
#     'elbow_flex': -28.65,
#     'wrist_flex': 11.46,
#     'wrist_roll': 0.0,
#     'gripper': 50.0
# }
```

**⚠️ Warning**: Assumes `qpos` order matches `JOINT_NAMES` order exactly.

---

### `convert_to_list()`

Converts dictionary format to joint positions array.

```python
convert_to_list(dictionary) -> list[float]
```

**Parameters**:
- `dictionary`: Dict with joint angles (degrees) and gripper (0-100)

**Returns**: List of 6 joint positions in radians

**Implementation**:
```python
[
    dictionary['shoulder_pan'] * 3.14159 / 180.0,
    dictionary['shoulder_lift'] * 3.14159 / 180.0,
    dictionary['elbow_flex'] * 3.14159 / 180.0,
    dictionary['wrist_flex'] * 3.14159 / 180.0,
    dictionary['wrist_roll'] * 3.14159 / 180.0,
    dictionary['gripper'] * 3.14159 / 100.0  # 0-100 → rad
]
```

**Example**:
```python
pose_dict = {
    'shoulder_pan': 45.0,
    'shoulder_lift': 0.0,
    'elbow_flex': 90.0,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 50.0
}
qpos_list = convert_to_list(pose_dict)
# qpos_list ≈ [0.785, 0.0, 1.571, 0.0, 0.0, 1.571]
```

**Note**: Uses `3.14159` instead of `math.pi` (less precise).

---

### `set_initial_pose()`

Sets robot to specified joint configuration.

```python
set_initial_pose(d, position_dict) -> None
```

**Parameters**:
- `d`: MuJoCo data structure
- `position_dict`: Dict with joint angles (degrees) and gripper (0-100)

**Implementation**:
```python
pos = convert_to_list(position_dict)
d.qpos = pos
```

**Example**:
```python
m = mujoco.MjModel.from_xml_path('model/scene_urdf.xml')
d = mujoco.MjData(m)

start_pose = {
    'shoulder_pan': 0.0,
    'shoulder_lift': 0.0,
    'elbow_flex': 0.0,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 0.0
}

set_initial_pose(d, start_pose)
# Robot is now at zero configuration
```

**⚠️ Important**: This does NOT call `mujoco.mj_forward()`, so dependent quantities (e.g., `d.xpos`) won't update until next physics step.

**Comparison with `so101_mujoco_utils2.py`**:
```python
# Legacy (this file)
set_initial_pose(d, pose)

# Modern (utils2)
set_initial_pose(m, d, pose)  # Also calls mj_forward
```

---

### `send_position_command()`

Commands robot actuators to target positions.

```python
send_position_command(d, position_dict) -> None
```

**Parameters**:
- `d`: MuJoCo data structure
- `position_dict`: Dict with target joint angles and gripper

**Implementation**:
```python
pos = convert_to_list(position_dict)
d.ctrl = pos
```

**Example**:
```python
target = {
    'shoulder_pan': 45.0,
    'shoulder_lift': -30.0,
    'elbow_flex': 90.0,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 50.0
}

# In simulation loop:
send_position_command(d, target)
mujoco.mj_step(m, d)
```

**How It Works**:
1. Converts dict to radians list
2. Sets `d.ctrl[0:6] = pos`
3. MuJoCo's position actuators compute torques
4. Torques applied during `mj_step()`

**Actuator Behavior**:
```
tau = kp * (ctrl - qpos) + kv * (0 - qvel)
```
where `kp` and `kv` are defined in the URDF.

---

## Usage Example

### Complete Script

```python
import time
import mujoco
import mujoco.viewer
from so101_mujoco_utils import set_initial_pose, send_position_command

m = mujoco.MjModel.from_xml_path('model/scene_urdf.xml')
d = mujoco.MjData(m)

# Define poses
home = {
    'shoulder_pan': 0.0,
    'shoulder_lift': 0.0,
    'elbow_flex': 0.0,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 0.0
}

# Set initial pose
set_initial_pose(d, home)

# Simulation loop
with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()

        # Command target position
        send_position_command(d, home)

        # Step physics
        mujoco.mj_step(m, d)

        # Update viewer
        viewer.sync()

        # Real-time pacing
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
```

---

## Limitations

### 1. Hardcoded Joint Order

**Problem**: Assumes `qpos[0]` = shoulder_pan, `qpos[1]` = shoulder_lift, etc.

**Risk**: If URDF joint order changes, this breaks silently.

**Solution**: Use `so101_mujoco_utils2.py` which looks up joints by name.

### 2. No Model Argument

**Problem**: Functions take only `d`, not `(m, d)`.

**Impact**: Cannot use robust joint name lookup.

**Solution**: Use modern utilities.

### 3. Pi Approximation

**Problem**: Uses `3.14159` instead of `math.pi`.

**Impact**: Small rounding errors in conversions.

**Fix**:
```python
import math
# Use math.pi instead of 3.14159
```

### 4. No Validation

**Problem**: No checks for:
- Missing keys in dictionary
- Out-of-range values
- Invalid types

**Impact**: Silent errors or crashes.

**Solution**: Add validation or use utilities with better error handling.

### 5. No Advanced Features

**Missing**:
- Motion primitives (move_to_pose, hold_position)
- Real-time plotting
- Robust conversions

**Alternative**: Use `so101_mujoco_utils2.py`.

---

## Migration to `so101_mujoco_utils2.py`

### Conversion Table

| Legacy (utils.py) | Modern (utils2.py) |
|-------------------|--------------------|
| `convert_to_dictionary(qpos)` | `get_positions_dict(m, d)` |
| `convert_to_list(dict)` | `convert_to_list(dict)` (same) |
| `set_initial_pose(d, pose)` | `set_initial_pose(m, d, pose)` |
| `send_position_command(d, pose)` | `send_position_command(m, d, pose)` |

### Example Migration

**Before (legacy)**:
```python
from so101_mujoco_utils import set_initial_pose, send_position_command

set_initial_pose(d, pose)
send_position_command(d, pose)
```

**After (modern)**:
```python
from so101_mujoco_utils2 import set_initial_pose, send_position_command

set_initial_pose(m, d, pose)  # Add 'm'
send_position_command(m, d, pose)  # Add 'm'
```

**Benefits**:
- Robust joint lookup
- Calls `mj_forward()` automatically
- Consistent with other utilities

---

## When This File is Still Useful

### 1. Quick Prototypes
If you need a throwaway script and don't care about robustness.

### 2. Educational
Shows minimal implementation without abstraction.

### 3. Legacy Compatibility
Maintaining old scripts that use this API.

### 4. No Model Available
If you somehow only have `d` but not `m` (rare).

---

## Comparison with Modern Utilities

| Feature | `so101_mujoco_utils.py` | `so101_mujoco_utils2.py` |
|---------|-------------------------|--------------------------|
| Joint lookup | Index-based | Name-based |
| API | `fn(d, ...)` | `fn(m, d, ...)` |
| Pi constant | 3.14159 | `math.pi` |
| `mj_forward()` | No | Yes |
| Motion functions | No | Yes (move_to_pose, hold) |
| Plotting | No | Yes (RealtimeJointPlotter) |
| Validation | Minimal | Better |
| Recommended | ❌ Legacy | ✅ Preferred |

---

## Source Code Reference

### File Structure
```python
so101_mujoco_utils.py (33 lines)
├── convert_to_dictionary()    # Lines 4-12
├── convert_to_list()          # Lines 14-22
├── set_initial_pose()         # Lines 25-27
└── send_position_command()    # Lines 30-32
```

### Dependencies
```python
import time
import mujoco
```

**Note**: No NumPy required (unlike utils2).

---

## Best Practices

### If Using This File

1. **Double-check joint order**: Verify URDF hasn't changed
2. **Add validation**: Check dictionary keys exist
3. **Call mj_forward()**: After `set_initial_pose()`, call `mujoco.mj_forward(m, d)`
4. **Use math.pi**: Replace `3.14159` with `math.pi`
5. **Consider migration**: Evaluate if `utils2` would be better

### If Not Using This File

1. **Use `so101_mujoco_utils2.py`**: More robust
2. **Update imports**: Change import statements
3. **Add `m` argument**: Update function calls

---

## Related Files

- **so101_mujoco_utils2.py**: Modern replacement with more features
- **run_mujoco_simulation_startingpose.py**: Uses this legacy file
- **run_mujoco_simulation.py**: Uses modern utilities

---

## Summary

### Pros ✅
- Simple, minimal code
- No dependencies beyond MuJoCo
- Easy to understand
- Lightweight

### Cons ❌
- Assumes fixed joint order
- Less robust than modern version
- No advanced features
- Pi approximation error

### Recommendation
**For new projects**: Use `so101_mujoco_utils2.py`
**For legacy scripts**: This file works but consider migrating
**For learning**: Good example of minimal utilities

---

## Quick Reference

```python
# Import
from so101_mujoco_utils import (
    convert_to_dictionary,
    convert_to_list,
    set_initial_pose,
    send_position_command
)

# Convert qpos array to dict
pose_dict = convert_to_dictionary(d.qpos)

# Convert dict to qpos array
qpos_list = convert_to_list(pose_dict)

# Set robot to pose
set_initial_pose(d, pose_dict)

# Command actuators
send_position_command(d, target_dict)
```
