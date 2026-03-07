# so101_mujoco_utils2.py

## Overview

**Modern** utility module providing robust robot control functions, motion primitives, and real-time plotting for the SO101 arm. This is the recommended version for new projects.

**File Path**: `simulation_code/so101_mujoco_utils2.py`

**Status**: ✅ **Current/Recommended** - Preferred over `so101_mujoco_utils.py`

## Purpose

Provides comprehensive utilities for:
- **Robust conversions**: Name-based joint lookup (not index-based)
- **Motion primitives**: `move_to_pose()`, `hold_position()`
- **Real-time plotting**: Web-based Dash/Plotly visualization
- **Helper functions**: Coordinate conversions, actuator control

## Module Contents

```python
so101_mujoco_utils2.py
├── Constants
│   └── JOINT_NAMES
├── Conversion Functions
│   ├── _deg2rad()
│   ├── _rad2deg()
│   ├── convert_to_dictionary()
│   └── convert_to_list()
├── MuJoCo Helpers
│   ├── _joint_qpos_index()
│   ├── _actuator_index()
│   ├── get_positions_dict()
│   ├── set_initial_pose()
│   ├── send_position_command()
│   └── _step_realtime()
├── Plotting
│   └── RealtimeJointPlotter (class)
└── Motion Primitives
    ├── move_to_pose()
    └── hold_position()
```

---

## Constants

### JOINT_NAMES
```python
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
```

Default joint ordering for the SO101 robot.

---

## Conversion Functions

### Private Helpers

```python
_deg2rad(x: float) -> float     # Degrees to radians
_rad2deg(x: float) -> float     # Radians to degrees
```

Use `math.pi` for precision.

### `convert_to_dictionary()`

```python
convert_to_dictionary(qpos) -> dict[str, float]
```

**⚠️ Warning**: Index-based, not robust. Use `get_positions_dict(m, d)` instead.

**Legacy**: Kept for backward compatibility only.

### `convert_to_list()`

```python
convert_to_list(position_dict) -> list[float]
```

Converts pose dictionary to radians list.

**Example**:
```python
pose = {
    "shoulder_pan": 45.0,
    "shoulder_lift": -30.0,
    "elbow_flex": 90.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 50.0,
}
qpos_list = convert_to_list(pose)
# qpos_list ≈ [0.785, -0.524, 1.571, 0.0, 0.0, 1.571]
```

---

## MuJoCo Helper Functions

### `_joint_qpos_index()`

```python
_joint_qpos_index(m, joint_name: str) -> int
```

Returns the `qpos` index for a named joint using MuJoCo's name lookup.

**Example**:
```python
idx = _joint_qpos_index(m, "shoulder_pan")
# idx = 0 (usually, but robust to URDF changes)
```

**Why robust**: Uses `mujoco.mj_name2id()` instead of hardcoded indices.

### `_actuator_index()`

```python
_actuator_index(m, actuator_name: str) -> int
```

Returns the `ctrl` index for a named actuator.

### `get_positions_dict()`

```python
get_positions_dict(m, d) -> dict[str, float]
```

**Preferred** method to read joint positions.

**Returns**: Dictionary with:
- Joint angles in **degrees** (shoulder_pan through wrist_roll)
- Gripper in **0-100 range**

**Example**:
```python
import mujoco
m = mujoco.MjModel.from_xml_path("model/scene_urdf.xml")
d = mujoco.MjData(m)

mujoco.mj_step(m, d)  # Step once

pos = get_positions_dict(m, d)
# pos = {
#     "shoulder_pan": 0.5,
#     "shoulder_lift": -2.3,
#     "elbow_flex": 15.7,
#     "wrist_flex": 8.2,
#     "wrist_roll": 0.0,
#     "gripper": 12.3
# }
```

**Advantages over `convert_to_dictionary()`**:
- Uses joint name lookup (robust)
- Takes `(m, d)` instead of just `qpos`
- More accurate (uses `math.pi`)

### `set_initial_pose()`

```python
set_initial_pose(m, d, position_dict) -> None
```

Sets robot to specified configuration and updates kinematics.

**Example**:
```python
start_pose = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -90.0,
    "elbow_flex": 90.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}

set_initial_pose(m, d, start_pose)
# Robot is now at specified pose
# Kinematics updated via mj_forward()
```

**Implementation**:
```python
target = convert_to_list(position_dict)
for i, name in enumerate(JOINT_NAMES):
    qadr = _joint_qpos_index(m, name)
    d.qpos[qadr] = target[i]
mujoco.mj_forward(m, d)  # Important!
```

**Difference from legacy**:
- Legacy: `set_initial_pose(d, pose)` - no `mj_forward()`
- Modern: `set_initial_pose(m, d, pose)` - includes `mj_forward()`

### `send_position_command()`

```python
send_position_command(m, d, position_dict) -> None
```

Commands position actuators to target pose.

**Example**:
```python
target = {
    "shoulder_pan": 45.0,
    "shoulder_lift": -30.0,
    ...
}

# In simulation loop:
send_position_command(m, d, target)
mujoco.mj_step(m, d)
viewer.sync()
```

**Implementation**:
```python
target = convert_to_list(position_dict)
for i, name in enumerate(JOINT_NAMES):
    a_id = _actuator_index(m, name)
    d.ctrl[a_id] = target[i]
```

### `_step_realtime()`

```python
_step_realtime(m, step_start: float) -> None
```

Sleeps to maintain real-time pacing.

**Internal use**: Called by motion primitives.

---

## Real-time Plotting

### RealtimeJointPlotter (Class)

Web-based real-time plotter using Dash and Plotly.

#### Constructor

```python
RealtimeJointPlotter(
    joint_names = JOINT_NAMES,
    max_points: int = 2000
)
```

**Parameters**:
- `joint_names`: Joints to plot (default: all 6)
- `max_points`: Maximum data points in rolling window

**Example**:
```python
plotter = RealtimeJointPlotter(max_points=4000)
```

#### Methods

##### `start()`

```python
plotter.start(
    host: str = "127.0.0.1",
    port: int = 8050,
    update_ms: int = 100
) -> None
```

Starts Dash server in background thread.

**Parameters**:
- `host`: Server address
- `port`: Server port
- `update_ms`: Plot refresh interval (milliseconds)

**Example**:
```python
plotter.start(host="127.0.0.1", port=8050, update_ms=100)
# Open browser to: http://127.0.0.1:8050
```

**Dependencies**: Requires `dash` and `plotly` packages.

##### `sample()`

```python
plotter.sample(m, d, now: Optional[float] = None) -> None
```

Records one data point.

**Parameters**:
- `m`: MuJoCo model
- `d`: MuJoCo data
- `now`: Timestamp (optional, defaults to `time.time()`)

**Example**:
```python
# In simulation loop:
for _ in range(1000):
    mujoco.mj_step(m, d)
    plotter.sample(m, d)  # Record state after step
    viewer.sync()
```

**Important**: Call **after** `mj_step()` to record realized state.

##### `stop()`

```python
plotter.stop() -> None
```

Stops plotting (daemon thread exits automatically).

#### Complete Example

```python
from so101_mujoco_utils2 import RealtimeJointPlotter, move_to_pose
import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path("model/scene_urdf.xml")
d = mujoco.MjData(m)

# Create and start plotter
plotter = RealtimeJointPlotter(max_points=4000)
plotter.start(host="127.0.0.1", port=8050, update_ms=100)

# Motion with plotting
target = {"shoulder_pan": 45.0, ...}

with mujoco.viewer.launch_passive(m, d) as viewer:
    move_to_pose(m, d, viewer, target, duration=2.0, plotter=plotter)

# View plots at: http://127.0.0.1:8050
```

#### Plot Features

- **Interactive**: Zoom, pan, hover for values
- **Multi-trace**: All joints on one plot
- **Auto-scroll**: Rolling time window
- **Legend**: Toggle individual joints
- **Web-based**: No blocking GUI

#### Internal Architecture

```python
class RealtimeJointPlotter:
    _lock: threading.Lock        # Thread safety
    _t: deque                    # Time buffer
    _y: dict[str, deque]         # Joint position buffers
    _dash_thread: Thread         # Background server
```

- **Thread-safe**: Uses locks for concurrent access
- **Circular buffers**: `deque(maxlen=max_points)` for efficiency
- **Streaming**: Uses `extendData` for incremental updates

---

## Motion Primitives

### `move_to_pose()`

Smoothly moves robot from current pose to target using linear interpolation and position actuators.

```python
move_to_pose(
    m,
    d,
    viewer,
    desired_position: dict[str, float],
    duration: float,
    realtime: bool = True,
    plotter: Optional[RealtimeJointPlotter] = None
) -> None
```

**Parameters**:
| Name | Type | Description |
|------|------|-------------|
| `m` | MjModel | MuJoCo model |
| `d` | MjData | MuJoCo data |
| `viewer` | Viewer | Viewer instance |
| `desired_position` | dict | Target pose (degrees + gripper 0-100) |
| `duration` | float | Motion time (seconds) |
| `realtime` | bool | Real-time pacing (default True) |
| `plotter` | Plotter | Optional plotter |

**Algorithm**:
```python
1. Read starting pose: p0 = get_positions_dict(m, d)
2. For each timestep:
   a. Compute alpha = (t - t0) / duration
   b. Interpolate: p_cmd = (1-alpha)*p0 + alpha*p_target
   c. Send command: send_position_command(m, d, p_cmd)
   d. Step simulation: mj_step(m, d)
   e. Record data (if plotter)
   f. Update viewer
   g. Sleep (if realtime)
```

**Example**:
```python
from so101_mujoco_utils2 import set_initial_pose, move_to_pose

start = {"shoulder_pan": 0.0, ...}
target1 = {"shoulder_pan": 90.0, ...}
target2 = {"shoulder_pan": -90.0, ...}

set_initial_pose(m, d, start)

with mujoco.viewer.launch_passive(m, d) as viewer:
    move_to_pose(m, d, viewer, target1, duration=2.0, realtime=True)
    move_to_pose(m, d, viewer, target2, duration=2.0, realtime=True)
    move_to_pose(m, d, viewer, start, duration=2.0, realtime=True)
```

**Characteristics**:
- **Smooth**: Linear interpolation ensures continuous motion
- **Timed**: Completes in exactly `duration` seconds (if realtime)
- **Safe**: Uses position actuators (force-limited)

### `hold_position()`

Maintains current pose for specified duration.

```python
hold_position(
    m,
    d,
    viewer,
    duration: float,
    realtime: bool = True,
    plotter: Optional[RealtimeJointPlotter] = None
) -> None
```

**Parameters**: Same as `move_to_pose()` except no `desired_position`.

**Algorithm**:
```python
1. Read current pose: p_hold = get_positions_dict(m, d)
2. For duration:
   a. Send command: send_position_command(m, d, p_hold)
   b. Step, record, update, sleep
```

**Example**:
```python
move_to_pose(m, d, viewer, target, duration=2.0)
hold_position(m, d, viewer, duration=3.0)  # Hold for 3 seconds
```

**Use Cases**:
- Pause between motions
- Let transients settle
- Demonstrate position hold
- Wait for external event

---

## Usage Patterns

### Pattern 1: Sequential Motions

```python
poses = [pose1, pose2, pose3, pose4]
for target in poses:
    move_to_pose(m, d, viewer, target, duration=2.0, realtime=True)
    hold_position(m, d, viewer, duration=0.5, realtime=True)
```

### Pattern 2: With Plotting

```python
plotter = RealtimeJointPlotter(max_points=4000)
plotter.start(port=8050)

move_to_pose(m, d, viewer, target, duration=2.0, plotter=plotter)
# Open http://127.0.0.1:8050 to see live plot
```

### Pattern 3: Fast Simulation (No Realtime)

```python
move_to_pose(m, d, None, target, duration=2.0, realtime=False)
# Runs as fast as possible, no viewer update
```

### Pattern 4: Cyclic Motion

```python
while viewer.is_running():
    move_to_pose(m, d, viewer, pose_a, duration=1.0, realtime=True)
    move_to_pose(m, d, viewer, pose_b, duration=1.0, realtime=True)
```

---

## Advantages Over Legacy

| Feature | Legacy (utils.py) | Modern (utils2.py) |
|---------|-------------------|---------------------|
| Joint lookup | Index-based | Name-based ✅ |
| API consistency | `fn(d, ...)` | `fn(m, d, ...)` ✅ |
| Pi precision | 3.14159 | `math.pi` ✅ |
| `mj_forward()` | Manual | Automatic ✅ |
| Motion functions | ❌ | ✅ `move_to_pose()`, `hold_position()` |
| Plotting | ❌ | ✅ `RealtimeJointPlotter` |
| Type hints | ❌ | ✅ Full annotations |
| Robustness | Basic | High ✅ |

---

## Dependencies

```python
# Standard library
from __future__ import annotations
import time
import math
import threading
from collections import deque
from typing import Optional

# Third-party
import mujoco

# Optional (for plotting)
from dash import Dash, dcc, html, no_update
from dash.dependencies import Input, Output
import plotly.graph_objects as go
```

**Note**: Dash/Plotly only required if using `RealtimeJointPlotter`.

---

## Best Practices

### 1. Always Use `get_positions_dict()` to Read State

```python
# Good
pos = get_positions_dict(m, d)

# Avoid
pos = convert_to_dictionary(d.qpos)  # Not robust
```

### 2. Set Initial Pose Before Viewer

```python
# Good
set_initial_pose(m, d, start_pose)
with mujoco.viewer.launch_passive(m, d) as viewer:
    ...

# Bad (robot falls on startup)
with mujoco.viewer.launch_passive(m, d) as viewer:
    set_initial_pose(m, d, start_pose)  # Too late
```

### 3. Record Data After Physics Step

```python
# Good
mujoco.mj_step(m, d)
plotter.sample(m, d)  # Realized state

# Bad
plotter.sample(m, d)  # Commanded state
mujoco.mj_step(m, d)
```

### 4. Use Type Hints

```python
def my_function(m: mujoco.MjModel, d: mujoco.MjData) -> dict[str, float]:
    return get_positions_dict(m, d)
```

---

## Troubleshooting

### Plotter Shows "No samples yet"

**Cause**: `plotter.sample()` not being called

**Solution**: Ensure it's in the simulation loop after `mj_step()`

### Plot Page Won't Load

**Cause**: Dash/Plotly not installed

**Solution**: `pip install dash plotly`

### Robot Jumps at Start

**Cause**: Initial pose not set before viewer

**Solution**: Call `set_initial_pose(m, d, start)` before `launch_passive()`

### Motion Not Smooth

**Cause**: `duration` too short or timestep too large

**Solution**: Increase duration or check `m.opt.timestep` (should be ~0.002s)

---

## Related Files

- **run_mujoco_simulation.py**: Example using position control
- **run_mujoco_simulation2.py**: Example with PID control
- **so101_mujoco_pid_utils.py**: PID-based motion functions
- **so101_mujoco_utils.py**: Legacy version (not recommended)

---

## API Quick Reference

### Conversion
```python
_deg2rad(degrees) -> radians
_rad2deg(radians) -> degrees
convert_to_list(pose_dict) -> qpos_list
```

### State Access
```python
get_positions_dict(m, d) -> pose_dict
```

### Control
```python
set_initial_pose(m, d, pose_dict)
send_position_command(m, d, pose_dict)
```

### Motion
```python
move_to_pose(m, d, viewer, target, duration, realtime, plotter)
hold_position(m, d, viewer, duration, realtime, plotter)
```

### Plotting
```python
plotter = RealtimeJointPlotter(max_points=4000)
plotter.start(host="127.0.0.1", port=8050, update_ms=100)
plotter.sample(m, d)
plotter.stop()
```

---

## Summary

**Strengths** ✅:
- Robust name-based joint lookup
- Comprehensive motion primitives
- Real-time web-based plotting
- Full type annotations
- Automatic kinematics updates
- Backward compatible API

**Use For**:
- All new projects
- Position-controlled motions
- Real-time visualization
- Educational demonstrations
- Robust simulations

**Recommended Workflow**:
1. Import utilities
2. Load model
3. Set initial pose
4. Start plotter (optional)
5. Run motion sequence
6. View results in browser
