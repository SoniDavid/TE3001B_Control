# mujoco_simulation.py

## Overview

Basic MuJoCo simulation script that demonstrates the minimal setup needed to visualize the SO101 robot arm with real-time joint position plotting.

**File Path**: `simulation_code/mujoco_simulation.py`

## Purpose

This script serves as a simple introduction to:
- Loading a MuJoCo model from XML
- Launching the interactive 3D viewer
- Running physics simulation steps
- Recording and plotting joint data in real-time

## Dependencies

```python
import time
import mujoco
import mujoco.viewer
from so101_mujoco_utils2 import RealtimeJointPlotter
```

## Key Components

### 1. Model Loading
```python
m = mujoco.MjModel.from_xml_path("model/scene_urdf.xml")
d = mujoco.MjData(m)
```
- Loads the complete scene (robot + environment)
- Creates MuJoCo data structure for simulation state

### 2. Real-time Plotter Setup
```python
plotter = RealtimeJointPlotter(max_points=4000)
plotter.start(host="127.0.0.1", port=8050, update_ms=100)
```
- Initializes Dash/Plotly web server
- Buffers up to 4000 data points
- Updates plot every 100ms
- Accessible at http://127.0.0.1:8050

### 3. Simulation Loop
```python
with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()

        mujoco.mj_step(m, d)
        plotter.sample(m, d)

        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

        viewer.sync()

        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
```

#### Loop Components:
- **Duration**: Runs for 30 seconds
- **Physics Step**: `mujoco.mj_step(m, d)` advances simulation
- **Data Recording**: `plotter.sample(m, d)` captures joint states
- **Viewer Update**: `viewer.sync()` updates 3D display
- **Contact Visualization**: Toggles contact point display every 2 seconds
- **Timing**: Maintains real-time pacing

## Usage

### Running the Script
```bash
cd simulation_code
python mujoco_simulation.py
```

### What to Expect
1. **3D Viewer Window**: Opens showing the SO101 arm
2. **Web Browser**: Navigate to http://127.0.0.1:8050 for live plots
3. **Duration**: Simulation runs for 30 seconds
4. **Robot Behavior**: Passive simulation (no active control, gravity effects)

### Viewer Controls
- **Left Mouse**: Rotate view
- **Right Mouse**: Zoom
- **Middle Mouse**: Pan
- **Scroll Wheel**: Zoom in/out
- **Double Click**: Select body
- **Ctrl+Right**: Apply forces

## Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `max_points` | 4000 | Maximum data points in plot buffer |
| `host` | "127.0.0.1" | Plotting server address |
| `port` | 8050 | Plotting server port |
| `update_ms` | 100 | Plot update interval (ms) |
| `duration` | 30 | Simulation duration (seconds) |

## Output

- **3D Visualization**: Interactive robot model viewer
- **Real-time Plots**: Joint positions vs. time for all 6 joints
- **Console**: Simulation progress (if any warnings/errors)

## Code Flow

```
1. Load MuJoCo model (scene_urdf.xml)
2. Initialize data structure
3. Start plotting server (background thread)
4. Launch passive viewer
5. For 30 seconds:
   a. Step physics simulation
   b. Record joint positions
   c. Toggle contact point visualization
   d. Update viewer display
   e. Sleep to maintain real-time
6. Cleanup and exit
```

## Modifications

### Change Simulation Duration
```python
while viewer.is_running() and time.time() - start < 60:  # 60 seconds
```

### Disable Contact Point Toggle
```python
# Comment out or remove:
# with viewer.lock():
#     viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)
```

### Change Plot Server Port
```python
plotter.start(host="127.0.0.1", port=9000, update_ms=100)
```

### Increase Plot Buffer Size
```python
plotter = RealtimeJointPlotter(max_points=10000)
```

## Troubleshooting

### Issue: Viewer closes immediately
**Solution**: Check that the XML file path is correct relative to the script location.

### Issue: Plot page doesn't load
**Solution**:
- Ensure Dash/Plotly are installed: `pip install dash plotly`
- Check if port 8050 is available
- Look for error messages in console

### Issue: Robot falls through floor
**Solution**: This is expected in passive mode (no control). The robot responds to gravity and contact forces.

## Related Files

- **so101_mujoco_utils2.py**: Provides `RealtimeJointPlotter` class
- **model/scene_urdf.xml**: Complete scene definition
- **model/robot_from_urdf.xml**: Robot-only model

## Learning Objectives

This script demonstrates:
1. Minimal MuJoCo simulation setup
2. Real-time data visualization
3. Passive viewer interaction
4. Basic timing and synchronization
5. Contact visualization features

## Next Steps

After understanding this script, explore:
- **run_mujoco_simulation.py**: Position control
- **run_mujoco_simulation2.py**: PID control with perturbations
- **so101_mujoco_utils2.py**: Implementation of plotting and utilities
