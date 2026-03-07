# SO101 Robot Arm MuJoCo Simulation - Documentation

This documentation provides a comprehensive overview of the SO101 robotic arm simulation project using MuJoCo physics engine.

## Project Overview

This project implements a physics-based simulation of the SO101 robotic arm using MuJoCo. It includes:

- **Robot Model**: Complete URDF-based model with 6 degrees of freedom (5 arm joints + gripper)
- **Control Systems**: PID control with perturbation modeling for realistic testing
- **Visualization**: Real-time plotting of joint positions using Dash/Plotly
- **Simulation Scripts**: Multiple ready-to-run simulation scenarios

## Quick Start

### Installation

```bash
cd simulation_code
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

### Running Simulations

```bash
# Basic simulation with real-time plotting
python run_mujoco_simulation.py

# PID control with perturbations
python run_mujoco_simulation2.py

# Simple demo with starting pose
python run_mujoco_simulation_startingpose.py

# Basic MuJoCo viewer
python mujoco_simulation.py
```

### Real-time Plotting

When running simulations with plotting enabled, open your browser to:
- http://127.0.0.1:8050

## Project Structure

```
control_lab_4.1/
├── docs/                           # Documentation (this folder)
│   ├── README.md                   # This file
│   └── source_code/                # Detailed documentation of all source files
├── simulation_code/                # Main simulation code
│   ├── model/                      # Robot model files
│   │   ├── assets/                 # 3D meshes (STL files)
│   │   ├── robot_from_urdf.xml     # Robot-only MJCF model
│   │   └── scene_urdf.xml          # Complete scene with robot
│   ├── mujoco_simulation.py        # Basic simulation demo
│   ├── run_mujoco_simulation.py    # Position control demo
│   ├── run_mujoco_simulation2.py   # PID control with plotting
│   ├── run_mujoco_simulation_startingpose.py  # Starting pose demo
│   ├── so101_control.py            # Core control algorithms (PID, perturbations)
│   ├── so101_mujoco_pid_utils.py   # PID motion utilities
│   ├── so101_mujoco_utils.py       # Basic robot control utilities (legacy)
│   ├── so101_mujoco_utils2.py      # Enhanced utilities with plotting
│   └── requirements.txt            # Python dependencies
└── ros2_ws/                        # ROS2 workspace (future integration)
```

## Robot Specifications

### SO101 Arm Joints

| Joint Name      | Type  | Range (deg)    | Description           |
|----------------|-------|----------------|-----------------------|
| shoulder_pan   | Hinge | -110 to 110    | Base rotation         |
| shoulder_lift  | Hinge | -100 to 100    | Shoulder pitch        |
| elbow_flex     | Hinge | -97 to 97      | Elbow joint           |
| wrist_flex     | Hinge | -95 to 95      | Wrist pitch           |
| wrist_roll     | Hinge | -157 to 163    | Wrist roll            |
| gripper        | Hinge | -10 to 100     | Gripper (0-100 range) |

### Actuators

- **Motor Type**: STS3215 servos
- **Force Range**: -3.35 to 3.35 N·m
- **Control Type**: Position control (PD servo)
- **Default PD Gains**: kp=998.22, kv=2.731

## Documentation Files

### Source Code Documentation

All source files are documented in detail in the `source_code/` directory:

1. **[mujoco_simulation.md](source_code/mujoco_simulation.md)** - Basic simulation viewer
2. **[run_mujoco_simulation.md](source_code/run_mujoco_simulation.md)** - Position control demo
3. **[run_mujoco_simulation2.md](source_code/run_mujoco_simulation2.md)** - PID control demo
4. **[run_mujoco_simulation_startingpose.md](source_code/run_mujoco_simulation_startingpose.md)** - Starting pose demo
5. **[so101_control.md](source_code/so101_control.md)** - Core control algorithms
6. **[so101_mujoco_pid_utils.md](source_code/so101_mujoco_pid_utils.md)** - PID motion utilities
7. **[so101_mujoco_utils.md](source_code/so101_mujoco_utils.md)** - Basic utilities (legacy)
8. **[so101_mujoco_utils2.md](source_code/so101_mujoco_utils2.md)** - Enhanced utilities
9. **[robot_model.md](source_code/robot_model.md)** - MJCF/URDF model documentation

## Key Features

### 1. Position Control
- Smooth interpolation between poses
- Real-time trajectory execution
- Position hold functionality

### 2. PID Control
- Joint-space PID controller
- Per-joint tunable gains
- Anti-windup protection
- Torque limiting

### 3. Perturbation Modeling
- Sinusoidal disturbances
- Colored noise injection
- Random impulse perturbations
- Measurement noise simulation

### 4. Real-time Visualization
- Live joint position plotting via Dash/Plotly
- Configurable update rates
- Multi-joint tracking
- Web-based interface

## Dependencies

- **MuJoCo** (>=3.0.0): Physics simulation engine
- **NumPy** (>=1.24.0): Numerical computing
- **Dash** (>=2.14.0): Web application framework for plotting
- **Plotly** (>=5.17.0): Interactive plotting library

## Common Use Cases

### Example 1: Test Position Control
```python
from so101_mujoco_utils2 import set_initial_pose, move_to_pose
import mujoco

m = mujoco.MjModel.from_xml_path("model/scene_urdf.xml")
d = mujoco.MjData(m)

target = {
    "shoulder_pan": 45.0,
    "shoulder_lift": -30.0,
    # ... other joints
}

with mujoco.viewer.launch_passive(m, d) as viewer:
    move_to_pose(m, d, viewer, target, duration=2.0, realtime=True)
```

### Example 2: PID Control with Perturbations
```python
from so101_mujoco_pid_utils import move_to_pose_pid, build_default_pid

pid = build_default_pid()
# Tune gains as needed
pid.gains["shoulder_pan"].kp = 50.0
pid.gains["shoulder_pan"].kd = 2.0

move_to_pose_pid(m, d, viewer, target, duration=2.0, pid=pid)
```

### Example 3: Real-time Plotting
```python
from so101_mujoco_utils2 import RealtimeJointPlotter

plotter = RealtimeJointPlotter(max_points=4000)
plotter.start(host="127.0.0.1", port=8050, update_ms=100)

# In simulation loop:
plotter.sample(m, d)
```

## Troubleshooting

### Issue: Viewer closes immediately
- Check that the simulation loop duration is sufficient
- Ensure viewer.is_running() is checked in your loop

### Issue: Robot falls through floor
- Verify initial pose is set before opening viewer
- Check that mujoco.mj_forward(m, d) is called after setting qpos

### Issue: Plotting page doesn't load
- Ensure Dash and Plotly are installed: `pip install dash plotly`
- Check that port 8050 is not blocked by firewall
- Verify plotter.sample() is being called in the simulation loop

### Issue: Unstable PID control
- Reduce Kp, Ki, Kd gains
- Increase damping in joint properties
- Check for measurement noise
- Verify timestep is appropriate (typically 0.002s)

## Contributing

When adding new features:
1. Document all functions with docstrings
2. Update this documentation
3. Follow the existing code style
4. Test with various robot configurations

## License

This project is part of the IRS Control Lab coursework.

## Contact

For questions or issues, please contact the course instructor or teaching assistants.
