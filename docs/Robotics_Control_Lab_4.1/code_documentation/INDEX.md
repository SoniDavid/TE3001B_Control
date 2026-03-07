# Source Code Documentation Index

Complete documentation for all source files in the SO101 MuJoCo simulation project.

## Quick Navigation

### Simulation Scripts
1. **[mujoco_simulation.md](mujoco_simulation.md)** - Basic simulation with real-time plotting
2. **[run_mujoco_simulation.md](run_mujoco_simulation.md)** - Position control demo
3. **[run_mujoco_simulation2.md](run_mujoco_simulation2.md)** - PID control with perturbations
4. **[run_mujoco_simulation_startingpose.md](run_mujoco_simulation_startingpose.md)** - Starting pose demo

### Core Libraries
5. **[so101_control.md](so101_control.md)** - PID controllers and perturbation models
6. **[so101_mujoco_pid_utils.md](so101_mujoco_pid_utils.md)** - PID motion primitives
7. **[so101_mujoco_utils.md](so101_mujoco_utils.md)** - Basic utilities (legacy)
8. **[so101_mujoco_utils2.md](so101_mujoco_utils2.md)** - Enhanced utilities (recommended)

### Model
9. **[robot_model.md](robot_model.md)** - MJCF/URDF robot model documentation

---

## File Overview Table

| File | Type | Complexity | Status | Best For |
|------|------|------------|--------|----------|
| mujoco_simulation.py | Demo | Simple | ✅ Current | Learning basics, plotting |
| run_mujoco_simulation.py | Demo | Simple | ✅ Current | Position control, motion |
| run_mujoco_simulation2.py | Demo | Advanced | ✅ Current | PID control, research |
| run_mujoco_simulation_startingpose.py | Demo | Very Simple | ✅ Current | First steps, beginners |
| so101_control.py | Library | Advanced | ✅ Current | Control algorithms |
| so101_mujoco_pid_utils.py | Library | Medium | ✅ Current | PID motions |
| so101_mujoco_utils.py | Library | Simple | ⚠️ Legacy | Compatibility only |
| so101_mujoco_utils2.py | Library | Medium | ✅ Recommended | New projects |
| robot_from_urdf.xml | Model | - | ✅ Current | Robot definition |
| scene_urdf.xml | Model | - | ✅ Current | Complete scene |

---

## Learning Path

### Beginner Path
1. Start with **[run_mujoco_simulation_startingpose.md](run_mujoco_simulation_startingpose.md)**
2. Read **[so101_mujoco_utils.md](so101_mujoco_utils.md)** for basic utilities
3. Try **[run_mujoco_simulation.md](run_mujoco_simulation.md)** for motion
4. Explore **[robot_model.md](robot_model.md)** to understand the robot

### Intermediate Path
1. Read **[so101_mujoco_utils2.md](so101_mujoco_utils2.md)** for modern utilities
2. Study **[mujoco_simulation.md](mujoco_simulation.md)** for plotting
3. Understand **[so101_control.md](so101_control.md)** for control theory
4. Experiment with modifying simulation scripts

### Advanced Path
1. Master **[run_mujoco_simulation2.md](run_mujoco_simulation2.md)**
2. Study **[so101_mujoco_pid_utils.md](so101_mujoco_pid_utils.md)** implementation
3. Tune PID gains using **[so101_control.md](so101_control.md)**
4. Implement custom controllers
5. Modify **[robot_model.md](robot_model.md)** for variations

---

## By Feature

### Real-time Plotting
- **[mujoco_simulation.md](mujoco_simulation.md)** - Basic plotting demo
- **[run_mujoco_simulation2.md](run_mujoco_simulation2.md)** - PID with plots
- **[so101_mujoco_utils2.md](so101_mujoco_utils2.md)** - `RealtimeJointPlotter` class

### Motion Control
- **[run_mujoco_simulation.md](run_mujoco_simulation.md)** - Position actuators
- **[run_mujoco_simulation2.md](run_mujoco_simulation2.md)** - PID torque control
- **[so101_mujoco_utils2.md](so101_mujoco_utils2.md)** - `move_to_pose()`, `hold_position()`
- **[so101_mujoco_pid_utils.md](so101_mujoco_pid_utils.md)** - PID motion primitives

### Control Algorithms
- **[so101_control.md](so101_control.md)** - `JointPID`, `PerturbationModel`
- **[so101_mujoco_pid_utils.md](so101_mujoco_pid_utils.md)** - High-level PID functions

### Robot Configuration
- **[robot_model.md](robot_model.md)** - Full model documentation
- **[so101_mujoco_utils2.md](so101_mujoco_utils2.md)** - `set_initial_pose()`

### Data Visualization
- **[so101_mujoco_utils2.md](so101_mujoco_utils2.md)** - `RealtimeJointPlotter`
- **[mujoco_simulation.md](mujoco_simulation.md)** - Example usage

---

## By Use Case

### "I want to run my first simulation"
→ **[run_mujoco_simulation_startingpose.md](run_mujoco_simulation_startingpose.md)**

### "I need to move the robot between poses"
→ **[run_mujoco_simulation.md](run_mujoco_simulation.md)**
→ **[so101_mujoco_utils2.md](so101_mujoco_utils2.md)**

### "I want to test PID controllers"
→ **[run_mujoco_simulation2.md](run_mujoco_simulation2.md)**
→ **[so101_control.md](so101_control.md)**

### "I need real-time data plotting"
→ **[mujoco_simulation.md](mujoco_simulation.md)**
→ **[so101_mujoco_utils2.md](so101_mujoco_utils2.md)** (RealtimeJointPlotter)

### "I want to understand the robot model"
→ **[robot_model.md](robot_model.md)**

### "I need to add perturbations/disturbances"
→ **[so101_control.md](so101_control.md)** (PerturbationModel)
→ **[run_mujoco_simulation2.md](run_mujoco_simulation2.md)**

### "I'm migrating from legacy code"
→ **[so101_mujoco_utils.md](so101_mujoco_utils.md)** (old API)
→ **[so101_mujoco_utils2.md](so101_mujoco_utils2.md)** (new API)

---

## Detailed File Descriptions

### 1. mujoco_simulation.md
- **Lines of Code**: 36
- **Dependencies**: `mujoco`, `so101_mujoco_utils2`
- **Key Features**: Basic viewer, real-time plotting, contact visualization
- **Difficulty**: ⭐ Very Simple
- **Typical Use**: Learning MuJoCo basics, testing plotting

### 2. run_mujoco_simulation.md
- **Lines of Code**: 50
- **Dependencies**: `mujoco`, `so101_mujoco_utils2`
- **Key Features**: Position control, smooth motion, hold positions
- **Difficulty**: ⭐⭐ Simple
- **Typical Use**: Demonstrations, simple motion sequences

### 3. run_mujoco_simulation2.md
- **Lines of Code**: 49
- **Dependencies**: `mujoco`, `so101_mujoco_utils2`, `so101_mujoco_pid_utils`
- **Key Features**: PID control, perturbations, real-time plotting
- **Difficulty**: ⭐⭐⭐⭐ Advanced
- **Typical Use**: Research, control tuning, robustness testing

### 4. run_mujoco_simulation_startingpose.md
- **Lines of Code**: 38
- **Dependencies**: `mujoco`, `so101_mujoco_utils` (legacy)
- **Key Features**: Minimal example, static hold
- **Difficulty**: ⭐ Very Simple
- **Typical Use**: First-time users, beginners

### 5. so101_control.md
- **Lines of Code**: 178
- **Dependencies**: `numpy`, `mujoco`
- **Key Classes**: `JointPID`, `PerturbationModel`
- **Difficulty**: ⭐⭐⭐⭐ Advanced
- **Typical Use**: Control algorithm implementation

### 6. so101_mujoco_pid_utils.md
- **Lines of Code**: 171
- **Dependencies**: `mujoco`, `numpy`, `so101_control`
- **Key Functions**: `move_to_pose_pid()`, `hold_position_pid()`
- **Difficulty**: ⭐⭐⭐ Medium
- **Typical Use**: PID-based motion generation

### 7. so101_mujoco_utils.md
- **Lines of Code**: 33
- **Dependencies**: `mujoco`
- **Status**: ⚠️ Legacy
- **Difficulty**: ⭐⭐ Simple
- **Typical Use**: Backward compatibility

### 8. so101_mujoco_utils2.md
- **Lines of Code**: 290
- **Dependencies**: `mujoco`, `numpy`, `dash`, `plotly`
- **Key Classes**: `RealtimeJointPlotter`
- **Key Functions**: `move_to_pose()`, `hold_position()`, `get_positions_dict()`
- **Difficulty**: ⭐⭐⭐ Medium
- **Typical Use**: New projects, recommended utilities

### 9. robot_model.md
- **File Format**: MJCF (XML)
- **Joints**: 6 (5 arm + 1 gripper)
- **Total Mass**: ~0.73 kg
- **Actuators**: Position-controlled (STS3215)
- **Difficulty**: ⭐⭐ Simple (to use), ⭐⭐⭐⭐ Advanced (to modify)

---

## Code Dependencies Graph

```
run_mujoco_simulation_startingpose.py
    └── so101_mujoco_utils.py (legacy)

mujoco_simulation.py
    └── so101_mujoco_utils2.py
        └── RealtimeJointPlotter

run_mujoco_simulation.py
    └── so101_mujoco_utils2.py
        ├── move_to_pose()
        └── hold_position()

run_mujoco_simulation2.py
    ├── so101_mujoco_utils2.py
    │   ├── set_initial_pose()
    │   └── RealtimeJointPlotter
    └── so101_mujoco_pid_utils.py
        ├── move_to_pose_pid()
        ├── hold_position_pid()
        └── so101_control.py
            ├── JointPID
            └── PerturbationModel
```

---

## Quick Tips

### For Beginners
- Start with simple demos (startingpose → simulation → simulation2)
- Read documentation in order of complexity
- Run examples before modifying code

### For Intermediate Users
- Focus on utils2 and pid_utils
- Understand the difference between position and torque control
- Experiment with PID gain tuning

### For Advanced Users
- Study so101_control.py implementation
- Modify perturbation models
- Implement custom controllers
- Extend plotting capabilities

---

## External Resources

### MuJoCo Documentation
- **Official Docs**: https://mujoco.readthedocs.io
- **Python Bindings**: https://mujoco.readthedocs.io/en/stable/python.html
- **MJCF Reference**: https://mujoco.readthedocs.io/en/stable/XMLreference.html

### Control Theory
- **PID Tuning**: Åström & Hägglund, "Advanced PID Control"
- **Robotics**: Siciliano et al., "Robotics: Modelling, Planning and Control"

### Python Libraries
- **Dash**: https://dash.plotly.com/
- **Plotly**: https://plotly.com/python/
- **NumPy**: https://numpy.org/doc/

---

## Getting Help

### File-Specific Questions
1. Identify the file in this index
2. Read the detailed documentation
3. Check "Related Files" section
4. Try the examples in the documentation

### General Questions
1. Start with main [README.md](../README.md)
2. Review troubleshooting sections
3. Check code comments
4. Consult MuJoCo documentation

---

## Contributing

When adding new files:
1. Create documentation in this directory
2. Update this INDEX.md
3. Update main [README.md](../README.md)
4. Follow existing documentation style
5. Include code examples
6. Add troubleshooting section

---

## Last Updated

Documentation created: 2026-02-11

**Total Documentation Pages**: 9
**Total Lines Documented**: ~1200+
