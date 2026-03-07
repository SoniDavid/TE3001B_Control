# Inspection Trajectory — CMM Surface Probe Simulation

## Real-World Application

The trajectory simulates a **robotic arm performing automated dimensional inspection on a CNC-machined part**. The end-effector represents a contact or non-contact measurement probe (touch-trigger probe, laser triangulator, or ultrasonic sensor) that must be positioned at specific feature points with a precise standoff distance.

This is the operating principle of industrial **Coordinate Measuring Machines (CMMs)** and inline quality control robots used in:
- Aerospace component inspection (turbine blades, engine casings)
- Automotive part verification (cylinder heads, transmission housings)
- Electronics PCB inspection (via-hole depth, pad presence)
- Medical device manufacturing (implant geometry verification)

The robot visits **3 inspection regions** on the part, with two features per region inspected sequentially at low height (the probe stays at measurement standoff while scanning adjacent features — exactly how a real CMM probes a cluster of holes on the same face). Between regions the arm rises to a safe transit height to avoid collisions with part features.

---

## Waypoint Table

All positions are **relative to `center`**, which is set from the robot's actual end-effector position at node startup. This makes the trajectory portable — it executes correctly regardless of where the robot is when the node launches.

| WP | Label | Δx (m) | Δy (m) | Δz (m) | Height | Purpose | Dwell |
|---|---|---|---|---|---|---|---|
| 1 | `transit_A` | −0.09 | +0.07 | 0.00 | High | Approach region A from above | — |
| 2 | `inspect_F1` | −0.09 | +0.07 | **−0.10** | **Low** | Probe bore F1 | **1.5 s** |
| 3 | `inspect_F2` | +0.01 | +0.09 | **−0.10** | **Low** | Probe bore F2 — *lateral move, stays low* | **1.5 s** |
| 4 | `transit_B` | +0.10 | 0.00 | 0.00 | High | Ascend + transit to region B | — |
| 5 | `inspect_F3` | +0.10 | 0.00 | **−0.10** | **Low** | Probe bore F3 | **1.5 s** |
| 6 | `inspect_F4` | +0.06 | −0.08 | **−0.10** | **Low** | Probe bore F4 — *lateral move, stays low* | **1.5 s** |
| 7 | `transit_C` | −0.06 | −0.05 | 0.00 | High | Ascend + transit to region C | — |
| 8 | `inspect_F5` | −0.06 | −0.05 | **−0.10** | **Low** | Surface flatness scan point | **1.5 s** |
| 9 | `home` | 0.00 | 0.00 | 0.00 | High | Return to center | — |

**Z separation**: 0.10 m (> 0.08 m minimum requirement)

---

## Path Geometry

```
Top view (XY plane, relative to center):

        Y
        ^
        |
  F2 ● |     ● F1
  (+.01,+.09) (-0.09,+.07)
        |
        |  CENTER
        |
  F3 ●-+--              ← region B, rightmost
  (+.10, 0)
        |
     F4 ●
   (+.06,-0.08)
        |
   F5 ●
  (-0.06,-0.05)
        +─────────────> X

Arrows show traversal order:
  center → WP1 → WP2 ──lateral──> WP3 → WP4 → WP5 ──lateral──> WP6 → WP7 → WP8 → home
```

```
Side view (Z over time):

  z_high ──●──────────────●────────────────●────────────●──
           WP1            WP4              WP7           home
            │              │                │
  z_low     ●──●           ●──●             ●
           WP2 WP3        WP5 WP6          WP8
         [F1][F2]lat    [F3][F4]lat       [F5]
```

---

## Key Design Decisions

### Lateral moves at low height (WP2→WP3 and WP5→WP6)

This is the most significant complexity factor. When two features lie on the same surface at the same Z level, a real CMM does not retract between them — it traverses laterally at measurement height. This:

1. **Reduces cycle time** (no unnecessary Z motion between adjacent features)
2. **Creates a more demanding control scenario**: the controller must simultaneously track X, Y, and Z while maintaining the low standoff height. Any Z error at this phase would mean the probe contacts the part or loses signal.
3. **Provides richer data for perturbation analysis**: perturbations during lateral low-height moves are more critical than during transit.

### Asymmetric feature layout

The 5 features (F1–F5) are not arranged on a regular grid. This reflects the reality of machined parts where features are located at functionally determined positions, not at evenly spaced intervals. The asymmetric layout means the robot visits all four quadrants of the XY plane (approximately) and makes diagonal moves, testing the controller across a variety of trajectory directions.

### Three regions

Grouping features into 3 regions (A: F1+F2, B: F3+F4, C: F5) creates natural structure in the data:
- Region A: upper-left cluster, visited with a diagonal lateral move
- Region B: right-side cluster, diagonal lateral move downward
- Region C: lower-left single feature, standalone inspection

---

## Timing

With default parameters (`segment_sec=2.0`, `dwell_sec=1.5`):

| Phase | Count | Duration each | Total |
|---|---|---|---|
| Segments (transitions) | 9 | 2.0 s | 18.0 s |
| Dwells (inspections) | 5 | 1.5 s | 7.5 s |
| **Full cycle** | | | **~25.5 s** |

One cycle = all 9 waypoints visited and returned to center. With `loop_traj=true` (default), the trajectory repeats continuously.

---

## Comparison with Lab 4.2 Figure-Eight

| Aspect | Lab 4.2 figure-eight | Inspection trajectory |
|---|---|---|
| Shape | Continuous smooth curve | Discrete waypoints + interpolation |
| Z motion | Fixed Z throughout | Alternates between 2 Z levels |
| Controller demand | Smooth, predictable tracking | Variable: diagonal transitions + precision holds at low Z |
| Dwell periods | None | 5 × 1.5 s inspection holds |
| Real-world story | Generic motion | CMM part inspection |
| Controller | Same Cartesian PD | Same Cartesian PD |
| Interface | Same TwistStamped → MoveIt | Same |
| Perturbation sensitivity | Uniform across path | Highest during low-Z lateral moves |

---

## RMSE Interpretation

- **Segment RMSE**: tracking error during interpolated transitions — shows how well the PD controller follows smooth references
- **Dwell RMSE**: steady-state error while holding at inspection point — shows positioning accuracy and disturbance rejection
- The CSV column `phase` (`"segment"` or `"dwell"`) allows separating these two regimes in post-processing
