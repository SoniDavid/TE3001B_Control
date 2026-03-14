"""
PCB Component Placement Trajectory — relative Cartesian offsets from actual
starting EE position.

Design rationale
----------------
Absolute Cartesian waypoints fail in practice because the FK estimate of the
home position may differ from the robot's actual EE position, giving the IK
solver a large cold-start error that causes divergence and near-singularity
behaviour.  The working xarm_perturbations package solves this by using
RELATIVE offsets from wherever the arm happens to be at startup.  This
trajectory mirrors that approach.

The arm's starting position is captured once from the first /joint_states FK
and used as the trajectory centre for every trial.

Coordinate frame (xArm Lite 6 ROS base frame):
  +x — forward reach    +y — physical LEFT    +z — vertical UP

Motion pattern (pure axial moves — no diagonals, minimises singularity risk):
  Each site: descend ↓ (z only) → place [dwell] → ascend ↑ (z only)
  Lateral transits: at HIGH z (x or y only)

Visit order (physical RIGHT first to stay away from robot body):
  site_A (far-right) → site_B (near-right) → site_C (near-left) → site_D (far-left) → HOME

8 distinct positions (4 HIGH + 4 LOW), vertical separation = DZ = 0.090 m ✓

Offsets (from centre = actual starting EE position):
  DX  = ±0.040 m  forward / back
  DY  = ±0.045 m  left / right
  DZ  =  0.090 m  vertical drop
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple


def _step_axis_vector(axis: str) -> np.ndarray:
    axis = str(axis).lower()
    if axis == 'x':
        return np.array([1.0, 0.0, 0.0])
    if axis == 'y':
        return np.array([0.0, 1.0, 0.0])
    if axis == 'z':
        return np.array([0.0, 0.0, 1.0])
    raise ValueError(f"Unsupported step axis '{axis}'. Use x, y, or z.")

# ---------------------------------------------------------------------------
# Relative offset constants  [m]
# ---------------------------------------------------------------------------

DX = 0.040   # forward / back half-range
DY = 0.045   # left / right half-range
DZ = 0.090   # vertical drop from HIGH to LOW  (separation = DZ ✓ ≥ 0.08 m)

SEGMENT_SEC = 3.0   # transition time between waypoints [s]

# (dx, dy, dz_high=0 → dz_low=-DZ, dwell_high, dwell_low, label)
# Each pad is visited twice: HIGH (approach, short dwell) then LOW (place, longer dwell)
# A "depart" step (no dwell) returns to HIGH before the next lateral transit.
_SITES: List[Tuple] = [
    (+DX, -DY, "site_A"),   # far-right  (physical right first)
    (-DX, -DY, "site_B"),   # near-right
    (-DX, +DY, "site_C"),   # near-left  (side switch at HIGH)
    (+DX, +DY, "site_D"),   # far-left
]


def _build_waypoints(centre: np.ndarray) -> List[Tuple]:
    """
    Expand relative offsets into absolute (x, y, z, dwell_sec, label) tuples.

    Sequence per site:
      HIGH (approach, 1.5 s dwell) → LOW (place, 2.0 s dwell) → HIGH (depart, no dwell)
    Lateral moves happen between depart-HIGH and next approach-HIGH at constant z.

    Returns list of (x, y, z, dwell, label) — the full motion sequence.
    """
    cx, cy, cz = centre
    z_high = cz            # transit / approach height  = starting z
    z_low = cz - DZ      # placement height           = starting z − DZ

    wps = []
    for (dx, dy, label) in _SITES:
        wps.append((cx + dx, cy + dy, z_high, 1.5, f"{label}_high"))   # approach
        wps.append((cx + dx, cy + dy, z_low, 2.0, f"{label}_low"))    # place
        wps.append((cx + dx, cy + dy, z_high, 0.0, f"{label}_depart"))  # depart

    return wps


# ---------------------------------------------------------------------------
# Trajectory data class
# ---------------------------------------------------------------------------

@dataclass
class TrajPoint:
    p: np.ndarray   # (3,) desired position [m]
    pd: np.ndarray   # (3,) desired velocity [m/s]
    pdd: np.ndarray   # (3,) desired acceleration [m/s²]
    wp_idx: int
    label: str
    phase: str          # "segment" | "dwell"


def _quintic(tau: float) -> Tuple[float, float, float]:
    t2, t3, t4, t5 = tau**2, tau**3, tau**4, tau**5
    h = 10 * t3 - 15 * t4 + 6 * t5
    hd = 30 * t2 - 60 * t3 + 30 * t4
    hdd = 60 * tau - 180 * t2 + 120 * t3
    return h, hd, hdd


class PCBTrajectory:
    """
    Piecewise quintic trajectory centred on the actual robot EE position.

    Args:
        centre: (3,) actual end-effector position [m] obtained from FK at
                startup.  All waypoint offsets are relative to this point.
    """

    def __init__(self, centre: np.ndarray):
        self._centre = centre.copy()
        self._waypoints = _build_waypoints(centre)   # absolute (x,y,z,dwell,label)

        # Full point chain: centre → WP1 … WPN → centre
        self._pts = [centre] + [np.array(wp[:3]) for wp in self._waypoints] + [centre]
        self._dwells = [0.0] + [wp[3] for wp in self._waypoints] + [0.0]
        self._labels = ["home"] + [wp[4] for wp in self._waypoints] + ["home"]

        T = SEGMENT_SEC
        events = []
        t = 0.0
        for i in range(len(self._pts) - 1):
            events.append((t, t + T, "segment", i))
            t += T
            d = self._dwells[i + 1]
            if d > 0.0:
                events.append((t, t + d, "dwell", i + 1))
                t += d
        self._events = events
        self._total_time = t

    @property
    def home(self) -> np.ndarray:
        return self._centre.copy()

    @property
    def waypoints(self) -> List[Tuple]:
        """Absolute (x, y, z, dwell, label) tuples — for markers / metadata."""
        return self._waypoints

    @property
    def total_time(self) -> float:
        return self._total_time

    @property
    def path_points(self) -> List[np.ndarray]:
        return [p.copy() for p in self._pts]

    def at(self, t: float) -> TrajPoint:
        t = float(np.clip(t, 0.0, self._total_time))
        T = SEGMENT_SEC
        for (t0, t1, etype, idx) in self._events:
            if t <= t1:
                if etype == "segment":
                    pa = self._pts[idx]
                    pb = self._pts[idx + 1]
                    tau = float(np.clip((t - t0) / T, 0.0, 1.0))
                    h, hd, hdd = _quintic(tau)
                    p = pa + h * (pb - pa)
                    pd = (hd / T) * (pb - pa)
                    pdd = (hdd / T**2) * (pb - pa)
                    return TrajPoint(p, pd, pdd, idx, self._labels[idx + 1], "segment")
                else:
                    p = self._pts[idx].copy()
                    return TrajPoint(p, np.zeros(3), np.zeros(3),
                                     idx, self._labels[idx], "dwell")
        return TrajPoint(self._centre.copy(), np.zeros(3), np.zeros(3),
                         len(self._pts) - 1, "home", "dwell")


class StepTrajectory:
    """
    Hold the initial EE position, then apply a single Cartesian setpoint step.

    This is intended for step-response experiments. The reference jumps from the
    startup EE position to a constant offset target at `hold_before_s`, then
    remains there for `hold_after_s`.
    """

    def __init__(
        self,
        centre: np.ndarray,
        axis: str = 'x',
        step_m: float = 0.010,
        hold_before_s: float = 2.0,
        hold_after_s: float = 6.0,
    ):
        self._centre = centre.copy()
        self._axis = str(axis).lower()
        self._step_m = float(step_m)
        self._hold_before_s = max(0.0, float(hold_before_s))
        self._hold_after_s = max(0.1, float(hold_after_s))

        direction = _step_axis_vector(self._axis)
        self._target = self._centre + self._step_m * direction
        self._label = f"step_{self._axis}_{self._step_m:+.3f}m"
        self._waypoints = [
            (self._target[0], self._target[1], self._target[2],
             self._hold_after_s, self._label)
        ]
        self._total_time = self._hold_before_s + self._hold_after_s

    @property
    def home(self) -> np.ndarray:
        return self._centre.copy()

    @property
    def waypoints(self) -> List[Tuple]:
        return self._waypoints

    @property
    def total_time(self) -> float:
        return self._total_time

    @property
    def path_points(self) -> List[np.ndarray]:
        return [self._centre.copy(), self._target.copy()]

    def at(self, t: float) -> TrajPoint:
        t = float(np.clip(t, 0.0, self._total_time))
        if t < self._hold_before_s:
            return TrajPoint(
                self._centre.copy(), np.zeros(3), np.zeros(3),
                0, 'home', 'dwell')
        return TrajPoint(
            self._target.copy(), np.zeros(3), np.zeros(3),
            1, self._label, 'dwell')


def build_trajectory(
    centre: np.ndarray,
    mode: str = 'pcb',
    step_axis: str = 'x',
    step_m: float = 0.010,
    step_hold_before_s: float = 2.0,
    step_hold_after_s: float = 6.0,
):
    mode = str(mode).lower()
    if mode == 'pcb':
        return PCBTrajectory(centre=centre)
    if mode == 'step':
        return StepTrajectory(
            centre=centre,
            axis=step_axis,
            step_m=step_m,
            hold_before_s=step_hold_before_s,
            hold_after_s=step_hold_after_s,
        )
    raise ValueError(f"Unsupported trajectory_mode '{mode}'. Use 'pcb' or 'step'.")
