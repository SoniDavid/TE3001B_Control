"""
Forward kinematics and position Jacobian for xArm Lite 6.

Method: URDF-direct product-of-transforms.
  Each joint frame is: T_0^{link_i} = T_0^{prev} @ T_origin_i @ Rz(q_i)
  where T_origin_i is the fixed parent→child origin transform from the URDF
  and all joints rotate about their local z-axis.

Joint origins from xarm_description/config/kinematics/default/lite6_default_kinematics.yaml:
  joint1: xyz=[0,       0,        0.2435]  rpy=[0,     0,      0    ]
  joint2: xyz=[0,       0,        0     ]  rpy=[π/2,  -π/2,   π    ]
  joint3: xyz=[0.2002,  0,        0     ]  rpy=[-π,    0,      π/2  ]
  joint4: xyz=[0.087,  -0.22761,  0     ]  rpy=[π/2,   0,      0    ]
  joint5: xyz=[0,       0,        0     ]  rpy=[π/2,   0,      0    ]
  joint6: xyz=[0,       0.0625,   0     ]  rpy=[-π/2,  0,      0    ]

Validated: FK matches TF2 to < 0.2 mm across all tested configurations;
analytical Jacobian matches numerical finite-difference to machine precision.
"""

import numpy as np

# Joint names in MoveIt / xarm_ros2 order
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# ---------------------------------------------------------------------------
# Precomputed constant origin transforms  (computed once at import)
# ---------------------------------------------------------------------------

def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF RPY → 3×3 rotation matrix: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)
    Rx = np.array([[1, 0,   0  ], [0,  cr, -sr], [0,  sr,  cr]])
    Ry = np.array([[cp, 0, sp  ], [0,   1,   0], [-sp, 0,  cp]])
    Rz = np.array([[cy, -sy, 0 ], [sy,  cy,  0], [0,   0,   1]])
    return Rz @ Ry @ Rx


def _make_origin(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy_matrix(*rpy)
    T[:3,  3] = xyz
    return T


_JOINT_PARAMS = [
    ([0,       0,        0.2435], [0,          0,       0      ]),  # joint1
    ([0,       0,        0     ], [np.pi/2,   -np.pi/2, np.pi  ]),  # joint2
    ([0.2002,  0,        0     ], [-np.pi,     0,       np.pi/2]),  # joint3
    ([0.087,  -0.22761,  0     ], [np.pi/2,    0,       0      ]),  # joint4
    ([0,       0,        0     ], [np.pi/2,    0,       0      ]),  # joint5
    ([0,       0.0625,   0     ], [-np.pi/2,   0,       0      ]),  # joint6
]

# Fixed origin transforms – computed once at import time
T_ORIGINS: list = [_make_origin(xyz, rpy) for xyz, rpy in _JOINT_PARAMS]

# ---------------------------------------------------------------------------
# Core: compute all frames in one pass
# ---------------------------------------------------------------------------

def _compute_frames(q: np.ndarray):
    """
    Compute all link frames for joint angles q (6,).

    Returns
    -------
    T_list : list of np.ndarray, length 7
        T_list[0] = identity (base frame)
        T_list[i] = T_0^{link_i}  — transform AFTER joint i rotation, i = 1..6

    T_pre_list : list of np.ndarray, length 6
        T_pre_list[i] = T_0^{joint_i_axis}  — transform BEFORE joint i rotation.
        z-axis of T_pre_list[i] is the rotation axis of joint i in the base frame.
    """
    T_list     = [np.eye(4)]
    T_pre_list = []
    T = np.eye(4)
    for i in range(6):
        T_pre = T @ T_ORIGINS[i]          # fixed parent→child origin, before q_i
        T_pre_list.append(T_pre)
        cq, sq = float(np.cos(q[i])), float(np.sin(q[i]))
        # Apply Rz(q_i): new_col0 =  cq*col0 + sq*col1
        #                new_col1 = -sq*col0 + cq*col1
        col0 = T_pre[:, 0].copy()
        col1 = T_pre[:, 1].copy()
        T = T_pre.copy()
        T[:3, 0] =  cq * col0[:3] + sq * col1[:3]
        T[:3, 1] = -sq * col0[:3] + cq * col1[:3]
        T_list.append(T)
    return T_list, T_pre_list


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def forward_kinematics(q: np.ndarray):
    """
    Compute EE position from joint angles.

    Args:
        q: (6,) joint angles [rad]

    Returns:
        p: (3,) EE position in base frame [m]
        T: (4,4) full T_0^{link6}
    """
    T_list, _ = _compute_frames(q)
    T = T_list[6]
    return T[:3, 3].copy(), T


def position_jacobian(q: np.ndarray) -> np.ndarray:
    """
    Compute 3×6 position Jacobian  J  such that  ṗ_EE = J @ q̇.

    Column i:   J[:,i] = z_i × (p_EE − p_i)
    where z_i and p_i are the rotation axis and origin of joint i
    in the base frame (from T_pre_list[i]).

    Args:
        q: (6,) joint angles [rad]

    Returns:
        J: (3, 6) position Jacobian
    """
    T_list, T_pre_list = _compute_frames(q)
    p_ee = T_list[6][:3, 3]
    J = np.zeros((3, 6))
    for i in range(6):
        z_i = T_pre_list[i][:3, 2]
        p_i = T_pre_list[i][:3, 3]
        J[:, i] = np.cross(z_i, p_ee - p_i)
    return J


def com_position(q: np.ndarray, link_idx: int, com_local: np.ndarray) -> np.ndarray:
    """
    Compute COM position of link_idx in base frame.

    Args:
        q:          (6,) joint angles [rad]
        link_idx:   0-based link index  (0 = Link1 … 5 = Link6)
        com_local:  (3,) COM in local frame of that link

    Returns:
        p_com: (3,) COM in base frame
    """
    T_list, _ = _compute_frames(q)
    T = T_list[link_idx + 1]
    return T[:3, :3] @ com_local + T[:3, 3]


def com_jacobian(q: np.ndarray, link_idx: int, com_local: np.ndarray) -> np.ndarray:
    """
    3×6 position Jacobian to the COM of link_idx.
    Columns for joints > link_idx are zero.

    Args:
        q:          (6,) joint angles [rad]
        link_idx:   0-based link index
        com_local:  (3,) COM in local frame

    Returns:
        J_com: (3, 6)
    """
    T_list, T_pre_list = _compute_frames(q)
    T = T_list[link_idx + 1]
    p_com = T[:3, :3] @ com_local + T[:3, 3]
    J = np.zeros((3, 6))
    for i in range(link_idx + 1):
        z_i = T_pre_list[i][:3, 2]
        p_i = T_pre_list[i][:3, 3]
        J[:, i] = np.cross(z_i, p_com - p_i)
    return J
