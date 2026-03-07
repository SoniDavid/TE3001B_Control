"""
Forward kinematics and position Jacobian for xArm Lite 6.

Modified DH parameters (Craig 1989 convention):
  T_{i-1}^i = Rot_x(alpha_{i-1}) * Trans_x(a_{i-1}) * Rot_z(theta_i) * Trans_z(d_i)

Columns: [alpha_{i-1}, a_{i-1}, d_i, theta_offset_i]
"""

import numpy as np

DH = np.array([
    [0,            0,      0.2433,  0         ],  # Link 1
    [-np.pi / 2,   0,      0,      -np.pi / 2 ],  # Link 2
    [0,            0.2,    0,       0          ],  # Link 3
    [0,            0.087,  0.2276,  np.pi / 2  ],  # Link 4
    [-np.pi / 2,   0,      0,       0          ],  # Link 5
    [np.pi / 2,    0,      0.0615,  0          ],  # Link 6
])

# Joint names in MoveIt / xarm_ros2 order
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


def _dh_mat(alpha: float, a: float, d: float, theta: float) -> np.ndarray:
    """Single modified-DH homogeneous transform (4x4)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct,      -st,       0,      a      ],
        [st * ca,  ct * ca, -sa,    -sa * d ],
        [st * sa,  ct * sa,  ca,     ca * d ],
        [0,        0,         0,     1      ],
    ])


def _all_transforms(q: np.ndarray):
    """Return list of T_0^k for k=0..6 (length 7, index 0 = identity)."""
    Ts = [np.eye(4)]
    T = np.eye(4)
    for i in range(6):
        al, a, d, off = DH[i]
        T = T @ _dh_mat(al, a, d, q[i] + off)
        Ts.append(T.copy())
    return Ts


def forward_kinematics(q: np.ndarray):
    """
    Compute EE position from joint angles.

    Args:
        q: (6,) joint angles [rad]

    Returns:
        p: (3,) EE position in base frame [m]
        T: (4,4) full T_0^6
    """
    Ts = _all_transforms(q)
    return Ts[6][:3, 3].copy(), Ts[6]


def position_jacobian(q: np.ndarray) -> np.ndarray:
    """
    Compute 3x6 position Jacobian.

    For Craig's modified DH, joint i+1 rotates about z_i (z-axis of frame {i}),
    which lives in Ts[i] = T_0^i.
    Column i: J[:,i] = z_i x (p_EE - p_i)  where z_i, p_i come from Ts[i].

    Args:
        q: (6,) joint angles [rad]

    Returns:
        J: (3, 6) position Jacobian
    """
    Ts = _all_transforms(q)
    p_ee = Ts[6][:3, 3]
    J = np.zeros((3, 6))
    for i in range(6):
        z = Ts[i][:3, 2]
        p = Ts[i][:3, 3]
        J[:, i] = np.cross(z, p_ee - p)
    return J


def com_position(q: np.ndarray, link_idx: int, com_local: np.ndarray) -> np.ndarray:
    """
    Compute COM position of link_idx in base frame.

    Args:
        q: (6,) joint angles
        link_idx: 0-based link index (0=Link1 .. 5=Link6)
        com_local: (3,) COM in local frame of that link

    Returns:
        p_com: (3,) COM in base frame
    """
    Ts = _all_transforms(q)
    T = Ts[link_idx + 1]
    return (T[:3, :3] @ com_local + T[:3, 3])


def com_jacobian(q: np.ndarray, link_idx: int, com_local: np.ndarray) -> np.ndarray:
    """
    3x6 position Jacobian to the COM of link_idx.
    Columns for joints > link_idx are zero.

    Args:
        q: (6,) joint angles
        link_idx: 0-based link index
        com_local: (3,) COM in local frame

    Returns:
        J_com: (3, 6)
    """
    Ts = _all_transforms(q)
    T = Ts[link_idx + 1]
    p_com = T[:3, :3] @ com_local + T[:3, 3]
    J = np.zeros((3, 6))
    for i in range(link_idx + 1):
        z = Ts[i][:3, 2]
        p = Ts[i][:3, 3]
        J[:, i] = np.cross(z, p_com - p)
    return J
