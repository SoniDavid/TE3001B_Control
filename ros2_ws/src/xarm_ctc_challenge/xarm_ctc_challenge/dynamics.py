"""
Nominal rigid-body dynamics for xArm Lite 6.

Model:  M(q)*q_ddot + C(q,qd)*qd + G(q) + F(qd) = tau

Mass matrix:    M = sum_i m_i * J_Gi^T @ J_Gi  + I_diag_approx
Gravity:        G = sum_i m_i * J_Gi^T @ [0, 0, 9.81]
Coriolis:       C*qd via Christoffel symbols from dM/dq_k (numerical)
Friction:       F = B * qd  (viscous)

Link parameters are approximate nominal values for xArm Lite 6.
"""

import numpy as np
from .kinematics import com_jacobian

# ------------------------------------------------------------------
# Link physical parameters (approximate, xArm Lite 6 nominal)
# ------------------------------------------------------------------

# Link masses [kg]  (sum ≈ 3.5 kg, consistent with 4.45 kg arm weight)
LINK_MASSES = np.array([0.84, 1.02, 0.72, 0.62, 0.16, 0.14])

# COM position in each link's LOCAL frame [m]
LINK_COMS = [
    np.array([0.00, 0.00, 0.12]),   # Link 1: along z_1
    np.array([0.10, 0.00, 0.00]),   # Link 2: along x_2 (upper arm)
    np.array([0.04, 0.00, 0.00]),   # Link 3: along x_3 (forearm)
    np.array([0.00, 0.00, 0.11]),   # Link 4: along z_4
    np.array([0.00, 0.00, 0.00]),   # Link 5: at joint centre
    np.array([0.00, 0.00, 0.03]),   # Link 6: along z_6
]

# Small approximate rotational inertia contribution [kg*m²] (diagonal)
I_DIAG_APPROX = np.array([0.08, 0.08, 0.05, 0.03, 0.01, 0.005])

# Viscous friction coefficients [Nm/(rad/s)]
B_FRICTION = np.array([0.40, 0.40, 0.30, 0.20, 0.10, 0.10])

# Gravity direction in base frame (upward vector for potential energy gradient)
_G_VEC = np.array([0.0, 0.0, 9.81])


# ------------------------------------------------------------------
# Public API
# ------------------------------------------------------------------

def mass_matrix(q: np.ndarray) -> np.ndarray:
    """
    Compute 6x6 joint-space mass matrix M(q).

    M = sum_i  m_i * J_Gi^T @ J_Gi  + diag(I_approx)

    Args:
        q: (6,) joint angles [rad]

    Returns:
        M: (6, 6) symmetric positive-definite mass matrix [kg*m²]
    """
    M = np.diag(I_DIAG_APPROX.copy())
    for i in range(6):
        Jg = com_jacobian(q, i, LINK_COMS[i])
        M += LINK_MASSES[i] * (Jg.T @ Jg)
    return M


def gravity_torques(q: np.ndarray) -> np.ndarray:
    """
    Compute gravity torque vector G(q).

    G = sum_i  m_i * J_Gi^T @ [0, 0, 9.81]

    (positive G means joints must exert torque to hold against gravity)

    Args:
        q: (6,) joint angles [rad]

    Returns:
        G: (6,) gravity torques [Nm]
    """
    G = np.zeros(6)
    for i in range(6):
        Jg = com_jacobian(q, i, LINK_COMS[i])
        G += LINK_MASSES[i] * (Jg.T @ _G_VEC)
    return G


def coriolis_torques(q: np.ndarray, qd: np.ndarray) -> np.ndarray:
    """
    Compute Coriolis/centripetal vector C(q,qd)*qd using Christoffel symbols.

    Γ_{ijk} = 0.5 * (dM_{ij}/dq_k + dM_{ik}/dq_j - dM_{jk}/dq_i)
    (C*qd)_i = sum_{j,k}  Γ_{ijk} * qd_j * qd_k

    Args:
        q:  (6,) joint angles [rad]
        qd: (6,) joint velocities [rad/s]

    Returns:
        Cqd: (6,) Coriolis torques [Nm]
    """
    eps = 1e-5
    n = 6

    # dM/dq_k  for k = 0..5
    dM = np.zeros((n, n, n))
    for k in range(n):
        qp, qm = q.copy(), q.copy()
        qp[k] += eps
        qm[k] -= eps
        dM[:, :, k] = (mass_matrix(qp) - mass_matrix(qm)) / (2.0 * eps)

    # Christoffel summation
    Cqd = np.zeros(n)
    for i in range(n):
        for j in range(n):
            for k in range(n):
                gamma = 0.5 * (dM[i, j, k] + dM[i, k, j] - dM[j, k, i])
                Cqd[i] += gamma * qd[j] * qd[k]
    return Cqd


def friction_torques(qd: np.ndarray) -> np.ndarray:
    """
    Viscous friction torques F(qd) = B * qd.

    Args:
        qd: (6,) joint velocities [rad/s]

    Returns:
        F: (6,) friction torques [Nm]
    """
    return B_FRICTION * qd


def get_dynamics(q: np.ndarray, qd: np.ndarray):
    """
    Convenience: return (M, Cqd, G, F) for control law.

    Returns:
        M:   (6,6) mass matrix
        Cqd: (6,)  C(q,qd)*qd
        G:   (6,)  gravity torques
        F:   (6,)  friction torques
    """
    M = mass_matrix(q)
    Cqd = coriolis_torques(q, qd)
    G = gravity_torques(q)
    F = friction_torques(qd)
    return M, Cqd, G, F
