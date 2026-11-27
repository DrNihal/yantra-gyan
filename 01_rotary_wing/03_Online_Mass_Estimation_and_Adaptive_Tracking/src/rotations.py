"""SO(3) rotation utilities."""

import numpy as np


def hat(vector: np.ndarray) -> np.ndarray:
    """Return the skew matrix such that hat(v) @ w = v x w."""
    x, y, z = vector
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ])


def vee(skew_matrix: np.ndarray) -> np.ndarray:
    """Return the vector represented by a 3x3 skew-symmetric matrix."""
    return np.array([
        skew_matrix[2, 1],
        skew_matrix[0, 2],
        skew_matrix[1, 0],
    ])


def project_to_so3(matrix: np.ndarray) -> np.ndarray:
    """
    Project a numerically drifted matrix back onto SO(3).

    Numerical integration can make R^T R slightly different from I.
    The SVD projection removes that accumulated numerical drift.
    """
    U, _, Vt = np.linalg.svd(matrix)
    rotation = U @ Vt

    # SVD can produce a reflection.  Force determinant +1.
    if np.linalg.det(rotation) < 0.0:
        U[:, -1] *= -1.0
        rotation = U @ Vt

    return rotation


def desired_attitude(force_vector: np.ndarray, yaw: float) -> np.ndarray:
    """
    Construct R_d from the desired force direction and desired yaw.

    The body z-axis is aligned with the required force direction.
    """
    force_norm = np.linalg.norm(force_vector)

    if force_norm < 1e-9:
        body_z = np.array([0.0, 0.0, 1.0])
    else:
        body_z = force_vector / force_norm

    # Horizontal yaw reference.
    yaw_axis = np.array([
        np.cos(yaw),
        np.sin(yaw),
        0.0,
    ])

    # Construct the second body axis orthogonal to body_z.
    body_y = np.cross(body_z, yaw_axis)

    # Guard against an ill-conditioned cross product.
    if np.linalg.norm(body_y) < 1e-9:
        body_y = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
    else:
        body_y /= np.linalg.norm(body_y)

    # Complete the right-handed orthonormal basis.
    body_x = np.cross(body_y, body_z)
    body_x /= np.linalg.norm(body_x)

    return np.column_stack((body_x, body_y, body_z))
