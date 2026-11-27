"""Rotation and Euler-angle utility functions."""

# NumPy provides trigonometric and matrix operations.
import numpy as np


def Rx(roll):
    # Compute sine and cosine once so the rotation matrix remains readable.
    c, s = np.cos(roll), np.sin(roll)

    # Return the standard rotation matrix about the x-axis.
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c],
    ])


def Ry(pitch):
    # Compute sine and cosine once.
    c, s = np.cos(pitch), np.sin(pitch)

    # Return the standard rotation matrix about the y-axis.
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c],
    ])


def Rz(yaw):
    # Compute sine and cosine once.
    c, s = np.cos(yaw), np.sin(yaw)

    # Return the standard rotation matrix about the z-axis.
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ])


def euler_to_R(eta):
    # Extract roll, pitch and yaw from the ZYX Euler-angle vector.
    phi, theta, psi = eta

    # Compose yaw, pitch and roll to obtain the body-to-inertial rotation.
    return Rz(psi) @ Ry(theta) @ Rx(phi)


def W_euler(eta):
    # Extract roll and pitch; yaw does not appear in this kinematic matrix.
    phi, theta, _ = eta

    # Pre-compute the required trigonometric quantities.
    cphi, sphi = np.cos(phi), np.sin(phi)
    ctheta = np.cos(theta)
    ttheta = np.tan(theta)

    # Protect the simulation from division by an Euler-angle singularity.
    if abs(ctheta) < 1e-6:
        raise FloatingPointError("Euler-angle pitch is too close to ±90 degrees.")

    # Return eta_dot = W(eta) * omega for the ZYX convention.
    return np.array([
        [1.0, sphi * ttheta, cphi * ttheta],
        [0.0, cphi, -sphi],
        [0.0, sphi / ctheta, cphi / ctheta],
    ])
