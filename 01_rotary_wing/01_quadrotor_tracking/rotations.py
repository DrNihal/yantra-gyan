"""Rotation and Euler-angle utility functions."""

# Import NumPy for trigonometric functions and matrix operations.
import numpy as np


# Rotation matrix for a roll rotation about body/inertial x.
def Rx(roll):
    # Calculate cosine and sine once to keep the matrix readable.
    c, s = np.cos(roll), np.sin(roll)

    # Return the standard x-axis rotation matrix.
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c],
    ])


# Rotation matrix for a pitch rotation about y.
def Ry(pitch):
    # Calculate cosine and sine once.
    c, s = np.cos(pitch), np.sin(pitch)

    # Return the standard y-axis rotation matrix.
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c],
    ])


# Rotation matrix for a yaw rotation about z.
def Rz(yaw):
    # Calculate cosine and sine once.
    c, s = np.cos(yaw), np.sin(yaw)

    # Return the standard z-axis rotation matrix.
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ])


# Construct the complete ZYX rotation matrix.
def euler_to_R(eta):
    # Extract roll, pitch and yaw from the attitude vector.
    phi, theta, psi = eta

    # Apply yaw, then pitch, then roll according to the ZYX convention.
    return Rz(psi) @ Ry(theta) @ Rx(phi)


# Convert body angular velocity to ZYX Euler-angle rates.
def W_euler(eta):
    # Extract roll and pitch; yaw is not needed in this kinematic matrix.
    phi, theta, _ = eta

    # Calculate sine/cosine terms once.
    cphi, sphi = np.cos(phi), np.sin(phi)

    # Calculate cosine and tangent of pitch.
    ctheta = np.cos(theta)
    ttheta = np.tan(theta)

    # Return the matrix satisfying eta_dot = W(eta) * omega.
    return np.array([
        [1.0, sphi * ttheta, cphi * ttheta],
        [0.0, cphi, -sphi],
        [0.0, sphi / ctheta, cphi / ctheta],
    ])


# Convert a rotation matrix back to ZYX Euler angles.
def R_to_euler(R):
    # Recover pitch using the ZYX relationship R[2,0] = -sin(theta).
    theta = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))

    # Recover roll from the remaining matrix elements.
    phi = np.arctan2(R[2, 1], R[2, 2])

    # Recover yaw from the first two elements of the first column.
    psi = np.arctan2(R[1, 0], R[0, 0])

    # Return the three Euler angles.
    return np.array([phi, theta, psi])


# Wrap an angle to the interval [-pi, pi).
def wrap_angle(a):
    # Add pi, take modulo 2*pi, and subtract pi.
    return (a + np.pi) % (2 * np.pi) - np.pi
