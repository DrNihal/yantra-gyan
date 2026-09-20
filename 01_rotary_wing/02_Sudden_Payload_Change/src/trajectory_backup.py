"""Desired 3-D trajectory for Yantra Gyan #02."""

# NumPy is used for trigonometric functions and vector construction.
import numpy as np

# Import the trajectory parameters from the central configuration file.
from config import HELIX_RADIUS as R, HELIX_RATE as w, CLIMB_RATE as vz, Z0


def desired_trajectory(t):
    """Return desired position, velocity, acceleration, yaw and yaw rate."""

    # Desired x position: a circular path shifted so x(0) = 0.
    x = R * (np.cos(w * t) - 1.0)

    # Desired y position: circular motion in the horizontal plane.
    y = R * np.sin(w * t)

    # Desired z position: constant-rate climb.
    z = Z0 + vz * t

    # Differentiate x(t) to obtain the desired x velocity.
    vx = -R * w * np.sin(w * t)

    # Differentiate y(t) to obtain the desired y velocity.
    vy = R * w * np.cos(w * t)

    # The desired vertical velocity is constant.
    vz_d = vz

    # Differentiate the horizontal velocities to obtain acceleration.
    ax = -R * w**2 * np.cos(w * t)
    ay = -R * w**2 * np.sin(w * t)

    # Constant climb rate means zero vertical acceleration.
    az = 0.0

    # Point the desired heading approximately along the direction of travel.
    psi = w * t + 0.5 * np.pi

    # The derivative of the desired heading is constant.
    psi_dot = w

    # Return all trajectory quantities required by the controller.
    return (
        np.array([x, y, z]),
        np.array([vx, vy, vz_d]),
        np.array([ax, ay, az]),
        psi,
        psi_dot,
    )
