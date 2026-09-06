"""Desired 3-D trajectory for Yantra Gyan #01."""

# Import NumPy for trigonometric functions and vector creation.
import numpy as np

# Import the trajectory constants defined in config.py.
from config import HELIX_RADIUS as R, HELIX_RATE as w, CLIMB_RATE as vz, Z0


# Define the desired trajectory as a function of time.
def desired_trajectory(t):
    # Desired x position: a circle shifted so the trajectory starts at x = 0.
    x = R * (np.cos(w * t) - 1.0)

    # Desired y position: circular motion in the horizontal plane.
    y = R * np.sin(w * t)

    # Desired z position: constant-rate climb.
    z = Z0 + vz * t

    # Desired x velocity obtained by differentiating x(t).
    vx = -R * w * np.sin(w * t)

    # Desired y velocity obtained by differentiating y(t).
    vy = R * w * np.cos(w * t)

    # Desired vertical velocity is constant.
    vz_d = vz

    # Desired x acceleration obtained by differentiating vx(t).
    ax = -R * w**2 * np.cos(w * t)

    # Desired y acceleration obtained by differentiating vy(t).
    ay = -R * w**2 * np.sin(w * t)

    # The desired vertical velocity is constant, so vertical acceleration is zero.
    az = 0.0

    # Make vehicle heading follow the direction of horizontal travel.
    psi = w * t + 0.5 * np.pi

    # The desired yaw rate is therefore constant.
    psi_dot = w

    # Return position, velocity, acceleration, yaw and yaw rate.
    return (
        np.array([x, y, z]),
        np.array([vx, vy, vz_d]),
        np.array([ax, ay, az]),
        psi,
        psi_dot,
    )
