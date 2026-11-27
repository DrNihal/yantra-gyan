"""Reference trajectory generation."""

import numpy as np


def desired_trajectory(t: float):
    """
    Generate the desired position, velocity, acceleration and yaw.

    The reference is a smooth 3-D helix.  Keeping the trajectory
    analytical gives the controller access to exact p_d, v_d and a_d
    without introducing a separate trajectory-generation error.
    """
    radius = 2.0
    angular_rate = 0.18
    vertical_rate = 0.10

    # Desired inertial position.
    position = np.array([
        radius * np.cos(angular_rate * t),
        radius * np.sin(angular_rate * t),
        1.5 + vertical_rate * t,
    ])

    # First derivative of the reference position.
    velocity = np.array([
        -radius * angular_rate * np.sin(angular_rate * t),
        radius * angular_rate * np.cos(angular_rate * t),
        vertical_rate,
    ])

    # Second derivative of the reference position.
    acceleration = np.array([
        -radius * angular_rate**2 * np.cos(angular_rate * t),
        -radius * angular_rate**2 * np.sin(angular_rate * t),
        0.0,
    ])

    # A constant yaw keeps the experiment focused on translation,
    # mass adaptation and geometric attitude tracking.
    yaw = 0.0

    return position, velocity, acceleration, yaw
