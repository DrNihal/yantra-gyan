"""External force and moment disturbance model."""

import numpy as np

from config import DISTURBANCE_TIME


def external_disturbance(t):
    """
    Apply a smooth lateral force and a small body-moment disturbance.

    The disturbance starts after the payload event so that the
    experiment tests both parameter change and disturbance rejection.
    """
    if t < DISTURBANCE_TIME:
        return np.zeros(3), np.zeros(3)

    tau = t - DISTURBANCE_TIME

    force = np.array([
        0.45 * np.sin(0.65 * tau),
        0.35 * np.cos(0.50 * tau),
        0.0,
    ])

    moment = np.array([
        0.008 * np.sin(0.40 * t),
        0.006 * np.cos(0.35 * t),
        0.0,
    ])

    return force, moment
