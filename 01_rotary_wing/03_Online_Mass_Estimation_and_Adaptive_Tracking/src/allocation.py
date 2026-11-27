"""Quadrotor wrench-to-rotor allocation."""

import numpy as np

from config import ARM_LENGTH, K_F, K_M, OMEGA_MAX


# Wrench ordering:
#
#     [total thrust, roll moment, pitch moment, yaw moment]
#
# Rotor thrust is proportional to omega^2.
#
# The matrix encodes the chosen rotor geometry and alternating rotor
# reaction-torque signs.
ALLOCATION_MATRIX = np.array([
    [
        K_F, K_F, K_F, K_F
    ],
    [
        K_F * ARM_LENGTH / np.sqrt(2),
        -K_F * ARM_LENGTH / np.sqrt(2),
        -K_F * ARM_LENGTH / np.sqrt(2),
        K_F * ARM_LENGTH / np.sqrt(2),
    ],
    [
        -K_F * ARM_LENGTH / np.sqrt(2),
        -K_F * ARM_LENGTH / np.sqrt(2),
        K_F * ARM_LENGTH / np.sqrt(2),
        K_F * ARM_LENGTH / np.sqrt(2),
    ],
    [
        K_M, -K_M, K_M, -K_M
    ],
])

ALLOCATION_INVERSE = np.linalg.inv(ALLOCATION_MATRIX)


def allocate_wrench(wrench):
    """
    Convert desired body wrench into rotor speeds.

    Saturation is applied to rotor squared-speed commands before the
    delivered wrench is reconstructed.  Therefore the plant receives
    the wrench that the saturated actuators can actually produce.
    """
    rotor_speed_squared = ALLOCATION_INVERSE @ wrench

    # Rotor thrust cannot be negative.
    rotor_speed_squared = np.clip(
        rotor_speed_squared,
        0.0,
        OMEGA_MAX**2,
    )

    rotor_speed = np.sqrt(rotor_speed_squared)

    # Reconstruct the wrench actually delivered after saturation.
    delivered_wrench = ALLOCATION_MATRIX @ rotor_speed_squared

    return rotor_speed, delivered_wrench
