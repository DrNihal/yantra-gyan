"""Quadrotor thrust/moment allocation.

The requested wrench is converted to rotor thrusts, limited by the rotor
speed constraint, and then mapped back to the actually delivered wrench.
"""

# NumPy is used to build and solve the allocation matrix.
import numpy as np


def allocation_matrix(vehicle):
    """Return the X-configuration mapping from rotor thrusts to body wrench."""

    # Read the arm length and rotor coefficients from the vehicle definition.
    l = vehicle.arm
    kf = vehicle.kf
    km = vehicle.km

    # In an X configuration, each rotor is l/sqrt(2) from the body axes.
    a = l / np.sqrt(2.0)

    # The matrix maps [f1,f2,f3,f4] to [T,tau_x,tau_y,tau_z].
    return np.array([
        [1, 1, 1, 1],
        [a, -a, -a, a],
        [-a, a, -a, a],
        [km / kf, km / kf, -km / kf, -km / kf],
    ], dtype=float)


def wrench_to_rotor_omega(wrench, vehicle):
    """Convert total thrust/moments into physically limited rotor speeds."""

    # Build the allocation matrix for the vehicle actuator geometry.
    M = allocation_matrix(vehicle)

    # Solve M*f = wrench for the four rotor thrusts.
    rotor_thrust = np.linalg.solve(M, wrench)

    # Rotor thrust cannot be negative.
    rotor_thrust = np.clip(
        rotor_thrust,
        0.0,
        vehicle.kf * vehicle.max_omega**2,
    )

    # Use f_i = k_f * omega_i^2 to convert thrust into rotor speed.
    rotor_omega = np.sqrt(rotor_thrust / vehicle.kf)

    # Return both quantities because both are useful for analysis.
    return rotor_omega, rotor_thrust


def rotor_thrust_to_wrench(rotor_thrust, vehicle):
    """Reconstruct the wrench actually delivered after actuator saturation."""
    M = allocation_matrix(vehicle)
    return M @ np.asarray(rotor_thrust, dtype=float)
