"""Quadrotor control allocation."""

# Import NumPy for matrix construction and numerical operations.
import numpy as np


# Construct the allocation matrix for the selected X-configuration convention.
def allocation_matrix(vehicle):
    # Read the rotor arm length from the vehicle model.
    l = vehicle.arm

    # Read the thrust coefficient.
    kf = vehicle.kf

    # Read the reaction-torque coefficient.
    km = vehicle.km

    # Each rotor is l/sqrt(2) away from the body x/y axes in an X layout.
    a = l / np.sqrt(2.0)

    # Return the matrix mapping rotor thrusts to total thrust and moments.
    return np.array([
        [1, 1, 1, 1],
        [a, -a, -a, a],
        [-a, a, -a, a],
        [km / kf, km / kf, -km / kf, -km / kf],
    ], dtype=float)


# Convert desired total wrench into individual rotor speeds and thrusts.
def wrench_to_rotor_omega(wrench, vehicle):
    # Build the allocation matrix for this vehicle.
    M = allocation_matrix(vehicle)

    # Solve M*f = wrench for the four rotor thrusts.
    f = np.linalg.solve(M, wrench)

    # Negative thrust is not physically available, so clip it to zero.
    f = np.clip(f, 0.0, vehicle.kf * vehicle.max_omega**2)

    # Convert rotor thrust to angular speed using f = kf * omega^2.
    omega = np.sqrt(f / vehicle.kf)

    # Return both rotor speed and thrust for analysis.
    return omega, f
