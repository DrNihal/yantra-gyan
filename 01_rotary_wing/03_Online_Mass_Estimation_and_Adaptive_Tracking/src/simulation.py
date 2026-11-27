"""Closed-loop simulation runner."""

import numpy as np

from config import (
    DT,
    T_FINAL,
    MASS_NOMINAL,
    VELOCITY_NOISE_STD,
    ACCEL_FILTER_TAU,
    RNG_SEED,
)
from trajectory import desired_trajectory
from rotations import desired_attitude
from controller import (
    translational_control,
    estimate_mass,
    geometric_attitude_control,
)
from allocation import allocate_wrench
from disturbance import external_disturbance
from dynamics import plant_parameters, state_derivative, integrate_state


def run_case(adaptive):
    """Run one complete closed-loop experiment."""
    time = np.arange(0.0, T_FINAL + DT, DT)
    n = len(time)

    position = np.zeros((n, 3))
    velocity = np.zeros((n, 3))
    rotation = np.zeros((n, 3, 3))
    angular_velocity = np.zeros((n, 3))

    desired_position = np.zeros((n, 3))
    position_error = np.zeros((n, 3))
    acceleration = np.zeros((n, 3))

    mass_actual = np.zeros(n)
    mass_estimate = np.zeros(n)
    rotor_speed = np.zeros((n, 4))
    thrust_command = np.zeros(n)
    attitude_error_norm = np.zeros(n)

    # Deterministic measurement noise makes the experiment reproducible.
    rng = np.random.default_rng(RNG_SEED)

    # Initial vehicle state is exactly on the desired trajectory.
    position[0], velocity[0], _, _ = desired_trajectory(0.0)
    rotation[0] = np.eye(3)

    # The estimator starts from the same nominal value used by the
    # fixed controller.
    current_mass_estimate = MASS_NOMINAL

    # Previous noisy velocity is required for numerical acceleration
    # estimation.
    previous_velocity_measurement = velocity[0].copy()
    acceleration_estimate = np.zeros(3)

    for k in range(n - 1):
        t = time[k]

        desired_position[k], desired_velocity, desired_acceleration, desired_yaw = (
            desired_trajectory(t)
        )

        # The plant parameters are the truth.  The controller only sees
        # current_mass_estimate.
        actual_mass, actual_inertia = plant_parameters(t)
        mass_actual[k] = actual_mass

        # Simulated velocity sensor.
        velocity_measurement = (
            velocity[k]
            + rng.normal(
                0.0,
                VELOCITY_NOISE_STD,
                3,
            )
        )

        # Numerical differentiation produces an acceleration estimate.
        raw_acceleration = (
            velocity_measurement
            - previous_velocity_measurement
        ) / DT

        # First-order low-pass filter reduces differentiation noise.
        alpha = DT / (ACCEL_FILTER_TAU + DT)
        acceleration_estimate += alpha * (
            raw_acceleration - acceleration_estimate
        )

        previous_velocity_measurement = velocity_measurement

        # Estimate the changing mass using the previous commanded thrust.
        if k == 0:
            previous_thrust = current_mass_estimate * 9.81
        else:
            previous_thrust = thrust_command[k - 1]

        current_mass_estimate = estimate_mass(
            current_mass_estimate,
            previous_thrust,
            acceleration_estimate,
            DT,
            adaptive=adaptive,
        )

        mass_estimate[k] = current_mass_estimate

        # Nonlinear translational control.
        desired_force = translational_control(
            position[k],
            velocity[k],
            desired_position[k],
            desired_velocity,
            desired_acceleration,
            current_mass_estimate,
        )

        # The desired force determines the required vehicle attitude.
        desired_rotation = desired_attitude(
            desired_force,
            desired_yaw,
        )

        # Geometric SO(3) attitude controller.
        desired_moment, attitude_error = geometric_attitude_control(
            rotation[k],
            angular_velocity[k],
            desired_rotation,
            actual_inertia,
        )

        # Convert desired force magnitude + moments into a rotor wrench.
        desired_wrench = np.r_[
            np.linalg.norm(desired_force),
            desired_moment,
        ]

        # Rotor allocation and saturation.
        # The returned delivered wrench is what the plant actually gets.
        rotor_speed[k], delivered_wrench = allocate_wrench(
            desired_wrench
        )

        thrust_command[k] = np.linalg.norm(desired_force)
        attitude_error_norm[k] = np.linalg.norm(attitude_error)

        # External force/moment disturbance.
        disturbance_force, disturbance_moment = external_disturbance(t)

        # Evaluate the true plant dynamics.
        derivative = state_derivative(
            position[k],
            velocity[k],
            rotation[k],
            angular_velocity[k],
            delivered_wrench,
            disturbance_force,
            disturbance_moment,
            actual_mass,
            actual_inertia,
        )

        # Integrate the actual plant state.
        (
            position[k + 1],
            velocity[k + 1],
            rotation[k + 1],
            angular_velocity[k + 1],
            acceleration[k],
        ) = integrate_state(
            position[k],
            velocity[k],
            rotation[k],
            angular_velocity[k],
            derivative,
            DT,
        )

        position_error[k] = desired_position[k] - position[k]

    # Final sample bookkeeping.
    desired_position[-1] = desired_trajectory(time[-1])[0]
    position_error[-1] = desired_position[-1] - position[-1]
    mass_actual[-1] = plant_parameters(time[-1])[0]
    mass_estimate[-1] = current_mass_estimate
    rotor_speed[-1] = rotor_speed[-2]
    thrust_command[-1] = thrust_command[-2]
    attitude_error_norm[-1] = attitude_error_norm[-2]
    acceleration[-1] = acceleration[-2]

    return {
        "time": time,
        "position": position,
        "velocity": velocity,
        "rotation": rotation,
        "angular_velocity": angular_velocity,
        "desired_position": desired_position,
        "position_error": position_error,
        "acceleration": acceleration,
        "mass_actual": mass_actual,
        "mass_estimate": mass_estimate,
        "rotor_speed": rotor_speed,
        "thrust_command": thrust_command,
        "attitude_error_norm": attitude_error_norm,
    }
