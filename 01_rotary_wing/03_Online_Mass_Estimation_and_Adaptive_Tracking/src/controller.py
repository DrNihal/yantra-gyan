"""Nonlinear translational, adaptive and geometric attitude control."""

import numpy as np

from config import (
    GRAVITY,
    ESTIMATOR_GAIN,
    MASS_MIN,
    MASS_MAX,
    KP,
    KD,
    K_R,
    K_OMEGA,
    FORCE_LIMIT,
)
from rotations import vee, desired_attitude


E3 = np.array([0.0, 0.0, 1.0])


def translational_control(
    position,
    velocity,
    desired_position,
    desired_velocity,
    desired_acceleration,
    mass_estimate,
):
    """
    Calculate the desired inertial force.

    Equation implemented:

        a_c = a_d + Kp(p_d - p) + Kd(v_d - v)

        F_d = m_hat (a_c + g e_3)

    The estimated mass is deliberately used here.  This is the
    mechanism through which parameter estimation affects control.
    """
    position_error = desired_position - position
    velocity_error = desired_velocity - velocity

    # Nonlinear feedback law operating directly on the translational
    # dynamics rather than on a linearized hover model.
    commanded_acceleration = (
        desired_acceleration
        + KP @ position_error
        + KD @ velocity_error
    )

    # Convert desired acceleration into the force required by the
    # current estimated vehicle mass, including gravity compensation.
    desired_force = mass_estimate * (
        commanded_acceleration + GRAVITY * E3
    )

    # Keep the requested force inside the simulation's actuator envelope.
    force_norm = np.linalg.norm(desired_force)
    if force_norm > FORCE_LIMIT:
        desired_force = desired_force * FORCE_LIMIT / force_norm

    return desired_force


def estimate_mass(
    previous_mass,
    thrust_previous,
    acceleration_estimate,
    dt,
    adaptive=True,
):
    """
    Update the online mass estimate.

    From vertical dynamics:

        T = m (a_z + g)

    therefore:

        m_inst = T / (a_z + g)

    A first-order adaptation law is used rather than feeding one noisy
    instantaneous estimate directly into the controller.
    """
    if not adaptive:
        return previous_mass

    denominator = acceleration_estimate[2] + GRAVITY

    # When the denominator is too small, the instantaneous estimate
    # would be numerically ill-conditioned.  In that situation, retain
    # the previous estimate instead of generating a bad parameter.
    if abs(denominator) <= 1.0:
        return previous_mass

    instantaneous_mass = thrust_previous / denominator

    # Bounds protect the controller from one pathological measurement.
    instantaneous_mass = np.clip(
        instantaneous_mass,
        MASS_MIN,
        MASS_MAX,
    )

    # First-order adaptation.
    updated_mass = previous_mass + dt * ESTIMATOR_GAIN * (
        instantaneous_mass - previous_mass
    )

    return float(np.clip(updated_mass, MASS_MIN, MASS_MAX))


def geometric_attitude_control(
    rotation,
    angular_velocity,
    desired_rotation,
    inertia,
):
    """
    Calculate the geometric SO(3) body-moment command.

    The attitude error is

        e_R = 1/2 vee(R_d^T R - R^T R_d).

    The nonlinear rigid-body coupling term Omega x J Omega is retained.
    """
    attitude_error = 0.5 * vee(
        desired_rotation.T @ rotation
        - rotation.T @ desired_rotation
    )

    desired_angular_velocity = np.zeros(3)

    moment = (
        -K_R @ attitude_error
        -K_OMEGA @ (
            angular_velocity
            - rotation.T @ desired_rotation @ desired_angular_velocity
        )
        + np.cross(angular_velocity, inertia @ angular_velocity)
    )

    return moment, attitude_error
