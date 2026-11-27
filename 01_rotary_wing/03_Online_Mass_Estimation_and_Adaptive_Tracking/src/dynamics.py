"""6-DOF nonlinear quadrotor plant dynamics."""

import numpy as np

from config import (
    GRAVITY,
    MASS_INITIAL,
    MASS_PAYLOAD,
    PAYLOAD_TIME,
    INERTIA_INITIAL,
    INERTIA_PAYLOAD,
)
from rotations import hat, project_to_so3


E3 = np.array([0.0, 0.0, 1.0])


def plant_parameters(t):
    """Return the actual plant mass and inertia at time t."""
    if t < PAYLOAD_TIME:
        return MASS_INITIAL, INERTIA_INITIAL

    return MASS_PAYLOAD, INERTIA_PAYLOAD


def state_derivative(
    position,
    velocity,
    rotation,
    angular_velocity,
    delivered_wrench,
    disturbance_force,
    disturbance_moment,
    mass,
    inertia,
):
    """
    Evaluate the nonlinear 6-DOF rigid-body equations.

    Translation:

        m v_dot = R [0,0,T]^T + d_F - m g e_3

    Rotation:

        J Omega_dot =
            tau + d_tau - Omega x (J Omega)

    Position and attitude kinematics:

        p_dot = v
        R_dot = R hat(Omega)
    """
    total_thrust = delivered_wrench[0]
    delivered_moment = delivered_wrench[1:4]

    # Rotor thrust acts along the body z-axis and is rotated into
    # the inertial frame by R.
    thrust_body = np.array([0.0, 0.0, total_thrust])
    thrust_inertial = rotation @ thrust_body

    # Nonlinear translational acceleration.
    acceleration = (
        thrust_inertial
        + disturbance_force
        - mass * GRAVITY * E3
    ) / mass

    # Nonlinear rigid-body rotational acceleration.
    angular_acceleration = np.linalg.solve(
        inertia,
        delivered_moment
        + disturbance_moment
        - np.cross(
            angular_velocity,
            inertia @ angular_velocity,
        ),
    )

    return velocity, acceleration, rotation @ hat(angular_velocity), angular_acceleration


def integrate_state(
    position,
    velocity,
    rotation,
    angular_velocity,
    derivative,
    dt,
):
    """Perform one explicit integration step and re-project R onto SO(3)."""
    position_dot, acceleration, rotation_dot, angular_acceleration = derivative

    position_next = position + velocity * dt
    velocity_next = velocity + acceleration * dt

    # Second-order rotation update improves the local approximation to
    # exp(hat(Omega)*dt) without requiring a matrix exponential.
    rotation_increment = (
        np.eye(3)
        + hat(angular_velocity) * dt
        + 0.5 * hat(angular_velocity) @ hat(angular_velocity) * dt**2
    )

    rotation_next = project_to_so3(rotation @ rotation_increment)
    angular_velocity_next = angular_velocity + angular_acceleration * dt

    return (
        position_next,
        velocity_next,
        rotation_next,
        angular_velocity_next,
        acceleration,
    )
