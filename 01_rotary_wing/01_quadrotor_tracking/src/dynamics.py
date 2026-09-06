"""6-DOF rigid-body quadrotor dynamics."""

# Import NumPy for vector and matrix calculations.
import numpy as np

# Import attitude kinematics and the Euler-to-rotation conversion.
from rotations import euler_to_R, W_euler


# Evaluate the derivative of the complete quadrotor state.
def quadrotor_rhs(t, state, vehicle, control, disturbance):
    # Read the actual plant mass.
    m = vehicle.mass

    # Read the actual plant inertia tensor.
    J = vehicle.J

    # Read gravitational acceleration.
    g = vehicle.gravity

    # Extract linear velocity from the state.
    v = state[3:6]

    # Extract Euler angles from the state.
    eta = state[6:9]

    # Extract body angular velocity from the state.
    omega = state[9:12]

    # Convert Euler angles into the inertial-to-body/body-to-inertial convention
    # used consistently by the simulation.
    R = euler_to_R(eta)

    # Read total commanded thrust.
    T = control["T"]

    # Read commanded body moments.
    tau = control["tau"]

    # Gravity points downward in the inertial frame.
    gravity = np.array([0.0, 0.0, -g])

    # The thrust vector is +body-z in this model; rotate it into inertial coordinates.
    thrust_inertial = R @ np.array([0.0, 0.0, T])

    # Newton's second law: acceleration = gravity + thrust/mass + disturbance/mass.
    v_dot = gravity + thrust_inertial / m + disturbance / m

    # Euler-angle rates follow eta_dot = W(eta) * omega.
    eta_dot = W_euler(eta) @ omega

    # Euler rigid-body equation gives angular acceleration.
    omega_dot = np.linalg.solve(
        J,
        tau - np.cross(omega, J @ omega),
    )

    # Return [position_dot, velocity_dot, attitude_dot, angular_velocity_dot].
    return np.r_[v, v_dot, eta_dot, omega_dot]
