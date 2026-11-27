"""6-DOF nonlinear quadrotor dynamics with a time-varying actual plant."""

# NumPy is used for vector and matrix operations.
import numpy as np

# Import the attitude kinematics and rotation conversion.
from rotations import euler_to_R, W_euler

# Import the payload timing and actual plant definitions.
from config import ACTUAL_NOMINAL, ACTUAL_PAYLOAD, PAYLOAD_TIME


def actual_vehicle_at_time(t, payload_enabled):
    """Return the actual plant parameters at time t."""

    # Before the payload event, the physical vehicle is the nominal vehicle.
    if not payload_enabled or t <= PAYLOAD_TIME:
        return ACTUAL_NOMINAL

    # After the event, the actual mass and inertia have changed.
    return ACTUAL_PAYLOAD


def quadrotor_rhs(t, state, payload_enabled, control, disturbance):
    """Evaluate the nonlinear 6-DOF plant equations."""

    # Select the actual physical plant at this instant.
    vehicle = actual_vehicle_at_time(t, payload_enabled)

    # Read the actual mass and inertia. The controller does not receive these.
    m = vehicle.mass
    J = vehicle.J
    g = vehicle.gravity

    # Extract inertial velocity from the state.
    v = state[3:6]

    # Extract Euler attitude.
    eta = state[6:9]

    # Extract body angular velocity.
    omega = state[9:12]

    # Convert Euler angles into the body-to-inertial rotation matrix.
    R = euler_to_R(eta)

    # Use the wrench actually delivered by the saturated rotor commands.
    # This is essential: actuator saturation must affect the plant dynamics.
    T = control["T_delivered"]
    tau = control["tau_delivered"]

    # Gravity acts downward in the inertial frame.
    gravity = np.array([0.0, 0.0, -g])

    # The rotors generate thrust along the vehicle body-z axis.
    thrust_inertial = R @ np.array([0.0, 0.0, T])

    # Newton's second law includes the external disturbance force.
    v_dot = gravity + thrust_inertial / m + disturbance / m

    # Euler-angle kinematics convert body rates into Euler-angle rates.
    eta_dot = W_euler(eta) @ omega

    # Euler's rigid-body equation retains the nonlinear gyroscopic coupling.
    omega_dot = np.linalg.solve(
        J,
        tau - np.cross(omega, J @ omega),
    )

    # Return the complete state derivative.
    return np.r_[v, v_dot, eta_dot, omega_dot]
