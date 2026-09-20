"""Fixed-parameter nonlinear geometric controller for Yantra Gyan #02.

The controller is intentionally unaware of the payload event. This allows the
experiment to isolate the effect of changing plant parameters.
"""

# NumPy provides vectors, matrices and cross products used by the controller.
import numpy as np

# Import the desired reference trajectory.
from trajectory import desired_trajectory

# Import Euler-to-rotation conversion for the actual vehicle attitude.
from rotations import euler_to_R

# Import physical actuator allocation.
from allocation import wrench_to_rotor_omega, rotor_thrust_to_wrench

# Import the parameters known by the controller.
from config import CONTROLLER_VEHICLE, KP, KV, KR, KOMEGA


def hat(v):
    """Return the skew-symmetric matrix associated with a 3-vector."""

    # The hat map converts a vector into a matrix that represents a cross product.
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


def vee(S):
    """Return the vector represented by a skew-symmetric matrix."""

    # Extract the three independent elements of the skew-symmetric matrix.
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def force_command(t, p, v):
    """Compute the nonlinear translational force command."""

    # The controller intentionally uses its fixed nominal mass.
    m = CONTROLLER_VEHICLE.mass

    # Read gravity from the controller model.
    g = CONTROLLER_VEHICLE.gravity

    # Obtain the desired position, velocity and acceleration.
    pd, vd, ad, _, _ = desired_trajectory(t)

    # Position error is actual minus desired position.
    ep = p - pd

    # Velocity error is actual minus desired velocity.
    ev = v - vd

    # Nonlinear feedback-acceleration command.
    ac = ad - KP @ ep - KV @ ev

    # Convert the desired acceleration into an inertial-frame force.
    Fc = m * (ac + np.array([0.0, 0.0, g]))

    # Return force and errors for use by the attitude loop and logging.
    return Fc, ep, ev


def desired_rotation_from_force(Fc, psi_d):
    """Construct desired attitude from desired force direction and yaw."""

    # The desired body-z axis must align with the requested force direction.
    force_norm = np.linalg.norm(Fc)
    if force_norm < 1e-8:
        # This fallback avoids division by zero in pathological conditions.
        b3 = np.array([0.0, 0.0, 1.0])
    else:
        b3 = Fc / force_norm

    # Build a horizontal reference vector from the desired yaw.
    b1c = np.array([np.cos(psi_d), np.sin(psi_d), 0.0])

    # Use a cross product to obtain the desired body-y direction.
    b2 = np.cross(b3, b1c)
    n = np.linalg.norm(b2)

    # If the vectors are nearly parallel, use a safe horizontal fallback.
    if n < 1e-8:
        b1c = np.array([1.0, 0.0, 0.0])
        b2 = np.cross(b3, b1c)
        n = np.linalg.norm(b2)

    # Normalize the desired body-y direction.
    b2 /= n

    # Complete the right-handed orthonormal frame.
    b1 = np.cross(b2, b3)

    # Assemble the desired rotation matrix from its three body axes.
    return np.column_stack((b1, b2, b3))


def desired_attitude_kinematics(t, p, v):
    """Numerically obtain desired angular velocity and acceleration."""

    # A small time step is used only for differentiating the desired attitude.
    h = 2e-4

    # Calculate the current desired force and desired yaw.
    Fc, _, _ = force_command(t, p, v)
    _, _, _, psi, _ = desired_trajectory(t)

    # Construct the desired current rotation matrix.
    Rd = desired_rotation_from_force(Fc, psi)

    # Evaluate the force and yaw slightly ahead of the current time.
    Fcp, _, _ = force_command(t + h, p, v)
    _, _, _, psip, _ = desired_trajectory(t + h)

    # Evaluate the force and yaw slightly behind the current time.
    tm = max(0.0, t - h)
    Fcm, _, _ = force_command(tm, p, v)
    _, _, _, psim, _ = desired_trajectory(tm)

    # Construct future and past desired attitudes.
    Rdp = desired_rotation_from_force(Fcp, psip)
    Rdm = desired_rotation_from_force(Fcm, psim)

    # Use a one-sided derivative at t=0 because a negative time is not needed.
    if t < h:
        Rdot = (Rdp - Rd) / h
    else:
        # Use a centered finite difference away from the initial point.
        Rdot = (Rdp - Rdm) / (2.0 * h)

    # Use a centered second derivative of the desired rotation.
    Rddot = (Rdp - 2.0 * Rd + Rdm) / (h * h)

    # Map Rdot to the desired body angular velocity skew matrix.
    Omega_hat = Rd.T @ Rdot

    # Extract the desired angular velocity vector.
    omega_d = vee(0.5 * (Omega_hat - Omega_hat.T))

    # Differentiate the body-frame desired angular velocity matrix.
    Omega_hat_dot = Rdot.T @ Rdot + Rd.T @ Rddot

    # Extract desired angular acceleration.
    omega_dot_d = vee(0.5 * (Omega_hat_dot - Omega_hat_dot.T))

    # Return all desired attitude quantities.
    return Rd, omega_d, omega_dot_d


def controller(t, state):
    """Compute thrust, moments and rotor commands from the current state."""

    # The controller keeps using nominal inertia even after the payload event.
    J = CONTROLLER_VEHICLE.J

    # Extract position from the 12-state vector.
    p = state[:3]

    # Extract inertial velocity.
    v = state[3:6]

    # Convert actual Euler angles to the current rotation matrix.
    R = euler_to_R(state[6:9])

    # Extract body angular velocity.
    omega = state[9:12]

    # Generate desired force and translational tracking errors.
    Fc, ep, ev = force_command(t, p, v)

    # Generate desired attitude and desired angular-rate quantities.
    Rd, omega_d, omega_dot_d = desired_attitude_kinematics(t, p, v)

    # Project the desired inertial force onto the current body-z axis.
    # This is the total thrust the current attitude can generate.
    T = float(Fc @ R[:, 2])

    # A rotor can only generate positive thrust.
    T = max(0.0, T)

    # Geometric attitude error on SO(3).
    eR = 0.5 * vee(Rd.T @ R - R.T @ Rd)

    # Transport desired angular velocity into the current body frame.
    transport = R.T @ Rd @ omega_d

    # Angular-rate tracking error in the current body frame.
    eOmega = omega - transport

    # Nonlinear geometric moment command.
    tau = (
        -KR @ eR
        -KOMEGA @ eOmega
        + np.cross(omega, J @ omega)
        - J @ (hat(omega) @ transport - R.T @ Rd @ omega_dot_d)
    )

    # Combine total thrust and body moments into the requested wrench.
    wrench_commanded = np.r_[T, tau]

    # Convert the requested wrench into four rotor commands with physical saturation.
    rotor_omega, rotor_thrust = wrench_to_rotor_omega(
        wrench_commanded,
        CONTROLLER_VEHICLE,
    )

    # Reconstruct the wrench actually delivered by the saturated rotors.
    wrench_delivered = rotor_thrust_to_wrench(
        rotor_thrust,
        CONTROLLER_VEHICLE,
    )

    # Return commanded and delivered quantities separately so actuator effects
    # can be distinguished from the controller request in the analysis.
    return {
        "T": T,
        "tau": tau,
        "T_delivered": float(wrench_delivered[0]),
        "tau_delivered": wrench_delivered[1:4],
        "rotor_omega": rotor_omega,
        "rotor_thrust": rotor_thrust,
        "Rd": Rd,
        "omega_d": omega_d,
        "position_error": ep,
        "velocity_error": ev,
        "attitude_error": eR,
    }
