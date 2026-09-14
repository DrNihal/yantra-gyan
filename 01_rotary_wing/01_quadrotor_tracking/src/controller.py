"""Nonlinear translational + geometric attitude controller.

The implementation follows controller_design.md line by line:
1. Position/velocity feedback creates a desired force.
2. Desired force determines desired thrust direction.
3. Desired thrust direction and yaw create a desired rotation.
4. Geometric attitude error creates desired moments.
5. Moments and thrust are allocated to the four rotors.
"""

# Import NumPy for vector, matrix and cross-product operations.
import numpy as np

# Import the desired trajectory.
from trajectory import desired_trajectory

# Import Euler-to-rotation conversion.
from rotations import euler_to_R

# Import the rotor allocation function.
from allocation import wrench_to_rotor_omega

# Import controller parameters.
from config import CONTROLLER_VEHICLE, KP, KV, KR, KOMEGA


# Return the skew-symmetric matrix ("hat" map) of a 3-vector.
def hat(v):
    # The hat operator converts a vector into a matrix used for cross products.
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


# Return the vector ("vee" map) corresponding to a skew-symmetric matrix.
def vee(S):
    # Extract the three independent off-diagonal terms.
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


# Compute the translational force requested by the outer loop.
def force_command(t, p, v):
    # Use the mass known by the controller, not the actual plant mass.
    m = CONTROLLER_VEHICLE.mass

    # Read gravity from the controller's model.
    g = CONTROLLER_VEHICLE.gravity

    # Obtain desired position, velocity and acceleration.
    pd, vd, ad, _, _ = desired_trajectory(t)

    # Calculate position tracking error.
    ep = p - pd

    # Calculate velocity tracking error.
    ev = v - vd

    # Generate commanded acceleration using position and velocity feedback.
    ac = ad - KP @ ep - KV @ ev

    # Convert commanded acceleration into a force vector.
    Fc = m * (ac + np.array([0.0, 0.0, g]))

    # Return the force command and tracking errors for logging.
    return Fc, ep, ev


# Construct the desired rotation matrix from desired force and yaw.
def desired_rotation_from_force(Fc, psi_d):
    # Normalize the desired force to obtain the desired body-z direction.
    b3 = Fc / np.linalg.norm(Fc)

    # Define a horizontal reference direction from desired yaw.
    b1c = np.array([np.cos(psi_d), np.sin(psi_d), 0.0])

    # Use a cross product to construct the desired body-y direction.
    b2 = np.cross(b3, b1c)

    # Calculate the norm so that b2 can be normalized.
    n = np.linalg.norm(b2)

    # If the two reference directions become nearly parallel,
    # use a safe fallback heading.
    if n < 1e-8:
        b1c = np.array([1.0, 0.0, 0.0])

        # Recalculate the cross product with the fallback direction.
        b2 = np.cross(b3, b1c)

        # Recalculate its norm.
        n = np.linalg.norm(b2)

    # Normalize the desired body-y axis.
    b2 /= n

    # Complete the right-handed orthonormal basis.
    b1 = np.cross(b2, b3)

    # Assemble the desired rotation matrix from its body axes.
    return np.column_stack((b1, b2, b3))


# Calculate desired attitude and its first two time derivatives.
def desired_attitude_kinematics(t, p, v):
    # Small time increment used for numerical differentiation.
    h = 2e-4

    # Calculate the current desired force using the current tracking error.
    Fc, _, _ = force_command(t, p, v)

    # Read desired yaw at the current time.
    _, _, _, psi, _ = desired_trajectory(t)

    # Construct the current desired rotation matrix.
    Rd = desired_rotation_from_force(Fc, psi)

    # Calculate the force command slightly in the future.
    Fcp, _, _ = force_command(t + h, p, v)

    # Calculate desired yaw slightly in the future.
    _, _, _, psip, _ = desired_trajectory(t + h)

    # Calculate the force command slightly in the past.
    Fcm, _, _ = force_command(max(0.0, t - h), p, v)

    # Calculate desired yaw slightly in the past.
    _, _, _, psim, _ = desired_trajectory(max(0.0, t - h))

    # Construct desired rotations at the future and past times.
    Rdp = desired_rotation_from_force(Fcp, psip)
    Rdm = desired_rotation_from_force(Fcm, psim)

    # Use forward difference at the initial time.
    if t < h:
        Rdot = (Rdp - Rd) / h

        # Second derivative using a one-sided approximation.
        Rddot = (Rdp - 2.0 * Rd + Rdm) / (h * h)

    # Otherwise use the centered finite difference.
    else:
        Rdot = (Rdp - Rdm) / (2.0 * h)

        # Calculate the second derivative.
        Rddot = (Rdp - 2.0 * Rd + Rdm) / (h * h)

    # Map the desired rotation derivative to the skew matrix of desired angular velocity.
    Omega_hat = Rd.T @ Rdot

    # Extract the desired angular velocity vector.
    omega_d = vee(0.5 * (Omega_hat - Omega_hat.T))

    # Differentiate the body-frame desired angular velocity matrix.
    Omega_hat_dot = Rdot.T @ Rdot + Rd.T @ Rddot

    # Extract desired angular acceleration.
    omega_dot_d = vee(0.5 * (Omega_hat_dot - Omega_hat_dot.T))

    # Return desired attitude kinematics.
    return Rd, omega_d, omega_dot_d


# Compute the complete control command.
def controller(t, state):
    # Read the nominal mass used by the controller.
    m = CONTROLLER_VEHICLE.mass

    # Read the nominal inertia used by the controller.
    J = CONTROLLER_VEHICLE.J

    # Extract position from the 12-state vector.
    p = state[:3]

    # Extract velocity.
    v = state[3:6]

    # Convert the actual Euler-angle state into a rotation matrix.
    R = euler_to_R(state[6:9])

    # Extract actual body angular velocity.
    omega = state[9:12]

    # Calculate desired force and translational errors.
    Fc, ep, ev = force_command(t, p, v)

    # Calculate desired attitude, angular velocity and angular acceleration.
    Rd, omega_d, omega_dot_d = desired_attitude_kinematics(t, p, v)

    # The actual thrust acts along the current body-z axis.
    # Project the requested force onto that axis.
    T = float(Fc @ R[:, 2])

    # A rotor cannot produce negative total thrust.
    T = max(0.0, T)

    # Compute geometric attitude error on SO(3).
    eR = 0.5 * vee(Rd.T @ R - R.T @ Rd)

    # Compute angular velocity error in the current body frame.
    eOmega = omega - R.T @ Rd @ omega_d

    # Transport desired angular velocity from desired body frame to current body frame.
    transport = R.T @ Rd @ omega_d

    # Calculate the nonlinear body moment command.
    tau = (
        -KR @ eR
        -KOMEGA @ eOmega
        + np.cross(omega, J @ omega)
        - J @ (hat(omega) @ transport - R.T @ Rd @ omega_dot_d)
    )

    # Combine total thrust and three body moments into the commanded wrench.
    wrench = np.r_[T, tau]

    # Convert the wrench into four physical rotor speeds and thrusts.
    rotor_omega, rotor_thrust = wrench_to_rotor_omega(
        wrench,
        CONTROLLER_VEHICLE,
    )

    # Return all useful controller outputs so they can be logged and plotted.
    return {
        "T": T,
        "tau": tau,
        "rotor_omega": rotor_omega,
        "rotor_thrust": rotor_thrust,
        "Rd": Rd,
        "omega_d": omega_d,
        "position_error": ep,
        "velocity_error": ev,
        "attitude_error": eR,
    }
