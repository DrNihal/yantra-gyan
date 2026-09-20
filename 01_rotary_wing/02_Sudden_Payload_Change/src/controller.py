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

    # Construct the desired attitude directly from the desired force.
    # The baseline uses geometric PD attitude feedback rather than a numerical
    # desired-rate feedforward term; this avoids differentiating a feedback
    # force command and keeps the baseline analytically transparent.
    _, _, _, psi_d, _ = desired_trajectory(t)
    Rd = desired_rotation_from_force(Fc, psi_d)

    # Project the desired inertial force onto the current body-z axis.
    # This is the total thrust the current attitude can generate.
    T = float(Fc @ R[:, 2])

    # A rotor can only generate positive thrust.
    T = max(0.0, T)

    # Geometric attitude error on SO(3).
    eR = 0.5 * vee(Rd.T @ R - R.T @ Rd)

    # For the baseline experiment, desired body-rate feedforward is omitted.
    # The rate error therefore reduces to the measured body rate.
    eOmega = omega

    # Nonlinear geometric PD moment command with rigid-body coupling compensation.
    tau = (
        -KR @ eR
        -KOMEGA @ eOmega
        + np.cross(omega, J @ omega)
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
        "position_error": ep,
        "velocity_error": ev,
        "attitude_error": eR,
    }
