"""Simulation runner for the three Yantra Gyan #01 cases."""

# Import NumPy for the common simulation time grid and initial state.
import numpy as np

# Import SciPy's adaptive ODE integrator.
from scipy.integrate import solve_ivp

# Import simulation settings and the three plant/controller models.
from config import T_FINAL, MAX_STEP
from config import ACTUAL_NOMINAL, ACTUAL_UNCERTAIN

# Import the nonlinear controller.
from controller import controller

# Import the disturbance model.
from disturbance import external_force

# Import the physical plant equations.
from dynamics import quadrotor_rhs

# Import the desired trajectory.
from trajectory import desired_trajectory


# Define the initial 12-state vector.
def initial_state():
    # Start at x=0, y=0, z=1 m.
    # All velocities, angles and angular rates start at zero.
    return np.array([
        0.0, 0.0, 1.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
    ])


# Run one complete experimental case.
def run_case(case_name, actual_vehicle, disturbance_enabled):
    # Create a common 100 Hz output grid for all cases.
    # Using the same grid makes case-to-case comparison easier.
    t_eval = np.linspace(
        0.0,
        T_FINAL,
        int(T_FINAL * 100) + 1,
    )

    # Define the ODE right-hand side passed to solve_ivp.
    def rhs(t, state):
        # Compute the controller command from the current state.
        c = controller(t, state)

        # Calculate the external disturbance for this case.
        d = external_force(t, disturbance_enabled)

        # Return the actual plant state derivative.
        return quadrotor_rhs(
            t,
            state,
            actual_vehicle,
            c,
            d,
        )

    # Numerically integrate the nonlinear plant equations.
    sol = solve_ivp(
        rhs,
        (0.0, T_FINAL),
        initial_state(),
        t_eval=t_eval,
        rtol=2e-6,
        atol=2e-8,
        max_step=MAX_STEP,
        method="RK45",
    )

    # Stop if the numerical solver reports failure.
    if not sol.success:
        raise RuntimeError(sol.message)

    # Calculate desired position at every saved simulation time.
    desired = np.array([
        desired_trajectory(t)[0]
        for t in sol.t
    ])

    # Allocate arrays for controller outputs and disturbance history.
    logs = {
        "T": np.zeros(len(sol.t)),
        "tau": np.zeros((len(sol.t), 3)),
        "rotor_omega": np.zeros((len(sol.t), 4)),
        "disturbance": np.zeros((len(sol.t), 3)),
    }

    # Re-evaluate the controller at every saved sample for logging.
    for k, (t, state) in enumerate(zip(sol.t, sol.y.T)):
        # Calculate controller output.
        c = controller(t, state)

        # Store total thrust.
        logs["T"][k] = c["T"]

        # Store body moments.
        logs["tau"][k] = c["tau"]

        # Store all four rotor speeds.
        logs["rotor_omega"][k] = c["rotor_omega"]

        # Store disturbance force.
        logs["disturbance"][k] = external_force(
            t,
            disturbance_enabled,
        )

    # Return time, state history, desired trajectory and logs.
    return sol.t, sol.y.T, desired, logs
