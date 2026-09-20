"""Simulation runner for the three Yantra Gyan #02 experiments."""

# NumPy provides the initial state and saved numerical arrays.
import numpy as np

# solve_ivp performs adaptive numerical integration of the nonlinear ODE.
from scipy.integrate import solve_ivp

# Import numerical and experiment settings.
from config import T_FINAL, MAX_STEP, OUTPUT_HZ, PAYLOAD_TIME

# Import the nonlinear plant equations.
from dynamics import quadrotor_rhs

# Import the fixed-parameter nonlinear controller.
from controller import controller

# Import the desired trajectory for output-grid evaluation.
from trajectory import desired_trajectory

# Import the disturbance model for logging.
from disturbance import external_force


def initial_state():
    """Return the initial 12-state vector."""

    # Start exactly at the initial desired position and with zero attitude/rates.
    return np.array([
        0.0, 0.0, 1.0,          # x, y, z [m]
        0.0, 0.0, 0.0,          # vx, vy, vz [m/s]
        0.0, 0.0, 0.0,          # roll, pitch, yaw [rad]
        0.0, 0.0, 0.0,          # p, q, r [rad/s]
    ])


def run_case(case_name, payload_enabled, disturbance_enabled):
    """Run one experiment and return states plus all logged controller data."""

    # Build a common output time grid so all cases can be compared sample-by-sample.
    t_eval = np.linspace(
        0.0,
        T_FINAL,
        int(T_FINAL * OUTPUT_HZ) + 1,
    )

    def rhs(t, state):
        # Calculate the control command using the controller's fixed nominal model.
        control = controller(t, state)

        # Calculate the selected external disturbance.
        disturbance = external_force(t, disturbance_enabled)

        # Propagate the actual plant, whose mass/inertia may change at PAYLOAD_TIME.
        return quadrotor_rhs(t, state, payload_enabled, control, disturbance)

    # A discontinuous parameter change is an intentional part of this experiment.
    # Integrate up to the event and then restart the solver with the post-event plant.
    if payload_enabled:
        pre_times = t_eval[t_eval <= PAYLOAD_TIME]
        post_times = t_eval[t_eval > PAYLOAD_TIME]

        sol_pre = solve_ivp(
            rhs,
            (0.0, PAYLOAD_TIME),
            initial_state(),
            t_eval=pre_times,
            rtol=2e-6,
            atol=2e-8,
            max_step=MAX_STEP,
            method="RK45",
        )
        if not sol_pre.success:
            raise RuntimeError(f"{case_name} failed before payload event: {sol_pre.message}")

        # Restart exactly at the payload event with the changed plant parameters.
        sol_post = solve_ivp(
            rhs,
            (PAYLOAD_TIME, T_FINAL),
            sol_pre.y[:, -1],
            t_eval=post_times,
            rtol=2e-6,
            atol=2e-8,
            max_step=MAX_STEP,
            method="RK45",
        )
        if not sol_post.success:
            raise RuntimeError(f"{case_name} failed after payload event: {sol_post.message}")

        t = np.concatenate((sol_pre.t, sol_post.t))
        states = np.vstack((sol_pre.y.T, sol_post.y.T))
    else:
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
        if not sol.success:
            raise RuntimeError(f"{case_name} failed: {sol.message}")
        t = sol.t
        states = sol.y.T

    # Calculate the desired position at every saved sample.
    desired = np.array([desired_trajectory(ti)[0] for ti in t])

    # Allocate arrays for controller outputs and disturbance history.
    logs = {
        "T": np.zeros(len(t)),
        "T_delivered": np.zeros(len(t)),
        "tau": np.zeros((len(t), 3)),
        "tau_delivered": np.zeros((len(t), 3)),
        "rotor_omega": np.zeros((len(t), 4)),
        "rotor_thrust": np.zeros((len(t), 4)),
        "disturbance": np.zeros((len(t), 3)),
        "mass": np.zeros(len(t)),
        "payload_active": np.zeros(len(t), dtype=bool),
    }

    # Re-evaluate the controller at each saved sample for analysis and plots.
    for k, (ti, state) in enumerate(zip(t, states)):
        control = controller(ti, state)
        logs["T"][k] = control["T"]
        logs["T_delivered"][k] = control["T_delivered"]
        logs["tau"][k] = control["tau"]
        logs["tau_delivered"][k] = control["tau_delivered"]
        logs["rotor_omega"][k] = control["rotor_omega"]
        logs["rotor_thrust"][k] = control["rotor_thrust"]
        logs["disturbance"][k] = external_force(ti, disturbance_enabled)
        logs["mass"][k] = 1.8 if payload_enabled and ti > PAYLOAD_TIME else 1.5
        logs["payload_active"][k] = payload_enabled and ti > PAYLOAD_TIME

    return t, states, desired, logs
