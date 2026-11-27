"""Entry point for Yantra Gyan #02."""

# Path provides a clean cross-platform way to create result directories.
from pathlib import Path

# JSON stores the final numerical summary in a human-readable format.
import json

# NumPy stores complete simulation histories for reproducibility.
import numpy as np

# Import the actuator limit used in the analysis.
from config import CONTROLLER_VEHICLE, PAYLOAD_TIME

# Import the experiment runner.
from simulation import run_case

# Import performance metrics.
from analysis import tracking_metrics, compute_recovery_time, add_actuator_metrics

# Import the common plotting routine.
from plots import make_comparison_plots, position_error_norm


# Create the standard results directory structure.
RESULTS = Path("results")
FIGURES = RESULTS / "figures"
RAW_DATA = RESULTS / "raw_data"
SUMMARY = RESULTS / "summary"
FIGURES.mkdir(parents=True, exist_ok=True)
RAW_DATA.mkdir(parents=True, exist_ok=True)
SUMMARY.mkdir(parents=True, exist_ok=True)


# Define the three experiments.
cases = [
    # Case A: no payload change and no external disturbance.
    ("case_A_nominal", False, False),

    # Case B: sudden +20% mass/inertia change at 15 s, no disturbance.
    ("case_B_payload", True, False),

    # Case C: same payload change plus a smooth lateral disturbance.
    ("case_C_payload_disturbance", True, True),
]


# Hold complete histories so the comparison plots can use the same data.
results = {}

# Hold the final scalar metrics for JSON output.
summary = {}


# Run each experiment sequentially.
for name, payload_enabled, disturbance_enabled in cases:
    # Print progress so the user knows which experiment is being simulated.
    print(f"Running {name}...")

    # Run the nonlinear plant simulation.
    t, states, desired, logs = run_case(
        name,
        payload_enabled,
        disturbance_enabled,
    )

    # Store everything needed by the plotting module.
    results[name] = {
        "t": t,
        "states": states,
        "desired": desired,
        "logs": logs,
        "vehicle_max_omega": CONTROLLER_VEHICLE.max_omega,
    }

    # Calculate tracking metrics.
    metrics = tracking_metrics(t, states, desired)

    # Calculate the Euclidean position-error history.
    error_norm = position_error_norm(states, desired)

    # Define a practical recovery threshold of 10 cm.
    recovery_time = (
        compute_recovery_time(
            t,
            error_norm,
            PAYLOAD_TIME,
            threshold=0.10,
            hold_time=2.0,
        )
        if payload_enabled
        else np.nan
    )

    # Add recovery and actuator metrics to the summary.
    metrics["recovery_time_to_10cm_s"] = recovery_time
    metrics = add_actuator_metrics(
        metrics,
        logs["rotor_omega"],
        CONTROLLER_VEHICLE.max_omega,
    )

    # Record the experiment definition along with numerical results.
    summary[name] = {
        "controller_mass_kg": 1.5,
        "initial_actual_mass_kg": 1.5,
        "post_payload_actual_mass_kg": 1.8 if payload_enabled else 1.5,
        "payload_event_time_s": PAYLOAD_TIME if payload_enabled else None,
        "external_disturbance": disturbance_enabled,
        **metrics,
    }

    # Save the complete numerical history for reproducibility.
    np.savez(
        RAW_DATA / f"{name}_data.npz",
        time=t,
        states=states,
        desired=desired,
        thrust_commanded=logs["T"],
        thrust_delivered=logs["T_delivered"],
        moments_commanded=logs["tau"],
        moments_delivered=logs["tau_delivered"],
        rotor_omega=logs["rotor_omega"],
        rotor_thrust=logs["rotor_thrust"],
        disturbance=logs["disturbance"],
        mass=logs["mass"],
    )


# Generate the comparison figures after all three cases are available.
make_comparison_plots(results, FIGURES)


# Save the final metrics as JSON.
with open(SUMMARY / "summary.json", "w", encoding="utf-8") as f:
    json.dump(summary, f, indent=2, allow_nan=True)


# Save a small CSV-style text table that is easy to inspect without Python.
with open(SUMMARY / "comparison.txt", "w", encoding="utf-8") as f:
    f.write("Case | Pre-event RMSE [m] | Post-event RMSE [m] | Peak post-event error [m] | Recovery to 10 cm [s] | Max rotor speed [rad/s]\n")
    f.write("---\n")
    for name, m in summary.items():
        recovery = m["recovery_time_to_10cm_s"]
        recovery_text = "N/A" if np.isnan(recovery) and name == "case_A_nominal" else ("not reached" if np.isnan(recovery) else f"{recovery:.3f}")
        f.write(
            f"{name} | {m['pre_payload_rmse_m']:.5f} | "
            f"{m['post_payload_rmse_m']:.5f} | "
            f"{m['post_payload_peak_error_m']:.5f} | "
            f"{recovery_text} | {m['max_rotor_speed_rad_s']:.2f}\n"
        )


# Print the key results to the terminal.
print("\n=== Yantra Gyan #02 results ===")
for name, m in summary.items():
    recovery = m["recovery_time_to_10cm_s"]
    recovery_text = "N/A" if np.isnan(recovery) and name == "case_A_nominal" else ("not reached" if np.isnan(recovery) else f"{recovery:.3f} s")
    print(
        f"{name:28s} | "
        f"pre RMSE = {m['pre_payload_rmse_m']:.4f} m | "
        f"post RMSE = {m['post_payload_rmse_m']:.4f} m | "
        f"peak = {m['post_payload_peak_error_m']:.4f} m | "
        f"recovery = {recovery_text}"
    )
