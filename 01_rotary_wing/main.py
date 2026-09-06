"""Entry point for Yantra Gyan #01."""

# Import Path so result files are stored in a predictable directory.
from pathlib import Path

# Import JSON so numerical summary metrics can be saved in a readable format.
import json

# Import NumPy for saving arrays.
import numpy as np

# Import the actual plant models.
from config import ACTUAL_NOMINAL, ACTUAL_UNCERTAIN

# Import the simulation runner.
from simulation import run_case

# Import the metric calculator.
from analysis import metrics

# Import plotting functions.
from plots import make_plots


# Create the output directory if it does not already exist.
RESULTS = Path("results")
RESULTS.mkdir(exist_ok=True)


# Define the three experiments.
cases = [
    # Case A: nominal plant and no disturbance.
    ("case_A_nominal", ACTUAL_NOMINAL, False),

    # Case B: uncertain plant and no disturbance.
    ("case_B_uncertain", ACTUAL_UNCERTAIN, False),

    # Case C: uncertain plant plus external disturbance.
    ("case_C_disturbed", ACTUAL_UNCERTAIN, True),
]


# Create a dictionary to hold the final numerical summary.
summary = {}


# Run each experiment sequentially.
for name, vehicle, disturbance in cases:
    # Tell the user which case is currently running.
    print(f"Running {name}...")

    # Run the actual plant simulation.
    t, states, desired, logs = run_case(
        name,
        vehicle,
        disturbance,
    )

    # Calculate tracking metrics.
    m = metrics(
        t,
        states,
        desired,
    )

    # Store metrics under the case name.
    summary[name] = m

    # Generate and save the result plots.
    make_plots(
        t,
        states,
        desired,
        logs,
        name,
        RESULTS,
    )

    # Save raw numerical data so the figures can be reproduced later.
    np.savez(
        RESULTS / f"{name}_data.npz",
        time=t,
        states=states,
        desired=desired,
        thrust=logs["T"],
        moments=logs["tau"],
        rotor_omega=logs["rotor_omega"],
        disturbance=logs["disturbance"],
    )


# Save the summary metrics as human-readable JSON.
with open(RESULTS / "summary.json", "w") as f:
    json.dump(summary, f, indent=2)


# Print a compact comparison in the terminal.
print("\n=== Yantra Gyan #01 results ===")

# Print each case and its two primary metrics.
for name, m in summary.items():
    print(
        f"{name:20s} | "
        f"RMSE position = {m['rmse_position']:.4f} m | "
        f"max error = {m['max_position_error']:.4f} m"
    )
