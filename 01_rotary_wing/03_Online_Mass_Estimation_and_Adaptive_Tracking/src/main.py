"""Entry point for Yantra Gyan #03."""

from pathlib import Path
import sys

# Add the modular source directory to Python's import path so that
# `python main.py` works directly from the repository root.
SOURCE_DIRECTORY = Path(__file__).resolve().parent / "src"
sys.path.insert(0, str(SOURCE_DIRECTORY))

import numpy as np

from config import (
    PAYLOAD_TIME,
    DISTURBANCE_TIME,
)
from simulation import run_case
from analysis import compute_metrics
from plots import make_plots


ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"


def save_csv(result, filename):
    """Save the most useful time-history data for later analysis."""
    data = np.column_stack([
        result["time"],
        result["position"],
        result["desired_position"],
        result["position_error"],
        result["mass_actual"],
        result["mass_estimate"],
        result["rotor_speed"],
        result["attitude_error_norm"],
    ])

    header = (
        "time,"
        "px,py,pz,"
        "pdx,pdy,pdz,"
        "ex,ey,ez,"
        "mass_actual,mass_estimate,"
        "omega1,omega2,omega3,omega4,"
        "attitude_error"
    )

    np.savetxt(
        RESULTS / filename,
        data,
        delimiter=",",
        header=header,
        comments="",
    )


def main():
    """Run both cases, analyze them, and generate all outputs."""
    RESULTS.mkdir(parents=True, exist_ok=True)

    print("Running Case A — fixed parameter...")
    fixed = run_case(adaptive=False)

    print("Running Case B — online mass estimation...")
    adaptive = run_case(adaptive=True)

    fixed_metrics = compute_metrics(
        fixed,
        PAYLOAD_TIME,
    )

    adaptive_metrics = compute_metrics(
        adaptive,
        PAYLOAD_TIME,
    )

    save_csv(
        fixed,
        "case_A_fixed.csv",
    )

    save_csv(
        adaptive,
        "case_B_adaptive.csv",
    )

    make_plots(
        fixed,
        adaptive,
        RESULTS,
        PAYLOAD_TIME,
        DISTURBANCE_TIME,
    )

    print("\n===== Yantra Gyan #03 Results =====")
    print(
        f"Fixed post-event RMSE      : "
        f"{fixed_metrics['post_event_rmse_m']:.4f} m"
    )
    print(
        f"Adaptive post-event RMSE   : "
        f"{adaptive_metrics['post_event_rmse_m']:.4f} m"
    )
    print(
        f"Fixed maximum rotor speed  : "
        f"{fixed_metrics['max_rotor_speed_rad_s']:.2f} rad/s"
    )
    print(
        f"Adaptive maximum rotor     : "
        f"{adaptive_metrics['max_rotor_speed_rad_s']:.2f} rad/s"
    )
    print(
        f"Final estimated mass       : "
        f"{adaptive_metrics['final_mass_estimate_kg']:.4f} kg"
    )
    print("Results written to:", RESULTS)


if __name__ == "__main__":
    main()
