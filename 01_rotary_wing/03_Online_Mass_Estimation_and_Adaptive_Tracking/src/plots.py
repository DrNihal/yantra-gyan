"""Publication plots for Yantra Gyan #03."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def make_plots(fixed, adaptive, output_directory, payload_time, disturbance_time):
    """Generate the complete figure set."""
    output_directory = Path(output_directory)
    output_directory.mkdir(parents=True, exist_ok=True)

    t = fixed["time"]

    # ---------------------------------------------------------
    # Figure 1 — 3-D trajectory
    # ---------------------------------------------------------
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(
        fixed["position"][:, 0],
        fixed["position"][:, 1],
        fixed["position"][:, 2],
        label="Fixed-parameter",
    )

    ax.plot(
        adaptive["position"][:, 0],
        adaptive["position"][:, 1],
        adaptive["position"][:, 2],
        label="Adaptive",
    )

    ax.plot(
        fixed["desired_position"][:, 0],
        fixed["desired_position"][:, 1],
        fixed["desired_position"][:, 2],
        "--",
        label="Desired",
    )

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("Yantra Gyan #03 — 3-D Trajectory")
    ax.legend()

    fig.tight_layout()
    fig.savefig(output_directory / "01_trajectory.png", dpi=200)
    plt.close(fig)

    # ---------------------------------------------------------
    # Figure 2 — Position tracking error
    # ---------------------------------------------------------
    fig, ax = plt.subplots(figsize=(8, 4.8))

    ax.plot(
        t,
        np.linalg.norm(
            fixed["position_error"],
            axis=1,
        ),
        label="Fixed-parameter",
    )

    ax.plot(
        t,
        np.linalg.norm(
            adaptive["position_error"],
            axis=1,
        ),
        label="Adaptive",
    )

    ax.axvline(
        payload_time,
        linestyle="--",
        label="Payload added",
    )

    ax.axvline(
        disturbance_time,
        linestyle=":",
        label="Disturbance starts",
    )

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Position error norm [m]")
    ax.set_title("Position Tracking Error")
    ax.grid(alpha=0.25)
    ax.legend()

    fig.tight_layout()
    fig.savefig(output_directory / "02_tracking_error.png", dpi=200)
    plt.close(fig)

    # ---------------------------------------------------------
    # Figure 3 — Mass estimation
    # ---------------------------------------------------------
    fig, ax = plt.subplots(figsize=(8, 4.8))

    ax.plot(
        t,
        adaptive["mass_actual"],
        label="Actual mass",
    )

    ax.plot(
        t,
        adaptive["mass_estimate"],
        label="Estimated mass",
    )

    ax.axvline(
        payload_time,
        linestyle="--",
        label="Payload added",
    )

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Mass [kg]")
    ax.set_title("Online Mass Estimation")
    ax.grid(alpha=0.25)
    ax.legend()

    fig.tight_layout()
    fig.savefig(output_directory / "03_mass_estimation.png", dpi=200)
    plt.close(fig)

    # ---------------------------------------------------------
    # Figure 4 — Altitude error
    # ---------------------------------------------------------
    fig, ax = plt.subplots(figsize=(8, 4.8))

    ax.plot(
        t,
        fixed["position_error"][:, 2],
        label="Fixed-parameter",
    )

    ax.plot(
        t,
        adaptive["position_error"][:, 2],
        label="Adaptive",
    )

    ax.axvline(
        payload_time,
        linestyle="--",
        label="Payload added",
    )

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Z error [m]")
    ax.set_title("Altitude Tracking Error")
    ax.grid(alpha=0.25)
    ax.legend()

    fig.tight_layout()
    fig.savefig(output_directory / "04_altitude_error.png", dpi=200)
    plt.close(fig)

    # ---------------------------------------------------------
    # Figure 5 — Rotor speeds
    # ---------------------------------------------------------
    fig, ax = plt.subplots(figsize=(8, 4.8))

    for rotor_index in range(4):
        ax.plot(
            t,
            adaptive["rotor_speed"][:, rotor_index],
            linewidth=0.8,
            label=f"Rotor {rotor_index + 1}",
        )

    ax.axhline(
        900.0,
        linestyle="--",
        label="Rotor limit",
    )

    ax.axvline(
        payload_time,
        linestyle=":",
        label="Payload added",
    )

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Rotor speed [rad/s]")
    ax.set_title("Adaptive Case — Rotor Speeds")
    ax.grid(alpha=0.25)
    ax.legend(ncol=2)

    fig.tight_layout()
    fig.savefig(output_directory / "05_rotor_speed.png", dpi=200)
    plt.close(fig)

    # ---------------------------------------------------------
    # Figure 6 — SO(3) attitude error
    # ---------------------------------------------------------
    fig, ax = plt.subplots(figsize=(8, 4.8))

    ax.plot(
        t,
        fixed["attitude_error_norm"],
        label="Fixed-parameter",
    )

    ax.plot(
        t,
        adaptive["attitude_error_norm"],
        label="Adaptive",
    )

    ax.axvline(
        payload_time,
        linestyle="--",
        label="Payload added",
    )

    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"$\|e_R\|$")
    ax.set_title("Geometric Attitude Error")
    ax.grid(alpha=0.25)
    ax.legend()

    fig.tight_layout()
    fig.savefig(output_directory / "06_attitude_error.png", dpi=200)
    plt.close(fig)
