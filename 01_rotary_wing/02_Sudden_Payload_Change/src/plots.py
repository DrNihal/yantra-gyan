"""Plot generation for Yantra Gyan #02."""

# Path makes output directories platform-independent.
from pathlib import Path

# NumPy is used for error calculations and plotting support.
import numpy as np

# Matplotlib generates publication-ready PNG figures.
import matplotlib.pyplot as plt

# Import experiment constants.
from config import PAYLOAD_TIME, ACTUAL_PAYLOAD


def position_error_norm(states, desired):
    # Calculate the Euclidean norm of the three-axis position error.
    return np.linalg.norm(states[:, :3] - desired, axis=1)


def make_comparison_plots(results, outdir):
    """Create the common figures used for LinkedIn and the GitHub study."""

    # Convert the requested output directory into a Path object.
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    # Pull the three experiment histories from the result dictionary.
    A = results["case_A_nominal"]
    B = results["case_B_payload"]
    C = results["case_C_payload_disturbance"]

    # ---------------------------------------------------------------
    # Figure 1: Desired vs actual 3-D trajectory.
    # ---------------------------------------------------------------
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")

    # The desired trajectory is common to all cases.
    ax.plot(
        A["desired"][:, 0], A["desired"][:, 1], A["desired"][:, 2],
        "k--", linewidth=2.2, label="Desired trajectory"
    )

    # Plot the actual trajectory for each experimental case.
    ax.plot(
        A["states"][:, 0], A["states"][:, 1], A["states"][:, 2],
        linewidth=2.0, label="A — Nominal"
    )
    ax.plot(
        B["states"][:, 0], B["states"][:, 1], B["states"][:, 2],
        linewidth=2.0, label="B — +20% payload mass"
    )
    ax.plot(
        C["states"][:, 0], C["states"][:, 1], C["states"][:, 2],
        linewidth=2.0, label="C — +20% mass + disturbance"
    )

    # Mark the approximate location of the payload event on the desired path.
    event_index = np.argmin(np.abs(A["t"] - PAYLOAD_TIME))
    ax.scatter(
        A["desired"][event_index, 0],
        A["desired"][event_index, 1],
        A["desired"][event_index, 2],
        s=45,
        marker="o",
        label="Payload event"
    )

    # Label all three spatial axes with SI units.
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("Yantra Gyan #02 — Sudden Payload Change: 3D Tracking")
    ax.legend()
    fig.tight_layout()
    fig.savefig(outdir / "01_3D_tracking_comparison.png", dpi=200)
    plt.close(fig)

    # ---------------------------------------------------------------
    # Figure 2: Position-error norm.
    # ---------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(11, 6))

    # Plot the position-error norm for all three cases.
    for key, label in [
        ("case_A_nominal", "A — Nominal"),
        ("case_B_payload", "B — +20% payload mass"),
        ("case_C_payload_disturbance", "C — +20% mass + disturbance"),
    ]:
        item = results[key]
        error = position_error_norm(item["states"], item["desired"])
        ax.plot(item["t"], error, linewidth=2.0, label=label)

    # Draw the exact payload event time.
    ax.axvline(
        PAYLOAD_TIME,
        linestyle="--",
        linewidth=1.8,
        label="Payload added at 15 s",
    )

    # Shade the post-event interval so the change is visually obvious.
    ax.axvspan(PAYLOAD_TIME, 25.0, alpha=0.10)

    # Label the axes and add a descriptive title.
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Position error norm [m]")
    ax.set_title("Yantra Gyan #02 — Position Tracking Error")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(outdir / "02_tracking_error_comparison.png", dpi=200)
    plt.close(fig)

    # ---------------------------------------------------------------
    # Figure 3: Mass profile and event.
    # ---------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(B["t"], B["logs"]["mass"], linewidth=2.5, label="Actual mass")
    ax.axvline(PAYLOAD_TIME, linestyle="--", linewidth=1.8, label="Payload added")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Actual mass [kg]")
    ax.set_title("Yantra Gyan #02 — Sudden Change in Vehicle Mass")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(outdir / "03_mass_change.png", dpi=200)
    plt.close(fig)

    # ---------------------------------------------------------------
    # Figure 4: Rotor-speed response for the payload-only case.
    # ---------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(11, 6))
    for i in range(4):
        ax.plot(
            B["t"],
            B["logs"]["rotor_omega"][:, i],
            linewidth=1.7,
            label=f"Rotor {i + 1}",
        )
    ax.axvline(PAYLOAD_TIME, linestyle="--", linewidth=1.8, label="Payload added")
    ax.axhline(
        B["vehicle_max_omega"],
        linestyle=":",
        linewidth=1.6,
        label="Rotor limit",
    )
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Rotor speed [rad/s]")
    ax.set_title("Yantra Gyan #02 — Rotor-Speed Response")
    ax.grid(True, alpha=0.25)
    ax.legend(ncol=2)
    fig.tight_layout()
    fig.savefig(outdir / "04_rotor_speed_response.png", dpi=200)
    plt.close(fig)

    # ---------------------------------------------------------------
    # Figure 5: Vertical tracking error — useful for explaining the payload.
    # ---------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(11, 5))
    for key, label in [
        ("case_A_nominal", "A — Nominal"),
        ("case_B_payload", "B — +20% payload mass"),
        ("case_C_payload_disturbance", "C — +20% mass + disturbance"),
    ]:
        item = results[key]
        ez = item["states"][:, 2] - item["desired"][:, 2]
        ax.plot(item["t"], ez, linewidth=2.0, label=label)
    ax.axvline(PAYLOAD_TIME, linestyle="--", linewidth=1.8, label="Payload added")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Altitude error [m]")
    ax.set_title("Yantra Gyan #02 — Altitude Response to Payload Change")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(outdir / "05_altitude_error.png", dpi=200)
    plt.close(fig)
