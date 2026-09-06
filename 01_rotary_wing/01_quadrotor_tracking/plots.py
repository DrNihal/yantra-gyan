"""Plotting functions for Yantra Gyan #01."""

# Import Path so the output directory can be handled cleanly.
from pathlib import Path

# Import NumPy for angle conversion and array operations.
import numpy as np

# Import Matplotlib for plotting.
import matplotlib.pyplot as plt


# Generate all standard result figures for one experimental case.
def make_plots(t, states, desired, logs, case_name, outdir):
    # Convert the output path into a Path object.
    outdir = Path(outdir)

    # Create the directory if it does not exist.
    outdir.mkdir(parents=True, exist_ok=True)

    # Create a new 3-D figure.
    fig = plt.figure(figsize=(8, 6))

    # Add a 3-D axis.
    ax = fig.add_subplot(111, projection="3d")

    # Plot the desired trajectory.
    ax.plot(
        desired[:, 0],
        desired[:, 1],
        desired[:, 2],
        label="Desired",
    )

    # Plot the actual simulated trajectory.
    ax.plot(
        states[:, 0],
        states[:, 1],
        states[:, 2],
        label="Actual",
    )

    # Label the axes.
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")

    # Add a case-specific title.
    ax.set_title(f"{case_name}: 3D trajectory")

    # Show the two trajectories in a legend.
    ax.legend()

    # Adjust layout before saving.
    fig.tight_layout()

    # Save the trajectory figure.
    fig.savefig(
        outdir / f"{case_name}_trajectory.png",
        dpi=180,
    )

    # Close the figure so it does not remain in memory.
    plt.close(fig)

    # Create a position-error figure.
    fig = plt.figure(figsize=(8, 5))

    # Plot x-position error.
    plt.plot(
        t,
        states[:, 0] - desired[:, 0],
        label="x error",
    )

    # Plot y-position error.
    plt.plot(
        t,
        states[:, 1] - desired[:, 1],
        label="y error",
    )

    # Plot z-position error.
    plt.plot(
        t,
        states[:, 2] - desired[:, 2],
        label="z error",
    )

    # Label the axes.
    plt.xlabel("Time [s]")
    plt.ylabel("Position error [m]")

    # Add a title.
    plt.title(f"{case_name}: position tracking error")

    # Add a light grid for readability.
    plt.grid(True, alpha=0.25)

    # Add the legend.
    plt.legend()

    # Adjust the figure layout.
    fig.tight_layout()

    # Save the position-error figure.
    fig.savefig(
        outdir / f"{case_name}_position_error.png",
        dpi=180,
    )

    # Close the figure.
    plt.close(fig)

    # Create an attitude figure.
    fig = plt.figure(figsize=(8, 5))

    # Plot roll, pitch and yaw in degrees.
    for i, label in enumerate(["roll", "pitch", "yaw"]):
        plt.plot(
            t,
            np.rad2deg(states[:, 6 + i]),
            label=label,
        )

    # Label the axes.
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")

    # Add a title.
    plt.title(f"{case_name}: attitude")

    # Add a grid.
    plt.grid(True, alpha=0.25)

    # Add the legend.
    plt.legend()

    # Adjust layout.
    fig.tight_layout()

    # Save the attitude plot.
    fig.savefig(
        outdir / f"{case_name}_attitude.png",
        dpi=180,
    )

    # Close the figure.
    plt.close(fig)

    # Create a rotor-speed figure.
    fig = plt.figure(figsize=(8, 5))

    # Plot all four rotor speeds.
    for i in range(4):
        plt.plot(
            t,
            logs["rotor_omega"][:, i],
            label=f"Rotor {i + 1}",
        )

    # Label the axes.
    plt.xlabel("Time [s]")
    plt.ylabel("Rotor speed [rad/s]")

    # Add a title.
    plt.title(f"{case_name}: rotor speeds")

    # Add a grid.
    plt.grid(True, alpha=0.25)

    # Add the legend.
    plt.legend()

    # Adjust layout.
    fig.tight_layout()

    # Save the rotor-speed plot.
    fig.savefig(
        outdir / f"{case_name}_rotor_speed.png",
        dpi=180,
    )

    # Close the figure.
    plt.close(fig)
