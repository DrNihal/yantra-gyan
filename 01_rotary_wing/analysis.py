"""Tracking-performance metrics."""

# Import NumPy for vectorized error and RMS calculations.
import numpy as np


# Calculate position-tracking metrics.
def metrics(t, states, desired_positions):
    # Calculate x/y/z tracking error at every sample.
    e = states[:, 0:3] - desired_positions

    # Calculate Euclidean position-error magnitude at every sample.
    norm = np.linalg.norm(e, axis=1)

    # Calculate RMS error separately for x, y and z.
    rmse_xyz = np.sqrt(np.mean(e**2, axis=0))

    # Calculate RMS of the 3-D position-error magnitude.
    rmse_norm = np.sqrt(np.mean(norm**2))

    # Find the largest 3-D position error.
    max_norm = np.max(norm)

    # Return metrics in a dictionary for reporting.
    return {
        "rmse_x": rmse_xyz[0],
        "rmse_y": rmse_xyz[1],
        "rmse_z": rmse_xyz[2],
        "rmse_position": rmse_norm,
        "max_position_error": max_norm,
    }
