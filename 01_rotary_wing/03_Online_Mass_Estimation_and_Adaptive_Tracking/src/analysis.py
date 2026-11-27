"""Quantitative analysis of simulation results."""

import numpy as np


def position_rmse(position_error, mask=None):
    """Return RMS norm of the 3-D position error."""
    error = position_error if mask is None else position_error[mask]
    return float(np.sqrt(np.mean(np.sum(error**2, axis=1))))


def maximum_position_error(position_error, mask=None):
    """Return maximum Euclidean position error."""
    error = position_error if mask is None else position_error[mask]
    return float(np.max(np.linalg.norm(error, axis=1)))


def maximum_rotor_speed(rotor_speed):
    """Return the maximum rotor speed across all four rotors."""
    return float(np.max(rotor_speed))


def compute_metrics(result, payload_time):
    """Compute the main metrics used by the study."""
    t = result["time"]
    post_event = t >= payload_time
    late_window = t >= 20.0

    return {
        "post_event_rmse_m": position_rmse(
            result["position_error"],
            post_event,
        ),
        "post_event_max_error_m": maximum_position_error(
            result["position_error"],
            post_event,
        ),
        "late_rmse_m": position_rmse(
            result["position_error"],
            late_window,
        ),
        "max_rotor_speed_rad_s": maximum_rotor_speed(
            result["rotor_speed"]
        ),
        "final_mass_estimate_kg": float(
            result["mass_estimate"][-1]
        ),
    }
