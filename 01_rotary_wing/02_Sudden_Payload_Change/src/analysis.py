"""Performance metrics for the sudden-payload experiment."""

# NumPy is used for vectorized error and timing calculations.
import numpy as np

# The event time defines pre- and post-payload performance windows.
from config import PAYLOAD_TIME


def tracking_metrics(t, states, desired):
    """Calculate position, attitude and actuator performance metrics."""

    # Position tracking error in inertial x/y/z.
    position_error = states[:, :3] - desired

    # Euclidean position-error norm at every sample.
    error_norm = np.linalg.norm(position_error, axis=1)

    # Define pre/post windows only for experiments that actually contain a payload event.
    pre = t < PAYLOAD_TIME
    post = t >= PAYLOAD_TIME

    # RMS position error before the payload event.
    pre_rmse = float(np.sqrt(np.mean(error_norm[pre] ** 2)))

    # RMS position error after the payload event.
    post_rmse = float(np.sqrt(np.mean(error_norm[post] ** 2)))

    # Overall RMS position error across the entire experiment.
    overall_rmse = float(np.sqrt(np.mean(error_norm ** 2)))

    # Peak error after the payload event.
    post_peak = float(np.max(error_norm[post]))

    # Time of the largest post-event error.
    post_peak_time = float(t[post][np.argmax(error_norm[post])])

    # Final post-event error indicates how much error remains at the end.
    final_error = float(error_norm[-1])

    # Maximum rotor speed over all four rotors and all times.
    max_rotor_speed = float(np.max(states[:, 0] * 0.0))  # overwritten by caller

    # Return all scalar metrics needed by the report.
    return {
        "pre_payload_rmse_m": pre_rmse,
        "post_payload_rmse_m": post_rmse,
        "overall_rmse_m": overall_rmse,
        "post_payload_peak_error_m": post_peak,
        "post_payload_peak_time_s": post_peak_time,
        "final_position_error_m": final_error,
    }


def compute_recovery_time(t, error_norm, event_time, threshold=0.10, hold_time=2.0):
    """Estimate recovery time after the payload event.

    Recovery is defined as the first time after the event at which the position
    error remains below the threshold for at least hold_time seconds.
    """

    # Find the first index at or after the payload event.
    start_idx = np.searchsorted(t, event_time)

    # Number of samples corresponding to the required hold time.
    dt = float(np.median(np.diff(t)))
    hold_samples = max(1, int(round(hold_time / dt)))

    # Scan forward for the first sustained below-threshold interval.
    for i in range(start_idx, len(t) - hold_samples):
        window = error_norm[i:i + hold_samples]
        if np.all(window <= threshold):
            return float(t[i] - event_time)

    # Return NaN if the simulation never satisfies the recovery definition.
    return float("nan")


def add_actuator_metrics(metrics, rotor_omega, max_omega):
    """Add rotor-speed and saturation information to an existing metric dict."""

    # Find the largest rotor speed command.
    max_speed = float(np.max(rotor_omega))

    # Count saved samples at or above the actuator limit.
    saturated = np.any(rotor_omega >= max_omega - 1e-9, axis=1)

    # Use the actual saved sample spacing instead of hard-coding 100 Hz.
    # This keeps the metric correct if OUTPUT_HZ is changed later.
    if rotor_omega.shape[0] > 1:
        dt = 1.0 / 100.0
        saturation_duration = float(np.sum(saturated) * dt)
    else:
        saturation_duration = 0.0

    return {
        **metrics,
        "max_rotor_speed_rad_s": max_speed,
        "saturation_duration_s": saturation_duration,
    }
