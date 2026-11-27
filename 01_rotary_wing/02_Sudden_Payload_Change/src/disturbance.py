"""External disturbance model used in Case C."""

# NumPy is used to create the three-axis force vector.
import numpy as np

# The disturbance duration is aligned with the payload experiment.
from config import PAYLOAD_TIME


def external_force(t, enabled=True):
    """Return a smooth lateral disturbance force in the inertial x direction."""

    # Return zero when the disturbance is not part of the selected experiment.
    if not enabled or t < 15.0 or t > 25.0:
        return np.zeros(3)

    # Peak force is 10% of the post-payload vehicle weight.
    amplitude = 0.10 * 1.8 * 9.81

    # Normalize time over the 10-second disturbance interval.
    tau = (t - 15.0) / 10.0

    # Use a half-sine pulse so the disturbance starts and ends smoothly.
    fx = amplitude * np.sin(np.pi * tau)

    # Apply the force laterally; no direct vertical disturbance is introduced.
    return np.array([fx, 0.0, 0.0])
