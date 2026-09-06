"""External disturbance model for Case C."""

# Import NumPy for vector construction and trigonometry.
import numpy as np


# Define the external force acting on the vehicle.
def external_force(t, enabled=True):
    # If the disturbance is disabled or outside the disturbance interval,
    # return zero force.
    if not enabled or t < 15.0 or t > 25.0:
        return np.zeros(3)

    # Set disturbance peak to 10% of the uncertain vehicle's weight.
    amplitude = 0.10 * 1.8 * 9.81

    # Normalize time so that tau goes from 0 to 1 during the disturbance.
    tau = (t - 15.0) / 10.0

    # Use a half-sine pulse so the disturbance starts and ends smoothly.
    fx = amplitude * np.sin(np.pi * tau)

    # Apply the force only in the inertial x direction.
    return np.array([fx, 0.0, 0.0])
