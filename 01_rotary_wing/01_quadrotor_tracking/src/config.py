"""Configuration for Yantra Gyan #01.

Every physical parameter and controller gain is kept in one place so that
the experiment can be changed without hunting through the simulation code.
"""

# Import NumPy because the inertia tensors and controller gain matrices are arrays.
import numpy as np

# Use a dataclass to group related vehicle parameters into a readable object.
from dataclasses import dataclass


# Define a vehicle-parameter container.
@dataclass(frozen=True)
class Vehicle:
    # Mass of the vehicle in kilograms.
    mass: float

    # 3x3 rigid-body inertia tensor in kg*m^2.
    inertia: np.ndarray

    # Distance from the vehicle center to the rotor location in metres.
    arm: float = 0.25

    # Rotor thrust coefficient: thrust = kf * omega^2.
    kf: float = 1.5e-5

    # Rotor reaction-torque coefficient.
    km: float = 2.0e-7

    # Maximum allowed rotor speed in rad/s.
    max_omega: float = 900.0

    # Gravitational acceleration in m/s^2.
    gravity: float = 9.81

    # Provide the inertia tensor in a convenient property.
    @property
    def J(self):
        # Convert the supplied inertia to a floating-point NumPy array.
        return np.asarray(self.inertia, dtype=float)


# Define the model known by the controller.
CONTROLLER_VEHICLE = Vehicle(
    # Nominal mass assumed by the controller.
    mass=1.5,

    # Nominal diagonal inertia assumed by the controller.
    inertia=np.diag([0.030, 0.030, 0.055]),
)


# Define the nominal physical plant used in Case A.
ACTUAL_NOMINAL = Vehicle(
    # In Case A, actual mass equals controller mass.
    mass=1.5,

    # In Case A, actual inertia equals controller inertia.
    inertia=np.diag([0.030, 0.030, 0.055]),
)


# Define the uncertain physical plant used in Cases B and C.
ACTUAL_UNCERTAIN = Vehicle(
    # Increase actual mass by 20% while the controller remains unaware.
    mass=1.8,

    # Change inertia to represent a different actual vehicle.
    inertia=np.diag([0.036, 0.036, 0.0605]),
)


# Position-feedback gain matrix.
KP = np.diag([2.5, 2.5, 4.0])

# Velocity-feedback gain matrix.
KV = np.diag([2.0, 2.0, 2.5])

# Attitude-error gain matrix for the geometric controller.
KR = np.diag([2.5, 2.5, 1.2])

# Angular-rate gain matrix for the geometric controller.
KOMEGA = np.diag([0.25, 0.25, 0.12])

# Total simulation duration in seconds.
T_FINAL = 40.0

# Maximum numerical integration step in seconds.
MAX_STEP = 0.01

# Radius of the desired helix in metres.
HELIX_RADIUS = 2.0

# Angular rate of the helix in rad/s.
HELIX_RATE = 0.2

# Constant climb rate in m/s.
CLIMB_RATE = 0.05

# Initial desired altitude in metres.
Z0 = 1.0
