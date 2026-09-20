"""Configuration for Yantra Gyan #02 — Sudden Payload Change.

All physical parameters, controller gains, experiment timings, and numerical
settings are kept here so the simulation can be modified without searching
through the implementation files.
"""

# NumPy provides matrices used for inertia and controller gains.
import numpy as np

# Dataclass keeps the vehicle model readable and immutable during a run.
from dataclasses import dataclass


@dataclass(frozen=True)
class Vehicle:
    # Vehicle mass in kilograms.
    mass: float

    # Diagonal rigid-body inertia tensor in kg*m^2.
    inertia: np.ndarray

    # Distance from the center of mass to a rotor location in metres.
    arm: float = 0.25

    # Rotor thrust coefficient: f_i = k_f * omega_i^2.
    kf: float = 1.5e-5

    # Rotor reaction-torque coefficient used by the yaw allocation model.
    km: float = 2.0e-7

    # Maximum rotor speed in rad/s.
    max_omega: float = 900.0

    # Gravitational acceleration in m/s^2.
    gravity: float = 9.81

    @property
    def J(self):
        # Return the inertia as a floating-point NumPy array.
        return np.asarray(self.inertia, dtype=float)


# -----------------------------
# Controller model
# -----------------------------
# The controller deliberately keeps these parameters fixed for the entire
# experiment. It does NOT know when the payload is added.
CONTROLLER_VEHICLE = Vehicle(
    mass=1.5,
    inertia=np.diag([0.030, 0.030, 0.055]),
)


# -----------------------------
# Actual plant models
# -----------------------------
# Case A uses the nominal plant throughout the flight.
ACTUAL_NOMINAL = Vehicle(
    mass=1.5,
    inertia=np.diag([0.030, 0.030, 0.055]),
)

# Case B/C represent the actual vehicle after a 0.30 kg payload is added.
# The controller still uses CONTROLLER_VEHICLE above.
ACTUAL_PAYLOAD = Vehicle(
    mass=1.8,
    # Payload is represented as a 0.30 kg point mass at [0.12, 0.00, 0.10] m
    # relative to the original center of mass. The inertia increment follows
    # the parallel-axis theorem for a point mass.
    inertia=np.diag([0.033, 0.03732, 0.05932]),
)


# -----------------------------
# Nonlinear controller gains
# -----------------------------
# Position gains used in the translational nonlinear feedback law.
KP = np.diag([2.5, 2.5, 4.0])

# Velocity gains provide damping in the translational error dynamics.
KV = np.diag([2.0, 2.0, 2.5])

# Geometric attitude-error gains on SO(3).
KR = np.diag([2.5, 2.5, 1.2])

# Body-rate gains for the geometric attitude controller.
KOMEGA = np.diag([0.25, 0.25, 0.12])


# -----------------------------
# Experiment definition
# -----------------------------
# Total simulation time in seconds.
T_FINAL = 40.0

# Time at which the payload is attached to the actual plant.
PAYLOAD_TIME = 15.0

# Payload mass added to the nominal 1.5 kg vehicle.
PAYLOAD_MASS = 0.30

# Numerical integration maximum step in seconds.
MAX_STEP = 0.01

# Output sample rate used for plots and saved data.
OUTPUT_HZ = 100

# -----------------------------
# Desired trajectory
# -----------------------------
# Horizontal radius of the desired helix in metres.
HELIX_RADIUS = 2.0

# Horizontal angular rate of the desired helix in rad/s.
HELIX_RATE = 0.2

# Constant vertical climb rate in m/s.
CLIMB_RATE = 0.05

# Initial desired altitude in metres.
Z0 = 1.0
