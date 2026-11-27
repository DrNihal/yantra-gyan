"""Central simulation parameters for Yantra Gyan #03."""

import numpy as np

# Simulation timing.
DT = 0.004
T_FINAL = 30.0

# Physical constants.
GRAVITY = 9.81

# Controller's initial/nominal mass.
MASS_NOMINAL = 1.50

# Actual mass before and after the payload event.
MASS_INITIAL = 1.50
MASS_PAYLOAD = 1.80
PAYLOAD_TIME = 15.0

# Actual inertia before and after the payload event.
INERTIA_INITIAL = np.diag([0.020, 0.020, 0.035])
INERTIA_PAYLOAD = np.diag([0.024, 0.024, 0.042])

# Quadrotor geometry and thrust/drag coefficients.
ARM_LENGTH = 0.23
K_F = 1.0e-5
K_M = 1.5e-7
OMEGA_MAX = 900.0

# Translational controller gains.
KP = np.diag([4.0, 4.0, 4.0])
KD = np.diag([3.2, 3.2, 3.2])

# Geometric attitude gains.
K_R = np.diag([3.5, 3.5, 1.8])
K_OMEGA = np.diag([0.18, 0.18, 0.12])

# Online mass-estimator settings.
ESTIMATOR_GAIN = 1.6
MASS_MIN = 1.0
MASS_MAX = 2.2
ACCEL_FILTER_TAU = 0.10

# Sensor noise used for the velocity measurement.
VELOCITY_NOISE_STD = 0.01

# Smooth disturbance begins after the payload event.
DISTURBANCE_TIME = 18.0

# Maximum requested total force before the allocation stage.
FORCE_LIMIT = 45.0

# Initial condition / numerical seed.
RNG_SEED = 3
