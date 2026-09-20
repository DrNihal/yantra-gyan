# Yantra Gyan #02 — Sudden Payload Change in a Nonlinear Quadrotor

## Purpose

This repository is the second reproducible simulation study in the **Yantra Gyan** series.

The first study asked:

> What happens when the actual plant differs from the mathematical model used for controller design?

This study asks a more dynamic question:

> **What happens when the physical system itself changes during flight?**

A quadrotor follows a 3-D trajectory. At **t = 15 s**, a **0.30 kg payload is suddenly added**, increasing the vehicle mass from **1.50 kg to 1.80 kg**. The desired yaw is held fixed so the experiment isolates the payload-induced dynamics rather than adding a separate yaw maneuver. The actual inertia is also changed using a simple point-mass payload model and the parallel-axis theorem, so the parameter change is physically traceable rather than arbitrary.

The nonlinear geometric controller is **not informed** about the payload event. The numerical solver is explicitly restarted at the event time so the discontinuous plant-parameter change is handled cleanly. It continues using the original 1.50 kg mass and nominal inertia.

This creates a controlled benchmark for studying the limitations of fixed-parameter nonlinear control and motivates the next step: adaptive control and online parameter estimation.

The simulation also provides an analytical cross-check: the fixed nominal mass produces an approximately **0.49 m steady-state altitude error** after the 20% mass increase, matching the equilibrium predicted from the controller equation. The post-payload rotor speed rises to about **554 rad/s**, remaining below the 900 rad/s actuator limit.

---

## 1. Engineering question

The central question is:

> **Can a fixed-parameter nonlinear controller maintain trajectory tracking when an important plant parameter changes abruptly during flight?**

The study is deliberately structured around engineering questions rather than only a final plot:

- How much tracking error appears immediately after the payload change?
- How does the controller recover?
- How much additional control effort is required?
- Do the actuators approach their limits?
- How does an external disturbance interact with the payload change?
- What limitation of the fixed controller does this experiment expose?
- What controller capability should be introduced next?

---

## 2. Experimental cases

| Case | Actual plant | Disturbance | Controller knowledge |
|---|---|---|---|
| A — Nominal | 1.50 kg throughout | None | 1.50 kg |
| B — Payload | 1.50 → 1.80 kg at 15 s | None | Still assumes 1.50 kg |
| C — Payload + disturbance | 1.50 → 1.80 kg at 15 s | Smooth lateral force | Still assumes 1.50 kg |

The controller is intentionally kept identical in all three cases.

Only the **actual plant** and disturbance conditions change.

This is important because it prevents us from changing the controller and plant at the same time and then being unable to identify the source of the performance change.

---

## 3. Controller used

The baseline controller is the same nonlinear geometric-control architecture established in Yantra Gyan #01:

```text
Desired trajectory
        ↓
Position / velocity error
        ↓
Nonlinear translational control
        ↓
Desired force
        ↓
Desired attitude on SO(3)
        ↓
Geometric attitude control
        ↓
Desired thrust + moments
        ↓
Rotor allocation
        ↓
Four rotor commands
        ↓
6-DOF nonlinear quadrotor
        ↓
Actual state
        └──────────────→ feedback
```

No PID or LQR controller is introduced here.

The purpose of #02 is **not** to demonstrate an adaptive controller yet. It establishes the fixed-parameter baseline that an adaptive method can later be compared against.

---

## 4. Why a sudden payload change?

A payload change is a physically intuitive example of parameter variation.

Before the event:

$$
m = 1.5\;kg
$$

After the event:

$$
m = 1.8\;kg
$$

Therefore:

$$
\Delta m = 0.3\;kg
$$

and

$$
\frac{\Delta m}{m_0}=20\%
$$

The controller does not update its mass parameter.

Therefore, immediately after the event:

$$
\boxed{m_{controller}\neq m_{actual}}
$$

This is different from simply starting a simulation with an uncertain mass. The uncertainty appears **during operation**.

---

## 5. Why this matters from a systems perspective

A real autonomous vehicle can experience changing operating conditions:

- payload pickup or release,
- fuel or battery mass changes,
- configuration changes,
- component degradation,
- external equipment attachment,
- changing aerodynamic characteristics.

A fixed model can therefore become less representative as the mission progresses.

This experiment isolates one such effect in a controlled environment.

---

## 6. Repository structure

```text
Yantra_Gyan_02_Sudden_Payload_Change/
│
├── README.md
├── dynamics_of_quadrotor.md
├── controller_design.md
├── results_and_discussion.md
├── YANTRA_GYAN_SIMULATION_TEMPLATE.md
├── requirements.txt
│
├── config.py
├── trajectory.py
├── rotations.py
├── controller.py
├── dynamics.py
├── allocation.py
├── disturbance.py
├── simulation.py
├── analysis.py
├── plots.py
├── main.py
│
└── results/
    ├── figures/
    ├── raw_data/
    └── summary/
```

---

## 7. Recommended learning path

### Step 1 — Understand the experiment

Read this README first.

### Step 2 — Understand the plant

Read `dynamics_of_quadrotor.md`.

Focus on how mass and inertia enter the translational and rotational equations.

### Step 3 — Understand the controller

Read `controller_design.md`.

Pay particular attention to the distinction between:

$$
\text{controller parameters}
$$

and

$$
\text{actual plant parameters}
$$

### Step 4 — Read the implementation

Recommended order:

```text
config.py
   ↓
trajectory.py
   ↓
controller.py
   ↓
dynamics.py
   ↓
simulation.py
   ↓
analysis.py
   ↓
plots.py
   ↓
main.py
```

The source code contains detailed comments explaining the purpose of important executable statements and how they connect to the mathematics.

### Step 5 — Run the simulation

```bash
pip install -r requirements.txt
python main.py
```

### Step 6 — Study the results

Read `results_and_discussion.md` and inspect all figures in `results/figures/`.

---

## 8. Important modelling distinction

The controller uses a fixed nominal model:

```text
Controller model
mass    = 1.50 kg
inertia = nominal
```

The actual plant changes during flight:

```text
Before 15 s
mass = 1.50 kg

After 15 s
mass = 1.80 kg
inertia = changed
```

This distinction is central to the experiment.

---

## 9. What this simulation does not claim

This is a controlled numerical benchmark.

It does **not** establish:

- robustness to arbitrary payload changes,
- global stability for all possible parameter variations,
- hardware-level flight performance,
- high-fidelity aerodynamic behaviour,
- sim-to-real accuracy.

The model does not include detailed motor/ESC dynamics, propeller inflow, blade-element aerodynamics, ground effect, battery-voltage variation, sensor noise, sensor latency, estimator dynamics, flexible-body effects, or payload swing dynamics.

If the payload itself is suspended or flexible, a future model should explicitly include payload dynamics rather than treating the payload only as a rigid mass increase.

---

## 10. Why this experiment is useful

The study follows the Yantra Gyan progression:

$$
\boxed{
\text{Baseline}
\rightarrow
\text{Parameter change}
\rightarrow
\text{Observe failure/degradation}
\rightarrow
\text{Identify limitation}
\rightarrow
\text{Improve controller}
}
$$

The next logical step is therefore not to add complexity for its own sake.

It is to ask whether the controller can **estimate the changing parameter and adapt its behaviour**.

### Next study

**Yantra Gyan #03 — Adaptive Nonlinear Control for a Changing-Mass Quadrotor**
