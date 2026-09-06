# Yantra Gyan #01 — Nonlinear Quadrotor Tracking Under Model Uncertainty

## 1. Study objective

This simulation investigates a fundamental robustness question in nonlinear UAV control:

> **How does a nonlinear quadrotor trajectory-tracking controller behave when the mathematical model used by the controller does not exactly match the actual plant, and when an external disturbance is introduced?**

The study uses a 6-DOF rigid-body quadrotor model and a nonlinear geometric controller.

The same controller is evaluated under three conditions:

| Case | Actual plant | External disturbance |
|---|---|---|
| **A — Nominal** | Controller model = actual plant | None |
| **B — Model uncertainty** | 20% mass increase + modified inertia | None |
| **C — Uncertainty + disturbance** | Same uncertainty as B | Smooth lateral force |

The purpose is not simply to generate a successful animation. The experiment evaluates:

- trajectory-tracking accuracy,
- effect of model uncertainty,
- disturbance rejection,
- control effort,
- rotor-speed demand,
- actuator saturation,
- and limitations of the simulation model.

---

# 2. What controller is used?

## Nonlinear Geometric Control

The first Yantra Gyan quadrotor study uses a **nonlinear geometric controller** consisting of:

1. a nonlinear translational/position controller, and
2. a geometric attitude controller formulated using rotation matrices on \(SO(3)\).

The control architecture is:

```text
Desired trajectory
       │
       ▼
Position / velocity errors
       │
       ▼
Nonlinear translational controller
       │
       ▼
Desired force vector F_c
       │
       ├──────────────► Desired thrust
       │
       ▼
Desired attitude R_d
       │
       ▼
Geometric attitude controller
       │
       ▼
Desired moments τ
       │
       ▼
Control allocation
       │
       ▼
Four rotor thrusts / speeds
       │
       ▼
6-DOF quadrotor plant
       │
       └──────────────► State feedback
```

The translational controller generates the force required to track the desired position. The desired force direction determines how the quadrotor must orient its thrust axis. The attitude controller then generates the body moments required to achieve that orientation.

---

# 3. Why was geometric nonlinear control selected for Yantra Gyan #01?

The controller is **not selected because it is universally the best controller for quadrotors**.

It is selected because it provides a strong and physically meaningful starting point for this series.

### 3.1 It is genuinely nonlinear

The controller operates with the nonlinear rigid-body dynamics rather than first linearizing the vehicle around a hover equilibrium.

The rotational dynamics retain the coupling term

\[
\boldsymbol{\omega}\times J\boldsymbol{\omega}.
\]

This makes it appropriate for a series focused on nonlinear modelling and control.

### 3.2 It matches the physics of a quadrotor

A quadrotor fundamentally controls its translational motion by changing the direction of its total thrust vector.

The control concept can therefore be understood as:

\[
\boxed{
\text{Position error}
\rightarrow
\text{Desired force}
\rightarrow
\text{Desired attitude}
\rightarrow
\text{Rotor commands}
}
\]

This provides a clean connection between the mathematics and the physical vehicle.

### 3.3 It provides a clean attitude representation

The attitude tracking error is calculated using rotation matrices rather than directly treating Euler-angle errors as three independent scalar errors.

The geometric attitude error is

\[
\mathbf e_R=
\frac{1}{2}
\left(
R_d^TR-R^TR_d
\right)^\vee.
\]

This is useful because attitude belongs naturally to the rotation manifold \(SO(3)\).

### 3.4 It provides a strong reference for future controllers

The same vehicle, trajectory and evaluation metrics can later be used to compare:

- analytical backstepping,
- sliding-mode control,
- adaptive nonlinear control,
- disturbance-observer-based control,
- fault-tolerant control,
- AI/ML-enhanced control,
- Deep Learning approaches,
- Physics-Informed Neural Networks (PINNs),
- and other learning-based methods.

This gives Yantra Gyan a consistent experimental baseline.

### 3.5 It remains understandable

The mathematical formulation is rigorous enough for a control-system study, while the implementation remains compact enough that another student or engineer can follow the code.

The objective is:

> **Understand the controller before trying to make it more complicated.**

---

# 4. Controller formulation

The position and velocity errors are

\[
\mathbf e_p=\mathbf p-\mathbf p_d
\]

and

\[
\mathbf e_v=\mathbf v-\mathbf v_d.
\]

The commanded acceleration is

\[
\mathbf a_c
=
\mathbf a_d
-
K_p\mathbf e_p
-
K_v\mathbf e_v.
\]

The desired force is

\[
\mathbf F_c
=
m_c(\mathbf a_c-\mathbf g),
\]

where \(m_c\) is the mass assumed by the controller.

The desired thrust direction is

\[
\mathbf b_{3d}
=
\frac{\mathbf F_c}{\|\mathbf F_c\|}.
\]

The desired thrust direction and desired yaw are then used to construct \(R_d\).

The geometric attitude error is

\[
\mathbf e_R=
\frac12
(R_d^TR-R^TR_d)^\vee.
\]

The angular-velocity error is

\[
\mathbf e_\Omega
=
\boldsymbol\omega
-
R^TR_d\boldsymbol\omega_d.
\]

The nonlinear moment command contains

\[
\boldsymbol\omega\times J_c\boldsymbol\omega
\]

to retain the rigid-body rotational coupling.

The final wrench

\[
[T,\tau_x,\tau_y,\tau_z]^T
\]

is converted into four rotor commands through the control-allocation matrix.

For the complete derivation, see:

**`controller_design.md`**

---

# 5. Why are PID and LQR not used?

Yantra Gyan is intentionally focused on **nonlinear control and progressively more advanced control/learning methods**.

PID and LQR are extremely useful engineering tools, but they are not the focus of this series.

They may be discussed later as comparison/reference controllers, but the primary experiments will investigate nonlinear and learning-based approaches.

The objective is not to claim that nonlinear control is always superior. The objective is to understand:

- where nonlinear methods provide an advantage,
- where they become difficult,
- what assumptions they require,
- how sensitive they are to uncertainty,
- and what happens when the assumptions fail.

---

# 6. Gain-selection methodology

A major principle of Yantra Gyan is:

> **Do not choose gains simply because they make one simulation look good.**

The gain-selection process should have a physical and analytical basis.

The initial gains for the translational controller are interpreted through the approximate tracking-error dynamics.

Ignoring disturbances and model mismatch,

\[
\ddot{\mathbf e}_p
+
K_v\dot{\mathbf e}_p
+
K_p\mathbf e_p
\approx0.
\]

For one decoupled axis, this resembles

\[
s^2+2\zeta\omega_n s+\omega_n^2=0.
\]

Therefore, a useful initial design relationship is

\[
k_p=\omega_n^2
\]

and

\[
k_v=2\zeta\omega_n.
\]

This provides a systematic starting point based on desired natural frequency and damping ratio.

The attitude gains are then selected using the same engineering philosophy: start from the expected rotational response and increase the gains only as necessary to achieve adequate attitude tracking without excessive control effort.

---

# 7. What are the gain-selection criteria?

The final gains are evaluated against several criteria.

## 7.1 Tracking performance

The controller should provide acceptable:

- position RMSE,
- maximum position error,
- velocity error,
- attitude error.

A lower RMSE is useful, but it is not the only criterion.

---

## 7.2 Transient response

We also examine:

- rise time,
- settling time,
- overshoot,
- peak error,
- recovery time after disturbance.

A controller that has a very small steady-state error but an unacceptable transient response is not considered a good design.

---

## 7.3 Control effort

Increasing gains indefinitely can reduce tracking error while dramatically increasing control demand.

Therefore we monitor:

\[
T,\quad
\tau_x,\quad
\tau_y,\quad
\tau_z.
\]

The controller should achieve the required performance without demanding unreasonable thrust or moments.

---

## 7.4 Actuator feasibility

The rotor commands must satisfy:

\[
0\leq\omega_i\leq\omega_{\max}.
\]

A controller that produces excellent tracking while continuously saturating the motors is **not considered a successful controller design**.

This is why rotor-speed plots are included in the results.

---

## 7.5 Robustness

The final gains should not be selected only from the nominal simulation.

They should also be evaluated under:

- mass uncertainty,
- inertia uncertainty,
- external disturbance,
- initial-condition variation,
- actuator limitations.

This is particularly important for Yantra Gyan #01 because the main research question concerns **model mismatch**.

---

# 8. Gain-tuning philosophy

The gain-selection process for this study is therefore:

\[
\boxed{
\text{Analytical starting point}
\rightarrow
\text{Nominal simulation}
\rightarrow
\text{Performance evaluation}
\rightarrow
\text{Actuator check}
\rightarrow
\text{Uncertainty/disturbance test}
\rightarrow
\text{Final gains}
}
\]

The objective is to find a reasonable compromise between:

\[
\boxed{
\text{Tracking accuracy}
+
\text{Transient response}
+
\text{Control effort}
+
\text{Actuator feasibility}
+
\text{Robustness}
}
\]

This methodology will be retained throughout the Yantra Gyan series.

---

# 9. Important distinction: tuning vs. proving robustness

The gains are selected using the nominal model as the starting point.

However, **robustness is not established by the tuning process itself**.

The controller must subsequently be tested against model uncertainty and disturbance.

Therefore:

```text
Gain design
     ↓
Nominal validation
     ↓
Uncertainty test
     ↓
Disturbance test
     ↓
Actuator-limit test
     ↓
Robustness assessment
```

A controller that performs well only under the exact model used during design should not be described as robust.

---

# 10. Simulation cases

## Case A — Nominal

The controller and actual plant use the same parameters.

Purpose:

- verify implementation,
- establish a baseline,
- confirm trajectory tracking.

## Case B — Model uncertainty

The controller continues to use:

\[
m_c=1.5\;kg
\]

while the actual plant uses:

\[
m_a=1.8\;kg.
\]

Selected inertia values are also modified.

Purpose:

- quantify sensitivity to model mismatch,
- evaluate degradation without retuning the controller.

## Case C — Model uncertainty + disturbance

Case B is retained and a smooth lateral disturbance is applied from 15–25 s.

Purpose:

- evaluate combined uncertainty and disturbance,
- examine recovery,
- evaluate additional actuator demand.

---

# 11. Simulation results

| Case | Position RMSE | Maximum position error |
|---|---:|---:|
| A — Nominal | **4.46 cm** | **14.27 cm** |
| B — 20% model uncertainty | **48.87 cm** | **54.79 cm** |
| C — Uncertainty + disturbance | **52.06 cm** | **70.53 cm** |

The detailed interpretation is provided in:

**`results_and_discussion.md`**

---

# 12. Actuator saturation

The simulation uses a maximum rotor speed of approximately

\[
900\;rad/s.
\]

The rotor-speed limit is reached briefly:

| Case | Approx. saturation duration |
|---|---:|
| A | 0.73 s |
| B | 0.94 s |
| C | 0.94 s |

This is intentionally reported rather than hidden.

A controller should be evaluated using both:

\[
\text{tracking performance}
\]

and

\[
\text{physical control authority}.
\]

---

# 13. Code architecture

The code is intentionally divided into small modules.

```text
main.py
  │
  ├── config.py
  │       └── parameters and gains
  │
  ├── trajectory.py
  │       └── desired position, velocity, acceleration and yaw
  │
  ├── controller.py
  │       ├── translational control
  │       ├── desired attitude
  │       └── geometric attitude control
  │
  ├── allocation.py
  │       └── wrench → rotor thrust/speed
  │
  ├── disturbance.py
  │       └── external disturbance
  │
  ├── dynamics.py
  │       └── 6-DOF rigid-body plant
  │
  ├── simulation.py
  │       └── numerical integration
  │
  ├── analysis.py
  │       └── quantitative metrics
  │
  └── plots.py
          └── result figures
```

---

# 14. Recommended reading order

If you are learning from this repository, follow this order.

### Step 1 — Understand the experiment

Read:

`README.md`

### Step 2 — Understand the plant

Read:

`dynamics_of_quadrotor.md`

### Step 3 — Understand the controller

Read:

`controller_design.md`

### Step 4 — Read the implementation

Recommended order:

```text
config.py
    ↓
trajectory.py
    ↓
rotations.py
    ↓
controller.py
    ↓
allocation.py
    ↓
dynamics.py
    ↓
disturbance.py
    ↓
simulation.py
    ↓
analysis.py
    ↓
plots.py
    ↓
main.py
```

### Step 5 — Run it

```bash
pip install -r requirements.txt
python main.py
```

### Step 6 — Understand the results

Read:

`results_and_discussion.md`

Then inspect:

`results/`

---

# 15. Code-comment philosophy

The source code is intentionally written for readability rather than minimum line count.

Important statements contain comments explaining:

1. **what the code does,**
2. **why it is required,**
3. **what physical quantity it represents,**
4. and, where useful, **which mathematical relationship it implements.**

The goal is that a reader can move directly between:

\[
\boxed{
\text{Equation}
\leftrightarrow
\text{Code}
\leftrightarrow
\text{Physical meaning}
}
\]

The comments should explain the engineering logic, not merely translate Python syntax into English.

---

# 16. Modelling assumptions and limitations

This is a controlled simulation benchmark, not a high-fidelity prediction of a particular commercial UAV.

The current model does not yet include detailed:

- motor/ESC dynamics,
- aerodynamic drag,
- rotor inflow,
- ground effect,
- battery-voltage variation,
- sensor noise,
- sensor latency,
- estimator dynamics,
- flexible-body effects,
- detailed aerodynamic wind interaction.

The plant state is represented using Euler angles. The attitude controller uses geometric rotation-matrix errors, but the Euler-angle state representation still has a singularity near:

\[
\theta=\pm90^\circ.
\]

Future aggressive-flight and VTOL transition studies should move to quaternion or direct \(SO(3)\) state propagation.

---

# 17. What this experiment does and does not demonstrate

### It demonstrates:

- implementation of a nonlinear quadrotor tracking controller,
- nominal trajectory tracking,
- sensitivity to model uncertainty,
- response to an external disturbance,
- control-effort behaviour,
- actuator-limit effects.

### It does not demonstrate:

- global robustness,
- global stability under arbitrary uncertainty,
- aerodynamic fidelity,
- real-flight performance,
- or sim-to-real accuracy.

These require additional studies.

---

# 18. Why this is the first Yantra Gyan experiment

The first experiment establishes the methodology that will be used throughout the series:

\[
\boxed{
\text{Model}
\rightarrow
\text{Controller}
\rightarrow
\text{Nominal validation}
\rightarrow
\text{Uncertainty}
\rightarrow
\text{Disturbance}
\rightarrow
\text{Limitations}
\rightarrow
\text{Next improvement}
}
\]

The intention is not to add complexity simply for the sake of using a more advanced algorithm.

Each new method should be introduced because the previous experiment exposes a specific limitation or research question.

---

# 19. Future progression

The broader Yantra Gyan roadmap will gradually progress toward:

\[
\text{Nonlinear control}
\rightarrow
\text{Adaptive / robust control}
\rightarrow
\text{AI/ML}
\rightarrow
\text{Deep Learning}
\rightarrow
\text{PINNs}
\rightarrow
\text{ROS 2}
\rightarrow
\text{High-fidelity simulation}
\rightarrow
\text{HIL}
\rightarrow
\text{Sim-to-Real}
\rightarrow
\text{Hardware}
\]

The objective is to understand where each method is useful, what assumptions it makes, what limitations it introduces, and whether the additional complexity is justified.

---

# 20. Next study

### Yantra Gyan #02 — Sudden Payload Change

Instead of representing uncertainty only as a fixed parameter mismatch, the next experiment will introduce a physically meaningful event:

> **The quadrotor experiences a sudden change in payload/mass during flight.**

This creates a natural progression toward:

- adaptive nonlinear control,
- online parameter estimation,
- disturbance observers,
- robustness analysis,
- and eventually learning-enhanced control.
#   y a n t r a - g y a n 
 
 #   y a n t r a - g y a n 
 
 