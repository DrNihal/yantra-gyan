# Yantra Gyan #03 — Online Mass Estimation & Adaptive Tracking

## 1. Study objective

Yantra Gyan #02 showed that a fixed-parameter nonlinear controller can
remain stable after a sudden payload change while losing trajectory
tracking accuracy.

The question for #03 is:

> **Can the controller recognize that the vehicle dynamics have changed,
> estimate the changed mass online, and use that information to recover
> tracking accuracy?**

This study keeps the same quadrotor/control philosophy used in #01/#02,
but adds an online parameter-estimation loop.

---

## 2. Simulation concept

The actual vehicle starts at:

\[
m_a=1.5\;kg
\]

and at \(t=15\;s\) the payload changes the actual mass to:

\[
m_a=1.8\;kg.
\]

The controller is **not directly told** that the payload has been added.

Two cases are evaluated:

| Case | Controller mass | Actual mass | Online estimation |
|---|---:|---:|---|
| A — Fixed parameter | 1.5 kg | 1.5 → 1.8 kg | No |
| B — Adaptive | 1.5 kg initial | 1.5 → 1.8 kg | Yes |

A smooth lateral disturbance is also introduced after the payload event.

---

## 3. Controller and plant

The simulation uses:

- 6-DOF nonlinear rigid-body quadrotor dynamics
- nonlinear translational control
- geometric attitude control on \(SO(3)\)
- explicit rotor allocation
- rotor-speed saturation
- changing mass and inertia
- noisy velocity measurement
- acceleration filtering
- online mass estimation
- external disturbance

The architecture is:

```text
Desired trajectory
        │
        ▼
Position / velocity errors
        │
        ▼
Nonlinear translational controller
        │
        ├──────────────► Desired force
        │
        ▼
Online mass estimate ─────► Updated force model
        │
        ▼
Desired attitude R_d
        │
        ▼
Geometric SO(3) attitude controller
        │
        ▼
Desired body wrench
        │
        ▼
Rotor allocation + saturation
        │
        ▼
6-DOF quadrotor plant
        │
        ├──────────────► State feedback
        │
        └──────────────► Measurements
                              │
                              ▼
                       Mass estimator
```

---

## 4. Why online mass estimation?

A fixed-parameter controller uses

\[
F_c=\hat m(a_c+ge_3)
\]

with \(\hat m=1.5\;kg\).

After the payload is added, the real vehicle requires more thrust than
the controller predicts.

For the adaptive case, the controller instead uses an evolving estimate

\[
\hat m(t).
\]

The estimator uses the vertical force relationship

\[
T \approx m(a_z+g)
\]

to form an instantaneous estimate

\[
m_{\mathrm{inst}}=
\frac{T}{a_z+g}.
\]

A bounded first-order adaptation law prevents a single noisy
measurement from immediately producing an unrealistic parameter:

\[
\dot{\hat m}
=
\gamma_m
\left(
m_{\mathrm{inst}}-\hat m
\right).
\]

This is intentionally transparent. It is a parameter-estimation
experiment, not yet a flight-ready estimator.

---

## 5. Gain selection

The translational controller is

\[
a_c=a_d+K_p(p_d-p)+K_d(v_d-v).
\]

For each translational axis, the starting point is the familiar
second-order interpretation

\[
k_p=\omega_n^2,\qquad
k_d=2\zeta\omega_n.
\]

The gains are then checked through:

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

The adaptive estimator gain introduces an additional trade-off:

- too small → slow parameter recovery,
- too large → greater sensitivity to acceleration/measurement noise.

---

## 6. Simulation cases

### Case A — Fixed-parameter controller

The controller continues to use:

\[
\hat m=1.5\;kg
\]

even after the actual vehicle becomes 1.8 kg.

Purpose:

- establish the #02-like baseline,
- quantify tracking degradation,
- provide the reference against which adaptation is evaluated.

### Case B — Online mass estimation

The controller starts with:

\[
\hat m(0)=1.5\;kg
\]

but updates the estimate during flight.

Purpose:

- detect the parameter change,
- observe estimator convergence,
- determine whether tracking accuracy recovers,
- check actuator demand.

---

## 7. Physical model

Translation:

\[
m\ddot p=R
\begin{bmatrix}
0\\0\\T
\end{bmatrix}
+d_F-mge_3.
\]

Rotation:

\[
J\dot\Omega+\Omega\times(J\Omega)
=
\tau+d_\tau.
\]

Attitude:

\[
\dot R=R\hat\Omega.
\]

The geometric attitude error is

\[
e_R=
\frac12
\left(
R_d^TR-R^TR_d
\right)^\vee.
\]

The moment command is

\[
\tau=
-K_Re_R
-K_\Omega(\Omega-\Omega_d)
+\Omega\times J\Omega.
\]

---

## 8. Results from the validated run

The full simulation is executed from `main.py` and produces the
following results:

- Fixed-parameter post-event RMSE: **0.4768 m**
- Adaptive post-event RMSE: **0.0635 m**
- Final estimated mass: **1.8015 kg**
- Actual post-event mass: **1.8000 kg**
- Maximum rotor speed, fixed: **675.4 rad/s**
- Maximum rotor speed, adaptive: **680.9 rad/s**
- Rotor limit: **900 rad/s**

The improvement therefore occurs without the adaptive case exceeding
the configured actuator-speed limit.

---

## 9. Code architecture

The implementation deliberately follows the Yantra Gyan #01 template.

```text
main.py
   │
   ├── config.py
   │      └── parameters, gains, simulation settings
   │
   ├── trajectory.py
   │      └── desired position, velocity, acceleration
   │
   ├── rotations.py
   │      └── SO(3) / hat / vee / projection utilities
   │
   ├── controller.py
   │      ├── translational controller
   │      ├── desired attitude
   │      ├── mass estimator
   │      └── geometric attitude controller
   │
   ├── allocation.py
   │      └── wrench → rotor thrust/speed
   │
   ├── disturbance.py
   │      └── external force/moment disturbance
   │
   ├── dynamics.py
   │      └── 6-DOF rigid-body plant
   │
   ├── simulation.py
   │      └── closed-loop numerical integration
   │
   ├── analysis.py
   │      └── quantitative metrics
   │
   └── plots.py
          └── publication figures
```

---

## 10. Recommended reading order

1. `README.md`
2. `dynamics_of_quadrotor.md`
3. `controller_design.md`
4. `src/config.py`
5. `src/trajectory.py`
6. `src/rotations.py`
7. `src/controller.py`
8. `src/allocation.py`
9. `src/disturbance.py`
10. `src/dynamics.py`
11. `src/simulation.py`
12. `src/analysis.py`
13. `src/plots.py`
14. `main.py`
15. `results_and_discussion.md`

Run:

```bash
pip install -r requirements.txt
python main.py
```

---

## 11. Code-comment philosophy

The source is intentionally written for readability rather than
minimum line count.

Important statements explain:

1. what the code does,
2. why it is required,
3. what physical quantity it represents,
4. and, where useful, which equation it implements.

The intended relationship is:

\[
\boxed{
\text{Equation}
\leftrightarrow
\text{Code}
\leftrightarrow
\text{Physical meaning}
}
\]

The comments explain engineering logic rather than simply translating
Python syntax into English.

---

## 12. Assumptions and limitations

This remains a controlled simulation benchmark.

It does not yet include detailed:

- motor/ESC dynamics,
- rotor inflow,
- aerodynamic drag,
- ground effect,
- battery-voltage variation,
- sensor latency,
- aerodynamic parameter identification,
- flexible-body effects,
- full state-estimator dynamics.

The mass estimator also relies on a simplified vertical force relation.
Consequently, disturbance/noise can bias the estimate.

The next study should examine robust parameter estimation when the
estimator is simultaneously exposed to noise, disturbance and model
uncertainty.

---

## 13. What #03 demonstrates

The experiment demonstrates the progression:

\[
\boxed{
\text{Parameter change}
\rightarrow
\text{Measurement}
\rightarrow
\text{Estimation}
\rightarrow
\text{Model update}
\rightarrow
\text{Control adaptation}
\rightarrow
\text{Tracking recovery}
}
\]

The key engineering question is therefore no longer only:

> Does the controller track?

It becomes:

> **Can the system identify what changed and use that information to
make a better control decision?**
