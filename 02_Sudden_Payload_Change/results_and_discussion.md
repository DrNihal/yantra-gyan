# Results and Discussion

The numerical values below are generated directly by the simulation and saved in `results/summary/summary.json`. The plant-parameter discontinuity at 15 s is handled by restarting the numerical integrator at the event, and actuator saturation is propagated back into the plant through the delivered rotor wrench.

## 1. Experimental setup

The same fixed-parameter nonlinear geometric controller is used in all cases.

### Case A — Nominal

- Actual mass: 1.50 kg throughout
- Actual inertia: nominal
- Payload event: none
- External disturbance: none

### Case B — Sudden payload change

- Actual mass: 1.50 kg before 15 s
- Actual mass: 1.80 kg after 15 s
- Actual inertia changes at the same instant
- Controller continues using 1.50 kg and nominal inertia
- External disturbance: none

### Case C — Payload + disturbance

Case B plus a smooth lateral force from 15–25 s.

---

## 2. Metrics used

The experiment reports:

### Pre-payload RMSE

$$
RMSE_{pre}
=
\sqrt{\frac{1}{N_{pre}}
\sum e_p^2}
$$

### Post-payload RMSE

$$
RMSE_{post}
=
\sqrt{\frac{1}{N_{post}}
\sum e_p^2}
$$

where:

$$
 e_p=\|\mathbf p-\mathbf p_d\|.
$$

Additional metrics include:

- peak post-payload position error,
- time of peak error,
- final position error,
- recovery time to 10 cm, when reached,
- maximum rotor speed,
- approximate actuator saturation duration.

The recovery metric is defined as the first time after the payload event at which the position-error norm remains below 0.10 m for at least 2 seconds.

---

## 3. Main results

Run:

```bash
python main.py
```

Then copy the values from:

```text
results/summary/summary.json
```

into the table below.

| Case | Pre-event RMSE | Post-event RMSE | Peak post-event error | Recovery to 10 cm | Max rotor speed |
|---|---:|---:|---:|---:|---:|
| A — Nominal | **3.52 cm** | **0.064 cm*** | **0.064 cm*** | **N/A*** | **518.36 rad/s** |
| B — Payload | **3.52 cm** | **48.36 cm** | **54.58 cm** | **Not reached** | **554.23 rad/s** |
| C — Payload + disturbance | **3.52 cm** | **52.77 cm** | **68.22 cm** | **Not reached** | **554.33 rad/s** |

*For Case A there is no payload event; the 15–40 s column is a comparison window, not a post-payload metric, and recovery time is therefore N/A.*

---

## 4. Case A — Nominal

Case A provides the reference behaviour.

Because the controller model and actual plant parameters are matched, there is no payload-induced model mismatch.

The purpose of this case is not to prove robustness. It establishes what the controller can achieve when its assumptions remain valid.

---

## 5. Case B — Sudden payload change

At:

$$
 t=15\;s
$$

the actual mass changes from:

$$
1.50\rightarrow1.80\;kg.
$$

The controller remains unaware of the change.

Immediately after the event, the same nominal force command produces less acceleration because:

$$
\dot{\mathbf v}
\supset
\frac{1}{m}R
\begin{bmatrix}0\\0\\T\end{bmatrix}.
$$

The controller therefore has to recover the trajectory through feedback error correction.
The simulation shows a particularly useful analytical result. Once the vehicle reaches a near-steady vertical condition, the controller commands approximately

$$
F_c=m_c(g-K_p e_z)
$$

while the actual plant requires $m_a g$. Therefore the approximate steady-state altitude error is

$$
e_z^{ss}
pprox
-\frac{(m_a-m_c)g}{m_cK_{p,z}}.
$$

For $m_c=1.5\;kg$, $m_a=1.8\;kg$, and $K_{p,z}=4$, this predicts

$$
e_z^{ss}
\approx-0.4905\;m.
$$

The simulated final altitude error is approximately **-0.4905 m**, which closely matches this analytical prediction. This is an important result: the persistent tracking error is not a numerical accident; it is a direct consequence of using a fixed incorrect mass in a controller without integral/adaptive compensation.


The simulated peak position error is approximately **0.546 m**, and the post-event RMSE is approximately **0.484 m**. The final position error is approximately **0.491 m**, while the maximum rotor speed remains about **554 rad/s**, below the 900 rad/s limit. The 10 cm recovery criterion is not reached during the 25-second post-event window.

The most important quantities to inspect are:

1. the transient increase in position error,
2. the additional thrust/rotor-speed demand,
3. whether the error returns to a small neighbourhood of the reference,
4. how long recovery takes.

---

## 6. Case C — Payload change + disturbance

Case C combines two effects:

$$
\text{changing plant parameters}
+
\text{external disturbance}.
$$

This case is useful because it tests whether the payload-induced mismatch leaves enough control authority to reject an additional disturbance.

The disturbance is intentionally simple and bounded. It should be interpreted as a robustness benchmark, not as a complete aerodynamic wind model. In Case C, the peak position error reaches approximately **0.682 m**, while the post-event RMSE is approximately **0.528 m**. The final position error remains approximately **0.491 m** because the fixed mass mismatch persists after the disturbance ends.

---

## 7. What should we learn from the comparison?

The important comparison is not simply the largest error.

The engineering questions are:

### 7.1 Does the feedback controller recover?

If the error returns to a small region after the payload event, the fixed controller has some tolerance to the parameter change.

### 7.2 How expensive is the recovery?

A large rotor-speed increase or saturation may indicate that the recovery is being achieved by consuming actuator margin.

### 7.3 Does the disturbance make the mismatch more significant?

If Case C shows substantially larger error or longer recovery, the combination of parameter variation and disturbance is reducing the available robustness margin.

### 7.4 What limitation remains?

The controller is reacting to the consequences of the parameter change through tracking error.

It does not explicitly know:

$$
\Delta m=0.30\;kg.
$$

That creates the motivation for an adaptive strategy.

---

## 8. Important interpretation

A successful recovery does **not** mean the fixed controller has solved the changing-parameter problem in general.

This experiment only evaluates one:

- trajectory,
- payload magnitude,
- payload timing,
- inertia change,
- disturbance profile,
- actuator model.

The results therefore provide a baseline for the next experiment rather than a general robustness guarantee.

---

## 9. Limitations

The payload is modelled as an instantaneous rigid mass/inertia change.

The simulation does not model:

- payload swing,
- flexible suspension dynamics,
- center-of-gravity migration,
- motor/ESC transient dynamics,
- detailed propeller aerodynamics,
- battery-voltage variation,
- sensor noise and latency,
- state-estimator dynamics.

A real suspended payload would introduce additional states and potentially important coupling dynamics.

---

## 10. Next engineering question

The experiment leads directly to the next step:

> **Can the vehicle estimate the changing mass/inertia online and adapt the nonlinear controller accordingly?**

That motivates:

- online parameter estimation,
- adaptive nonlinear control,
- disturbance estimation,
- robustness analysis,
- eventually learning-enhanced control.

**Next: Yantra Gyan #03 — Adaptive Nonlinear Control for a Changing-Mass Quadrotor.**
