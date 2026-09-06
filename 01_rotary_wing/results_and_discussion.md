# Results and Discussion

## 1. Experimental setup

The same controller and same trajectory are used in all cases.

### Case A — Nominal

Controller model = actual plant.

### Case B — Model uncertainty

Controller:

$$
m_c=1.5\;kg.
$$

Actual plant:

$$
m_a=1.8\;kg
$$

with modified inertia.

### Case C — Model uncertainty + disturbance

Case B plus a smooth lateral disturbance from 15–25 s.

## 2. Main results

| Case | Position RMSE | Maximum position error |
|---|---:|---:|
| A — Nominal | **0.0446 m** | **0.1427 m** |
| B — 20% model uncertainty | **0.4887 m** | **0.5479 m** |
| C — Uncertainty + disturbance | **0.5206 m** | **0.7053 m** |

## 3. Case A: nominal model

The controller has an accurate representation of the simulated plant.

The resulting position RMSE is approximately:

$$
4.46\;cm.
$$

This establishes the baseline.

It should not be interpreted as evidence of robustness because the controller is being tested under the same assumptions used to model the plant.

## 4. Case B: model uncertainty

The actual vehicle is 20% heavier and has different inertia, while the controller remains unchanged.

The RMSE increases to:

$$
48.87\;cm.
$$

This is the most significant observation of the first experiment.

The controller's model and plant are now different:

$$
\boxed{\text{Controller model}\neq\text{Plant}}
$$

The force and moment commands are therefore not perfectly matched to the actual simulated dynamics.

This demonstrates why nominal simulation results can be misleading when used as the only robustness test.

## 5. Case C: uncertainty + disturbance

The plant retains the parameter mismatch and receives a smooth lateral disturbance.

Peak disturbance:

$$
F_{d,\max}\approx1.77\;N.
$$

The RMSE becomes:

$$
52.06\;cm
$$

and maximum position error reaches:

$$
70.53\;cm.
$$

The disturbance adds another source of deviation and demands additional control authority.

## 6. Actuator demand

The rotor-speed limit is 900 rad/s.

The limit is reached briefly:

| Case | Approx. saturation duration |
|---|---:|
| A | 0.73 s |
| B | 0.94 s |
| C | 0.94 s |

This observation is important because a controller should not be evaluated only by tracking error.

A physically meaningful evaluation should consider:

$$
\boxed{
\text{Tracking}
+
\text{Control effort}
+
\text{Actuator feasibility}
}
$$

## 7. What did we learn?

### 7.1 Nominal performance is not enough

A controller can look excellent when the mathematical model is accurate.

That does not establish robustness.

### 7.2 Model uncertainty matters

A 20% mass change plus inertia variation causes a large increase in trajectory error.

### 7.3 Disturbance rejection must be measured

Watching the animation is not enough. We need quantitative error metrics.

### 7.4 Actuator limits are part of the control problem

A mathematically valid command can still be physically unavailable.

## 8. What this experiment does not prove

This is a controlled simulation benchmark.

It does not establish:

- robustness to arbitrary uncertainty,
- global stability under all conditions,
- aerodynamic fidelity,
- real flight performance,
- sim-to-real accuracy.

The model does not yet include detailed motor/ESC dynamics, aerodynamic drag and inflow, ground effect, battery-voltage variation, sensor noise, sensor latency or estimator dynamics.

## 9. Numerical/representation limitation

Euler angles are used for state storage and visualization. The attitude tracking error is computed using rotation matrices.

Euler angles become singular near:

$$
\theta=\pm90^\circ.
$$

For future aggressive VTOL transition studies, the state propagation should move to quaternions or rotation matrices.

## 10. Why this is a useful first Yantra Gyan experiment

The important output is not only the numerical RMSE.

The experiment establishes a repeatable workflow:

$$
\boxed{
\text{Nominal}
\rightarrow
\text{Uncertainty}
\rightarrow
\text{Disturbance}
\rightarrow
\text{Limitation}
\rightarrow
\text{Improved method}
}
$$

This allows the next experiment to address a specific limitation rather than adding complexity without a reason.

## 11. Next experiment

**Yantra Gyan #02 — Sudden Payload Change**

Instead of introducing uncertainty as an abstract parameter change, the vehicle mass will change during flight.

This gives a physically intuitive problem and creates a natural path toward:

- adaptive nonlinear control,
- parameter estimation,
- disturbance observers,
- robustness analysis,
- learning-enhanced control.
