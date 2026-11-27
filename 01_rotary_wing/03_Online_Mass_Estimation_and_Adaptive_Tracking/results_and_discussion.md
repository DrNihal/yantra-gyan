# Results and Discussion

## 1. Main result

The sudden payload event changes the actual mass from 1.5 kg to 1.8 kg.

The fixed controller continues using the old parameter and develops a
large tracking error.

The adaptive controller estimates the new mass and substantially
reduces the post-event tracking error.

## 2. Quantitative comparison

| Metric | Fixed parameter | Adaptive |
|---|---:|---:|
| Post-event RMSE | 0.4768 m | 0.0635 m |
| Final mass estimate | — | 1.8015 kg |
| Actual final mass | 1.8000 kg | 1.8000 kg |
| Maximum rotor speed | 675.4 rad/s | 680.9 rad/s |
| Rotor limit | 900 rad/s | 900 rad/s |

## 3. What changed?

The physical plant changed first.

The fixed controller did not know about that change.

The adaptive controller used measured motion to update the parameter
appearing directly in the force calculation:

\[
F_d=\hat m(a_c+ge_3).
\]

Therefore the controller gradually restored the force level required by
the heavier vehicle.

## 4. Why is the result important?

The experiment illustrates a distinction between:

\[
\text{stability}
\]

and

\[
\text{mission-level tracking accuracy}.
\]

A vehicle can remain dynamically controlled while its trajectory error
becomes unacceptable for the mission.

## 5. Actuator check

The maximum rotor speeds remain below 900 rad/s in both cases.

Therefore the tracking difference in this experiment is not explained by
one case simply losing all actuator authority.

## 6. Estimator behaviour

The final estimated mass is 1.8015 kg versus an actual mass of 1.8000 kg.

That is close, but this should not be interpreted as proof of a robust
mass estimator.

The experiment uses a simplified force/acceleration relationship and
controlled disturbances.

## 7. Engineering lesson

The progression from #02 to #03 is:

```text
#02
The vehicle changes
        ↓
Fixed controller loses tracking accuracy

#03
The vehicle changes
        ↓
Measure response
        ↓
Estimate parameter
        ↓
Update controller model
        ↓
Recover tracking accuracy
```

The next question is whether this adaptation remains reliable when the
information used by the estimator becomes less clean.

## 8. Limitations

The study does not establish:

- global adaptive stability,
- robustness to arbitrary disturbances,
- real-flight parameter-estimation performance,
- accurate aerodynamic identification,
- sim-to-real performance.

Those require separate experiments.

## 9. Next study

A natural next experiment is:

**Robust online parameter estimation under measurement noise,
disturbance and model uncertainty.**

That study can move from simple parameter adaptation toward a more
formal adaptive-control/observer structure.
