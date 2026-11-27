# Controller Design — Online Mass Estimation and Adaptive Tracking

## 1. Translational controller

Define

\[
e_p=p_d-p,
\qquad
e_v=v_d-v.
\]

The commanded acceleration is

\[
a_c=a_d+K_pe_p+K_de_v.
\]

The estimated mass converts this desired acceleration into a force:

\[
F_d=\hat m(a_c+ge_3).
\]

This is the central modification introduced in #03.

---

## 2. Desired attitude

The body thrust axis must point in the direction of the desired force:

\[
b_{3d}=\frac{F_d}{\|F_d\|}.
\]

A desired yaw direction is then used to construct the remaining body
axes and form

\[
R_d=[b_{1d}\;b_{2d}\;b_{3d}].
\]

---

## 3. Geometric attitude controller

The attitude error is

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

For this study \(\Omega_d=0\).

---

## 4. Online mass estimator

The vertical dynamics motivate

\[
T\approx m(a_z+g).
\]

Therefore,

\[
m_{\mathrm{inst}}
=
\frac{T}{a_z+g}.
\]

The implementation updates

\[
\dot{\hat m}
=
\gamma_m
(m_{\mathrm{inst}}-\hat m)
\]

with bounds

\[
1.0\leq\hat m\leq2.2\;kg.
\]

The bounds are not a physical claim about all quadrotors. They are
simulation safeguards preventing one noisy sample from producing an
unphysical controller parameter.

---

## 5. Why not directly replace the estimate?

Directly setting

\[
\hat m=m_{\mathrm{inst}}
\]

would make the controller highly sensitive to acceleration
measurement noise.

The filtered, bounded adaptation law provides a simple first step
toward a practical online parameter estimator.

---

## 6. Estimator limitations

The relation

\[
T=m(a_z+g)
\]

is an intentionally simplified identification relationship.

In a real vehicle, the estimate can be biased by:

- acceleration-estimation error,
- thrust-model uncertainty,
- rotor efficiency changes,
- attitude misalignment,
- aerodynamic force,
- external disturbance,
- sensor bias,
- filtering delay.

These limitations define the next research question.
