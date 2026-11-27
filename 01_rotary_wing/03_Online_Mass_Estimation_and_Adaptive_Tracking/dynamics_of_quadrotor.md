# Dynamics of the Quadrotor

## 1. State

The plant state is

\[
x=
\left[
p^T,\;v^T,\;R,\;\Omega^T
\right]^T
\]

where:

- \(p\) — inertial position,
- \(v\) — inertial velocity,
- \(R\in SO(3)\) — body-to-inertial rotation matrix,
- \(\Omega\) — body angular velocity.

## 2. Translational dynamics

The rotor thrust acts along the body \(z\)-axis:

\[
F_B=
\begin{bmatrix}
0\\0\\T
\end{bmatrix}.
\]

Therefore the inertial thrust is

\[
F_I=RF_B.
\]

The translational equation is

\[
m\dot v=RF_B+d_F-mge_3.
\]

and

\[
\dot p=v.
\]

## 3. Rotational dynamics

The rigid-body rotational equation is

\[
J\dot\Omega
=
\tau+d_\tau-\Omega\times(J\Omega).
\]

The cross-product term is retained; no hover linearization is used.

## 4. Attitude kinematics

\[
\dot R=R\hat\Omega.
\]

Numerical propagation is projected back to \(SO(3)\) after each step
to suppress integration drift.

## 5. Changing plant parameters

Before the payload event:

\[
m=1.5\;kg,\qquad
J=
\operatorname{diag}(0.020,0.020,0.035).
\]

After the event:

\[
m=1.8\;kg,\qquad
J=
\operatorname{diag}(0.024,0.024,0.042).
\]

The controller is not given the new parameters in the fixed case.

## 6. Rotor model

For rotor \(i\),

\[
f_i=k_f\omega_i^2.
\]

The rotor allocation maps the desired wrench

\[
[T,\tau_x,\tau_y,\tau_z]^T
\]

to rotor squared speeds.

The resulting rotor speeds are saturated at

\[
\omega_{\max}=900\;rad/s.
\]

The **saturated** rotor outputs are then used to calculate the
delivered plant wrench. This keeps actuator saturation physically
connected to the plant dynamics.
