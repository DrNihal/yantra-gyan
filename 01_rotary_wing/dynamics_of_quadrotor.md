# Dynamics of the Quadrotor

This document explains the physical model implemented in `dynamics.py`.

## 1. State vector

The simulation uses 12 states:

$$
\mathbf{x}=
\begin{bmatrix}
x&y&z&
v_x&v_y&v_z&
\phi&\theta&\psi&
p&q&r
\end{bmatrix}^{T}.
$$

They are grouped as:

- position $\mathbf p$
- velocity $\mathbf v$
- Euler attitude $\boldsymbol\eta$
- body angular velocity $\boldsymbol\omega$

## 2. Translational dynamics

Position:

$$
\dot{\mathbf p}=\mathbf v.
$$

Force balance:

$$
m\dot{\mathbf v}
=
m\mathbf g+
R(\boldsymbol\eta)
\begin{bmatrix}0\\0\\T\end{bmatrix}
+\mathbf F_d.
$$

Therefore:

$$
\dot{\mathbf v}
=
\mathbf g+
\frac{1}{m}R
\begin{bmatrix}0\\0\\T\end{bmatrix}
+\frac{1}{m}\mathbf F_d.
$$

The important physical point is that the quadrotor does not directly command inertial $x$ and $y$ forces. It changes attitude so that its thrust vector points in the required direction.

## 3. Attitude representation

The plant state uses ZYX Euler angles:

$$
R=R_z(\psi)R_y(\theta)R_x(\phi).
$$

The Euler-angle kinematics are:

$$
\dot{\boldsymbol\eta}
=
W(\boldsymbol\eta)\boldsymbol\omega
$$

with

$$
W=
\begin{bmatrix}
1&\sin\phi\tan\theta&\cos\phi\tan\theta\\
0&\cos\phi&-\sin\phi\\
0&\frac{\sin\phi}{\cos\theta}&\frac{\cos\phi}{\cos\theta}
\end{bmatrix}.
$$

Euler angles become singular at $\theta=\pm90^\circ$. This benchmark remains away from that region. Future aggressive VTOL/transition work should use quaternion or rotation-matrix state propagation.

## 4. Rotational dynamics

Euler's rigid-body equation:

$$
J\dot{\boldsymbol\omega}
+
\boldsymbol\omega\times(J\boldsymbol\omega)
=
\boldsymbol\tau.
$$

Therefore:

$$
\dot{\boldsymbol\omega}
=
J^{-1}
\left[
\boldsymbol\tau-
\boldsymbol\omega\times(J\boldsymbol\omega)
\right].
$$

The cross-product term is retained; the rotational axes are therefore not treated as independent linear systems.

## 5. Rotor model

Each rotor is modelled as

$$
f_i=k_f\omega_i^2.
$$

The total thrust is

$$
T=\sum_i f_i.
$$

Rotor reaction torque is represented using

$$
\tau_{z,i}=s_i k_m\omega_i^2.
$$

The four rotor thrusts are mapped to total thrust and body moments by the allocation matrix in `allocation.py`.

## 6. Disturbance

Case C applies:

$$
\mathbf F_d=
\begin{bmatrix}
F_x(t)\\0\\0
\end{bmatrix}.
$$

The disturbance is active from 15–25 s and has a smooth half-sine profile. Its peak is approximately 10% of the 1.8 kg vehicle weight:

$$
F_{d,\max}\approx1.77\;N.
$$

This is a controlled disturbance for benchmarking, not a claim that it is a complete aerodynamic wind model.

## 7. Numerical integration

The state equation is

$$
\dot{\mathbf x}=f(t,\mathbf x,\mathbf u).
$$

It is integrated using SciPy `solve_ivp` with RK45. The solver performs numerical integration; it is not part of the controller.
