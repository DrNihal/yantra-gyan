# Dynamics of the Quadrotor — Yantra Gyan #02

This document explains the nonlinear plant equations implemented in `dynamics.py` and, most importantly, how the payload event changes the actual dynamics.

---

## 1. State vector

The simulation uses the same 12-state representation as Yantra Gyan #01:

$$
\mathbf{x}=
\begin{bmatrix}
 x&y&z&
 v_x&v_y&v_z&
 \phi&\theta&\psi&
 p&q&r
\end{bmatrix}^{T}.
$$

The state is grouped into:

- inertial position $\mathbf p$,
- inertial velocity $\mathbf v$,
- ZYX Euler attitude $\boldsymbol\eta$,
- body angular velocity $\boldsymbol\omega$.

---

## 2. Translational dynamics

Newton's second law is:

$$
 m\dot{\mathbf v}
 =
 m\mathbf g
 +R
 \begin{bmatrix}0\\0\\T\end{bmatrix}
 +\mathbf F_d.
$$

Therefore:

$$
\dot{\mathbf v}
 =
 \mathbf g
 +\frac{1}{m}R
 \begin{bmatrix}0\\0\\T\end{bmatrix}
 +\frac{1}{m}\mathbf F_d.
$$

The key point for this experiment is the factor:

$$
\frac{1}{m}.
$$

For the same thrust command $T$, increasing the vehicle mass reduces the resulting acceleration.

---

## 3. What changes at the payload event?

Initially:

$$
m_0=1.50\;kg.
$$

At $t=15\;s$, the payload adds:

$$
\Delta m=0.30\;kg.
$$

The actual plant therefore becomes:

$$
m_a=1.80\;kg.
$$

The controller still uses:

$$
m_c=1.50\;kg.
$$

Hence:

$$
\boxed{m_c\neq m_a}
$$

Immediately after the event, a force command designed using $m_c$ produces a different acceleration on the actual plant.

---

## 4. Rotational dynamics

Euler's rigid-body equation is:

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

The payload experiment also changes the actual inertia tensor. To keep this change physically traceable, the 0.30 kg payload is represented as a point mass located at $[0.12,0,0.10]$ m relative to the original center of mass. Using the parallel-axis theorem gives the payload inertia contribution

$$
\Delta J_p = m_p\left(\|r_p\|^2I-r_pr_p^T\right),
$$

which gives the post-event plant inertia

$$
J_1=\operatorname{diag}(0.033,0.03732,0.05932)\;kg\,m^2.
$$

The controller continues using the original $J_0=\operatorname{diag}(0.030,0.030,0.055)$ kg m$^2$.

This means the mismatch is present in both translation and rotation.

---

## 5. Euler-angle kinematics

The attitude is represented using the ZYX convention:

$$
R=R_z(\psi)R_y(\theta)R_x(\phi).
$$

The kinematic relationship is:

$$
\dot{\boldsymbol\eta}
=W(\boldsymbol\eta)\boldsymbol\omega.
$$

The matrix $W$ becomes singular near:

$$
\theta=\pm90^\circ.
$$

The present trajectory stays away from this region. Future aggressive VTOL experiments should use quaternion or direct rotation-matrix propagation.

---

## 6. Rotor model

Each rotor produces thrust according to:

$$
f_i=k_f\omega_i^2.
$$

The four rotor thrusts are mapped into total thrust and body moments through the allocation matrix implemented in `allocation.py`.

Rotor speed is limited to:

$$
0\leq\omega_i\leq900\;rad/s.
$$

This matters because the payload event requires additional thrust to maintain the same trajectory.

---

## 7. External disturbance in Case C

Case C adds a smooth lateral force:

$$
\mathbf F_d(t)=
\begin{bmatrix}
F_x(t)\\0\\0
\end{bmatrix}.
$$

The disturbance is active from 15–25 s and follows a half-sine profile.

Its peak magnitude is:

$$
F_{d,max}=0.10(1.8)(9.81)\approx1.77\;N.
$$

The disturbance is intentionally simple. It is a benchmarking disturbance, not a detailed wind/aerodynamic model.

---

## 8. Numerical integration

The nonlinear plant is represented as:

$$
\dot{\mathbf x}=f(t,\mathbf x,\mathbf u).
$$

The simulation uses SciPy's adaptive RK45 integrator through `solve_ivp`.

The controller and plant are evaluated continuously during integration. A common 100 Hz output grid is then used to compare all cases.

---

## 9. Important physical interpretation

The payload event does not directly tell the controller that the mass changed.

Instead, the controller continues producing commands based on its original model while the plant responds according to its new parameters.

Therefore the experiment creates the following loop:

$$
\boxed{
\text{Parameter change}
\rightarrow
\text{model mismatch}
\rightarrow
\text{tracking deviation}
\rightarrow
\text{feedback correction}
}
$$

The next question is whether the controller can explicitly estimate the parameter change instead of relying only on feedback correction.
