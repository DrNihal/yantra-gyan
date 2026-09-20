# Controller Design — Fixed-Parameter Nonlinear Geometric Control

## 1. Purpose of the controller in this experiment

The purpose of Yantra Gyan #02 is to establish a **fixed-parameter nonlinear baseline**.

The controller is deliberately kept unchanged when the payload is added.

This lets us isolate the effect of a changing plant before introducing adaptive control.

The controller therefore uses:

$$
m_c=1.5\;kg
$$

and

$$
J_c=\operatorname{diag}(0.030,0.030,0.055).
$$

The actual plant becomes different after 15 s. The post-event inertia is derived from the assumed payload location in `config.py` using the parallel-axis theorem.

---

## 2. Translational control law

Let the position and velocity errors be:

$$
\mathbf e_p=\mathbf p-\mathbf p_d
$$

and

$$
\mathbf e_v=\mathbf v-\mathbf v_d.
$$

The desired acceleration is generated using nonlinear feedback around the reference acceleration:

$$
\mathbf a_c
=
\mathbf a_d
-K_p\mathbf e_p
-K_v\mathbf e_v.
$$

The controller then forms the desired force:

$$
\mathbf F_c
=
 m_c
\left(
\mathbf a_c-\mathbf g
\right).
$$

With $\mathbf g=[0,0,-g]^T$, this is implemented as:

$$
\mathbf F_c
=
 m_c
\left(
\mathbf a_c+
\begin{bmatrix}0\\0\\g\end{bmatrix}
\right).
$$

The important experiment design point is that $m_c$ never changes.

Because this baseline controller has no integral term and does not adapt its mass estimate, a persistent steady-state error can appear after a mass increase. This is not an implementation bug; it is a useful property of the experiment because it exposes the limitation that the next adaptive controller should address.

For a near-hover vertical condition, the approximate equilibrium satisfies

$$
m_c(g-K_{p,z}e_z)\approx m_a g,$$

which gives

$$
e_z^{ss}\approx-\frac{(m_a-m_c)g}{m_cK_{p,z}}.$$

With the selected values, the predicted error is approximately $-0.4905\;m$, providing a direct analytical check against the simulation.

---

## 3. Gain selection

The position gains were selected using the same second-order interpretation used in Yantra Gyan #01.

For one axis, approximately:

$$
\ddot e+k_v\dot e+k_p e\approx0.
$$

Comparing this with:

$$
\ddot e+2\zeta\omega_n\dot e+\omega_n^2e=0
$$

gives:

$$
 k_p=\omega_n^2,
$$

$$
 k_v=2\zeta\omega_n.
$$

The selected gains were then checked against:

- nominal tracking,
- transient response,
- actuator demand,
- payload-change response,
- disturbance response.

The final values are stored centrally in `config.py`.

---

## 4. Desired attitude from desired force

The quadrotor cannot directly command arbitrary inertial-frame acceleration.

Its thrust acts along the body $z$ axis.

Therefore the desired force direction is used to construct the desired body $z$ axis:

$$
\mathbf b_{3d}
=
\frac{\mathbf F_c}{\|\mathbf F_c\|}.
$$

The desired yaw supplies a horizontal reference direction.

Together these vectors define the desired attitude:

$$
R_d=[\mathbf b_{1d}\;\mathbf b_{2d}\;\mathbf b_{3d}].
$$

This is the key translation-to-attitude coupling in the architecture.

---

## 5. Geometric attitude error

The attitude error is computed directly on $SO(3)$:

$$
\mathbf e_R
=
\frac12
\left(
R_d^TR-R^TR_d
\right)^\vee.
$$

The desired angular velocity is transported into the current body frame:

$$
\boldsymbol\omega_{d,c}
=R^TR_d\boldsymbol\omega_d.
$$

The angular-rate error is:

$$
\mathbf e_\Omega
=
\boldsymbol\omega-
R^TR_d\boldsymbol\omega_d.
$$

---

## 6. Nonlinear moment command

For this baseline, the attitude command uses geometric proportional-derivative feedback with rigid-body coupling compensation:

$$
\boldsymbol\tau=
-K_R\mathbf e_R
-K_\Omega\boldsymbol\omega
+\boldsymbol\omega\times J_c\boldsymbol\omega.
$$

A numerical desired-rate feedforward term is deliberately omitted. This keeps the baseline transparent and avoids differentiating a feedback-generated desired-force signal.

The nonlinear rigid-body coupling term:

$$
\boldsymbol\omega\times J_c\boldsymbol\omega
$$

remains in the control law, so the attitude loop is not a linearized independent-axis controller.

---

## 7. Desired-yaw choice

The desired yaw is held at $\psi_d=0$ throughout this experiment. This removes an unnecessary initial heading maneuver and keeps the study focused on the payload-induced parameter change.

## 8. Rotor allocation

The controller produces the wrench:

$$
\mathbf u_c=
\begin{bmatrix}
T&\tau_x&\tau_y&\tau_z
\end{bmatrix}^T.
$$

The allocation matrix maps this wrench to individual rotor thrusts:

$$
\mathbf u_c=M\mathbf f.
$$

Then each rotor thrust is converted to speed using:

$$
\omega_i=\sqrt{\frac{f_i}{k_f}}.
$$

The rotor speed is limited to:

$$
\omega_{max}=900\;rad/s.
$$

---

## 9. What the controller does NOT do

The controller does not:

- detect the payload event explicitly,
- update its mass estimate,
- update its inertia estimate,
- identify payload parameters online,
- use an adaptive law,
- use a disturbance observer.

Those capabilities are intentionally reserved for the next experiment.

---

## 10. Why this baseline is useful

If an adaptive controller is introduced immediately, it becomes difficult to determine whether the improvement comes from adaptation or from an unrelated modelling/controller change.

The fixed controller therefore provides a clean reference:

$$
\boxed{
\text{Fixed model}
\rightarrow
\text{parameter change}
\rightarrow
\text{measured degradation}
}
$$

The next experiment can then ask whether online parameter estimation and adaptation reduce that degradation.
