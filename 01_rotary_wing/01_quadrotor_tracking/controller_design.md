# Controller Design

## 1. Objective

The controller must make:

$$
\mathbf p(t)\rightarrow\mathbf p_d(t).
$$

Define:

$$
\mathbf e_p=\mathbf p-\mathbf p_d
$$

and

$$
\mathbf e_v=\mathbf v-\mathbf v_d.
$$

## 2. Nonlinear translational control

The commanded acceleration is:

$$
\mathbf a_c
=
\mathbf a_d
-K_p\mathbf e_p
-K_v\mathbf e_v.
$$

The controller then calculates:

$$
\mathbf F_c
=
m_c(\mathbf a_c-\mathbf g).
$$

Since

$$
\mathbf g=[0,0,-g]^T,
$$

the implementation is equivalent to:

$$
\mathbf F_c
=
m_c
\left(
\mathbf a_c+
[0,0,g]^T
\right).
$$

This force contains the acceleration needed to follow the trajectory plus the force needed to counter gravity.

## 3. Desired thrust direction

Define:

$$
\mathbf b_{3d}
=
\frac{\mathbf F_c}{\|\mathbf F_c\|}.
$$

The desired body-z axis is therefore aligned with the requested force.

This is the key connection between position control and attitude control:

$$
\boxed{
\text{Position error}
\rightarrow
\text{desired force}
\rightarrow
\text{desired attitude}
}
$$

## 4. Desired yaw

The vehicle heading follows the horizontal direction of travel.

For:

$$
x_d=2[\cos(0.2t)-1]
$$

and

$$
y_d=2\sin(0.2t),
$$

the desired heading is chosen as:

$$
\psi_d=0.2t+\frac{\pi}{2}.
$$

The desired force direction and desired yaw are combined to construct $R_d$.

## 5. Geometric attitude control

The current attitude is $R$ and desired attitude is $R_d$.

The attitude error is:

$$
\mathbf e_R
=
\frac12
(R_d^TR-R^TR_d)^\vee.
$$

The angular-rate error is:

$$
\mathbf e_\Omega
=
\boldsymbol\omega
-
R^TR_d\boldsymbol\omega_d.
$$

The nonlinear moment command is:

$$
\boldsymbol\tau=
-K_R\mathbf e_R
-K_\Omega\mathbf e_\Omega
+\boldsymbol\omega\times J_c\boldsymbol\omega
-J_c
\left(
\hat{\boldsymbol\omega}R^TR_d\boldsymbol\omega_d
-
R^TR_d\dot{\boldsymbol\omega}_d
\right).
$$

The rigid-body nonlinear coupling:

$$
\boldsymbol\omega\times J_c\boldsymbol\omega
$$

is explicitly retained.

## 6. Rotor allocation

The controller produces:

$$
[T,\tau_x,\tau_y,\tau_z]^T.
$$

The allocation matrix converts this to four rotor thrusts:

$$
\mathbf u=M\mathbf f.
$$

Then:

$$
\omega_i=\sqrt{\frac{f_i}{k_f}}.
$$

The code also applies physical rotor-speed/thrust limits.

## 7. Why this controller?

This experiment is intended to study nonlinear control under uncertainty, not to claim that this is the only or optimal nonlinear controller.

PID and LQR are intentionally excluded from this series.

A later Yantra Gyan study will implement analytical backstepping separately and compare it against geometric control under identical conditions.
