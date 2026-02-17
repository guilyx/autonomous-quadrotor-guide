# Erwin Lejeune - 2026-02-16

# Control Theory Reference

This document covers the control algorithms implemented in
`quadrotor_sim.control`.

---

## 1. Cascaded PID

The most common quadrotor controller architecture.

### Outer loop (position)

For each axis `{x, y, z}`:

```
e = x_des - x
a_des = Kp * e + Ki * ∫e dt + Kd * ė
```

The desired accelerations map to a desired thrust vector, from which desired
roll/pitch angles are extracted:

```
T_des = m * ||a_des + g||
φ_des = arcsin((a_des_x * sin(ψ) - a_des_y * cos(ψ)) / ||a_des + g||)
θ_des = arcsin((a_des_x * cos(ψ) + a_des_y * sin(ψ)) / (||a_des + g|| * cos(φ)))
```

### Inner loop (attitude)

For each angle `{φ, θ, ψ}`:

```
e_att = angle_des - angle
τ = Kp_att * e_att + Kd_att * (ω_des - ω)
```

Output `[T, τx, τy, τz]` is sent to the mixer.

---

## 2. LQR

Linearise the quadrotor dynamics about hover:

```
ẋ = A * δx + B * δu
```

where `δx = x - x_hover` and `δu = u - u_hover`.

Solve the continuous-time Algebraic Riccati Equation:

```
A'P + PA - PBR⁻¹B'P + Q = 0
```

Gain: `K = R⁻¹ B' P`

Control law: `δu = -K * δx`

---

## 3. Geometric Control on SO(3)

Following Lee, Leok, McClamroch (CDC 2010).

### Position control

```
e_x = x - x_d
e_v = v - v_d
F_des = -Kx * e_x - Kv * e_v + m*g*e3 + m*a_d
```

### Desired attitude

Construct desired rotation `R_d` from `F_des` and desired yaw `ψ_d`:

```
b3_d = F_des / ||F_des||
b1_d = [-sin(ψ_d), cos(ψ_d), 0]'
b2_d = b3_d × b1_d / ||b3_d × b1_d||
b1_d = b2_d × b3_d
R_d = [b1_d, b2_d, b3_d]
```

### Attitude control

```
e_R = 0.5 * vee(R_d' R - R' R_d)
e_ω = ω - R' R_d ω_d
τ = -K_R * e_R - K_ω * e_ω + ω × I ω
```

### Thrust

```
T = F_des · R e3
```

---

## 4. Model Predictive Control

At each time step, solve the finite-horizon optimal control problem:

```
min  Σ_{k=0}^{N-1} [x_k' Q x_k + u_k' R u_k] + x_N' Q_f x_N
s.t. x_{k+1} = A_d x_k + B_d u_k
     u_min ≤ u_k ≤ u_max
     x_min ≤ x_k ≤ x_max
```

The linearised discrete model `(A_d, B_d)` is obtained from the continuous
hover-linearised model via zero-order hold discretisation.

Apply only the first control input, then re-solve at the next time step.

---

## 5. Sliding Mode Control

Define sliding surface per axis:

```
s = ė + λ * e
```

Control law:

```
u = u_eq + u_sw
u_eq = (nominal control to stay on surface)
u_sw = -K_s * sign(s)   (or saturation function for chattering reduction)
```

Properties: invariant to matched disturbances once on the sliding surface.

---

## 6. Backstepping Control

Recursive Lyapunov design in 4 layers:

1. **Position error** → desired velocity
2. **Velocity error** → desired thrust vector / attitude
3. **Attitude error** → desired angular rates
4. **Angular rate error** → motor torques

Each step defines a virtual control and a Lyapunov function whose derivative
is made negative semi-definite by the choice of the next layer's control.

The overall Lyapunov function is the sum:

```
V = V_pos + V_vel + V_att + V_rate
V̇ ≤ -α * V
```

guaranteeing exponential stability.
