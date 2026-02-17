# Erwin Lejeune - 2026-02-16

# Quadrotor Dynamics Reference

This document describes the mathematical model used in `quadrotor_sim.models`.

---

## Coordinate Frames

- **World frame** `W = {xW, yW, zW}`: fixed, NED or ENU (we use ENU: z-up).
- **Body frame** `B = {xB, yB, zB}`: origin at centre of mass, xB forward,
  yB left, zB up.

## State Vector

The full 6DOF state is a 12-dimensional vector:

```
x = [x, y, z, φ, θ, ψ, vx, vy, vz, p, q, r]
```

| Symbol | Quantity | Unit |
|---|---|---|
| x, y, z | Position in world frame | m |
| φ, θ, ψ | Roll, pitch, yaw (Euler ZYX) | rad |
| vx, vy, vz | Linear velocity in world frame | m/s |
| p, q, r | Angular velocity in body frame | rad/s |

## Rotation Matrix

ZYX Euler angles → rotation from body to world:

```
R = Rz(ψ) Ry(θ) Rx(φ)
```

where:

```
Rx(φ) = [[1, 0, 0], [0, cos(φ), -sin(φ)], [0, sin(φ), cos(φ)]]
Ry(θ) = [[cos(θ), 0, sin(θ)], [0, 1, 0], [-sin(θ), 0, cos(θ)]]
Rz(ψ) = [[cos(ψ), -sin(ψ), 0], [sin(ψ), cos(ψ), 0], [0, 0, 1]]
```

## Translational Dynamics (Newton)

```
m * v̇ = R * [0, 0, T]ᵀ + [0, 0, -m*g]ᵀ - Kd * v
```

- `T` = total thrust (sum of 4 motors)
- `g` = 9.81 m/s²
- `Kd` = diagonal drag coefficient matrix

## Rotational Dynamics (Euler)

```
I * ω̇ = τ - ω × (I * ω) - Jr * [0, 0, Ω_gyro]ᵀ × ω
```

- `I` = inertia tensor `diag(Ixx, Iyy, Izz)`
- `τ = [τx, τy, τz]ᵀ` = body torques from mixer
- `Jr` = rotor moment of inertia
- `Ω_gyro` = algebraic sum of rotor speeds (gyroscopic precession)

## Motor Model

Each motor is modelled as a first-order system:

```
τ_motor * ω̇_i + ω_i = ω_cmd_i
```

Thrust and torque produced:

```
T_i = kT * ω_i²
Q_i = kQ * ω_i²
```

## Mixer (X-frame configuration)

Numbering: front-left (1, CCW), front-right (2, CW), rear-right (3, CCW),
rear-left (4, CW). Arm length `L`.

```
T   = T1 + T2 + T3 + T4
τx  = L * (T1 - T2 - T3 + T4) / √2
τy  = L * (T1 + T2 - T3 - T4) / √2
τz  = kQ/kT * (-T1 + T2 - T3 + T4)
```

Inverse (mixer matrix `M`):

```
[f1, f2, f3, f4]ᵀ = M⁻¹ * [T, τx, τy, τz]ᵀ
```

## Euler Rate Kinematics

Relation between body angular rates and Euler angle derivatives:

```
[φ̇, θ̇, ψ̇]ᵀ = W⁻¹ * [p, q, r]ᵀ
```

where:

```
W = [[1, 0, -sin(θ)],
     [0, cos(φ), cos(θ)*sin(φ)],
     [0, -sin(φ), cos(θ)*cos(φ)]]
```

## Default Parameters (Crazyflie-like)

| Parameter | Value | Unit |
|---|---|---|
| mass | 0.027 | kg |
| arm length L | 0.0397 | m |
| Ixx | 1.4e-5 | kg m² |
| Iyy | 1.4e-5 | kg m² |
| Izz | 2.17e-5 | kg m² |
| kT | 2.98e-6 | N/(rad/s)² |
| kQ | 1.14e-7 | Nm/(rad/s)² |
| motor time constant | 0.02 | s |
| max RPM | 21000 | RPM |
| drag coeff (linear) | 0.01 | N/(m/s) |
| g | 9.81 | m/s² |

## Differential Flatness

The quadrotor is a differentially flat system with flat outputs `[x, y, z, ψ]`.
This property is exploited by:

- **Min-snap trajectory generation** (section 5 of algorithms.md)
- **Feedback linearisation tracking** (section 6)
- **Geometric control** (derives desired attitude from position error)

For a desired trajectory `[x_d(t), y_d(t), z_d(t), ψ_d(t)]`, all other
states and inputs can be algebraically determined from the flat outputs and
their derivatives up to the 4th order.
