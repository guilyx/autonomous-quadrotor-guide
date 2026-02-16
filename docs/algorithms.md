# Erwin Lejeune - 2026-02-16

# Algorithm Catalogue and Roadmap

This document lists every algorithm implemented (or planned) in the
`quadrotor_sim` library, grouped by domain. Each entry links to its
source module and its corresponding detailed doc page.

---

## 1. Dynamics and Models

### 1.1 Quadrotor 6DOF Rigid Body

- **Module**: `quadrotor_sim.models.quadrotor`
- **State vector**: `[x, y, z, φ, θ, ψ, vx, vy, vz, p, q, r]` (12 states)
- **Inputs**: 4 rotor thrusts `[f1, f2, f3, f4]`
- **Equations**: Newton-Euler formulation in body frame
- **Features**: configurable mass, inertia tensor, arm length, drag
- **Reference**: Mahony, Kumar, Corke, "Multirotor aerial vehicles: Modelling,
  estimation and control of quadrotor", IEEE RAM, 2012

### 1.2 Motor Model

- **Module**: `quadrotor_sim.models.motor`
- **Dynamics**: first-order lag `τ * ω̇ + ω = ω_cmd`
- **Thrust curve**: `T = kT * ω²`, `Q = kQ * ω²`
- **Constraints**: `ω_min ≤ ω ≤ ω_max`

### 1.3 Control Mixer

- **Module**: `quadrotor_sim.models.mixer`
- **Function**: maps `[T, τx, τy, τz]` → `[f1, f2, f3, f4]`
- **Configuration**: X-frame and +-frame supported
- **Inverse**: maps motor forces back to body wrench

---

## 2. State Estimation

Detailed theory: [docs/dynamics.md](dynamics.md)

### 2.1 Extended Kalman Filter (EKF)

- **Module**: `quadrotor_sim.estimation.ekf`
- **Sensors fused**: IMU (accelerometer + gyroscope) + GPS
- **State**: 12-state or reduced 6-state (position + velocity)
- **Reference**: Thrun, Burgard, Fox, "Probabilistic Robotics", Chapter 3

### 2.2 Unscented Kalman Filter (UKF)

- **Module**: `quadrotor_sim.estimation.ukf`
- **Advantage**: handles nonlinearities better than EKF (no Jacobians)
- **Sigma points**: Merwe scaled sigma point selection
- **Reference**: Julier & Uhlmann, 2004

### 2.3 Complementary Filter

- **Module**: `quadrotor_sim.estimation.complementary_filter`
- **Purpose**: lightweight attitude estimation from gyro + accel
- **Tuning**: single parameter α ∈ (0, 1)
- **Use case**: fast inner-loop attitude feedback

### 2.4 Particle Filter

- **Module**: `quadrotor_sim.estimation.particle_filter`
- **Method**: sequential importance resampling (SIR)
- **Advantage**: handles arbitrary nonlinear/non-Gaussian distributions
- **Trade-off**: computationally heavier than EKF/UKF

---

## 3. Control

Detailed theory: [docs/control.md](control.md)

### 3.1 Cascaded PID

- **Module**: `quadrotor_sim.control.pid_controller`
- **Architecture**: outer position PID → desired angles → inner attitude PID → motor commands
- **Gains**: `Kp`, `Ki`, `Kd` per axis (x, y, z, φ, θ, ψ)
- **Anti-windup**: integral clamping

### 3.2 LQR (Linear Quadratic Regulator)

- **Module**: `quadrotor_sim.control.lqr_controller`
- **Linearisation**: about hover equilibrium
- **Design**: solve continuous Algebraic Riccati Equation (ARE)
- **Tuning**: Q (state cost) and R (input cost) matrices

### 3.3 Geometric Control on SO(3)

- **Module**: `quadrotor_sim.control.geometric_controller`
- **Theory**: Lee, Leok, McClamroch, "Geometric tracking control of a
  quadrotor UAV on SE(3)", CDC 2010
- **Advantage**: singularity-free attitude representation
- **Features**: almost-global asymptotic stability

### 3.4 Model Predictive Control (MPC)

- **Module**: `quadrotor_sim.control.mpc_controller`
- **Method**: linearise dynamics at each step, solve QP over horizon
- **Constraints**: input saturation, state bounds
- **Solver**: SciPy or simple QP formulation

### 3.5 Sliding Mode Control

- **Module**: `quadrotor_sim.control.sliding_mode_controller`
- **Surface**: linear combination of tracking error and its derivative
- **Reaching law**: exponential + constant rate
- **Robustness**: invariant to matched disturbances

### 3.6 Backstepping Control

- **Module**: `quadrotor_sim.control.backstepping_controller`
- **Method**: recursive Lyapunov function design
- **Layers**: position → velocity → attitude → angular rate
- **Guarantee**: global asymptotic stability under model assumptions

---

## 4. 3D Path Planning

Detailed theory: [docs/planning.md](planning.md)

### 4.1 3D A*

- **Module**: `quadrotor_sim.planning.astar_3d`
- **Space**: voxelised 3D grid
- **Heuristic**: Euclidean distance (admissible)
- **Connectivity**: 26-connected neighbourhood

### 4.2 3D RRT

- **Module**: `quadrotor_sim.planning.rrt_3d`
- **Method**: randomly sample, extend tree towards sample
- **Collision check**: line-sphere / line-AABB intersection
- **Termination**: goal region reached or max iterations

### 4.3 3D RRT*

- **Module**: `quadrotor_sim.planning.rrt_star_3d`
- **Extension of**: RRT with rewiring for asymptotic optimality
- **Near-neighbour radius**: `r = γ * (log(n)/n)^(1/d)` where d=3
- **Reference**: Karaman & Frazzoli, IJRR, 2011

### 4.4 3D Potential Field

- **Module**: `quadrotor_sim.planning.potential_field_3d`
- **Attractive**: quadratic pull towards goal
- **Repulsive**: inverse-distance push from obstacles
- **Limitation**: local minima (addressed by random walk escape)

---

## 5. Trajectory Generation

### 5.1 Minimum-Snap Trajectory

- **Module**: `quadrotor_sim.planning.min_snap`
- **Theory**: Mellinger & Kumar, "Minimum snap trajectory generation and
  control for quadrotors", ICRA 2011
- **Method**: 7th-order polynomial per segment, minimise ∫ snap² dt
- **Constraints**: position, velocity, acceleration continuity at waypoints

### 5.2 Polynomial Trajectory

- **Module**: `quadrotor_sim.planning.polynomial_trajectory`
- **Method**: arbitrary-order polynomial interpolation
- **Boundary conditions**: user-specified at each waypoint
- **Use case**: simpler alternative to min-snap when snap optimality is not needed

---

## 6. Trajectory Tracking

### 6.1 Feedback Linearisation

- **Module**: `quadrotor_sim.tracking.feedback_linearisation`
- **Theory**: exploit differential flatness of quadrotor dynamics
- **Flat outputs**: `[x, y, z, ψ]`
- **Result**: reduces tracking to linear error dynamics

### 6.2 MPPI (Model Predictive Path Integral)

- **Module**: `quadrotor_sim.tracking.mppi`
- **Method**: sample K random control sequences, weight by cost
- **Advantage**: derivative-free, handles non-convex costs
- **Reference**: Williams et al., "Information-theoretic MPC", T-RO 2018

---

## 7. Swarm Algorithms

Detailed theory: [docs/swarm.md](swarm.md)

### 7.1 Reynolds Flocking

- **Module**: `quadrotor_sim.swarm.reynolds_flocking`
- **Rules**: separation, alignment, cohesion
- **Parameters**: neighbour radius, rule weights
- **Reference**: Reynolds, "Flocks, herds and schools", SIGGRAPH 1987

### 7.2 Consensus-Based Formation

- **Module**: `quadrotor_sim.swarm.consensus_formation`
- **Method**: distributed agreement on formation offsets via graph Laplacian
- **Graph**: configurable adjacency (ring, complete, custom)
- **Convergence**: guaranteed for connected graphs

### 7.3 Virtual Structure Formation

- **Module**: `quadrotor_sim.swarm.virtual_structure`
- **Method**: define a rigid virtual body; agents track assigned points on it
- **Advantage**: formation shape is exactly maintained
- **Trade-off**: centralised knowledge of virtual body state

### 7.4 Leader-Follower

- **Module**: `quadrotor_sim.swarm.leader_follower`
- **Method**: one leader follows a trajectory; followers maintain relative offsets
- **Control**: each follower uses its own PID/LQR to track offset
- **Scalability**: tree or chain topologies supported

### 7.5 Potential-Based Swarm Navigation

- **Module**: `quadrotor_sim.swarm.potential_swarm`
- **Method**: inter-agent repulsion + goal attraction + obstacle avoidance
- **Equilibrium**: Lennard-Jones-like potential for desired inter-agent spacing
- **Use case**: obstacle-rich environments

### 7.6 Area Coverage

- **Module**: `quadrotor_sim.swarm.coverage`
- **Method**: Voronoi-based Lloyd's algorithm in 2D/3D
- **Objective**: minimise coverage cost (each agent covers its Voronoi cell)
- **Reference**: Cortes, Martinez, Karatas, Bullo, "Coverage control for
  mobile sensing networks", T-RO 2004

---

## Implementation status

| # | Algorithm | Status |
|---|---|---|
| 1 | Quadrotor 6DOF | Planned |
| 2 | Motor model | Planned |
| 3 | Control mixer | Planned |
| 4 | EKF | Planned |
| 5 | UKF | Planned |
| 6 | Complementary filter | Planned |
| 7 | Particle filter | Planned |
| 8 | Cascaded PID | Planned |
| 9 | LQR | Planned |
| 10 | Geometric SO(3) | Planned |
| 11 | MPC | Planned |
| 12 | Sliding mode | Planned |
| 13 | Backstepping | Planned |
| 14 | 3D A* | Planned |
| 15 | 3D RRT | Planned |
| 16 | 3D RRT* | Planned |
| 17 | 3D Potential field | Planned |
| 18 | Min-snap trajectory | Planned |
| 19 | Polynomial trajectory | Planned |
| 20 | Feedback linearisation | Planned |
| 21 | MPPI | Planned |
| 22 | Reynolds flocking | Planned |
| 23 | Consensus formation | Planned |
| 24 | Virtual structure | Planned |
| 25 | Leader-follower | Planned |
| 26 | Potential swarm | Planned |
| 27 | Area coverage | Planned |
