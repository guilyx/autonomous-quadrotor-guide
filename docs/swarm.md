# Erwin Lejeune - 2026-02-16

# Swarm Algorithms Reference

This document covers the multi-agent algorithms in `quadrotor_sim.swarm`.

---

## 1. Reynolds Flocking

### Rules (per agent `i`)

1. **Separation**: steer away from neighbours that are too close.

   ```
   f_sep_i = -Σ_{j ∈ N_i, d<r_sep} (p_j - p_i) / ||p_j - p_i||²
   ```

2. **Alignment**: match velocity of nearby neighbours.

   ```
   f_ali_i = (1/|N_i|) Σ_{j ∈ N_i} v_j - v_i
   ```

3. **Cohesion**: steer towards the centroid of neighbours.

   ```
   f_coh_i = (1/|N_i|) Σ_{j ∈ N_i} p_j - p_i
   ```

### Combined force

```
f_i = w_sep * f_sep_i + w_ali * f_ali_i + w_coh * f_coh_i
```

### Parameters

- `r_percept`: perception radius (defines neighbourhood `N_i`)
- `r_sep`: separation radius
- `w_sep, w_ali, w_coh`: rule weights

---

## 2. Consensus-Based Formation

### Graph model

Agents are nodes in a communication graph `G = (V, E)` with adjacency
matrix `A` and Laplacian `L = D - A`.

### Protocol

Each agent updates its position towards a desired formation offset `δ_i`:

```
u_i = -Σ_{j ∈ N_i} a_ij * ((p_i - δ_i) - (p_j - δ_j))
     = -[L * (p - δ)]_i
```

### Convergence

For a connected graph, all agents converge to the desired formation
exponentially fast with rate `λ₂(L)` (algebraic connectivity).

### Supported topologies

Ring, complete, star, and arbitrary custom adjacency matrices.

---

## 3. Virtual Structure Formation

### Concept

Define a **virtual rigid body** with position `p_v(t)` and orientation `R_v(t)`.
Each agent `i` is assigned a fixed offset `d_i` in the virtual body frame.

### Desired position per agent

```
p_i_des(t) = p_v(t) + R_v(t) * d_i
```

### Control

Each agent uses its own controller (PID, LQR, etc.) to track `p_i_des(t)`.

### Advantage

The formation shape is exactly maintained (not just convergent).

### Trade-off

Requires all agents to know `p_v(t)` (centralised knowledge or broadcast).

---

## 4. Leader-Follower

### Architecture

- **Leader**: follows a predefined trajectory autonomously.
- **Followers**: maintain desired offsets relative to the leader (or to their
  parent in a tree topology).

### Control law for follower `i`

```
u_i = controller(p_i, p_leader + δ_i)
```

where `δ_i` is the desired offset in world or leader-body frame.

### Topologies

- **Star**: all followers reference the leader directly.
- **Chain**: follower `i` references follower `i-1`.
- **Tree**: arbitrary parent-child relationships.

---

## 5. Potential-Based Swarm Navigation

### Inter-agent potential

Lennard-Jones-like potential for desired spacing `d_des`:

```
U_ij(r) = ε * [(d_des/r)^a - (d_des/r)^b]
```

where `a > b > 0` (e.g. `a=4, b=2`). Equilibrium at `r = d_des`.

### Forces on agent `i`

```
f_i = -Σ_{j≠i} ∇U_ij + f_goal_i + f_obstacle_i
```

- `f_goal_i`: attractive force towards mission goal
- `f_obstacle_i`: repulsive force from obstacles

### Properties

Emergent lattice-like formations with guaranteed minimum spacing.

---

## 6. Area Coverage (Voronoi-Based)

### Problem

Partition a region `Q` among `N` agents such that each agent covers its
vicinity optimally.

### Algorithm (Lloyd's algorithm)

1. Compute Voronoi tessellation of `Q` given agent positions.
2. Each agent moves towards the centroid of its Voronoi cell:

   ```
   p_i ← centroid(V_i)
   ```

3. Repeat until convergence.

### Cost function

```
H(p) = Σ_{i=1}^{N} ∫_{V_i} ||q - p_i||² φ(q) dq
```

where `φ(q)` is a density function (importance weighting).

### Extension to 3D

Use 3D Voronoi cells (via Delaunay tetrahedralisation or approximate methods).

### Reference

Cortes, Martinez, Karatas, Bullo, "Coverage control for mobile sensing
networks", IEEE T-RO, 2004.
