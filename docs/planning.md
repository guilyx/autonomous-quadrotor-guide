# Erwin Lejeune - 2026-02-16

# 3D Path Planning Reference

This document covers the planning and trajectory generation algorithms in
`quadrotor_sim.planning`.

---

## 1. 3D A*

### Overview

Grid-based optimal search extended to 3D voxel space.

### Algorithm

1. Discretise workspace into 3D grid with resolution `δ`.
2. Mark obstacle cells.
3. Initialise open set with start node, `g(start) = 0`.
4. Loop:
   - Pop node with lowest `f = g + h` from open set.
   - If goal reached, reconstruct path.
   - Expand 26 neighbours (6-face + 12-edge + 8-corner).
   - For each neighbour: if `g_new < g_old`, update and add to open set.

### Heuristic

Euclidean distance (admissible and consistent in 3D).

### Complexity

`O(V log V)` with binary heap, where V = number of voxels.

---

## 2. 3D RRT

### Overview

Sampling-based path planner for high-dimensional configuration spaces.

### Algorithm

1. Initialise tree with start node.
2. Loop (max iterations):
   - Sample random point `q_rand` in workspace (with goal bias).
   - Find nearest node `q_near` in tree.
   - Steer from `q_near` towards `q_rand` by step size `η`.
   - If edge is collision-free, add `q_new` to tree.
   - If `q_new` is within goal radius, return path.

### Collision checking

Line-segment vs. sphere / axis-aligned bounding box intersection.

---

## 3. 3D RRT*

### Extension of RRT

After adding `q_new`:
1. Find all nodes within radius `r = γ * (log(n)/n)^(1/3)`.
2. Choose the one that minimises cost-to-come through `q_new`.
3. Rewire: for each near node, check if routing through `q_new` is cheaper.

### Properties

- Asymptotically optimal (converges to shortest path as n → ∞).
- Probabilistically complete.

---

## 4. 3D Potential Field

### Attractive potential

```
U_att(q) = 0.5 * ζ * ||q - q_goal||²
```

### Repulsive potential

For each obstacle `o` with influence distance `ρ₀`:

```
U_rep(q) = 0.5 * η * (1/ρ(q,o) - 1/ρ₀)²   if ρ(q,o) ≤ ρ₀
         = 0                                  otherwise
```

### Force (negative gradient)

```
F(q) = -∇U_att(q) - Σ ∇U_rep(q)
```

### Local minima escape

Random perturbation or Brownian motion when `||F|| < ε` and goal not reached.

---

## 5. Minimum-Snap Trajectory

### Problem

Given waypoints `{w0, w1, ..., wM}` and segment times `{T1, ..., TM}`, find
polynomial trajectories `σ_k(t)` per segment that minimise:

```
J = Σ_{k=1}^{M} ∫₀^{Tk} (d⁴σ_k/dt⁴)² dt
```

### Solution

Each segment is a 7th-order polynomial (8 coefficients). Constraints:
- Position continuity at waypoints.
- Velocity, acceleration, jerk continuity at internal waypoints.
- Boundary conditions (start/end at rest).

Assembled into a large QP and solved via block-matrix inversion or sparse QP.

### Why snap?

Quadrotors are differentially flat with flat outputs `[x, y, z, ψ]`.
The motor inputs depend on up to the 4th derivative of position (snap).
Minimising snap therefore minimises control effort.

---

## 6. Polynomial Trajectory

### Simpler alternative to min-snap

Given waypoints and desired order `n`, fit an `n`th-order polynomial per
segment with user-specified boundary conditions.

Useful when snap optimality is not required but smooth trajectories are needed.
