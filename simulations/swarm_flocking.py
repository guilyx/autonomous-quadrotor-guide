# Erwin Lejeune - 2026-02-16
"""Reynolds flocking simulation with N quadrotors."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from quadrotor_sim.swarm.reynolds_flocking import ReynoldsFlocking


def main() -> None:
    """Simulate Reynolds flocking behaviour in 3D."""
    N = 15
    rng = np.random.default_rng(42)
    positions = rng.uniform(0, 20, (N, 3))
    velocities = rng.uniform(-1, 1, (N, 3))

    flock = ReynoldsFlocking(r_percept=8.0, r_sep=2.0, w_sep=3.0, w_ali=1.5, w_coh=1.0)

    dt = 0.05
    steps = 300
    history = np.zeros((steps, N, 3))

    for t in range(steps):
        forces = flock.compute_forces(positions, velocities)
        velocities += forces * dt
        speed = np.linalg.norm(velocities, axis=1, keepdims=True)
        max_speed = 3.0
        velocities = np.where(
            speed > max_speed, velocities / speed * max_speed, velocities
        )
        positions += velocities * dt
        history[t] = positions.copy()

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    for i in range(N):
        ax.plot(history[:, i, 0], history[:, i, 1], history[:, i, 2], alpha=0.5)
        ax.scatter(*history[-1, i], s=30, marker="o")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"Reynolds Flocking ({N} agents)")
    plt.show()


if __name__ == "__main__":
    main()
