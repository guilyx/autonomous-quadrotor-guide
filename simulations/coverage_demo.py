# Erwin Lejeune - 2026-02-16
"""Voronoi-based area coverage (Lloyd's algorithm) demo."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from quadrotor_sim.swarm.coverage import CoverageController


def main() -> None:
    """Simulate 6 agents covering a 2-D workspace."""
    bounds = np.array([[0.0, 0.0], [20.0, 20.0]])
    N = 6
    rng = np.random.default_rng(42)
    pos = rng.uniform(0, 20, (N, 2))

    cc = CoverageController(bounds, resolution=0.5, gain=2.0)

    dt = 0.05
    steps = 200
    history = np.zeros((steps, N, 2))

    for t in range(steps):
        forces = cc.compute_forces(pos)
        pos += forces * dt
        # Keep inside workspace.
        pos = np.clip(pos, bounds[0] + 0.1, bounds[1] - 0.1)
        history[t] = pos.copy()

    fig, ax = plt.subplots(figsize=(8, 8))
    colours = plt.cm.Set2(np.linspace(0, 1, N))
    for i in range(N):
        ax.plot(history[:, i, 0], history[:, i, 1], "-", color=colours[i], alpha=0.5)
        ax.scatter(history[-1, i, 0], history[-1, i, 1], s=80, color=colours[i], zorder=5)
    ax.set_xlim(bounds[0, 0], bounds[1, 0])
    ax.set_ylim(bounds[0, 1], bounds[1, 1])
    ax.set_aspect("equal")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title(f"Voronoi Coverage ({N} agents)")
    ax.grid(True, alpha=0.3)
    plt.show()


if __name__ == "__main__":
    main()
