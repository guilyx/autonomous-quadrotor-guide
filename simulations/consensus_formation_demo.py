# Erwin Lejeune - 2026-02-16
"""Consensus-based formation convergence demo."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from quadrotor_sim.swarm.consensus_formation import ConsensusFormation


def main() -> None:
    """Simulate 4 agents converging to a square formation."""
    N = 4
    adj = np.ones((N, N)) - np.eye(N)
    side = 2.0
    offsets = np.array(
        [
            [-side / 2, -side / 2, 0],
            [side / 2, -side / 2, 0],
            [side / 2, side / 2, 0],
            [-side / 2, side / 2, 0.0],
        ]
    )
    ctrl = ConsensusFormation(adj, offsets, gain=2.0)

    rng = np.random.default_rng(42)
    pos = rng.uniform(-10, 10, (N, 3))

    dt = 0.01
    steps = 800
    history = np.zeros((steps, N, 3))

    for t in range(steps):
        forces = ctrl.compute_forces(pos)
        pos += forces * dt
        history[t] = pos.copy()

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    colours = ["r", "g", "b", "m"]
    for i in range(N):
        ax.plot(
            history[:, i, 0],
            history[:, i, 1],
            history[:, i, 2],
            f"{colours[i]}-",
            alpha=0.5,
        )
        ax.scatter(*history[-1, i], s=60, c=colours[i], marker="o")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"Consensus Formation ({N} agents → square)")
    plt.show()


if __name__ == "__main__":
    main()
