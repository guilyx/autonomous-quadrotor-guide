# Erwin Lejeune - 2026-02-16
"""Leader-follower formation flight demo."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from quadrotor_sim.swarm.leader_follower import LeaderFollower


def main() -> None:
    """Simulate a leader following a circular trajectory with 3 followers."""
    offsets = np.array([[2, 0, 0], [-2, 0, 0], [0, 2, 0.0]])
    lf = LeaderFollower(offsets, kp=3.0, kd=2.0)

    dt = 0.02
    steps = 500

    # Leader follows a helix.
    leader_positions = np.zeros((steps, 3))
    leader_velocities = np.zeros((steps, 3))
    for t in range(steps):
        s = t * dt
        leader_positions[t] = [3 * np.cos(0.5 * s), 3 * np.sin(0.5 * s), 0.5 * s]
        leader_velocities[t] = [-1.5 * np.sin(0.5 * s), 1.5 * np.cos(0.5 * s), 0.5]

    follower_positions = np.zeros((steps, 3, 3))
    follower_velocities = np.zeros((3, 3))
    follower_pos = leader_positions[0] + offsets

    for t in range(steps):
        forces = lf.compute_forces(
            leader_positions[t], leader_velocities[t], follower_pos, follower_velocities
        )
        follower_velocities += forces * dt
        follower_pos += follower_velocities * dt
        follower_positions[t] = follower_pos.copy()

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(*leader_positions.T, "k-", linewidth=2, label="Leader")
    colours = ["r", "g", "b"]
    for i in range(3):
        ax.plot(*follower_positions[:, i, :].T, f"{colours[i]}-", alpha=0.7, label=f"Follower {i}")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Leader-Follower Formation")
    ax.legend()
    plt.show()


if __name__ == "__main__":
    main()
