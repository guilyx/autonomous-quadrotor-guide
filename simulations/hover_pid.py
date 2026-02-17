# Erwin Lejeune - 2026-02-16
"""PID-controlled hover and step response simulation."""

from __future__ import annotations

import numpy as np

from quadrotor_sim.control.pid_controller import CascadedPIDController
from quadrotor_sim.models.quadrotor import Quadrotor
from quadrotor_sim.visualization import plot_state_history, plot_trajectory_3d


def main() -> None:
    """Run hover PID simulation: take off to 1 m then hold."""
    quad = Quadrotor()
    quad.reset(position=np.array([0.0, 0.0, 0.0]))

    ctrl = CascadedPIDController()
    target = np.array([0.0, 0.0, 1.0])

    dt = 0.001
    duration = 5.0
    steps = int(duration / dt)

    times = np.zeros(steps)
    states = np.zeros((steps, 12))

    for i in range(steps):
        wrench = ctrl.compute(quad.state, target, dt=dt)
        quad.step(wrench, dt)
        times[i] = quad.time
        states[i] = quad.state

    print(f"Final position: {quad.position}")
    print(f"Final altitude error: {abs(quad.position[2] - target[2]):.4f} m")

    plot_trajectory_3d(states[:, :3], title="PID Hover: Trajectory")
    plot_state_history(times, states, title="PID Hover: State History")


if __name__ == "__main__":
    main()
