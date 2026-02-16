# Erwin Lejeune - 2026-02-16

# Autonomous Quadrotor Guide

[![CI](https://github.com/guilyx/autonomous-quadrotor-guide/actions/workflows/ci.yml/badge.svg)](https://github.com/guilyx/autonomous-quadrotor-guide/actions/workflows/ci.yml)

Python sample codes and documents about Autonomous Quadrotor algorithms. This
project can be used as a technical guide book to study the algorithms and the
software architectures for beginners.

## Table of Contents

- [What is this?](#what-is-this)
- [Goal of this project](#goal-of-this-project)
- [Requirements](#requirements)
- [Quick start](#quick-start)
- [Algorithm catalogue](#algorithm-catalogue)
  - [Dynamics and models](#dynamics-and-models)
  - [State estimation](#state-estimation)
  - [Control](#control)
  - [3D path planning](#3d-path-planning)
  - [Trajectory generation](#trajectory-generation)
  - [Trajectory tracking](#trajectory-tracking)
  - [Swarm algorithms](#swarm-algorithms)
- [Simulations](#simulations)
- [Documents](#documents)
- [Project structure](#project-structure)
- [License](#license)
- [Contributing](#contributing)
- [Author](#author)

## What is this?

A collection of reference implementations covering the full autonomy stack for
quadrotor UAVs: dynamics modelling, state estimation, flight control, 3D path
planning, trajectory generation, and multi-agent swarm algorithms. Every module
is written in Python with NumPy/SciPy so you can study, modify, and extend it
freely.

## Goal of this project

Provide a single, well-tested repository that takes you from a bare quadrotor
physics model all the way to swarm coordination, with clear documentation for
every algorithm along the path.

## Requirements

- [Python 3.12+](https://www.python.org/)
- [uv](https://docs.astral.sh/uv/) (recommended package manager)

## Quick start

```bash
# Clone
git clone https://github.com/guilyx/autonomous-quadrotor-guide.git
cd autonomous-quadrotor-guide

# Install (creates .venv automatically)
uv sync --all-groups

# Run tests
uv run pytest

# Run a simulation
uv run python simulations/hover_pid.py
```

## Algorithm catalogue

### Dynamics and models

| Module | Description |
|---|---|
| `Quadrotor6DOF` | Full 12-state rigid body dynamics with Newton-Euler equations |
| `Motor` | First-order motor dynamics with RPM limits and thrust curve |
| `Mixer` | Control allocation: `[T, τx, τy, τz]` to individual motor forces |

### State estimation

| Algorithm | Description |
|---|---|
| Extended Kalman Filter | IMU + GPS sensor fusion for 6DOF state |
| Unscented Kalman Filter | Sigma-point propagation for nonlinear dynamics |
| Complementary Filter | Lightweight gyro + accelerometer attitude fusion |
| Particle Filter | Sequential Monte Carlo state estimation |

### Control

| Algorithm | Description |
|---|---|
| Cascaded PID | Inner attitude loop + outer position loop |
| LQR | Linear Quadratic Regulator on linearised hover model |
| Geometric SO(3) | Attitude control on the rotation group (Lee et al.) |
| Model Predictive Control | Receding-horizon optimisation on linearised model |
| Sliding Mode | Robust control with discontinuous switching surface |
| Backstepping | Lyapunov-based recursive controller design |

### 3D path planning

| Algorithm | Description |
|---|---|
| 3D A* | Grid-based optimal search in voxel space |
| 3D RRT | Rapidly-exploring random tree |
| 3D RRT* | Asymptotically optimal RRT variant |
| 3D Potential Field | Attractive/repulsive force field navigation |

### Trajectory generation

| Algorithm | Description |
|---|---|
| Minimum-Snap | Mellinger & Kumar polynomial optimisation |
| Polynomial Trajectory | Arbitrary-order polynomial waypoint interpolation |

### Trajectory tracking

| Algorithm | Description |
|---|---|
| Feedback Linearisation | Differential flatness-based tracking |
| MPPI | Model Predictive Path Integral sampling-based tracker |

### Swarm algorithms

| Algorithm | Description |
|---|---|
| Reynolds Flocking | Separation + alignment + cohesion rules |
| Consensus Formation | Graph-based distributed agreement |
| Virtual Structure | Rigid virtual body formation |
| Leader-Follower | Single leader, multiple followers |
| Potential-Based Swarm | Inter-agent artificial potential fields |
| Area Coverage | Voronoi-based distributed coverage |

## Simulations

| Script | Description |
|---|---|
| `simulations/hover_pid.py` | PID-controlled hover and step response |
| `simulations/waypoint_tracking.py` | 3D waypoint following with geometric control |
| `simulations/geometric_control_demo.py` | SO(3) geometric controller demo |
| `simulations/path_planning_3d.py` | RRT* planning through obstacle field |
| `simulations/min_snap_demo.py` | Minimum-snap trajectory through waypoints |
| `simulations/swarm_flocking.py` | Reynolds flocking with N quadrotors |
| `simulations/leader_follower_demo.py` | Leader-follower formation flight |

## Documents

Detailed algorithm references are in the `docs/` directory:

- [Algorithm catalogue and roadmap](docs/algorithms.md)
- [Quadrotor dynamics](docs/dynamics.md)
- [Control theory](docs/control.md)
- [3D path planning](docs/planning.md)
- [Swarm algorithms](docs/swarm.md)

## Project structure

```
src/quadrotor_sim/
├── models/          # 6DOF dynamics, motor, control mixer
├── estimation/      # EKF, UKF, complementary filter, particle filter
├── control/         # PID, LQR, geometric, MPC, sliding mode, backstepping
├── planning/        # A*3D, RRT3D, RRT*3D, potential field
├── tracking/        # feedback linearisation, MPPI
├── swarm/           # flocking, consensus, virtual structure, leader-follower, coverage
└── visualization/   # 3D plotting and animation helpers
simulations/         # Runnable demo scripts
tests/               # pytest test suite
docs/                # Algorithm documentation
```

## License

MIT

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).

## Author

[Erwin Lejeune](https://github.com/guilyx)
