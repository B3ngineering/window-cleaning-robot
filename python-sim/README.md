# Cable Robot Simulator

A Python simulator for a 4-cable parallel robot used in window cleaning applications.

## Overview

This simulator models a cable-driven parallel robot (CDPR) where four motors at the corners of a frame control the position of an end effector via cables.

```
Motor Layout (2m × 2m frame):

  M1 (0,2) -------- M2 (2,2)
      |\            /|
      | \  cables  / |
      |  \   EE   /  |
      |  /        \  |
      | /          \ |
  M4 (0,0) -------- M3 (2,0)
```

## Components

| Class | Purpose |
|-------|---------|
| `RobotConfig` | Physical parameters (motor positions, cable limits, speeds) |
| `CableRobotKinematics` | Inverse/forward kinematics calculations |
| `PIDController` | Per-motor PID control for cable length tracking |
| `CableRobotSimulator` | Main simulation loop integrating kinematics + control |
| `RealTimeVisualizer` | Live matplotlib visualization with keyboard control |

## Kinematics

**Inverse Kinematics (IK):** Given a desired position `[x, z]`, calculate the required cable lengths. This is simple geometry—each cable length is the distance from the end effector to its motor.

**Forward Kinematics (FK):** Given cable lengths, determine position. This is solved iteratively using damped least squares (Levenberg-Marquardt), since the system is overdetermined (4 cables, 2 DOF).

## Usage

### Run the Interactive Demo

```bash
cd python-sim
python3 cable_robot_simulator.py
```

### Keyboard Controls

| Key | Action |
|-----|--------|
| ↑ ↓ ← → | Move target position |
| C | Return to center |
| R | Reset PID controllers |

### Programmatic Usage

```python
from cable_robot_simulator import RobotConfig, CableRobotSimulator

config = RobotConfig()
sim = CableRobotSimulator(config)

# Move to a target position
sim.run_to_target(np.array([0.5, 1.5]))

# Or step manually
sim.set_desired_position(np.array([1.2, 0.8]))
for _ in range(100):
    sim.step()
    print(sim.ee_position)
```

## Visualization

The real-time display shows three plots:

1. **Robot View** — Cable configuration, end effector position, and target
2. **Tracking Error** — Position error over time
3. **Cable Lengths** — Actual vs desired cable lengths for all 4 motors

## Requirements

```
numpy
matplotlib
```

Install with:
```bash
pip install numpy matplotlib
```

## File Structure

```
python-sim/
├── cable_robot_simulator.py   # Main simulator code
├── demo_simulatory.py         # Additional demos
└── README.md
```
