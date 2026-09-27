# ECS-323: Course Project Submission

## Project: Robot Conga: Sequential Path Following for Multi-Agent Systems
**Student Name:** Ruvaifa  
**Roll Number:** 24211  
**Course:** ECS-323 (Multi-Agent Simulation and Control)  

---

## 1. Project Overview

This repository contains the complete implementation and verification of the **Robot Conga** sequential path-following algorithm for multi-agent systems, based on the research paper:
> *Robot Conga: A Leader-Follower Walking Approach to Sequential Path Following in Multi-Agent Systems* (Tiwari & Nath, 2025).

Unlike traditional rigid formation control or time-delay following ($t - \tau$), Robot Conga enforces spatial propagation:
$$s_i = s_L - i \cdot d$$
where each follower robot $i$ receives a reference state evaluated at its specific arc-length offset $s_i$ along a shared geometric path. The trajectory is tracked using a Lyapunov-derived nonlinear controller (Equation 8) coupled with singularity safeguards, velocity saturation, actuation delays (100 ms), and discrete noisy measurements (15 Hz).

---

## 2. Repository Structure

```
Project/ruvaifa_24211/
├── README.md              # Complete execution and navigation instructions
├── requirements.txt       # Project dependencies (numpy, matplotlib, pytest)
├── src/                   # Complete project source code
│   ├── robot_conga/       # Core mathematical and simulation package
│   │   ├── __init__.py    # API exports
│   │   ├── controller.py  # Equation (8) Lyapunov controller & singularity guard
│   │   ├── geometry.py    # Angle wrapping, clamp, Euclidean distance utilities
│   │   ├── metrics.py     # RMSE, max deviation, and spacing error evaluation
│   │   ├── models.py      # Dataclasses (Pose, Twist, PathState, ReferenceState)
│   │   ├── path.py        # Path interface (StraightPath, CirclePath, SCurvePath)
│   │   ├── plotting.py    # Matplotlib visualizer for trajectory and errors
│   │   ├── propagator.py  # Spatial arc-length reference propagator
│   │   └── simulator.py   # Unicycle dynamics, delay queue, sensor noise
│   ├── examples/          # Ready-to-run demo scripts
│   │   ├── straight_demo.py     # Straight-line tracking simulation (Vcmd = 0.1 m/s)
│   │   ├── circle_demo.py       # Circular arc tracking simulation (R = 2.0 m)
│   │   ├── conga_demo.py        # 4-robot Conga line on S-curve (generates 5 plots)
│   │   └── singularity_demo.py  # Denominator singularity guard verification
│   ├── test/              # Comprehensive test suite (31 tests)
│   │   ├── test_controller.py
│   │   ├── test_geometry.py
│   │   ├── test_metrics.py
│   │   ├── test_path.py
│   │   ├── test_propagator.py
│   │   └── test_simulator.py
│   ├── package.xml        # ROS 2 Humble ament_python manifest
│   ├── setup.py           # Python package build script
│   └── setup.cfg          # Package and test configuration
├── logs/                  # Notes on analytical model-free architecture
├── results/               # Generated performance plots (.png)
└── report/
    └── report.pdf         # Detailed academic project report
```

---

## 3. Installation & Prerequisites

The codebase is implemented in Python 3 (compatible with Python 3.10+) and ROS 2 Humble.

### Step 1: Navigate to the Project Directory
```bash
cd Project/ruvaifa_24211
```

### Step 2: Install Dependencies
```bash
pip install -r requirements.txt
```

---

## 4. Step-by-Step Execution Guide

All demonstration scripts run out-of-the-box without requiring code modifications.

### 4.1 Running the Multi-Robot Conga Simulation (Main Experiment)
Simulates a 4-robot formation (1 leader + 3 followers) maintaining 1.0 m spacing along an S-curve path under 100 ms actuation delay and 15 Hz measurement noise.
```bash
python src/examples/conga_demo.py
```
**Outputs Generated in `results/`:**
1. `results/xy_trajectory.png`: Spatial trajectories of leader and 3 followers along the path.
2. `results/position_error.png`: Tracking position error over time for all agents.
3. `results/heading_error.png`: Absolute heading error over time for all agents.
4. `results/inter_robot_spacing.png`: Inter-agent spacing between adjacent pairs compared to the 1.0 m target line.
5. `results/arc_length_position.png`: Temporal progression of arc lengths $s_i(t)$.

### 4.2 Running the Straight-Line Tracking Experiment
Evaluates single-agent trajectory tracking on a straight path at $V_{\text{cmd}} = 0.10\text{ m/s}$.
```bash
python src/examples/straight_demo.py
```
**Output:** `results/straight_demo.png`

### 4.3 Running the Circular Trajectory Experiment
Evaluates circular path tracking ($R = 2.0\text{ m}, \kappa = 0.5\text{ m}^{-1}, V_{\text{cmd}} = 0.10\text{ m/s}$).
```bash
python src/examples/circle_demo.py
```
**Output:** `results/circle_demo.png`

### 4.4 Running the Singularity Safety Guard Demonstration
Verifies that when heading approaches $\pm \pi/2$ (alignment with global Y-axis), the denominator guard explicitly detects numerical instability and raises `ControllerSingularityError`, preventing `NaN` or `inf` propagation.
```bash
python src/examples/singularity_demo.py
```

---

## 5. Running the Test Suite

A comprehensive test suite of 31 automated tests verifies geometry, paths, reference propagation, controller stability, unicycle simulation, and metrics.

### Using Pytest (Direct)
```bash
pytest src/test -v
```
**Expected Result:** `31 passed in ~0.25s` (100% pass rate).

### Using Colcon (ROS 2 Workspace)
From the `src/` directory or ROS 2 workspace root:
```bash
colcon build --packages-select robot_conga
colcon test --packages-select robot_conga --python-testing pytest
colcon test-result --verbose
```
**Expected Result:** `Summary: 31 tests, 0 errors, 0 failures, 0 skipped`.

---

## 6. Verification with Report

The quantitative results generated by running `python src/examples/conga_demo.py` match the tables and figures in `report/report.pdf`:

| Metric / Agent | Leader (Robot 0) | Follower 1 | Follower 2 | Follower 3 |
|---|---|---|---|---|
| **Position RMSE** | 20.32 mm | 68.31 mm | 44.87 mm | 17.02 mm |
| **Max Position Error** | 58.47 mm | 126.54 mm | 121.41 mm | 54.76 mm |
| **Heading RMSE** | 5.76 deg | 13.42 deg | 9.43 deg | 6.31 deg |
| **Spacing RMSE to Preceding Agent** | N/A | 34.62 mm (0 $\to$ 1) | 31.35 mm (1 $\to$ 2) | 27.93 mm (2 $\to$ 3) |

All plots referenced in the report are located in `results/` and the complete technical report is available at `report/report.pdf`.
