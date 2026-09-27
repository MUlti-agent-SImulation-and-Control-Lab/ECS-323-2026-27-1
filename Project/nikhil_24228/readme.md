# Bicycle Motion Tracking & State Estimation using Smartphone Sensor Fusion via Extended Kalman Filtering
Course: ECS323 — Control Systems / State-Space Analysis
Evaluation: Mid-Semester Project Submission
Team Members:
 * Nikhil Vashisht (Roll No. 24228)
 * Vatsal Agarwal (Roll No. 24369)
## Executive Summary
This project addresses 2D bicycle position and trajectory tracking using onboard smartphone sensors (Accelerometer, Gyroscope, GPS, and Magnetometer). Because bicycle kinematics involve non-linear couplings (v\cos\theta and v\sin\theta) and multi-input multi-output (MIMO) dynamics with an unobservable sensor bias (b_\omega), standard transfer function models G(s) are insufficient.
We model the bicycle using a 5-dimensional non-linear state space and apply an Extended Kalman Filter (EKF) to fuse fast, high-frequency IMU data with slow, low-frequency absolute GPS and magnetometer updates.
Current Project Scope & Source Code Note
> Note on Implementation:
> This mid-semester submission focuses on the theoretical framework, kinematic derivations, non-linear model formulation, analytical Jacobian derivation, and observability rank analysis.
> As planned in our timeline, no empirical source code (src/), model weights (logs/), or experimental data plots (results/) are included in this deliverable. The practical implementation phase—comprising real-time smartphone IMU/GPS logging, Flutter app development, noise covariance matrix (\mathbf{Q} and \mathbf{R}) tuning, and field trials—will take place in the second half of the semester.
> 
## Repository Structure
Project/nikhil_24228/
├── README.md               # Navigation and project overview (this file)
├── requirements.txt        # LaTeX compilation dependencies & future Python packages
├── src/                    # [Future Phase] Mobile App & EKF Filter Implementation
├── logs/                   # [Future Phase] Raw IMU & GPS trip logs
├── results/                # [Future Phase] Filter trajectories & state estimation plots
└── report/
    ├── report.pdf          # Mid-Semester Evaluation Report (Compiled PDF)
    └── report.tex          # Complete LaTeX source document

## Key Theoretical Framework
1. State Vector & System Inputs
The state vector \mathbf{x}_k \in \mathbb{R}^5 and input vector \mathbf{u}_k \in \mathbb{R}^2 are defined as:
 * P_{x,k}, P_{y,k}: Global East/North coordinates (m)
 * v_k: Forward body-frame speed (m/s)
 * \theta_k: Heading angle relative to East (rad)
 * b_{\omega,k}: Gyroscope zero-rate yaw bias (rad/s)
 * a_{m,k}, \omega_{m,k}: Accelerometer forward acceleration and gyro rate inputs
2. Discrete Kinematic Model
Obtained via forward Euler discretization (\Delta t \approx 0.01\text{--}0.02\,\text{s}):
3. EKF Process Jacobian Matrix (\mathbf{F}_k)
Linearization around the current state estimate \hat{\mathbf{x}}_{k-1\vert{}k-1} yields:
4. Observability Analysis Highlights
 * Heading-only Updates (Magnetometer, No GPS): \text{rank}(\mathcal{O}) = 2. Heading \theta and gyro bias b_\omega are jointly observable, but absolute position (P_x, P_y) and speed (v) drift without bound.
 * Full GPS + Magnetometer Fused: \text{rank}(\mathcal{O}) = 5. The entire state space—including the hidden gyro bias b_\omega—becomes locally observable in a single prediction step.
Report Location & Compilation
The primary deliverable for this evaluation is the complete mid-semester derivation report:
 * PDF Version: report/report.pdf
 * LaTeX Source: report/report.tex

## Roadmap for Next Phase (Post-Midsem)
 * Mobile Data Logging: Build sensor logging utilities to record synchronised IMU (100 Hz) and GPS (1 Hz) streams from smartphone rides.
 * Filter Tuning: Determine empirical process noise \mathbf{Q} and measurement noise \mathbf{R} matrices from sensor datasheets and stationary calibration logs.
 * Application Development: Implement the derived EKF predict/update recursion in Dart/Flutter (or native Android C++/Java) for real-time onboard trajectory tracking.
 * Validation: Compare estimated trajectories against raw GPS and baseline odometry in results/.
