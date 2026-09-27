# Model Weights & Execution Logs

This project implements an analytical Lyapunov-based kinematic state-feedback controller (Equation 8 from Tiwari & Nath, 2025) and unicycle dynamic simulation.

Because the control strategy is derived from first-principles analytical stability proofs rather than learned parameters, no neural network weights, learned checkpoints, or pre-trained model files are required.

Simulation execution traces, telemetry, and numerical metric outputs are computed dynamically during demonstration runs and visualized in the `results/` directory.
