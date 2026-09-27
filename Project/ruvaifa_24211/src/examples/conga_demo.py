"""Demo: Multi-agent Robot Conga sequential path following experiment.

Simulates 4 robots (1 leader + 3 followers) maintaining 1.0 m spatial separation
along a continuous trajectory under 100 ms actuation delay, 15 Hz sensor rate,
and Gaussian measurement noise. Generates all 5 required plots in results/.
"""

import math
import sys
from pathlib import Path

# Add project root to sys.path if not installed
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_conga import (
    CongaSimulator,
    SCurvePath,
    SimulatorConfig,
    compute_conga_metrics,
)
from robot_conga.plotting import generate_all_plots


def main() -> None:
    print("=================================================================")
    print("       ROBOT CONGA: MULTI-AGENT SEQUENTIAL PATH FOLLOWING        ")
    print("=================================================================")

    num_robots = 4
    desired_spacing = 1.0  # meters
    v_cmd = 0.10  # m/s

    # S-curve path with two smooth circular arcs
    path = SCurvePath(
        x0=0.0,
        y0=0.0,
        theta0=0.0,
        radius=2.5,
        sweep_angle=math.pi / 3,  # 60 degrees
        straight_entry=2.0,
        straight_exit=3.0,
    )

    config = SimulatorConfig(
        dt=0.01,
        actuation_delay=0.100,  # 100 ms actuation delay
        position_noise_std=0.02,  # 2 cm measurement noise
        heading_noise_std=math.radians(2.0),  # 2 degrees heading noise
        measurement_frequency=15.0,  # 15 Hz sensor pose updates
        seed=42,
    )

    # Initial leader position so that follower 3 starts at s = 0.0
    initial_leader_s = (num_robots - 1) * desired_spacing  # 3.0 m

    sim = CongaSimulator(
        path=path,
        num_robots=num_robots,
        spacing=desired_spacing,
        v_cmd=v_cmd,
        initial_leader_s=initial_leader_s,
        config=config,
    )

    duration = 50.0  # 50 seconds simulation
    print(f"Number of Robots:      {num_robots} (Leader + 3 Followers)")
    print(f"Inter-Robot Spacing:   {desired_spacing:.2f} m")
    print(f"Commanded Speed:       {v_cmd:.2f} m/s")
    print(f"Actuation Delay:       {config.actuation_delay * 1000:.0f} ms")
    print(f"Measurement Frequency: {config.measurement_frequency:.0f} Hz")
    print(f"Simulation Duration:   {duration:.1f} s")
    print("-" * 65)
    print("Running simulation...")

    result = sim.run(duration=duration)
    print("Simulation complete! Computing metrics...")

    # Compute metrics (exclude first 5s of initial settling)
    metrics = compute_conga_metrics(result, start_time=5.0)

    print("\n--- INDIVIDUAL ROBOT TRACKING PERFORMANCE ---")
    for rm in metrics.robot_metrics:
        role = "Leader" if rm.robot_idx == 0 else f"Follower {rm.robot_idx}"
        print(
            f"  {role:<12}: "
            f"Pos RMSE = {rm.position_rmse * 1000:6.2f} mm | "
            f"Max Pos Err = {rm.max_position_error * 1000:6.2f} mm | "
            f"Heading RMSE = {math.degrees(rm.heading_rmse):5.2f} deg"
        )

    print("\n--- INTER-ROBOT SPACING PERFORMANCE ---")
    for pair, rmse_val in metrics.spacing_rmse.items():
        max_dev = metrics.max_spacing_deviation[pair]
        mean_err = metrics.mean_spacing_error[pair]
        print(
            f"  Pair ({pair[0]} -> {pair[1]}): "
            f"Spacing RMSE = {rmse_val * 1000:6.2f} mm | "
            f"Max Deviation = {max_dev * 1000:6.2f} mm | "
            f"Mean Err = {mean_err * 1000:+6.2f} mm"
        )

    # Generate all 5 required plots in results/
    results_dir = Path("results")
    print(f"\nGenerating plots into directory: {results_dir.resolve()} ...")
    generate_all_plots(result, output_dir=str(results_dir))

    print("Plots successfully generated:")
    print("  1. results/xy_trajectory.png")
    print("  2. results/position_error.png")
    print("  3. results/heading_error.png")
    print("  4. results/inter_robot_spacing.png")
    print("  5. results/arc_length_position.png")
    print("=================================================================")


if __name__ == "__main__":
    main()
