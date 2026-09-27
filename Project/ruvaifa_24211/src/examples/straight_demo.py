"""Demo: Straight-line trajectory tracking with Robot Conga controller."""

import sys
from pathlib import Path

# Add project root to sys.path if not installed
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_conga import (
    CongaController,
    CongaSimulator,
    SimulatorConfig,
    StraightPath,
    compute_conga_metrics,
)
from robot_conga.plotting import plot_xy_trajectories


def main() -> None:
    print("=== Robot Conga: Straight-Line Tracking Experiment ===")
    v_cmd = 0.10  # 0.10 m/s as required by paper implementation plan
    path = StraightPath(x0=0.0, y0=0.0, theta0=0.0)

    config = SimulatorConfig(
        dt=0.01,
        actuation_delay=0.100,  # 100 ms actuation delay
        position_noise_std=0.02,  # 2 cm measurement noise
        heading_noise_std=0.035,  # ~2 degrees heading noise
        measurement_frequency=15.0,  # 15 Hz sensor rate
        seed=42,
    )

    sim = CongaSimulator(
        path=path,
        num_robots=1,
        spacing=1.0,
        v_cmd=v_cmd,
        initial_leader_s=0.0,
        config=config,
    )

    duration = 30.0
    print(f"Simulating straight-line motion at {v_cmd} m/s for {duration} seconds...")
    result = sim.run(duration=duration)

    metrics = compute_conga_metrics(result, start_time=5.0)
    lead_m = metrics.robot_metrics[0]
    print(f"Position RMSE:        {lead_m.position_rmse * 1000:.2f} mm")
    print(f"Max Position Error:   {lead_m.max_position_error * 1000:.2f} mm")
    print(f"Heading RMSE:         {lead_m.heading_rmse * 180 / 3.14159:.2f} deg")

    output_path = Path("results") / "straight_demo.png"
    plot_xy_trajectories(result, str(output_path), title="Straight-Line Tracking (Vcmd=0.1 m/s)")
    print(f"Plot saved to: {output_path}")


if __name__ == "__main__":
    main()
