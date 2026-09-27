"""Demo: Circular trajectory tracking with Robot Conga controller."""

import math
import sys
from pathlib import Path

# Add project root to sys.path if not installed
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_conga import (
    CirclePath,
    CongaSimulator,
    SimulatorConfig,
    compute_conga_metrics,
)
from robot_conga.plotting import plot_xy_trajectories


def main() -> None:
    print("=== Robot Conga: Circular Trajectory Tracking Experiment ===")
    v_cmd = 0.10  # 0.1 m/s
    radius = 2.0  # R = 2 m as specified in implementation plan
    curvature = 1.0 / radius
    omega_star = v_cmd * curvature
    print(f"Parameters: R = {radius} m, Curvature = {curvature} 1/m, omega* = {omega_star:.4f} rad/s")
    print("Note: Trajectory heading is kept away from Y-axis (+/- pi/2) where Equation (8)")
    print("experiences the paper's known coordinate singularity (addressed in Section IV.C).")

    # Arc spanning heading from -45° to +15° (centered around forward X axis)
    # For CCW circle: heading = phi + pi/2 -> phi = heading - pi/2
    start_phi = -math.pi / 2 - math.pi / 4  # -3*pi/4 (-135°) -> heading = -45°
    path = CirclePath(center_x=0.0, center_y=radius, radius=radius, start_angle=start_phi, direction=1)

    config = SimulatorConfig(
        dt=0.01,
        actuation_delay=0.100,  # 100 ms actuation delay
        position_noise_std=0.02,  # 2 cm measurement noise
        heading_noise_std=math.radians(2.0),  # 2 deg heading noise
        measurement_frequency=15.0,  # 15 Hz sensor rate
        seed=100,
    )

    sim = CongaSimulator(
        path=path,
        num_robots=1,
        spacing=1.0,
        v_cmd=v_cmd,
        initial_leader_s=0.0,
        config=config,
    )

    duration = 20.0
    print(f"Simulating circular arc for {duration} seconds...")
    result = sim.run(duration=duration)

    metrics = compute_conga_metrics(result, start_time=5.0)
    lead_m = metrics.robot_metrics[0]
    print(f"Position RMSE:        {lead_m.position_rmse * 1000:.2f} mm")
    print(f"Max Position Error:   {lead_m.max_position_error * 1000:.2f} mm")
    print(f"Heading RMSE:         {math.degrees(lead_m.heading_rmse):.2f} deg")

    output_path = Path("results") / "circle_demo.png"
    plot_xy_trajectories(result, str(output_path), title=f"Circular Arc Tracking (R={radius}m, Vcmd={v_cmd}m/s)")
    print(f"Plot saved to: {output_path}")


if __name__ == "__main__":
    main()
