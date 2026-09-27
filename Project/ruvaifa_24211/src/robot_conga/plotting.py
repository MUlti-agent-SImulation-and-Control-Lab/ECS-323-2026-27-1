"""Plotting utilities to visualize Robot Conga simulation results."""

import math
from pathlib import Path
from typing import Optional

import matplotlib
matplotlib.use("Agg")  # Non-interactive backend suitable for headless and CI environments
import matplotlib.pyplot as plt

from robot_conga.simulator import SimulationResult


def plot_xy_trajectories(
    result: SimulationResult,
    output_path: Optional[str] = None,
    title: str = "Robot Conga - XY Trajectories",
) -> plt.Figure:
    """Plot XY trajectories for reference path and all robots (leader + followers)."""
    fig, ax = plt.subplots(figsize=(8, 6), dpi=150)

    # Plot path reference
    ref_x = [step.reference_states[0].pose.x for step in result.steps]
    ref_y = [step.reference_states[0].pose.y for step in result.steps]
    ax.plot(ref_x, ref_y, "k--", linewidth=1.5, label="Leader Reference", alpha=0.7)

    # Robot colors
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]

    for r_idx in range(result.num_robots):
        traj = result.get_robot_trajectory(r_idx)
        xs = [p.x for p in traj]
        ys = [p.y for p in traj]
        label = "Leader (Robot 0)" if r_idx == 0 else f"Follower {r_idx}"
        c = colors[r_idx % len(colors)]
        ax.plot(xs, ys, color=c, linewidth=2.0, label=label)
        # Mark start and end
        ax.plot(xs[0], ys[0], marker="o", color=c, markersize=6)
        ax.plot(xs[-1], ys[-1], marker="s", color=c, markersize=6)

    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.set_xlabel("X [m]", fontsize=10)
    ax.set_ylabel("Y [m]", fontsize=10)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()

    if output_path:
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path)
    return fig


def plot_position_errors(
    result: SimulationResult,
    output_path: Optional[str] = None,
    title: str = "Position Error vs Time",
) -> plt.Figure:
    """Plot Euclidean position error vs time for each robot."""
    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=150)
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]

    for r_idx in range(result.num_robots):
        errors = result.get_position_errors(r_idx)
        label = "Leader" if r_idx == 0 else f"Follower {r_idx}"
        ax.plot(result.times, errors, color=colors[r_idx % len(colors)], label=label, linewidth=1.8)

    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.set_xlabel("Time [s]", fontsize=10)
    ax.set_ylabel("Position Error [m]", fontsize=10)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()

    if output_path:
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path)
    return fig


def plot_heading_errors(
    result: SimulationResult,
    output_path: Optional[str] = None,
    title: str = "Heading Error vs Time",
) -> plt.Figure:
    """Plot absolute heading error (in degrees) vs time for each robot."""
    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=150)
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]

    for r_idx in range(result.num_robots):
        errors_rad = result.get_heading_errors(r_idx)
        errors_deg = [math.degrees(e) for e in errors_rad]
        label = "Leader" if r_idx == 0 else f"Follower {r_idx}"
        ax.plot(result.times, errors_deg, color=colors[r_idx % len(colors)], label=label, linewidth=1.8)

    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.set_xlabel("Time [s]", fontsize=10)
    ax.set_ylabel("Heading Error [deg]", fontsize=10)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()

    if output_path:
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path)
    return fig


def plot_inter_robot_spacing(
    result: SimulationResult,
    output_path: Optional[str] = None,
    title: str = "Inter-Robot Spacing vs Time",
) -> plt.Figure:
    """Plot Euclidean distance between adjacent robot pairs over time."""
    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=150)
    all_dists = result.get_inter_robot_distances()
    colors = ["#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]

    # Target spacing line
    ax.axhline(
        y=result.desired_spacing,
        color="black",
        linestyle="--",
        linewidth=1.5,
        label=f"Desired Spacing ({result.desired_spacing:.2f} m)",
    )

    for i, (pair, dists) in enumerate(all_dists.items()):
        label = f"Robot {pair[0]} - Robot {pair[1]}"
        ax.plot(result.times, dists, color=colors[i % len(colors)], label=label, linewidth=1.8)

    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.set_xlabel("Time [s]", fontsize=10)
    ax.set_ylabel("Distance [m]", fontsize=10)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()

    if output_path:
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path)
    return fig


def plot_arc_length_positions(
    result: SimulationResult,
    output_path: Optional[str] = None,
    title: str = "Arc-Length Position (s) vs Time",
) -> plt.Figure:
    """Plot arc length s for leader and followers over time."""
    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=150)
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]

    for r_idx in range(result.num_robots):
        s_vals = result.get_robot_s(r_idx)
        label = f"Leader s (Robot 0)" if r_idx == 0 else f"Follower {r_idx} s"
        ax.plot(result.times, s_vals, color=colors[r_idx % len(colors)], label=label, linewidth=1.8)

    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.set_xlabel("Time [s]", fontsize=10)
    ax.set_ylabel("Arc-Length Position s [m]", fontsize=10)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()

    if output_path:
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path)
    return fig


def generate_all_plots(result: SimulationResult, output_dir: str = "results") -> None:
    """Generate and save all 5 required plots to the specified output directory.

    Args:
        result: SimulationResult from CongaSimulator.
        output_dir: Directory where png plot files will be saved.
    """
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    plot_xy_trajectories(result, str(out / "xy_trajectory.png"))
    plot_position_errors(result, str(out / "position_error.png"))
    plot_heading_errors(result, str(out / "heading_error.png"))
    plot_inter_robot_spacing(result, str(out / "inter_robot_spacing.png"))
    plot_arc_length_positions(result, str(out / "arc_length_position.png"))
    plt.close("all")
