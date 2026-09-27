"""Metrics evaluation for Robot Conga tracking and spacing performance."""

from dataclasses import dataclass
import math
from typing import Dict, List, Sequence, Tuple

from robot_conga.geometry import distance, wrap_to_pi
from robot_conga.models import Pose


def position_error(actual: Pose, reference: Pose) -> float:
    """Calculate Euclidean distance error between actual and reference poses.

    Args:
        actual: Ground truth or measured robot pose.
        reference: Target reference pose.

    Returns:
        Scalar Euclidean position error (meters).
    """
    return distance(actual, reference)


def heading_error(actual_theta: float, ref_theta: float) -> float:
    """Calculate absolute heading error wrapped to [0, pi].

    Args:
        actual_theta: Current robot orientation (radians).
        ref_theta: Reference orientation (radians).

    Returns:
        Absolute angular error in [0, pi] radians.
    """
    return abs(wrap_to_pi(actual_theta - ref_theta))


def rmse(errors: Sequence[float]) -> float:
    """Calculate Root Mean Square Error (RMSE) for a sequence of errors.

    Args:
        errors: Sequence of scalar error values.

    Returns:
        RMSE value. Returns 0.0 for empty sequence.
    """
    if not errors:
        return 0.0
    sum_sq = sum(float(e) ** 2 for e in errors)
    return math.sqrt(sum_sq / len(errors))


def max_error(errors: Sequence[float]) -> float:
    """Calculate the maximum absolute error in a sequence.

    Args:
        errors: Sequence of scalar error values.

    Returns:
        Maximum absolute error. Returns 0.0 for empty sequence.
    """
    if not errors:
        return 0.0
    return max(abs(float(e)) for e in errors)


def inter_robot_distance(pose_leader: Pose, pose_follower: Pose) -> float:
    """Calculate Euclidean distance between two robot poses.

    Args:
        pose_leader: Leader robot pose.
        pose_follower: Follower robot pose.

    Returns:
        Euclidean distance (meters).
    """
    return distance(pose_leader, pose_follower)


def spacing_error(actual_distance: float, desired_spacing: float) -> float:
    """Calculate spacing error: actual_distance - desired_spacing.

    Args:
        actual_distance: Measured or simulated Euclidean distance (meters).
        desired_spacing: Commanded spatial spacing (meters).

    Returns:
        Signed spacing error (meters). Positive means too far, negative means too close.
    """
    return float(actual_distance) - float(desired_spacing)


def max_spacing_deviation(spacing_errors: Sequence[float]) -> float:
    """Calculate maximum absolute spacing deviation from desired spacing.

    Args:
        spacing_errors: Sequence of signed spacing errors.

    Returns:
        Maximum absolute deviation (meters).
    """
    return max_error(spacing_errors)


@dataclass
class RobotMetrics:
    """Performance metrics for an individual robot."""
    robot_idx: int
    position_rmse: float
    max_position_error: float
    heading_rmse: float
    max_heading_error: float


@dataclass
class CongaSummary:
    """Consolidated metrics summary for a multi-robot Robot Conga formation."""
    robot_metrics: List[RobotMetrics]
    spacing_rmse: Dict[Tuple[int, int], float]
    max_spacing_deviation: Dict[Tuple[int, int], float]
    mean_spacing_error: Dict[Tuple[int, int], float]


def compute_conga_metrics(
    simulation_result: "SimulationResult",
    start_time: float = 0.0,
) -> CongaSummary:
    """Compute complete tracking and spacing metrics for a simulation run.

    Args:
        simulation_result: Result object from CongaSimulator.
        start_time: Optional warm-up cutoff time (seconds) to exclude initial transient.

    Returns:
        CongaSummary dataclass with per-robot and formation spacing metrics.
    """
    # Filter steps after start_time
    valid_indices = [
        i for i, t in enumerate(simulation_result.times) if t >= start_time
    ]

    robot_metrics: List[RobotMetrics] = []
    for r_idx in range(simulation_result.num_robots):
        pos_errs = [
            simulation_result.get_position_errors(r_idx)[i] for i in valid_indices
        ]
        head_errs = [
            simulation_result.get_heading_errors(r_idx)[i] for i in valid_indices
        ]
        robot_metrics.append(
            RobotMetrics(
                robot_idx=r_idx,
                position_rmse=rmse(pos_errs),
                max_position_error=max_error(pos_errs),
                heading_rmse=rmse(head_errs),
                max_heading_error=max_error(head_errs),
            )
        )

    # Formation spacing metrics between consecutive pairs
    all_dists = simulation_result.get_inter_robot_distances()
    spacing_rmse: Dict[Tuple[int, int], float] = {}
    max_dev: Dict[Tuple[int, int], float] = {}
    mean_err: Dict[Tuple[int, int], float] = {}

    desired = simulation_result.desired_spacing
    for pair, dist_list in all_dists.items():
        filtered_dists = [dist_list[i] for i in valid_indices]
        s_errs = [spacing_error(d, desired) for d in filtered_dists]
        spacing_rmse[pair] = rmse(s_errs)
        max_dev[pair] = max_spacing_deviation(s_errs)
        mean_err[pair] = sum(s_errs) / len(s_errs) if s_errs else 0.0

    return CongaSummary(
        robot_metrics=robot_metrics,
        spacing_rmse=spacing_rmse,
        max_spacing_deviation=max_dev,
        mean_spacing_error=mean_err,
    )
