"""Spatial reference propagator for Robot Conga."""

from typing import List, Optional

from robot_conga.models import Pose, ReferenceState, Twist
from robot_conga.path import Path


class ReferencePropagator:
    """Propagates reference trajectory states spatially along an arc-length path."""

    def __init__(
        self,
        path: Path,
        spacing: float = 1.0,
        number_of_robots: int = 4,
        v_cmd: float = 0.1,
    ) -> None:
        """Initialize reference propagator.

        Args:
            path: Arc-length parameterized Path instance.
            spacing: Desired inter-robot spatial spacing (meters). Must be > 0.
            number_of_robots: Total number of robots (leader + followers). Must be >= 1.
            v_cmd: Commanded linear speed along the trajectory (m/s).

        Raises:
            ValueError: If spacing <= 0 or number_of_robots < 1.
        """
        if spacing <= 0:
            raise ValueError(f"Spacing must be positive, got {spacing}")
        if number_of_robots < 1:
            raise ValueError(f"number_of_robots must be >= 1, got {number_of_robots}")

        self.path = path
        self.spacing = float(spacing)
        self.number_of_robots = int(number_of_robots)
        self.v_cmd = float(v_cmd)

    def compute_s(self, leader_s: float, robot_idx: int) -> float:
        """Compute the spatial arc length position for a robot index.

        Args:
            leader_s: Current arc length position of the leader (meters).
            robot_idx: Index of the robot (0 for leader, > 0 for followers).

        Returns:
            Spatial arc length s_i = leader_s - i * spacing.
        """
        return leader_s - robot_idx * self.spacing

    def get_reference_state(self, s: float, v_cmd: Optional[float] = None) -> ReferenceState:
        """Compute reference state at a specific arc length position.

        Args:
            s: Arc length along path (meters).
            v_cmd: Optional linear speed override (m/s).

        Returns:
            ReferenceState containing pose, twist, s, and curvature.

        Raises:
            ValueError: If s is negative.
        """
        if s < 0.0:
            raise ValueError(
                f"Cannot generate reference state for negative arc length s={s:.4f}. "
                "Path starts at s >= 0."
            )

        v = self.v_cmd if v_cmd is None else float(v_cmd)
        path_state = self.path.evaluate(s)
        u_star = v
        omega_star = v * path_state.curvature

        return ReferenceState(
            pose=Pose(x=path_state.x, y=path_state.y, theta=path_state.theta),
            twist=Twist(linear=u_star, angular=omega_star),
            s=s,
            curvature=path_state.curvature,
        )

    def propagate(
        self,
        leader_s: float,
        v_cmd: Optional[float] = None,
    ) -> List[ReferenceState]:
        """Compute reference states for all robots in the conga line.

        Args:
            leader_s: Current arc length position of the leader (meters).
            v_cmd: Commanded linear speed along path (m/s).

        Returns:
            List of ReferenceState objects for all robots (index 0 is leader).

        Raises:
            ValueError: If any robot's arc-length position is negative.
        """
        v = self.v_cmd if v_cmd is None else float(v_cmd)
        ref_states: List[ReferenceState] = []

        for i in range(self.number_of_robots):
            s_i = self.compute_s(leader_s, i)
            if s_i < 0.0:
                raise ValueError(
                    f"Robot {i} requested invalid negative arc length s_{i} = {s_i:.4f} "
                    f"(leader_s={leader_s:.4f}, spacing={self.spacing}). "
                    "Follower positions cannot precede the path origin."
                )
            ref_states.append(self.get_reference_state(s_i, v))

        return ref_states
