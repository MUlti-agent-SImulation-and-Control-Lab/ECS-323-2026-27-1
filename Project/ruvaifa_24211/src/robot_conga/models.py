"""Core data models and type definitions for Robot Conga."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose:
    """2D pose representation with orientation.

    Attributes:
        x: X-coordinate in global frame (meters).
        y: Y-coordinate in global frame (meters).
        theta: Heading angle in global frame (radians).
    """
    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class Twist:
    """Velocity representation in robot body frame.

    Attributes:
        linear: Linear velocity u (m/s).
        angular: Angular velocity omega (rad/s).
    """
    linear: float
    angular: float


@dataclass(frozen=True)
class PathState:
    """State of a path evaluated at a specific arc length.

    Attributes:
        s: Arc length along the path (meters).
        x: X-coordinate (meters).
        y: Y-coordinate (meters).
        theta: Tangent heading angle (radians).
        curvature: Signed path curvature kappa (1/m).
    """
    s: float
    x: float
    y: float
    theta: float
    curvature: float

    @property
    def pose(self) -> Pose:
        """Return pose as a Pose object."""
        return Pose(x=self.x, y=self.y, theta=self.theta)


@dataclass(frozen=True)
class ReferenceState:
    """Complete reference trajectory state for a robot.

    Attributes:
        pose: Desired reference pose (x*, y*, theta*).
        twist: Desired reference velocities (u*, omega*).
        s: Reference arc length position (meters).
        curvature: Path curvature at this arc length (1/m).
    """
    pose: Pose
    twist: Twist
    s: float
    curvature: float

    @property
    def x(self) -> float:
        """Reference X coordinate x*."""
        return self.pose.x

    @property
    def y(self) -> float:
        """Reference Y coordinate y*."""
        return self.pose.y

    @property
    def theta(self) -> float:
        """Reference heading theta*."""
        return self.pose.theta

    @property
    def u_star(self) -> float:
        """Reference linear velocity u*."""
        return self.twist.linear

    @property
    def omega_star(self) -> float:
        """Reference angular velocity omega*."""
        return self.twist.angular
