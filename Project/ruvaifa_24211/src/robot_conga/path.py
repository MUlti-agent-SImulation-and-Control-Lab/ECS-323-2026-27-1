"""Arc-length parameterized path abstraction and implementations."""

from abc import ABC, abstractmethod
import math
from typing import List, Optional, Tuple

from robot_conga.geometry import wrap_to_pi
from robot_conga.models import PathState


class Path(ABC):
    """Abstract base class for arc-length parameterized paths."""

    @abstractmethod
    def point(self, s: float) -> Tuple[float, float]:
        """Return (x, y) coordinates at arc length s.

        Args:
            s: Arc length along path (meters). Must be >= 0.

        Returns:
            Tuple of (x, y) coordinates.
        """
        pass

    @abstractmethod
    def heading(self, s: float) -> float:
        """Return tangent heading angle theta in radians at arc length s.

        Args:
            s: Arc length along path (meters). Must be >= 0.

        Returns:
            Tangent heading in radians in [-pi, pi).
        """
        pass

    @abstractmethod
    def curvature(self, s: float) -> float:
        """Return signed curvature kappa (1/meters) at arc length s.

        Args:
            s: Arc length along path (meters). Must be >= 0.

        Returns:
            Signed curvature in 1/m (positive for counter-clockwise turning).
        """
        pass

    def evaluate(self, s: float) -> PathState:
        """Evaluate full path state at arc length s.

        Args:
            s: Arc length along path (meters). Must be >= 0.

        Returns:
            PathState object with s, x, y, theta, and curvature.

        Raises:
            ValueError: If s is negative.
        """
        self._validate_s(s)
        x, y = self.point(s)
        theta = self.heading(s)
        kappa = self.curvature(s)
        return PathState(s=s, x=x, y=y, theta=theta, curvature=kappa)

    def _validate_s(self, s: float) -> None:
        """Validate non-negative arc length.

        Args:
            s: Arc length.

        Raises:
            ValueError: If s < 0.
        """
        if s < 0.0:
            raise ValueError(f"Invalid negative arc length s={s:.4f}. Arc length must be non-negative.")


class StraightPath(Path):
    """A straight-line path starting at (x0, y0) with heading theta0."""

    def __init__(
        self,
        x0: float = 0.0,
        y0: float = 0.0,
        theta0: float = 0.0,
        length: Optional[float] = None,
    ) -> None:
        """Initialize straight path.

        Args:
            x0: Initial X position (m).
            y0: Initial Y position (m).
            theta0: Path orientation in radians.
            length: Optional maximum path length (m).
        """
        self.x0 = float(x0)
        self.y0 = float(y0)
        self.theta0 = wrap_to_pi(float(theta0))
        self.length = float(length) if length is not None else None

    def point(self, s: float) -> Tuple[float, float]:
        self._validate_s(s)
        x = self.x0 + s * math.cos(self.theta0)
        y = self.y0 + s * math.sin(self.theta0)
        return x, y

    def heading(self, s: float) -> float:
        self._validate_s(s)
        return self.theta0

    def curvature(self, s: float) -> float:
        self._validate_s(s)
        return 0.0


class CirclePath(Path):
    """A circular path defined by center, radius, and initial angle."""

    def __init__(
        self,
        center_x: float = 0.0,
        center_y: float = 0.0,
        radius: float = 1.0,
        start_angle: float = 0.0,
        direction: int = 1,
    ) -> None:
        """Initialize circular path.

        Args:
            center_x: Center X coordinate (m).
            center_y: Center Y coordinate (m).
            radius: Radius of the circle (m). Must be > 0.
            start_angle: Starting angular position on the circle (radians).
            direction: +1 for counter-clockwise (CCW), -1 for clockwise (CW).

        Raises:
            ValueError: If radius <= 0 or direction not in (-1, 1).
        """
        if radius <= 0.0:
            raise ValueError(f"Circle radius must be strictly positive, got {radius}")
        if direction not in (-1, 1):
            raise ValueError(f"Direction must be 1 (CCW) or -1 (CW), got {direction}")

        self.center_x = float(center_x)
        self.center_y = float(center_y)
        self.radius = float(radius)
        self.start_angle = wrap_to_pi(float(start_angle))
        self.direction = direction

    def _angle_at(self, s: float) -> float:
        """Compute angular position on circle at arc length s."""
        return self.start_angle + self.direction * (s / self.radius)

    def point(self, s: float) -> Tuple[float, float]:
        self._validate_s(s)
        phi = self._angle_at(s)
        x = self.center_x + self.radius * math.cos(phi)
        y = self.center_y + self.radius * math.sin(phi)
        return x, y

    def heading(self, s: float) -> float:
        self._validate_s(s)
        phi = self._angle_at(s)
        # Tangent vector (dx/ds, dy/ds)
        # dx/ds = -direction * sin(phi)
        # dy/ds = direction * cos(phi)
        dx_ds = -float(self.direction) * math.sin(phi)
        dy_ds = float(self.direction) * math.cos(phi)
        return wrap_to_pi(math.atan2(dy_ds, dx_ds))

    def curvature(self, s: float) -> float:
        self._validate_s(s)
        return float(self.direction) / self.radius


class SCurvePath(Path):
    """An S-curve path composed of two smoothly connected circular arcs with opposite curvature.

    First arc: turns left (CCW) by angle sweep_angle with radius R.
    Second arc: turns right (CW) by angle sweep_angle with radius R.
    Optionally preceded and followed by straight segments.
    """

    def __init__(
        self,
        x0: float = 0.0,
        y0: float = 0.0,
        theta0: float = 0.0,
        radius: float = 2.0,
        sweep_angle: float = math.pi / 3,  # 60 degrees
        straight_entry: float = 1.0,
        straight_exit: float = 1.0,
    ) -> None:
        """Initialize S-curve path.

        Args:
            x0: Start X coordinate.
            y0: Start Y coordinate.
            theta0: Start heading angle.
            radius: Radius of the circular arcs.
            sweep_angle: Sweep angle of each curve (radians).
            straight_entry: Length of initial straight segment (m).
            straight_exit: Length of final straight segment (m).
        """
        if radius <= 0:
            raise ValueError(f"Radius must be positive, got {radius}")
        if sweep_angle <= 0:
            raise ValueError(f"Sweep angle must be positive, got {sweep_angle}")

        self.x0 = float(x0)
        self.y0 = float(y0)
        self.theta0 = wrap_to_pi(float(theta0))
        self.radius = float(radius)
        self.sweep_angle = float(sweep_angle)
        self.straight_entry = max(0.0, float(straight_entry))
        self.straight_exit = max(0.0, float(straight_exit))

        self.arc1_len = self.radius * self.sweep_angle
        self.arc2_len = self.radius * self.sweep_angle

        # Boundary arc-length milestones
        self.s1 = self.straight_entry
        self.s2 = self.s1 + self.arc1_len
        self.s3 = self.s2 + self.arc2_len
        self.total_length = self.s3 + self.straight_exit

        # Geometry of segment transitions
        # End of entry straight:
        self.p1_x = self.x0 + self.s1 * math.cos(self.theta0)
        self.p1_y = self.y0 + self.s1 * math.sin(self.theta0)
        self.th1 = self.theta0

        # Arc 1: turns left (CCW). Center is at p1 + R * normal (to left)
        # Normal to left of heading th1 is (-sin(th1), cos(th1))
        self.c1_x = self.p1_x - self.radius * math.sin(self.th1)
        self.c1_y = self.p1_y + self.radius * math.cos(self.th1)
        self.phi1_start = math.atan2(self.p1_y - self.c1_y, self.p1_x - self.c1_x)

        # End of Arc 1:
        self.phi1_end = self.phi1_start + self.sweep_angle
        self.p2_x = self.c1_x + self.radius * math.cos(self.phi1_end)
        self.p2_y = self.c1_y + self.radius * math.sin(self.phi1_end)
        self.th2 = wrap_to_pi(self.th1 + self.sweep_angle)

        # Arc 2: turns right (CW). Center is at p2 - R * normal (to right)
        # Normal to right of heading th2 is (sin(th2), -cos(th2))
        self.c2_x = self.p2_x + self.radius * math.sin(self.th2)
        self.c2_y = self.p2_y - self.radius * math.cos(self.th2)
        self.phi2_start = math.atan2(self.p2_y - self.c2_y, self.p2_x - self.c2_x)

        # End of Arc 2:
        self.phi2_end = self.phi2_start - self.sweep_angle
        self.p3_x = self.c2_x + self.radius * math.cos(self.phi2_end)
        self.p3_y = self.c2_y + self.radius * math.sin(self.phi2_end)
        self.th3 = wrap_to_pi(self.th2 - self.sweep_angle)

    def point(self, s: float) -> Tuple[float, float]:
        self._validate_s(s)
        if s <= self.s1:
            # Entry straight
            return (
                self.x0 + s * math.cos(self.theta0),
                self.y0 + s * math.sin(self.theta0),
            )
        elif s <= self.s2:
            # Arc 1 (turning left)
            ds = s - self.s1
            phi = self.phi1_start + ds / self.radius
            return (
                self.c1_x + self.radius * math.cos(phi),
                self.c1_y + self.radius * math.sin(phi),
            )
        elif s <= self.s3:
            # Arc 2 (turning right)
            ds = s - self.s2
            phi = self.phi2_start - ds / self.radius
            return (
                self.c2_x + self.radius * math.cos(phi),
                self.c2_y + self.radius * math.sin(phi),
            )
        else:
            # Exit straight
            ds = s - self.s3
            return (
                self.p3_x + ds * math.cos(self.th3),
                self.p3_y + ds * math.sin(self.th3),
            )

    def heading(self, s: float) -> float:
        self._validate_s(s)
        if s <= self.s1:
            return self.theta0
        elif s <= self.s2:
            ds = s - self.s1
            return wrap_to_pi(self.th1 + ds / self.radius)
        elif s <= self.s3:
            ds = s - self.s2
            return wrap_to_pi(self.th2 - ds / self.radius)
        else:
            return self.th3

    def curvature(self, s: float) -> float:
        self._validate_s(s)
        if s < self.s1:
            return 0.0
        elif s <= self.s2:
            return 1.0 / self.radius
        elif s <= self.s3:
            return -1.0 / self.radius
        else:
            return 0.0
