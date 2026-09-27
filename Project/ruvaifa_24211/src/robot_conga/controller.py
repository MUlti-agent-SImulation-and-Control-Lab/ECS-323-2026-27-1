"""Robot Conga controller implementing the paper's Equation (8)."""

import math
from typing import Optional

from robot_conga.geometry import clamp, wrap_to_pi
from robot_conga.models import Pose, ReferenceState, Twist


class ControllerSingularityError(RuntimeError):
    """Raised when the controller denominator cos(theta* + e3) approaches zero."""
    pass


class CongaController:
    """Robot Conga trajectory tracking controller based on Lyapunov stability analysis.

    Implements the paper's Equation (8):
        u = [u* * cos(theta*) - lambda3 * e1] / cos(theta* + e3)
        omega = omega* - lambda2 * e3 - lambda1 * e2

    where:
        e1 = x - x*
        e2 = y - y*
        e3 = wrap_to_pi(theta - theta*)
    """

    def __init__(
        self,
        lambda1: float = 4.5,
        lambda2: float = 7.5,
        lambda3: float = 2.5,
        singularity_threshold: float = 1e-3,
        max_linear_velocity: Optional[float] = 0.30,
        max_angular_velocity: Optional[float] = 1.50,
    ) -> None:
        """Initialize controller with gains, singularity threshold, and saturation limits.

        Args:
            lambda1: Feedback gain for lateral error e2. Must be > 0.
            lambda2: Feedback gain for heading error e3. Must be > 0.
            lambda3: Feedback gain for longitudinal error e1. Must be > 0.
            singularity_threshold: Minimum acceptable absolute value for cos(theta* + e3).
            max_linear_velocity: Maximum linear velocity limit (m/s) or None for unconstrained.
            max_angular_velocity: Maximum angular velocity limit (rad/s) or None for unconstrained.

        Raises:
            ValueError: If gains or singularity threshold are not strictly positive.
        """
        if lambda1 <= 0.0 or lambda2 <= 0.0 or lambda3 <= 0.0:
            raise ValueError(
                f"Controller gains must be strictly positive: "
                f"lambda1={lambda1}, lambda2={lambda2}, lambda3={lambda3}"
            )
        if singularity_threshold <= 0.0:
            raise ValueError(
                f"Singularity threshold must be strictly positive, got {singularity_threshold}"
            )

        self.lambda1 = float(lambda1)
        self.lambda2 = float(lambda2)
        self.lambda3 = float(lambda3)
        self.singularity_threshold = float(singularity_threshold)
        self.max_linear_velocity = (
            float(max_linear_velocity) if max_linear_velocity is not None else None
        )
        self.max_angular_velocity = (
            float(max_angular_velocity) if max_angular_velocity is not None else None
        )

    def compute(self, actual_pose: Pose, reference_state: ReferenceState) -> Twist:
        """Compute control inputs u and omega from actual pose and reference state.

        Args:
            actual_pose: Current actual (or measured) robot pose (x, y, theta).
            reference_state: Desired reference state containing pose and twist.

        Returns:
            Twist containing commanded linear velocity u and angular velocity omega.

        Raises:
            ControllerSingularityError: If |cos(theta* + e3)| < singularity_threshold.
        """
        # Tracking errors (Equation 3) with engineering safeguard wrap_to_pi on e3
        e1 = actual_pose.x - reference_state.pose.x
        e2 = actual_pose.y - reference_state.pose.y
        e3 = wrap_to_pi(actual_pose.theta - reference_state.pose.theta)

        theta_star = reference_state.pose.theta
        u_star = reference_state.twist.linear
        omega_star = reference_state.twist.angular

        # Denominator in Equation (8): cos(theta* + e3)
        denom = math.cos(theta_star + e3)

        if abs(denom) < self.singularity_threshold:
            raise ControllerSingularityError(
                f"Singularity detected: |cos(theta* + e3)| = {abs(denom):.6e} < "
                f"threshold ({self.singularity_threshold:.6e}). "
                f"theta*={theta_star:.4f}, e3={e3:.4f}."
            )

        # Equation (8)
        u_raw = (u_star * math.cos(theta_star) - self.lambda3 * e1) / denom
        omega_raw = omega_star - self.lambda2 * e3 - self.lambda1 * e2

        # Prevent NaN or inf from propagating
        if not math.isfinite(u_raw) or not math.isfinite(omega_raw):
            raise ControllerSingularityError(
                f"Non-finite control output computed: u={u_raw}, omega={omega_raw}"
            )

        # Apply velocity saturation limits if configured
        u = u_raw
        if self.max_linear_velocity is not None:
            u = clamp(u, -self.max_linear_velocity, self.max_linear_velocity)

        omega = omega_raw
        if self.max_angular_velocity is not None:
            omega = clamp(omega, -self.max_angular_velocity, self.max_angular_velocity)

        return Twist(linear=u, angular=omega)
