"""Unit tests for Robot Conga Equation (8) controller."""

import math
import pytest

from robot_conga.controller import CongaController, ControllerSingularityError
from robot_conga.models import Pose, ReferenceState, Twist


def test_controller_zero_error():
    """Requirement 1: Zero error. Output should equal reference input."""
    controller = CongaController(
        lambda1=4.5,
        lambda2=7.5,
        lambda3=2.5,
        max_linear_velocity=None,
        max_angular_velocity=None,
    )

    ref_pose = Pose(x=3.0, y=4.0, theta=0.2)
    ref_twist = Twist(linear=0.15, angular=0.05)
    ref_state = ReferenceState(pose=ref_pose, twist=ref_twist, s=5.0, curvature=1.0)

    # Actual pose identical to reference pose
    actual_pose = Pose(x=3.0, y=4.0, theta=0.2)

    cmd = controller.compute(actual_pose, ref_state)

    # u should equal u*, omega should equal omega*
    assert cmd.linear == pytest.approx(ref_twist.linear, abs=1e-9)
    assert cmd.angular == pytest.approx(ref_twist.angular, abs=1e-9)


def test_controller_positive_positional_error():
    """Requirement 2: Positive positional error. Controller reacts in expected direction."""
    controller = CongaController(
        lambda1=4.5,
        lambda2=7.5,
        lambda3=2.5,
        max_linear_velocity=None,
        max_angular_velocity=None,
    )

    # Reference facing along +x axis (theta* = 0)
    ref_pose = Pose(x=0.0, y=0.0, theta=0.0)
    ref_twist = Twist(linear=0.2, angular=0.0)
    ref_state = ReferenceState(pose=ref_pose, twist=ref_twist, s=0.0, curvature=0.0)

    # Robot ahead in x (e1 > 0): controller should reduce linear velocity
    actual_ahead = Pose(x=0.05, y=0.0, theta=0.0)
    cmd_ahead = controller.compute(actual_ahead, ref_state)
    assert cmd_ahead.linear < ref_twist.linear

    # Robot displaced to positive y (e2 > 0): controller should command negative omega to steer right
    actual_left = Pose(x=0.0, y=0.05, theta=0.0)
    cmd_left = controller.compute(actual_left, ref_state)
    assert cmd_left.angular < ref_twist.angular


def test_controller_heading_wrap():
    """Requirement 3: Heading wrap.

    +179° vs -179° should result in approximately -2° normalized error.
    """
    controller = CongaController(
        lambda1=4.5,
        lambda2=7.5,
        lambda3=2.5,
        max_linear_velocity=None,
        max_angular_velocity=None,
    )

    # theta* = -179 degrees = -3.124139 rad
    theta_ref = math.radians(-179.0)
    ref_pose = Pose(x=0.0, y=0.0, theta=theta_ref)
    ref_twist = Twist(linear=0.1, angular=0.0)
    ref_state = ReferenceState(pose=ref_pose, twist=ref_twist, s=0.0, curvature=0.0)

    # actual theta = +179 degrees = +3.124139 rad
    theta_actual = math.radians(179.0)
    actual_pose = Pose(x=0.0, y=0.0, theta=theta_actual)

    # e3 = wrap_to_pi(179 - (-179)) = wrap_to_pi(358) = -2 degrees
    cmd = controller.compute(actual_pose, ref_state)

    # omega = omega* - lambda2 * e3 - lambda1 * e2
    # e2 = 0, omega* = 0 -> omega = -lambda2 * (-2 deg in rad) > 0
    expected_e3 = math.radians(-2.0)
    expected_omega = -7.5 * expected_e3
    assert cmd.angular == pytest.approx(expected_omega, rel=1e-4)


def test_controller_singularity_guard():
    """Requirement 4: Singularity handling.

    When cos(theta* + e3) approaches zero (|cos| < threshold),
    ControllerSingularityError must be explicitly raised.
    """
    controller = CongaController(singularity_threshold=1e-3)

    # Let reference heading be pi/2, actual heading be pi/2 -> cos(pi/2) = 0
    ref_pose = Pose(x=0.0, y=0.0, theta=math.pi / 2)
    ref_twist = Twist(linear=0.1, angular=0.0)
    ref_state = ReferenceState(pose=ref_pose, twist=ref_twist, s=0.0, curvature=0.0)

    actual_pose = Pose(x=0.0, y=0.0, theta=math.pi / 2)

    with pytest.raises(ControllerSingularityError, match="Singularity detected"):
        controller.compute(actual_pose, ref_state)


def test_controller_velocity_saturation():
    """Requirement 5: Saturation. Output must respect configured velocity limits."""
    controller = CongaController(
        lambda1=10.0,
        lambda2=10.0,
        lambda3=10.0,
        max_linear_velocity=0.30,
        max_angular_velocity=1.50,
    )

    ref_pose = Pose(x=0.0, y=0.0, theta=0.0)
    ref_twist = Twist(linear=0.1, angular=0.0)
    ref_state = ReferenceState(pose=ref_pose, twist=ref_twist, s=0.0, curvature=0.0)

    # Huge errors that would produce unbounded velocity commands
    actual_far = Pose(x=-10.0, y=10.0, theta=0.0)
    cmd = controller.compute(actual_far, ref_state)

    assert abs(cmd.linear) <= 0.30
    assert abs(cmd.angular) <= 1.50


def test_controller_gain_validation():
    """Verify non-positive gains are rejected."""
    with pytest.raises(ValueError):
        CongaController(lambda1=0.0)
    with pytest.raises(ValueError):
        CongaController(lambda2=-1.0)
    with pytest.raises(ValueError):
        CongaController(lambda3=-0.5)
    with pytest.raises(ValueError):
        CongaController(singularity_threshold=0.0)
