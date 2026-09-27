"""Unit tests for the unicycle simulator, noise model, actuation delay, and multi-robot Conga."""

import math
import pytest

from robot_conga.controller import CongaController
from robot_conga.geometry import distance, wrap_to_pi
from robot_conga.models import Pose, ReferenceState, Twist
from robot_conga.path import StraightPath
from robot_conga.simulator import CongaSimulator, RobotSimulator, SimulatorConfig


def test_unicycle_dynamics_step():
    """Verify clean kinematic integration for unicycle dynamics."""
    config = SimulatorConfig(
        dt=0.1,
        actuation_delay=0.0,
        position_noise_std=0.0,
        heading_noise_std=0.0,
        max_linear_velocity=2.0,
        max_angular_velocity=2.0,
        max_linear_acceleration=None,
        max_angular_acceleration=None,
    )
    controller = CongaController(max_linear_velocity=2.0, max_angular_velocity=2.0)
    robot = RobotSimulator(
        initial_pose=Pose(x=0.0, y=0.0, theta=0.0),
        controller=controller,
        config=config,
    )

    ref = ReferenceState(
        pose=Pose(x=0.0, y=0.0, theta=0.0),
        twist=Twist(linear=1.0, angular=0.5),
        s=0.0,
        curvature=0.5,
    )

    # Step at t = 0
    t_pose, m_pose, cmd, act = robot.step(0.0, ref)

    # After dt = 0.1 with linear=1.0 and angular=0.5:
    # x = 0 + 1.0 * cos(0) * 0.1 = 0.1
    # y = 0 + 1.0 * sin(0) * 0.1 = 0.0
    # theta = 0 + 0.5 * 0.1 = 0.05
    assert t_pose.x == pytest.approx(0.1, abs=1e-5)
    assert t_pose.y == pytest.approx(0.0, abs=1e-5)
    assert t_pose.theta == pytest.approx(0.05, abs=1e-5)


def test_actuation_delay():
    """Verify that commanded inputs are delayed by actuation_delay."""
    delay = 0.100  # 100 ms
    config = SimulatorConfig(
        dt=0.02,
        actuation_delay=delay,
        position_noise_std=0.0,
        heading_noise_std=0.0,
        max_linear_velocity=1.0,
        max_angular_velocity=1.0,
        max_linear_acceleration=None,
        max_angular_acceleration=None,
    )
    controller = CongaController(max_linear_velocity=1.0, max_angular_velocity=1.0)
    robot = RobotSimulator(
        initial_pose=Pose(x=0.0, y=0.0, theta=0.0),
        controller=controller,
        config=config,
    )

    ref = ReferenceState(
        pose=Pose(x=0.0, y=0.0, theta=0.0),
        twist=Twist(linear=0.5, angular=0.0),
        s=0.0,
        curvature=0.0,
    )

    # Steps prior to t = 0.100 should have active_command = (0.0, 0.0)
    for i in range(4):
        t = i * 0.02
        t_pose, _, _, act = robot.step(t, ref)
        assert act.linear == pytest.approx(0.0)

    # Step at t = 0.100: the first command enqueued at t=0.0 is now ready
    t_pose, _, _, act = robot.step(0.10, ref)
    assert act.linear == pytest.approx(0.5)


def test_noise_separation():
    """Verify noise is added to measurement and true state is undisturbed."""
    config = SimulatorConfig(
        dt=0.05,
        actuation_delay=0.0,
        position_noise_std=0.05,
        heading_noise_std=0.05,
        seed=42,
    )
    controller = CongaController()
    robot = RobotSimulator(
        initial_pose=Pose(x=1.0, y=2.0, theta=0.0),
        controller=controller,
        config=config,
    )

    # True state initially exact
    assert robot.true_pose.x == 1.0
    assert robot.true_pose.y == 2.0
    # Measured state should differ from true pose due to noise
    assert robot.measured_pose.x != 1.0 or robot.measured_pose.y != 2.0


def test_stationary_convergence():
    """Requirement A: Stationary reference convergence.

    Verify e1 -> 0, e2 -> 0, e3 -> 0 when tracking a stationary reference.
    """
    config = SimulatorConfig(
        dt=0.01,
        actuation_delay=0.05,
        position_noise_std=0.0,
        heading_noise_std=0.0,
        max_linear_velocity=0.30,
        max_angular_velocity=1.50,
    )
    controller = CongaController(
        lambda1=4.5,
        lambda2=7.5,
        lambda3=2.5,
        max_linear_velocity=0.30,
        max_angular_velocity=1.50,
    )

    # Initial perturbed longitudinal pose and heading
    robot = RobotSimulator(
        initial_pose=Pose(x=0.15, y=0.0, theta=0.10),
        controller=controller,
        config=config,
    )

    # Stationary reference at origin
    ref = ReferenceState(
        pose=Pose(x=0.0, y=0.0, theta=0.0),
        twist=Twist(linear=0.0, angular=0.0),
        s=0.0,
        curvature=0.0,
    )

    # Simulate for 5 seconds
    t = 0.0
    for _ in range(500):
        t_pose, _, _, _ = robot.step(t, ref)
        t += 0.01

    # Longitudinal, lateral, and heading errors converge to zero
    assert abs(robot.true_pose.x) < 0.01
    assert abs(robot.true_pose.y) < 0.01
    assert abs(robot.true_pose.theta) < 0.01


def test_moving_reference_tracking_convergence():
    """Verify full 3-DOF convergence (e1, e2, e3 -> 0) when tracking moving trajectory."""
    config = SimulatorConfig(
        dt=0.01,
        actuation_delay=0.05,
        position_noise_std=0.0,
        heading_noise_std=0.0,
        max_linear_velocity=0.30,
        max_angular_velocity=1.50,
    )
    controller = CongaController(
        lambda1=4.5,
        lambda2=7.5,
        lambda3=2.5,
        max_linear_velocity=0.30,
        max_angular_velocity=1.50,
    )

    # Robot starts with perturbed pose: x, y, and theta
    robot = RobotSimulator(
        initial_pose=Pose(x=0.05, y=0.05, theta=0.05),
        controller=controller,
        config=config,
    )

    path = StraightPath(x0=0.0, y0=0.0, theta0=0.0)
    v_cmd = 0.10

    # Simulate tracking along straight line for 35 seconds
    t = 0.0
    s = 0.0
    for _ in range(3500):
        p_state = path.evaluate(s)
        ref = ReferenceState(
            pose=Pose(p_state.x, p_state.y, p_state.theta),
            twist=Twist(linear=v_cmd, angular=0.0),
            s=s,
            curvature=0.0,
        )
        t_pose, _, _, _ = robot.step(t, ref)
        s += v_cmd * 0.01
        t += 0.01

    # Verify that all errors have converged to practically zero
    final_e1 = abs(robot.true_pose.x - s)
    final_e2 = abs(robot.true_pose.y - 0.0)
    final_e3 = abs(wrap_to_pi(robot.true_pose.theta - 0.0))

    assert final_e1 < 0.01
    assert final_e2 < 0.01
    assert final_e3 < 0.01


def test_conga_multi_robot_simulation():
    """Verify Conga multi-robot simulation executes and records full history."""
    path = StraightPath(x0=0.0, y0=0.0, theta0=0.0)
    config = SimulatorConfig(
        dt=0.02,
        actuation_delay=0.05,
        position_noise_std=0.005,
        heading_noise_std=math.radians(0.5),
        seed=123,
    )
    sim = CongaSimulator(
        path=path,
        num_robots=4,
        spacing=1.0,
        v_cmd=0.1,
        initial_leader_s=5.0,
        config=config,
    )

    res = sim.run(duration=1.0)
    assert len(res.steps) == 50
    assert res.num_robots == 4

    # Check spacing distances are approximately 1.0
    distances = res.get_inter_robot_distances()
    assert (0, 1) in distances
    assert (1, 2) in distances
    assert (2, 3) in distances
    # Distance at end of run should be close to desired 1.0m
    for pair in [(0, 1), (1, 2), (2, 3)]:
        assert distances[pair][-1] == pytest.approx(1.0, abs=0.1)
