"""ROS-independent unicycle simulator with delay and measurement noise."""

from collections import deque
from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from robot_conga.controller import CongaController
from robot_conga.geometry import clamp, distance, wrap_to_pi
from robot_conga.models import Pose, ReferenceState, Twist
from robot_conga.path import Path
from robot_conga.propagator import ReferencePropagator


@dataclass
class SimulatorConfig:
    """Configuration parameters for the Robot Conga simulator.

    Attributes:
        dt: Integration time step (seconds). Default 0.01s (100 Hz).
        max_linear_velocity: Saturation limit for linear velocity (m/s).
        max_angular_velocity: Saturation limit for angular velocity (rad/s).
        max_linear_acceleration: Maximum linear acceleration limit (m/s^2).
        max_angular_acceleration: Maximum angular acceleration limit (rad/s^2).
        actuation_delay: Actuation delay in seconds (default 100 ms = 0.1s).
        position_noise_std: 1-sigma position measurement noise (meters).
        heading_noise_std: 1-sigma heading measurement noise (radians).
        measurement_frequency: Frequency of sensor pose updates (Hz). Default 15 Hz.
        seed: Random seed for reproducible noise generation.
    """
    dt: float = 0.01
    max_linear_velocity: float = 0.30
    max_angular_velocity: float = 1.50
    max_linear_acceleration: Optional[float] = 1.0
    max_angular_acceleration: Optional[float] = 3.0
    actuation_delay: float = 0.100  # 100 ms
    position_noise_std: float = 0.02  # 2 cm
    heading_noise_std: float = math.radians(2.0)  # 2 degrees
    measurement_frequency: float = 15.0  # 15 Hz
    seed: Optional[int] = None


class RobotSimulator:
    """Simulates a single non-holonomic unicycle robot with actuation delay and noise."""

    def __init__(
        self,
        initial_pose: Pose,
        controller: CongaController,
        config: SimulatorConfig,
        rng: Optional[np.random.Generator] = None,
    ) -> None:
        """Initialize single robot simulator.

        Args:
            initial_pose: Ground truth starting pose.
            controller: Controller instance for computing commands.
            config: Simulator configuration settings.
            rng: Optional NumPy random number generator.
        """
        self.config = config
        self.controller = controller
        self.rng = rng if rng is not None else np.random.default_rng(config.seed)

        # Ground truth state
        self.true_pose = Pose(initial_pose.x, initial_pose.y, wrap_to_pi(initial_pose.theta))
        self.current_twist = Twist(linear=0.0, angular=0.0)

        # Sensor state
        self.last_measurement_time: float = -1e9
        self.measured_pose = self._generate_measurement(self.true_pose)

        # Actuation delay queue: stores (time_to_apply, Twist)
        self.command_queue: deque[Tuple[float, Twist]] = deque()
        self.active_command = Twist(linear=0.0, angular=0.0)

    def _generate_measurement(self, pose: Pose) -> Pose:
        """Add Gaussian measurement noise to true pose."""
        if self.config.position_noise_std > 0.0:
            nx = float(self.rng.normal(0.0, self.config.position_noise_std))
            ny = float(self.rng.normal(0.0, self.config.position_noise_std))
        else:
            nx, ny = 0.0, 0.0

        if self.config.heading_noise_std > 0.0:
            nth = float(self.rng.normal(0.0, self.config.heading_noise_std))
        else:
            nth = 0.0

        return Pose(
            x=pose.x + nx,
            y=pose.y + ny,
            theta=wrap_to_pi(pose.theta + nth),
        )

    def step(
        self,
        current_time: float,
        reference_state: ReferenceState,
    ) -> Tuple[Pose, Pose, Twist, Twist]:
        """Perform one simulation step of duration config.dt.

        Args:
            current_time: Current simulation time (seconds).
            reference_state: Desired reference state for this robot.

        Returns:
            Tuple of (true_pose, measured_pose, commanded_twist, actual_twist).
        """
        dt = self.config.dt

        # 1. Update noisy measurement at specified frequency (e.g., 15 Hz)
        meas_period = 1.0 / self.config.measurement_frequency
        if (current_time - self.last_measurement_time) >= (meas_period - 1e-9):
            self.measured_pose = self._generate_measurement(self.true_pose)
            self.last_measurement_time = current_time

        # 2. Compute controller command using the latest measured pose
        cmd_twist = self.controller.compute(self.measured_pose, reference_state)

        # 3. Enqueue command into actuation delay queue
        apply_time = current_time + self.config.actuation_delay
        self.command_queue.append((apply_time, cmd_twist))

        # 4. Extract ready commands from queue
        while self.command_queue and self.command_queue[0][0] <= current_time:
            _, self.active_command = self.command_queue.popleft()

        # 5. Apply acceleration limits
        target_u = self.active_command.linear
        target_omega = self.active_command.angular

        cur_u = self.current_twist.linear
        if self.config.max_linear_acceleration is not None:
            max_du = self.config.max_linear_acceleration * dt
            cur_u += clamp(target_u - cur_u, -max_du, max_du)
        else:
            cur_u = target_u
        cur_u = clamp(cur_u, -self.config.max_linear_velocity, self.config.max_linear_velocity)

        cur_omega = self.current_twist.angular
        if self.config.max_angular_acceleration is not None:
            max_domega = self.config.max_angular_acceleration * dt
            cur_omega += clamp(target_omega - cur_omega, -max_domega, max_domega)
        else:
            cur_omega = target_omega
        cur_omega = clamp(cur_omega, -self.config.max_angular_velocity, self.config.max_angular_velocity)

        self.current_twist = Twist(linear=cur_u, angular=cur_omega)

        # 6. Unicycle dynamics integration for true pose
        new_x = self.true_pose.x + self.current_twist.linear * math.cos(self.true_pose.theta) * dt
        new_y = self.true_pose.y + self.current_twist.linear * math.sin(self.true_pose.theta) * dt
        new_theta = wrap_to_pi(self.true_pose.theta + self.current_twist.angular * dt)

        self.true_pose = Pose(x=new_x, y=new_y, theta=new_theta)

        return self.true_pose, self.measured_pose, cmd_twist, self.current_twist


@dataclass
class SimulationStep:
    """Data recorded at a single simulation time step."""
    time: float
    leader_s: float
    true_poses: List[Pose]
    measured_poses: List[Pose]
    reference_states: List[ReferenceState]
    command_twists: List[Twist]
    actual_twists: List[Twist]


@dataclass
class SimulationResult:
    """Container for recorded simulation run."""
    times: List[float] = field(default_factory=list)
    steps: List[SimulationStep] = field(default_factory=list)
    num_robots: int = 0
    desired_spacing: float = 1.0

    def get_robot_trajectory(self, robot_idx: int) -> List[Pose]:
        """Return list of true poses over time for robot index."""
        return [step.true_poses[robot_idx] for step in self.steps]

    def get_reference_trajectory(self, robot_idx: int) -> List[Pose]:
        """Return list of reference poses over time for robot index."""
        return [step.reference_states[robot_idx].pose for step in self.steps]

    def get_robot_s(self, robot_idx: int) -> List[float]:
        """Return list of arc-length reference positions for robot index."""
        return [step.reference_states[robot_idx].s for step in self.steps]

    def get_position_errors(self, robot_idx: int) -> List[float]:
        """Return Euclidean position errors over time for robot index."""
        errors = []
        for step in self.steps:
            p_true = step.true_poses[robot_idx]
            p_ref = step.reference_states[robot_idx].pose
            errors.append(distance(p_true, p_ref))
        return errors

    def get_heading_errors(self, robot_idx: int) -> List[float]:
        """Return absolute heading errors in radians over time for robot index."""
        errors = []
        for step in self.steps:
            th_true = step.true_poses[robot_idx].theta
            th_ref = step.reference_states[robot_idx].pose.theta
            errors.append(abs(wrap_to_pi(th_true - th_ref)))
        return errors

    def get_inter_robot_distances(self) -> Dict[Tuple[int, int], List[float]]:
        """Return inter-robot Euclidean distances between adjacent pairs (i, i+1)."""
        distances: Dict[Tuple[int, int], List[float]] = {}
        for pair_idx in range(self.num_robots - 1):
            distances[(pair_idx, pair_idx + 1)] = []

        for step in self.steps:
            for i in range(self.num_robots - 1):
                d = distance(step.true_poses[i], step.true_poses[i + 1])
                distances[(i, i + 1)].append(d)

        return distances


class CongaSimulator:
    """Multi-robot simulator orchestrating spatial reference propagation and unicycle robots."""

    def __init__(
        self,
        path: Path,
        num_robots: int = 4,
        spacing: float = 1.0,
        v_cmd: float = 0.1,
        initial_leader_s: Optional[float] = None,
        initial_poses: Optional[List[Pose]] = None,
        controllers: Optional[List[CongaController]] = None,
        config: Optional[SimulatorConfig] = None,
    ) -> None:
        """Initialize Conga multi-robot simulator.

        Args:
            path: Trajectory path instance.
            num_robots: Total number of robots (leader + followers).
            spacing: Desired inter-robot arc length spacing (m).
            v_cmd: Leader commanded progression speed (m/s).
            initial_leader_s: Starting leader arc-length position.
            initial_poses: Initial poses for all robots (defaults to reference poses).
            controllers: List of controllers (defaults to standard CongaController).
            config: Simulator configuration parameters.
        """
        self.path = path
        self.num_robots = num_robots
        self.spacing = spacing
        self.v_cmd = v_cmd
        self.config = config if config is not None else SimulatorConfig()
        self.rng = np.random.default_rng(self.config.seed)

        # Ensure leader starts far enough so all followers have s >= 0
        min_leader_s = float(num_robots - 1) * spacing
        if initial_leader_s is None:
            self.leader_s = min_leader_s
        else:
            if initial_leader_s < min_leader_s:
                raise ValueError(
                    f"initial_leader_s={initial_leader_s:.2f} is less than required minimum "
                    f"{min_leader_s:.2f} for {num_robots} robots with spacing {spacing:.2f}m."
                )
            self.leader_s = float(initial_leader_s)

        self.propagator = ReferencePropagator(
            path=self.path,
            spacing=self.spacing,
            number_of_robots=self.num_robots,
            v_cmd=self.v_cmd,
        )

        # Setup initial reference states
        initial_refs = self.propagator.propagate(self.leader_s, self.v_cmd)

        # Setup controllers
        if controllers is None:
            self.controllers = [
                CongaController(
                    lambda1=4.5,
                    lambda2=7.5,
                    lambda3=2.5,
                    max_linear_velocity=self.config.max_linear_velocity,
                    max_angular_velocity=self.config.max_angular_velocity,
                )
                for _ in range(num_robots)
            ]
        else:
            if len(controllers) != num_robots:
                raise ValueError(f"Expected {num_robots} controllers, got {len(controllers)}")
            self.controllers = controllers

        # Setup robot instances
        self.robots: List[RobotSimulator] = []
        for i in range(num_robots):
            init_pose = initial_poses[i] if initial_poses is not None else initial_refs[i].pose
            # Sub-generator for each robot for reproducible independent noise
            robot_rng = np.random.default_rng(
                None if self.config.seed is None else self.config.seed + i + 100
            )
            robot = RobotSimulator(
                initial_pose=init_pose,
                controller=self.controllers[i],
                config=self.config,
                rng=robot_rng,
            )
            self.robots.append(robot)

    def run(self, duration: float) -> SimulationResult:
        """Run the multi-robot simulation for a specified duration in seconds.

        Args:
            duration: Total duration of simulation run (seconds).

        Returns:
            SimulationResult containing recorded trajectory and telemetry data.
        """
        num_steps = int(math.ceil(duration / self.config.dt))
        result = SimulationResult(
            num_robots=self.num_robots,
            desired_spacing=self.spacing,
        )

        current_time = 0.0
        for _ in range(num_steps):
            # Advance leader reference along path
            ref_states = self.propagator.propagate(self.leader_s, self.v_cmd)

            true_poses: List[Pose] = []
            meas_poses: List[Pose] = []
            cmd_twists: List[Twist] = []
            act_twists: List[Twist] = []

            for i in range(self.num_robots):
                t_p, m_p, c_tw, a_tw = self.robots[i].step(current_time, ref_states[i])
                true_poses.append(t_p)
                meas_poses.append(m_p)
                cmd_twists.append(c_tw)
                act_twists.append(a_tw)

            step = SimulationStep(
                time=current_time,
                leader_s=self.leader_s,
                true_poses=true_poses,
                measured_poses=meas_poses,
                reference_states=ref_states,
                command_twists=cmd_twists,
                actual_twists=act_twists,
            )
            result.times.append(current_time)
            result.steps.append(step)

            # Advance simulation clock and leader arc-length
            self.leader_s += self.v_cmd * self.config.dt
            current_time += self.config.dt

        return result
