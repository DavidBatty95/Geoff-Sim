import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from .g1_constants import G1_29DOF_JOINTS


ARM_JOINTS = tuple(G1_29DOF_JOINTS[15:])
ARM_OFFSET = 15
BODY_DOF = len(G1_29DOF_JOINTS)

# Limits are copied from g1_29dof_mode_15_brainco_hand.urdf.  The speed limit is
# deliberately much lower than the actuator limits in that file.
ARM_LIMITS: Dict[str, Tuple[float, float]] = {
    "left_shoulder_pitch_joint": (-3.0892, 2.6704),
    "left_shoulder_roll_joint": (-1.5882, 2.2515),
    "left_shoulder_yaw_joint": (-2.618, 2.618),
    "left_elbow_joint": (-1.0472, 2.0944),
    "left_wrist_roll_joint": (-1.972222054, 1.972222054),
    "left_wrist_pitch_joint": (-1.614429558, 1.614429558),
    "left_wrist_yaw_joint": (-1.614429558, 1.614429558),
    "right_shoulder_pitch_joint": (-3.0892, 2.6704),
    "right_shoulder_roll_joint": (-2.2515, 1.5882),
    "right_shoulder_yaw_joint": (-2.618, 2.618),
    "right_elbow_joint": (-1.0472, 2.0944),
    "right_wrist_roll_joint": (-1.972222054, 1.972222054),
    "right_wrist_pitch_joint": (-1.614429558, 1.614429558),
    "right_wrist_yaw_joint": (-1.614429558, 1.614429558),
}
ARM_MAX_SPEEDS = {name: 0.8 for name in ARM_JOINTS}
START_POSITION_TOLERANCE = 0.05


class TargetPublicationGate:
    """Coalesce identical refreshes; changed targets never wait for a timer."""
    def __init__(self,period=0.):
        if not math.isfinite(period) or not 0.<=period<=.05:
            raise ValueError('repeat publication period must be between 0 and 50 ms')
        self.period = period
        self.targets = None
        self.stamp = None

    def accept(self,targets,now):
        if (self.targets==targets and self.stamp is not None and
                0.<=now-self.stamp<self.period):
            return False
        self.targets = list(targets)
        self.stamp = now
        return True


class TrajectoryError(ValueError):
    pass


@dataclass(frozen=True)
class TrajectoryPointData:
    positions: Tuple[float, ...]
    time_from_start: float
    velocities: Tuple[float, ...] = ()
    accelerations: Tuple[float, ...] = ()
    effort: Tuple[float, ...] = ()


@dataclass(frozen=True)
class ParsedTrajectory:
    joint_names: Tuple[str, ...]
    arm_indices: Tuple[int, ...]
    points: Tuple[TrajectoryPointData, ...]


def parse_trajectory(
    joint_names: Sequence[str],
    points: Sequence[TrajectoryPointData],
    initial_arm_positions: Sequence[float],
) -> ParsedTrajectory:
    names = tuple(joint_names)
    if not names:
        raise TrajectoryError("trajectory has no joints")
    if len(set(names)) != len(names):
        raise TrajectoryError("trajectory contains duplicate joint names")
    unknown = [name for name in names if name not in ARM_LIMITS]
    if unknown:
        raise TrajectoryError(f"joints are not controlled arm joints: {unknown}")
    if not points:
        raise TrajectoryError("trajectory has no points")
    if len(initial_arm_positions) != len(ARM_JOINTS):
        raise TrajectoryError("initial arm position has the wrong dimension")
    if not all(math.isfinite(float(value)) for value in initial_arm_positions):
        raise TrajectoryError("initial arm position contains non-finite values")

    arm_indices = tuple(ARM_JOINTS.index(name) for name in names)
    parsed_points: List[TrajectoryPointData] = []
    previous_time = -1.0
    previous = [float(initial_arm_positions[index]) for index in arm_indices]
    for point in points:
        positions = tuple(float(value) for value in point.positions)
        time_from_start = float(point.time_from_start)
        optional_fields = (point.velocities, point.accelerations, point.effort)
        if len(positions) != len(names):
            raise TrajectoryError("point position dimension does not match joint names")
        if any(field and len(field) != len(names) for field in optional_fields):
            raise TrajectoryError("point field dimension does not match joint names")
        values = positions + tuple(float(value) for field in optional_fields for value in field)
        if not math.isfinite(time_from_start) or not all(math.isfinite(value) for value in values):
            raise TrajectoryError("trajectory contains non-finite values")
        if time_from_start < 0.0 or time_from_start <= previous_time:
            raise TrajectoryError("trajectory times must be non-negative and strictly increasing")
        for name, value in zip(names, positions):
            lower, upper = ARM_LIMITS[name]
            if value < lower or value > upper:
                raise TrajectoryError(f"{name} position {value} is outside [{lower}, {upper}]")
        if point.velocities:
            for name, velocity in zip(names, point.velocities):
                if abs(float(velocity)) > ARM_MAX_SPEEDS[name]:
                    raise TrajectoryError(
                        f"{name} velocity exceeds the {ARM_MAX_SPEEDS[name]} rad/s speed limit"
                    )

        duration = time_from_start if previous_time < 0.0 else time_from_start - previous_time
        normalized_positions = list(positions)
        for index, (name, before, after) in enumerate(zip(names, previous, positions)):
            distance = abs(after - before)
            if duration == 0.0:
                if distance > START_POSITION_TOLERANCE:
                    raise TrajectoryError(
                        f"{name} has an instantaneous position change "
                        f"from {before:.4f} to {after:.4f}"
                    )
                normalized_positions[index] = before
            if duration > 0.0 and distance / duration > ARM_MAX_SPEEDS[name]:
                raise TrajectoryError(f"{name} exceeds the {ARM_MAX_SPEEDS[name]} rad/s speed limit")
        positions = tuple(normalized_positions)
        parsed_points.append(
            TrajectoryPointData(
                positions,
                time_from_start,
                tuple(float(value) for value in point.velocities),
                tuple(float(value) for value in point.accelerations),
                tuple(float(value) for value in point.effort),
            )
        )
        previous = list(positions)
        previous_time = time_from_start
    return ParsedTrajectory(names, arm_indices, tuple(parsed_points))


def interpolate_trajectory(
    trajectory: ParsedTrajectory,
    initial_arm_positions: Sequence[float],
    elapsed: float,
) -> List[float]:
    output = list(initial_arm_positions)
    elapsed = max(0.0, float(elapsed))
    previous_time = 0.0
    previous = [output[index] for index in trajectory.arm_indices]
    for point in trajectory.points:
        if elapsed <= point.time_from_start:
            duration = point.time_from_start - previous_time
            fraction = 1.0 if duration <= 0.0 else (elapsed - previous_time) / duration
            fraction = max(0.0, min(1.0, fraction))
            for index, before, after in zip(trajectory.arm_indices, previous, point.positions):
                output[index] = before + fraction * (after - before)
            return output
        previous_time = point.time_from_start
        previous = list(point.positions)
    for index, value in zip(trajectory.arm_indices, trajectory.points[-1].positions):
        output[index] = value
    return output


def merge_arm_targets(locomotion: Sequence[float], arm_positions: Sequence[float]) -> List[float]:
    if len(locomotion) != BODY_DOF or len(arm_positions) != len(ARM_JOINTS):
        raise ValueError("invalid locomotion or arm target dimension")
    merged = list(locomotion)
    merged[ARM_OFFSET:] = arm_positions
    return merged


class ArmCommandCompositorCore:
    def __init__(
        self,
        input_timeout: float = 0.25,
        goal_tolerance: float = 0.05,
        completion_timeout: float = 1.0,
    ) -> None:
        if not math.isfinite(input_timeout) or input_timeout <= 0.0:
            raise ValueError("input_timeout must be positive")
        if not math.isfinite(goal_tolerance) or goal_tolerance < 0.0:
            raise ValueError("goal_tolerance must be non-negative")
        if not math.isfinite(completion_timeout) or completion_timeout < 0.0:
            raise ValueError("completion_timeout must be non-negative")
        self.input_timeout = float(input_timeout)
        self.goal_tolerance = float(goal_tolerance)
        self.completion_timeout = float(completion_timeout)
        self.locomotion: Optional[List[float]] = None
        self.locomotion_time: Optional[float] = None
        self.measured: Optional[List[float]] = None
        self.measured_time: Optional[float] = None
        self.authority = False
        self.arm_hold: Optional[List[float]] = None
        self.trajectory: Optional[ParsedTrajectory] = None
        self.trajectory_initial: Optional[List[float]] = None
        self.trajectory_start = 0.0
        self.outcome: Optional[Tuple[str, str]] = None

    @staticmethod
    def _valid_body(values: Sequence[float]) -> bool:
        return len(values) == BODY_DOF and all(math.isfinite(float(value)) for value in values)

    def update_locomotion(self, values: Sequence[float], now: float) -> bool:
        if not self._valid_body(values):
            return False
        self.locomotion = list(values)
        self.locomotion_time = float(now)
        return True

    def update_measured(self, values: Sequence[float], now: float) -> bool:
        if not self._valid_body(values):
            return False
        self.measured = [float(value) for value in values]
        self.measured_time = float(now)
        return True

    def _fresh(self, timestamp: Optional[float], now: float) -> bool:
        return timestamp is not None and 0.0 <= now - timestamp <= self.input_timeout

    def acquire(self, now: float) -> Tuple[bool, str]:
        if not self._fresh(self.measured_time, now) or self.measured is None:
            return False, "cannot acquire arm authority without fresh LowState"
        self.authority = True
        self.arm_hold = list(self.measured[ARM_OFFSET:])
        self.trajectory = None
        self.trajectory_initial = None
        self.outcome = None
        return True, "arm authority acquired from measured positions"

    def release(self) -> None:
        if self.trajectory is not None:
            self.outcome = ("released", "arm authority was released")
        self.authority = False
        self.arm_hold = None
        self.trajectory = None
        self.trajectory_initial = None

    def start_trajectory(self, trajectory: ParsedTrajectory, now: float) -> None:
        if not self.authority or self.arm_hold is None:
            raise TrajectoryError("arm authority is not held")
        if self.trajectory is not None:
            raise TrajectoryError("another trajectory is active")
        if not self._fresh(self.measured_time, now) or self.measured is None:
            raise TrajectoryError("cannot start trajectory without fresh LowState")
        self.arm_hold = list(self.measured[ARM_OFFSET:])
        self.trajectory = trajectory
        self.trajectory_initial = list(self.arm_hold)
        self.trajectory_start = float(now)
        self.outcome = None

    def cancel(self, now: float) -> None:
        if self.authority:
            if self._fresh(self.measured_time, now) and self.measured is not None:
                self.arm_hold = list(self.measured[ARM_OFFSET:])
            elif self.trajectory is not None and self.trajectory_initial is not None:
                self.arm_hold = interpolate_trajectory(
                    self.trajectory, self.trajectory_initial, now - self.trajectory_start
                )
        self.trajectory = None
        self.trajectory_initial = None
        self.outcome = ("canceled", "trajectory canceled; holding safe arm positions")

    def output(self, now: float) -> Optional[List[float]]:
        if not self.authority:
            if self.locomotion is None or not self._fresh(self.locomotion_time, now):
                return None
            return list(self.locomotion)
        if not self._fresh(self.locomotion_time, now):
            self._abort_stale("locomotion command is stale")
            return None
        if not self._fresh(self.measured_time, now):
            self._abort_stale("LowState is stale")
            return None
        if self.locomotion is None or self.arm_hold is None:
            return None

        if self.trajectory is not None and self.trajectory_initial is not None:
            elapsed = now - self.trajectory_start
            self.arm_hold = interpolate_trajectory(self.trajectory, self.trajectory_initial, elapsed)
            end_time = self.trajectory.points[-1].time_from_start
            if elapsed >= end_time:
                errors = [
                    abs(self.measured[ARM_OFFSET + index] - self.arm_hold[index])
                    for index in self.trajectory.arm_indices
                ]
                if max(errors, default=0.0) <= self.goal_tolerance:
                    self.trajectory = None
                    self.trajectory_initial = None
                    self.outcome = ("succeeded", "trajectory reached within tolerance")
                elif elapsed > end_time + self.completion_timeout:
                    self.trajectory = None
                    self.trajectory_initial = None
                    self.outcome = ("goal_tolerance", "measured arm did not reach the goal tolerance")
        return merge_arm_targets(self.locomotion, self.arm_hold)

    def _abort_stale(self, message: str) -> None:
        if self.trajectory is not None:
            self.outcome = ("stale", message)
            self.trajectory = None
            self.trajectory_initial = None


def _duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


def _message_trajectory(trajectory) -> Tuple[Tuple[str, ...], Tuple[TrajectoryPointData, ...]]:
    points = tuple(
        TrajectoryPointData(
            tuple(point.positions),
            _duration_seconds(point.time_from_start),
            tuple(point.velocities),
            tuple(point.accelerations),
            tuple(point.effort),
        )
        for point in trajectory.points
    )
    return tuple(trajectory.joint_names), points


def _run_ros_node() -> None:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor,SingleThreadedExecutor

    rclpy.init()
    node = ArmCommandCompositorNode()
    threads = int(node.get_parameter('executor_threads').value)
    executor = SingleThreadedExecutor() if threads==1 else MultiThreadedExecutor(num_threads=threads)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.stop_execution()
        deadline = time.monotonic()+1.
        while rclpy.ok() and node._execution_waiter is not None and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.01)
        executor.shutdown(timeout_sec=1.)
        node.destroy_node()
        rclpy.shutdown()


try:
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from rclpy.task import Future
    from std_msgs.msg import Float64MultiArray,Bool
    from std_srvs.srv import SetBool
    from unitree_hg.msg import LowState

    LATEST_RELIABLE_QOS = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )

    class ArmCommandCompositorNode(Node):
        def __init__(self) -> None:
            super().__init__("g1_arm_command_compositor")
            self.declare_parameter("output_rate", 100.0)
            self.declare_parameter("input_timeout", 0.25)
            self.declare_parameter("goal_tolerance", 0.05)
            self.declare_parameter("completion_timeout", 1.0)
            self.declare_parameter("lowstate_topic", "/lowstate")
            threads = int(self.declare_parameter('executor_threads',2).value)
            if threads not in (1,2):
                raise ValueError('executor_threads must be 1 or 2')
            self._execution_waiter = None
            self.publication_gate = TargetPublicationGate(float(self.declare_parameter('repeat_publish_period',0.).value))
            self.publish_on_locomotion = bool(self.declare_parameter('publish_on_locomotion',True).value)
            rate = float(self.get_parameter("output_rate").value)
            if not math.isfinite(rate) or rate <= 0.0:
                raise ValueError("output_rate must be positive")
            self.core = ArmCommandCompositorCore(
                float(self.get_parameter("input_timeout").value),
                float(self.get_parameter("goal_tolerance").value),
                float(self.get_parameter("completion_timeout").value),
            )
            self._lock = threading.RLock()
            self._publisher = self.create_publisher(
                Float64MultiArray, "/g1/body_position_commands", LATEST_RELIABLE_QOS
            )
            self.create_subscription(
                Float64MultiArray,
                "/g1/locomotion_position_commands",
                self._locomotion_callback,
                LATEST_RELIABLE_QOS,
            )
            self.create_subscription(
                LowState,
                str(self.get_parameter("lowstate_topic").value),
                self._lowstate_callback,
                LATEST_RELIABLE_QOS,
            )
            self.create_service(SetBool, "~/set_arm_authority", self._authority_callback)
            self._action = ActionServer(
                self,
                FollowJointTrajectory,
                "/arm_trajectory_controller/follow_joint_trajectory",
                execute_callback=self._execute_callback,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
                callback_group=ReentrantCallbackGroup(),
            )
            self.create_timer(1.0 / rate, self._publish)
            self._authority_pub = self.create_publisher(Bool,'/g1/arm_authority',1)
            self.create_timer(.05,lambda:self._authority_pub.publish(Bool(data=self.core.authority)))

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds * 1.0e-9

        def stop_execution(self):
            with self._lock:
                if self._execution_waiter is not None and not self._execution_waiter.done():
                    self.core.cancel(self._now())
                    self._execution_waiter.set_result(('shutdown','ROS shutdown'))

        def _locomotion_callback(self, msg: Float64MultiArray) -> None:
            with self._lock:
                valid = self.core.update_locomotion(msg.data, self._now())
            if not valid:
                self.get_logger().warning("Ignoring malformed locomotion command")
            elif self.publish_on_locomotion:
                # A completed inference need not wait another 0--10 ms for the
                # arm timer. The same authority, freshness and trajectory core
                # composes this immediate publication; the timer remains the
                # watchdog and arm-interpolation heartbeat.
                self._publish()

        def _lowstate_callback(self, msg: LowState) -> None:
            if len(msg.motor_state) < BODY_DOF:
                self.get_logger().warning("Ignoring short LowState")
                return
            positions = [float(state.q) for state in msg.motor_state[:BODY_DOF]]
            with self._lock:
                valid = self.core.update_measured(positions, self._now())
            if not valid:
                self.get_logger().warning("Ignoring malformed LowState")

        def _authority_callback(self, request, response):
            with self._lock:
                if request.data:
                    response.success, response.message = self.core.acquire(self._now())
                else:
                    self.core.release()
                    response.success = True
                    response.message = "arm authority released; locomotion pass-through restored"
            return response

        def _parse_request(self, request) -> ParsedTrajectory:
            names, points = _message_trajectory(request.trajectory)
            now = self._now()
            if not self.core._fresh(self.core.measured_time, now) or self.core.measured is None:
                raise TrajectoryError("cannot validate trajectory without fresh LowState")
            return parse_trajectory(names, points, self.core.measured[ARM_OFFSET:])

        def _goal_callback(self, goal_request):
            with self._lock:
                if not self.core.authority or self.core.trajectory is not None or self._execution_waiter is not None:
                    return GoalResponse.REJECT
                try:
                    self._parse_request(goal_request)
                except TrajectoryError as error:
                    self.get_logger().warning(f"Rejecting arm trajectory: {error}")
                    return GoalResponse.REJECT
            return GoalResponse.ACCEPT

        def _cancel_callback(self, _goal_handle):
            with self._lock:
                self.core.cancel(self._now())
            return CancelResponse.ACCEPT

        async def _execute_callback(self, goal_handle):
            result = FollowJointTrajectory.Result()
            with self._lock:
                try:
                    trajectory = self._parse_request(goal_handle.request)
                    self.core.start_trajectory(trajectory, self._now())
                    waiter = Future()
                    self._execution_waiter = waiter
                except TrajectoryError as error:
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = str(error)
                    goal_handle.abort()
                    return result

            try:
                # The existing output/watchdog timer resolves this future.
                # Awaiting yields the executor instead of reserving a worker
                # thread for the whole arm trajectory.
                status,message = await waiter
                result.error_string = message
                if status == "succeeded":
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    goal_handle.succeed()
                elif status == "canceled":
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    goal_handle.canceled()
                elif status == "goal_tolerance":
                    result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    goal_handle.abort()
                else:
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    goal_handle.abort()
                return result
            finally:
                with self._lock:
                    if self._execution_waiter is waiter:
                        self._execution_waiter = None

        def _publish(self) -> None:
            with self._lock:
                now = self._now()
                targets = self.core.output(now)
                waiter = self._execution_waiter
                if waiter is not None and not waiter.done() and self.core.outcome is not None:
                    waiter.set_result(self.core.outcome)
                if targets is not None and not self.publication_gate.accept(targets,now):
                    return
                if targets is None:
                    return
                # Keep output selection and publication in the same critical
                # section. Action cancellation/authority changes must not slip
                # between them, nor may concurrent output calls publish B then A.
                msg = Float64MultiArray()
                msg.data = targets
                self._publisher.publish(msg)

except ImportError:
    ArmCommandCompositorNode = None


def main(args=None) -> None:
    del args
    if ArmCommandCompositorNode is None:
        raise RuntimeError("ROS 2 Python dependencies are unavailable")
    _run_ros_node()
