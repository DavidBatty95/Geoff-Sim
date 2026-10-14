import json
import math
import time
from dataclasses import replace

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray, Marker
from unitree_social_navigation_msgs.msg import PredictedTrajectories
from tf2_ros import Buffer, TransformException, TransformListener

from .bridge_safety import (
    clamp_velocity,
    scan_is_fresh,
    stop_velocity,
    trajectory_makes_target_progress,
    trajectory_static_cost,
)
from .pointcloud import iter_xyz
from .motion_envelope import load_motion_envelope, advance_planar_pose
from .command_envelope import CommandEnvelopeGate,goal_signature
from .prediction_risk import PreparedPredictions,prepare_predictions,time_aligned_prediction_risk
from .escape_search import EscapeSnapshot,EscapeSearchWorker,search_escape,checked_escape,stopped_tail


LATEST_SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def _duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class HumanCollisionAvoidance(Node):
    def __init__(self) -> None:
        super().__init__("human_collision_avoidance")
        self.declare_parameter("input_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("output_cmd_vel_topic", "/cmd_vel_safe")
        self.declare_parameter("debug_topic", "/human_collision_avoidance_debug")
        self.declare_parameter("planner_state_topic", "/human_avoidance_state")
        self.declare_parameter("safe_patch_topic", "/human_avoidance_safe_patch")
        self.declare_parameter("predicted_cloud_topic", "/predicted_dynamic_obstacles")
        self.declare_parameter("predicted_trajectories_topic", "/predicted_person_trajectories")
        self.declare_parameter("tracked_cloud_topic", "/tracked_dynamic_obstacles")
        self.declare_parameter("mujoco_people_topic", "/mujoco/pedestrians")
        self.declare_parameter("use_mujoco_people", True)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("danger_radius", 1.20)
        self.declare_parameter("critical_radius", 0.70)
        self.declare_parameter("trajectory_horizon", 3.0)
        self.declare_parameter("trajectory_dt", 0.2)
        self.declare_parameter("corridor_width", 0.85)
        self.declare_parameter("corridor_critical_width", 0.45)
        self.declare_parameter("corridor_min_length", 0.9)
        self.declare_parameter("max_avoid_linear", 0.18)
        self.declare_parameter("max_reverse_linear", 0.07)
        self.declare_parameter("max_lateral_linear", 0.10)
        self.declare_parameter("max_avoid_angular", 0.65)
        self.declare_parameter("hold_time", 0.8)
        self.declare_parameter("input_timeout", 0.6)
        self.declare_parameter("require_recent_input_for_avoidance", False)
        self.declare_parameter("min_input_speed_for_avoidance", 0.03)
        self.declare_parameter("stop_on_human_risk", True)
        self.declare_parameter("emergency_stop_radius", 1.35)
        self.declare_parameter("robot_self_filter_radius", 0.45)
        self.declare_parameter("rate_hz", 12.0)
        self.declare_parameter("hold_start_location", False)
        self.declare_parameter("return_position_tolerance", 0.18)
        self.declare_parameter("return_yaw_tolerance", 0.25)
        self.declare_parameter("return_linear_speed", 0.12)
        self.declare_parameter("return_angular_speed", 0.35)
        self.declare_parameter("return_linear_heading_tolerance", 0.55)
        self.declare_parameter("return_max_tilt", 0.45)
        self.declare_parameter("enable_open_space_bias", True)
        self.declare_parameter("wall_repulsion_clearance", 1.15)
        self.declare_parameter("wall_repulsion_range", 2.4)
        self.declare_parameter("wall_repulsion_gain", 0.11)
        self.declare_parameter("wall_repulsion_max_linear", 0.12)
        self.declare_parameter("wall_repulsion_max_angular", 0.35)
        self.declare_parameter("people_repulsion_range", 2.2)
        self.declare_parameter("predicted_repulsion_range", 1.5)
        self.declare_parameter("predicted_repulsion_corridor_width", 0.75)
        self.declare_parameter("predicted_repulsion_forward_scale", 0.10)
        self.declare_parameter("people_repulsion_gain", 0.08)
        self.declare_parameter("people_repulsion_max_linear", 0.08)
        self.declare_parameter("people_repulsion_max_angular", 0.30)
        self.declare_parameter("local_risk_radius", 2.4)
        self.declare_parameter("max_repulsion_points", 16)
        self.declare_parameter("escape_search_horizon", 1.2)
        self.declare_parameter("escape_search_dt", 0.2)
        self.declare_parameter("escape_progress_weight", 0.15)
        self.declare_parameter("escape_reverse_penalty", 0.25)
        self.declare_parameter("repulsion_timeout", 0.8)
        self.declare_parameter("startup_grace_time", 2.0)
        self.declare_parameter("person_cloud_timeout", 0.8)
        self.declare_parameter("prediction_collision_radius", 1.0)
        self.declare_parameter("prediction_uncertainty_scale", 0.5)
        self.declare_parameter("prediction_min_confidence", 0.35)
        self.declare_parameter("scan_timeout", 0.6)
        self.declare_parameter("static_escape_clearance", 0.48)
        self.declare_parameter("predictive_escape_search", False)
        self.declare_parameter('async_predictive_escape_search',False)
        self.declare_parameter('escape_result_max_age',.2)
        self.async_predictive_escape_search = bool(self.get_parameter('async_predictive_escape_search').value)
        self.escape_result_max_age = float(self.get_parameter('escape_result_max_age').value)
        if not 0.<self.escape_result_max_age<=.5:
            raise ValueError('escape_result_max_age must be in (0, 0.5] seconds')
        self.escape_worker = None
        self.cached_escape = None
        self.async_escape_state = 'idle'
        self.async_escape_rejections = 0
        self.declare_parameter("use_planned_sequence", False)
        self.declare_parameter('require_command_envelope',False)
        self.require_command_envelope = bool(self.get_parameter('require_command_envelope').value)
        if self.async_predictive_escape_search and not self.require_command_envelope:
            raise ValueError('async_predictive_escape_search requires require_command_envelope for intent fencing')
        if self.async_predictive_escape_search:
            self.escape_worker = EscapeSearchWorker()
        self.command_gate = CommandEnvelopeGate()
        self.declare_parameter("odom_timeout", 0.4)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.declare_parameter("response_delay", 0.10)
        self.motion_envelope = load_motion_envelope(self)
        self.response_delay = float(self.get_parameter("response_delay").value)
        self.latest_predictions = []
        self.prediction_epoch = None
        self.measured_velocity = (0.0, 0.0, 0.0)
        self.policy_command = None
        self.policy_command_stamp = None
        self.odom_stamp = None
        self.odom_pose = None

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.danger_radius = float(self.get_parameter("danger_radius").value)
        self.critical_radius = float(self.get_parameter("critical_radius").value)
        self.trajectory_horizon = float(self.get_parameter("trajectory_horizon").value)
        self.trajectory_dt = float(self.get_parameter("trajectory_dt").value)
        self.corridor_width = float(self.get_parameter("corridor_width").value)
        self.corridor_critical_width = float(self.get_parameter("corridor_critical_width").value)
        self.corridor_min_length = float(self.get_parameter("corridor_min_length").value)
        self.max_avoid_linear = float(self.get_parameter("max_avoid_linear").value)
        self.max_reverse_linear = float(self.get_parameter("max_reverse_linear").value)
        self.max_lateral_linear = float(self.get_parameter("max_lateral_linear").value)
        self.max_avoid_angular = float(self.get_parameter("max_avoid_angular").value)
        self.hold_time = float(self.get_parameter("hold_time").value)
        self.input_timeout = float(self.get_parameter("input_timeout").value)
        self.require_recent_input_for_avoidance = bool(self.get_parameter("require_recent_input_for_avoidance").value)
        self.min_input_speed_for_avoidance = float(self.get_parameter("min_input_speed_for_avoidance").value)
        self.stop_on_human_risk = bool(self.get_parameter("stop_on_human_risk").value)
        self.emergency_stop_radius = float(self.get_parameter("emergency_stop_radius").value)
        self.robot_self_filter_radius = float(self.get_parameter("robot_self_filter_radius").value)
        self.hold_start_location = bool(self.get_parameter("hold_start_location").value)
        self.return_position_tolerance = float(self.get_parameter("return_position_tolerance").value)
        self.return_yaw_tolerance = float(self.get_parameter("return_yaw_tolerance").value)
        self.return_linear_speed = float(self.get_parameter("return_linear_speed").value)
        self.return_angular_speed = float(self.get_parameter("return_angular_speed").value)
        self.return_linear_heading_tolerance = float(self.get_parameter("return_linear_heading_tolerance").value)
        self.return_max_tilt = float(self.get_parameter("return_max_tilt").value)
        self.enable_open_space_bias = bool(self.get_parameter("enable_open_space_bias").value)
        self.wall_repulsion_clearance = float(self.get_parameter("wall_repulsion_clearance").value)
        self.wall_repulsion_range = float(self.get_parameter("wall_repulsion_range").value)
        self.wall_repulsion_gain = float(self.get_parameter("wall_repulsion_gain").value)
        self.wall_repulsion_max_linear = float(self.get_parameter("wall_repulsion_max_linear").value)
        self.wall_repulsion_max_angular = float(self.get_parameter("wall_repulsion_max_angular").value)
        self.people_repulsion_range = float(self.get_parameter("people_repulsion_range").value)
        self.predicted_repulsion_range = float(self.get_parameter("predicted_repulsion_range").value)
        self.predicted_repulsion_corridor_width = float(self.get_parameter("predicted_repulsion_corridor_width").value)
        self.predicted_repulsion_forward_scale = float(self.get_parameter("predicted_repulsion_forward_scale").value)
        self.people_repulsion_gain = float(self.get_parameter("people_repulsion_gain").value)
        self.people_repulsion_max_linear = float(self.get_parameter("people_repulsion_max_linear").value)
        self.people_repulsion_max_angular = float(self.get_parameter("people_repulsion_max_angular").value)
        self.local_risk_radius = float(self.get_parameter("local_risk_radius").value)
        self.use_mujoco_people = bool(self.get_parameter("use_mujoco_people").value)
        self.max_repulsion_points = int(self.get_parameter("max_repulsion_points").value)
        self.escape_search_horizon = float(self.get_parameter("escape_search_horizon").value)
        self.escape_search_dt = float(self.get_parameter("escape_search_dt").value)
        self.escape_progress_weight = float(self.get_parameter("escape_progress_weight").value)
        self.escape_reverse_penalty = float(self.get_parameter("escape_reverse_penalty").value)
        self.repulsion_timeout = float(self.get_parameter("repulsion_timeout").value)
        self.startup_grace_time = float(self.get_parameter("startup_grace_time").value)
        self.person_cloud_timeout = float(self.get_parameter("person_cloud_timeout").value)
        self.prediction_collision_radius = float(self.get_parameter("prediction_collision_radius").value)
        self.empty_prediction_collision_radius = self.prediction_collision_radius
        self.recovery_min_clearance = .80
        self.prediction_uncertainty_scale = float(self.get_parameter("prediction_uncertainty_scale").value)
        self.prediction_min_confidence = float(self.get_parameter("prediction_min_confidence").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.static_escape_clearance = float(self.get_parameter("static_escape_clearance").value)
        self.predictive_escape_search = bool(self.get_parameter("predictive_escape_search").value)
        self.use_planned_sequence = bool(self.get_parameter("use_planned_sequence").value)
        self.command_sequence = None
        self.goal_signature = None
        self.mission_active = False
        self.mission_state_time = None
        self.scan_frame = None
        self.scan_stamp = None
        rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.supervisor_period_s = 1.0/rate_hz
        self.supervisor_tick_started = None
        self.supervisor_tick_interval_ms = None
        self.supervisor_search_ms = 0.
        if self.critical_radius > self.danger_radius:
            raise ValueError("critical_radius must be less than or equal to danger_radius")
        if self.trajectory_dt <= 0.0 or self.escape_search_dt <= 0.0:
            raise ValueError("trajectory_dt and escape_search_dt must be positive")
        if self.max_repulsion_points < 1:
            raise ValueError("max_repulsion_points must be at least 1")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_cmd = Twist()
        self.latest_cmd_time = None
        self.avoid_until = self.get_clock().now()
        self.avoid_cmd = Twist()
        self.closest_risk = math.inf
        self.home_pose = None
        self.wall_force = (0.0, 0.0)
        self.wall_force_time = None
        self.scan_obstacles = []
        self.people_forces = {}
        self.person_cloud_times = {}
        self.last_risk_source = "none"
        self.last_risk_point = None
        self.last_avoid_reason = "none"
        self.avoid_side = 0.0
        self.avoid_side_until = self.get_clock().now()
        self.started_at = self.get_clock().now()
        self.forward_escape_until = self.get_clock().now()
        self.planner_state = ""
        self.planner_state_time = None
        self.safe_patch: tuple[float, float] | None = None
        self.safe_patch_time = None

        input_topic = str(self.get_parameter("input_cmd_vel_topic").value)
        output_topic = str(self.get_parameter("output_cmd_vel_topic").value)

        self.pub = self.create_publisher(Twist, output_topic, 1)
        self.visual_pub = self.create_publisher(MarkerArray, "/navigation/command_markers", 1)
        self.last_visual_publish = -math.inf
        self.debug_pub = self.create_publisher(String, str(self.get_parameter("debug_topic").value), 10)
        self.create_subscription(
            Twist,
            input_topic,
            self._cmd_callback,
            1,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("planner_state_topic").value),
            self._planner_state_callback,
            1,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("safe_patch_topic").value),
            self._safe_patch_callback,
            10,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("predicted_cloud_topic").value),
            lambda msg: self._cloud_callback(msg, "predicted"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PredictedTrajectories,
            str(self.get_parameter("predicted_trajectories_topic").value),
            self._predicted_trajectories_callback,
            1,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("tracked_cloud_topic").value),
            lambda msg: self._cloud_callback(msg, "tracked"),
            qos_profile_sensor_data,
        )
        if self.use_mujoco_people:
            self.create_subscription(
                String,
                str(self.get_parameter("mujoco_people_topic").value),
                self._mujoco_people_callback,
                10,
            )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._scan_callback,
            LATEST_SENSOR_QOS,
        )
        if self.use_planned_sequence or self.require_command_envelope:
            self.create_subscription(String,'/human_avoidance_command_sequence',self._sequence_callback,1)
            self.create_subscription(Bool,'/benchmark/goal_active',self._mission_callback,1)
            self.create_subscription(PoseStamped,'/benchmark/current_goal',self._goal_callback,1)
        self.create_timer(1.0 / rate_hz, self._timer_callback)
        self.create_subscription(Bool,'/navigation/carrying',self._payload_callback,1)
        self.create_subscription(Odometry, "/odom", self._odom_callback, LATEST_SENSOR_QOS)
        self.create_subscription(TwistStamped, "/g1/policy_cmd_vel", self._policy_command_callback, LATEST_SENSOR_QOS)
        self.get_logger().info(f"Human collision avoidance filter ready: {input_topic} -> {output_topic}")

    def _clear_sequence(self):
        self.command_sequence = None

    def _payload_callback(self,msg):
        radius = self.empty_prediction_collision_radius+(.16 if msg.data else 0.)
        if radius != self.prediction_collision_radius:
            self._invalidate_escape()
        self.prediction_collision_radius = radius
        self.recovery_min_clearance = .96 if msg.data else .80
        self.motion_envelope = replace(self.motion_envelope,payload_active=bool(msg.data))

    def _goal_callback(self,msg):
        signature = goal_signature(msg)
        if signature != self.goal_signature:
            self._invalidate_escape()
        if self.require_command_envelope and signature != self.goal_signature:
            self._fence_command_input()
        self._clear_sequence()
        self.goal_signature = signature

    def _mission_callback(self,msg):
        if bool(msg.data) != self.mission_active:
            self._invalidate_escape()
        if self.require_command_envelope and bool(msg.data) != self.mission_active:
            self._fence_command_input()
        self.mission_active = bool(msg.data)
        self.mission_state_time = self.get_clock().now()
        if not self.mission_active:
            self._clear_sequence()

    def _sequence_callback(self,msg):
        if self.require_command_envelope:
            try:
                sequence = json.loads(msg.data)
            except (ValueError,TypeError):
                self.command_gate.reject('invalid_json')
                return
            now = self.get_clock().now()
            previous_publisher = self.command_gate.publisher
            error = self.command_gate.accept(sequence,now.nanoseconds*1e-9,self.robot_frame,
                self.goal_signature,self.mission_active,min(.25,self.input_timeout))
            if error is not None:
                return
            if previous_publisher != self.command_gate.publisher:
                self._invalidate_escape()
            self.latest_cmd = self._twist_from_velocity(tuple(sequence['first']))
            self.latest_cmd_time = rclpy.time.Time(nanoseconds=int(sequence['stamp']*1e9),clock_type=now.clock_type)
            self.planner_state = sequence['state']
            self.planner_state_time = self.latest_cmd_time
            self.command_sequence = sequence if sequence.get('then') is not None else None
            return
        self._clear_sequence()
        try:
            sequence = json.loads(msg.data)
            if (sequence.get('frame') != self.robot_frame or sequence.get('active') is not True or
                    sequence.get('then') is None):
                return
            if any(not isinstance(sequence.get(key),list) or len(sequence[key]) != 3 for key in ('first','then')):
                return
            values = [sequence['stamp'],sequence['duration'],*sequence['first'],*sequence['then']]
            if not all(type(value) in (int,float) and math.isfinite(value) for value in values):
                return
            if not 0. <= sequence['duration'] <= 10.:
                return
            if sequence['stamp'] > self.get_clock().now().nanoseconds*1e-9:
                return
            self.command_sequence = sequence
        except (ValueError,KeyError,TypeError,AttributeError):
            return

    def _sequence_parameters(self,command,seconds):
        sequence = self.command_sequence
        if not self.use_planned_sequence or sequence is None or not self.mission_active or self.mission_state_time is None:
            return None
        if self.goal_signature is None or sequence.get('goal') != self.goal_signature:
            return None
        age = seconds-sequence['stamp']
        mission_age = seconds-self.mission_state_time.nanoseconds*1e-9
        if not 0. <= age <= min(.25,self.input_timeout) or not 0. <= mission_age <= 1.:
            return None
        if any(abs(a-b) > 1e-4 for a,b in zip(command,sequence['first'])):
            return None
        return sequence['duration']-age,self.motion_envelope.normalize(sequence['then'])

    def _odom_callback(self, msg: Odometry) -> None:
        velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
        if all(math.isfinite(value) for value in velocity):
            self.measured_velocity = velocity
            self.odom_stamp = _duration_seconds(msg.header.stamp)
            self.odom_pose = None
            if msg.header.frame_id == self.target_frame and msg.child_frame_id == self.robot_frame:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                pose = (p.x, p.y, self._yaw_from_quat(q), self._tilt_from_quat(q))
                if all(math.isfinite(value) for value in pose):
                    self.odom_pose = pose

    def _policy_command_callback(self, msg: TwistStamped) -> None:
        values = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        if all(math.isfinite(value) for value in values):
            self.policy_command = values
            self.policy_command_stamp = _duration_seconds(msg.header.stamp)

    def _cmd_callback(self, msg: Twist) -> None:
        if self.require_command_envelope:
            return  # The Twist remains a visualization/legacy interface only.
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()

    def _planner_state_callback(self, msg: String) -> None:
        if self.require_command_envelope:
            return
        self.planner_state = msg.data
        self.planner_state_time = self.get_clock().now()

    def _fence_command_input(self):
        self.command_gate.fence(self.get_clock().now().nanoseconds*1e-9)
        self.latest_cmd = Twist()
        self.latest_cmd_time = None
        self.planner_state = ''
        self.planner_state_time = None
        self._clear_sequence()

    def _invalidate_escape(self):
        self.cached_escape = None
        self.async_escape_state = 'invalidated'
        if self.escape_worker is not None:
            self.escape_worker.invalidate()

    def _safe_patch_callback(self, msg: PoseStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            return
        self.safe_patch = (float(msg.pose.position.x), float(msg.pose.position.y))
        self.safe_patch_time = self.get_clock().now()

    def _scan_callback(self, msg: LaserScan) -> None:
        self.scan_frame = msg.header.frame_id
        self.scan_stamp = _duration_seconds(msg.header.stamp)
        fx = 0.0
        fy = 0.0
        considered = 0
        obstacles = []
        max_range = max(0.0, self.wall_repulsion_range)
        clearance = max(1e-3, self.wall_repulsion_clearance)
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or distance <= msg.range_min:
                continue
            if distance > max_range:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            obstacles.append((distance * math.cos(angle), distance * math.sin(angle)))
            if distance > clearance:
                continue
            strength = ((clearance - distance) / clearance) ** 2
            fx -= math.cos(angle) * strength
            fy -= math.sin(angle) * strength
            considered += 1
        if considered > 0:
            fx /= considered
            fy /= considered
        self.wall_force = (fx, fy)
        self.wall_force_time = self.get_clock().now()
        self.scan_obstacles = obstacles

    def _cloud_callback(self, msg: PointCloud2, source: str) -> None:
        if self._typed_safety_fresh():
            return
        msg_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_seconds = self.get_clock().now().nanoseconds * 1e-9
        if msg_stamp > 0.0 and now_seconds - msg_stamp > self.person_cloud_timeout:
            self.people_forces.pop(source, None)
            self.get_logger().warn(
                f"Ignoring stale {source} person cloud age={now_seconds - msg_stamp:.2f}s",
                throttle_duration_sec=1.0,
            )
            return
        self.person_cloud_times[source] = self.get_clock().now()
        home_hold_active = self.hold_start_location and self.home_pose is not None
        if (self.get_clock().now() - self.started_at).nanoseconds * 1e-9 < self.startup_grace_time:
            return
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.robot_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(
                f"No transform {self.target_frame} <- {self.robot_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        yaw = self._yaw_from_quat(transform.transform.rotation)
        trajectory = self._planned_trajectory()

        risk_points = []
        local_risk_points = []
        force_candidates = []
        closest = math.inf
        closest_corridor = math.inf
        source_range = self.predicted_repulsion_range if source == "predicted" else self.people_repulsion_range
        mujoco_stamp = self.person_cloud_times.get("mujoco")
        mujoco_fresh = (
            self.use_mujoco_people
            and mujoco_stamp is not None
            and (self.get_clock().now() - mujoco_stamp).nanoseconds * 1e-9 <= self.person_cloud_timeout
        )
        hard_source = source != "predicted" and not (source == "tracked" and mujoco_fresh)
        for x, y, _ in iter_xyz(msg):
            dx = x - robot_x
            dy = y - robot_y
            distance = math.hypot(dx, dy)
            if distance < self.robot_self_filter_radius:
                continue
            if distance < closest:
                closest = distance
                if hard_source:
                    self.last_risk_point = (source, x, y, distance)
            local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
            local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            corridor_distance = self._distance_to_trajectory(local_x, local_y, trajectory)
            if corridor_distance < closest_corridor:
                closest_corridor = corridor_distance
            if distance <= source_range and (
                source != "predicted" or corridor_distance <= self.predicted_repulsion_corridor_width
            ):
                strength = ((source_range - distance) / max(1e-3, source_range)) ** 2
                force_candidates.append((distance, -local_x / max(distance, 1e-3) * strength, -local_y / max(distance, 1e-3) * strength))
            if distance > self.local_risk_radius:
                continue
            if distance <= self.danger_radius:
                risk_points.append((dx, dy, distance, distance, "radius"))
                local_risk_points.append((local_x, local_y, "radius"))
            elif corridor_distance <= self.corridor_width:
                risk_points.append((dx, dy, distance, corridor_distance, "corridor"))
                local_risk_points.append((local_x, local_y, "corridor"))
        if force_candidates:
            force_candidates.sort(key=lambda item: item[0])
            selected = force_candidates[: max(1, self.max_repulsion_points)]
            forward_scale = self.predicted_repulsion_forward_scale if source == "predicted" else 1.0
            self.people_forces[source] = (
                forward_scale * sum(item[1] for item in selected) / len(selected),
                sum(item[2] for item in selected) / len(selected),
                self.get_clock().now(),
            )
        else:
            self.people_forces.pop(source, None)
        # The legacy prediction cloud is an inflated visualization corridor.
        # It has no time dimension, so it is suitable only for soft repulsion.
        if not hard_source:
            return
        self.closest_risk = closest
        if risk_points:
            self.last_risk_source = source
        if not risk_points:
            if self.get_clock().now() >= self.avoid_until:
                self.closest_risk = math.inf
            return

        # Move away from the centre of predicted human-risk points in robot-local coordinates.
        mean_dx = sum(point[0] for point in risk_points) / len(risk_points)
        mean_dy = sum(point[1] for point in risk_points) / len(risk_points)
        local_x = math.cos(yaw) * mean_dx + math.sin(yaw) * mean_dy
        local_y = -math.sin(yaw) * mean_dx + math.cos(yaw) * mean_dy
        nearest = min(point[2] for point in risk_points)
        nearest_corridor = min(point[3] for point in risk_points)
        has_radius_risk = any(point[4] == "radius" for point in risk_points)
        radius_urgency = max(
            0.0,
            min(
                1.0,
                (self.danger_radius - nearest)
                / max(1e-3, self.danger_radius - self.critical_radius),
            ),
        )
        corridor_urgency = max(
            0.0,
            min(
                1.0,
                (self.corridor_width - nearest_corridor)
                / max(1e-3, self.corridor_width - self.corridor_critical_width),
            ),
        )
        urgency = max(radius_urgency, corridor_urgency)

        cmd = Twist()
        if not self._input_stale():
            cmd.linear.x = float(self.latest_cmd.linear.x)
            cmd.linear.y = float(self.latest_cmd.linear.y)
            cmd.angular.z = float(self.latest_cmd.angular.z)

        input_active = self._input_active()
        if self.require_recent_input_for_avoidance and not input_active and not home_hold_active:
            self.avoid_cmd = Twist()
            self.avoid_until = self.get_clock().now() + Duration(seconds=min(self.hold_time, 0.5))
            self.last_avoid_reason = "inactive_input_risk_hold"
            return
        if not input_active and not home_hold_active:
            self.avoid_cmd = Twist()
            self.avoid_until = self.get_clock().now()
            self.last_avoid_reason = "inactive_input_observe_only"
            return

        if self.stop_on_human_risk:
            if self._safe_patch_command_is_valid(cmd):
                self.avoid_cmd = cmd
                self.avoid_until = self.get_clock().now() + Duration(seconds=min(0.2, self.hold_time))
                self.last_avoid_reason = "validated_safe_patch"
                return
            if nearest <= self.emergency_stop_radius:
                self.avoid_cmd = self._twist_from_velocity(stop_velocity())
                self.avoid_until = self.get_clock().now() + Duration(seconds=self.hold_time)
                self.last_avoid_reason = "emergency_stop_radius"
                return
            self.avoid_cmd = self._twist_from_velocity(stop_velocity())
            self.avoid_until = self.get_clock().now() + Duration(seconds=self.hold_time)
            self.last_avoid_reason = "trajectory_risk_stop"
            return

        cmd = self._best_escape_cmd(cmd, local_risk_points, urgency, has_radius_risk)
        if has_radius_risk or urgency >= 0.55:
            self.avoid_cmd = cmd
        else:
            self.avoid_cmd = self._apply_open_space_bias(cmd)
        self.avoid_until = self.get_clock().now() + Duration(seconds=self.hold_time)
        self.last_avoid_reason = "trajectory_risk"

    def _predicted_trajectories_callback(self, msg: PredictedTrajectories) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            self.get_logger().warn(
                f"Ignoring prediction frame {msg.header.frame_id}; expected {self.target_frame}",
                throttle_duration_sec=2.0,
            )
            return
        epoch = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        now = self.get_clock().now()
        now_seconds = now.nanoseconds * 1e-9
        raw_age = now_seconds - epoch if epoch > 0.0 else 0.0
        if raw_age < -0.05:
            self.get_logger().warn("Ignoring future-dated person prediction", throttle_duration_sec=2.0)
            return
        age = max(0.0, raw_age)
        if age > self.person_cloud_timeout:
            return
        pose = self._robot_pose()
        if pose is None:
            return
        trajectories = []
        for trajectory in msg.trajectories:
            samples = []
            for point in trajectory.points:
                samples.append((
                    _duration_seconds(point.time_offset),
                    float(point.position.x),
                    float(point.position.y),
                    float(point.position_covariance[0]),
                    float(point.position_covariance[4]),
                    float(point.confidence),
                ))
            trajectories.append(samples)
        if self.prediction_epoch is not None and epoch<self.prediction_epoch:
            self._invalidate_escape()
        self.latest_predictions = prepare_predictions(trajectories,.1)
        self.prediction_epoch = epoch
        self.person_cloud_times["predicted_typed"] = now

    def _validate_final_command(self, cmd: Twist, pose, now) -> tuple[Twist, str | None]:
        pose = pose[:3]
        command = self.motion_envelope.normalize((cmd.linear.x, cmd.linear.y, cmd.angular.z))
        seconds = now.nanoseconds * 1e-9
        duration,next_command = math.inf,None
        sequence = self._sequence_parameters(command,seconds)
        if sequence is not None:
            duration,next_command = sequence
            if duration <= 0.:
                command,duration,next_command = next_command,math.inf,None
        if self.prediction_epoch is None or not 0.0 <= seconds - self.prediction_epoch <= self.person_cloud_timeout:
            return Twist(), "stale_predictions_stop"
        if self.odom_stamp is None or not 0.0 <= seconds - self.odom_stamp <= getattr(self, "odom_timeout", 0.4):
            return Twist(), "stale_odometry_stop"
        # The planner handles long-range social choices; this final check covers
        # response + braking with margin, re-evaluated for every output command.
        braking_time = max(abs(self.measured_velocity[0]) / max(0.01, self.motion_envelope.accel_x), abs(self.measured_velocity[1]) / max(0.01, self.motion_envelope.accel_y))
        horizon = min(self.trajectory_horizon, max(1.0, self.response_delay + braking_time + 0.5))
        if self.planner_state == "CLEARANCE_RECOVERY" and self.planner_state_time is not None and (now - self.planner_state_time).nanoseconds * 1e-9 <= self.input_timeout:
            horizon = min(horizon, 1.0)
        filtered = self.policy_command if self.policy_command_stamp is not None and 0.0 <= seconds - self.policy_command_stamp <= 0.4 else None
        rollout = self.motion_envelope.rollout(pose, command, self.measured_velocity, horizon, 0.1, self.response_delay,
                                               duration,next_command,filtered)
        risk = time_aligned_prediction_risk(self.latest_predictions, seconds - self.prediction_epoch, pose, command, horizon, 0.1, self.prediction_collision_radius, self.prediction_uncertainty_scale, self.prediction_min_confidence, rollout)
        if risk is not None:
            stop_rollout = self.motion_envelope.rollout(pose, (0.0, 0.0, 0.0), self.measured_velocity, horizon, 0.1, self.response_delay, filtered_command=filtered)
            stop_risk = time_aligned_prediction_risk(self.latest_predictions, seconds - self.prediction_epoch, pose, (0.0, 0.0, 0.0), horizon, 0.1, math.inf, self.prediction_uncertainty_scale, self.prediction_min_confidence, stop_rollout)
            if risk[0] >= self.recovery_min_clearance and stop_risk is not None:
                improves = risk[0] > stop_risk[0]+.05
                if not improves and risk[0] >= stop_risk[0]-.001:
                    tail = self._tail_clearance(pose,command,rollout,seconds,horizon)
                    stopped_tail = self._tail_clearance(pose,(0.,0.,0.),stop_rollout,seconds,horizon)
                    improves = tail is not None and stopped_tail is not None and tail > stopped_tail+.05
                if improves:
                    return self._twist_from_velocity(command), "clearance_recovery"
            if self.predictive_escape_search and stop_risk is not None and stop_risk[0] < self.prediction_collision_radius:
                search_started = time.perf_counter()
                escape = (self._async_predictive_escape(pose,seconds,horizon,filtered,stop_risk[0],command)
                          if self.async_predictive_escape_search else
                          self._validated_predictive_escape(pose,seconds,horizon,filtered,stop_risk[0],command))
                self.supervisor_search_ms += (time.perf_counter()-search_started)*1000.
                if escape is not None:
                    return self._twist_from_velocity(escape), "validated_escape_async" if self.async_predictive_escape_search else "validated_escape"
            clearance, conflict_time, x, y = risk
            self.last_risk_source = "predicted_typed"
            self.last_risk_point = ("predicted_typed", x, y, clearance)
            self.last_avoid_reason = f"predicted_space_time_stop_{conflict_time:.1f}s"
            return Twist(), "temporal_override"
        return self._twist_from_velocity(command), "planned_sequence" if sequence is not None else None

    def _tail_clearance(self,pose,command,rollout,seconds,horizon):
        risk = time_aligned_prediction_risk(self.latest_predictions,seconds-self.prediction_epoch,
            pose,command,horizon,.1,math.inf,self.prediction_uncertainty_scale,
            self.prediction_min_confidence,rollout,minimum_time=max(0.,horizon-.3))
        return None if risk is None else risk[0]

    def _validated_predictive_escape(self, pose, seconds, horizon, filtered, stop_clearance, preferred):
        """Find a checked alternative when stopping also predicts a conflict."""
        snapshot = self._escape_snapshot(pose,seconds,horizon,filtered,stop_clearance,preferred)
        return None if snapshot is None else search_escape(snapshot)

    def _escape_snapshot(self,pose,seconds,horizon,filtered,stop_clearance,preferred):
        if (self.scan_frame != self.robot_frame or self.scan_stamp is None or
                not 0. <= seconds-self.scan_stamp <= self.input_timeout):
            return None
        predictions = (self.latest_predictions if isinstance(self.latest_predictions,PreparedPredictions)
                       else prepare_predictions(self.latest_predictions,.1))
        return EscapeSnapshot(tuple(pose),seconds,horizon,self.trajectory_horizon,self.motion_envelope,
            tuple(self.measured_velocity),None if filtered is None else tuple(filtered),
            predictions,self.prediction_epoch,tuple(tuple(point) for point in self.scan_obstacles),
            self.static_escape_clearance,max(self.static_escape_clearance,self.wall_repulsion_range),
            self.response_delay,self.prediction_uncertainty_scale,self.prediction_min_confidence,
            self.recovery_min_clearance,stop_clearance,tuple(preferred))

    def _async_predictive_escape(self,pose,seconds,horizon,filtered,stop_clearance,preferred):
        snapshot = self._escape_snapshot(pose,seconds,horizon,filtered,stop_clearance,preferred)
        if snapshot is None:
            self.cached_escape = None
            self.async_escape_state = 'stale_scan'
            return None
        self._collect_escape_result()
        # Only one job may exist. The worker receives no mutable node state.
        self.escape_worker.submit(snapshot)
        cached = self.cached_escape
        if (cached is None or cached.generation != self.escape_worker.generation or
                not 0.<=seconds-cached.snapshot.seconds<=self.escape_result_max_age or
                not 0.<=time.monotonic()-cached.submitted_at<=self.escape_result_max_age):
            self.cached_escape = None
            self.async_escape_state = 'waiting_for_current_result'
            return None
        if cached.command is None:
            self.async_escape_state = 'no_feasible_proposal'
            return None
        # The worker's old clearance is never a publication certificate.
        checked = checked_escape(snapshot,cached.command,stopped_tail(snapshot))
        if checked is None:
            self.cached_escape = None
            self.async_escape_rejections += 1
            self.async_escape_state = 'revalidation_rejected'
            return None
        self.async_escape_state = 'revalidated'
        return checked[0]

    def _collect_escape_result(self):
        if self.escape_worker is not None:
            result = self.escape_worker.poll(self.escape_result_max_age)
            if result is not None:
                self.cached_escape = result

    def _typed_safety_fresh(self) -> bool:
        return self.prediction_min_confidence <= 1.0 and self.prediction_epoch is not None and 0.0 <= self.get_clock().now().nanoseconds * 1e-9 - self.prediction_epoch <= self.person_cloud_timeout

    def _safe_patch_command_is_valid(
        self,
        cmd: Twist,
    ) -> bool:
        if self.planner_state != "LOCAL_SAFE_SPACE" or self.planner_state_time is None:
            return False
        state_age = (self.get_clock().now() - self.planner_state_time).nanoseconds * 1e-9
        if self.safe_patch is None or self.safe_patch_time is None:
            return False
        patch_age = (self.get_clock().now() - self.safe_patch_time).nanoseconds * 1e-9
        if state_age < 0.0 or state_age > 0.5 or patch_age < 0.0 or patch_age > 0.5 or not self._scan_fresh():
            return False
        vx = float(cmd.linear.x)
        vy = float(cmd.linear.y)
        wz = float(cmd.angular.z)
        if math.hypot(vx, vy) + abs(wz) < self.min_input_speed_for_avoidance:
            return False
        if self._escape_hits_static(vx, vy, wz):
            return False
        robot_pose = self._robot_pose()
        if robot_pose is None:
            return False
        dx = self.safe_patch[0] - robot_pose[0]
        dy = self.safe_patch[1] - robot_pose[1]
        local_patch_x = math.cos(robot_pose[2]) * dx + math.sin(robot_pose[2]) * dy
        local_patch_y = -math.sin(robot_pose[2]) * dx + math.cos(robot_pose[2]) * dy
        return trajectory_makes_target_progress(
            vx,
            vy,
            wz,
            local_patch_x,
            local_patch_y,
            self.escape_search_horizon,
            self.escape_search_dt,
        )

    def _mujoco_people_callback(self, msg: String) -> None:
        # The typed path checks the actual executable trajectory each tick.
        # An additional timeless radius stop would veto every safe passing arc.
        if self._typed_safety_fresh():
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        people = payload.get("people", []) if isinstance(payload, dict) else []
        points = []
        for item in people:
            try:
                points.append((float(item.get("x", 0.0)), float(item.get("y", 0.0)), float(item.get("radius_m", 0.42))))
            except (TypeError, ValueError):
                continue
        if not points:
            self.people_forces.pop("mujoco", None)
            return
        if (self.get_clock().now() - self.started_at).nanoseconds * 1e-9 < self.startup_grace_time:
            return
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.robot_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(
                f"No transform {self.target_frame} <- {self.robot_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        yaw = self._yaw_from_quat(transform.transform.rotation)
        trajectory = self._planned_trajectory()
        risk_points = []
        local_risk_points = []
        force_candidates = []
        closest = math.inf
        closest_corridor = math.inf
        source_range = self.people_repulsion_range
        for x, y, radius in points:
            dx = x - robot_x
            dy = y - robot_y
            center_distance = math.hypot(dx, dy)
            clearance = max(0.0, center_distance - radius)
            if center_distance < self.robot_self_filter_radius:
                continue
            if clearance < closest:
                closest = clearance
                self.last_risk_point = ("mujoco", x, y, clearance)
            local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
            local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            corridor_distance = max(0.0, self._distance_to_trajectory(local_x, local_y, trajectory) - radius)
            closest_corridor = min(closest_corridor, corridor_distance)
            if clearance <= source_range:
                strength = ((source_range - clearance) / max(1e-3, source_range)) ** 2
                force_candidates.append((clearance, -local_x / max(center_distance, 1e-3) * strength, -local_y / max(center_distance, 1e-3) * strength))
            if clearance > self.local_risk_radius:
                continue
            if clearance <= self.danger_radius:
                risk_points.append((dx, dy, clearance, clearance, "radius"))
                local_risk_points.append((local_x, local_y, "radius"))
            elif corridor_distance <= self.corridor_width:
                risk_points.append((dx, dy, clearance, corridor_distance, "corridor"))
                local_risk_points.append((local_x, local_y, "corridor"))

        now = self.get_clock().now()
        self.person_cloud_times["mujoco"] = now
        if force_candidates:
            force_candidates.sort(key=lambda item: item[0])
            selected = force_candidates[: max(1, self.max_repulsion_points)]
            self.people_forces["mujoco"] = (
                sum(item[1] for item in selected) / len(selected),
                sum(item[2] for item in selected) / len(selected),
                now,
            )
        else:
            self.people_forces.pop("mujoco", None)
        self.closest_risk = closest
        if risk_points:
            self.last_risk_source = "mujoco"
        elif now >= self.avoid_until:
            self.closest_risk = math.inf
            return
        if not risk_points:
            return

        nearest = min(point[2] for point in risk_points)
        nearest_corridor = min(point[3] for point in risk_points)
        has_radius_risk = any(point[4] == "radius" for point in risk_points)
        radius_urgency = max(0.0, min(1.0, (self.danger_radius - nearest) / max(1e-3, self.danger_radius - self.critical_radius)))
        corridor_urgency = max(0.0, min(1.0, (self.corridor_width - nearest_corridor) / max(1e-3, self.corridor_width - self.corridor_critical_width)))
        urgency = max(radius_urgency, corridor_urgency)
        cmd = Twist()
        if not self._input_stale():
            cmd.linear.x = float(self.latest_cmd.linear.x)
            cmd.linear.y = float(self.latest_cmd.linear.y)
            cmd.angular.z = float(self.latest_cmd.angular.z)
        input_active = self._input_active()
        if not input_active:
            self.avoid_cmd = Twist()
            self.avoid_until = now
            self.last_avoid_reason = "mujoco_inactive_input_observe_only"
            return
        if self.stop_on_human_risk:
            if self._safe_patch_command_is_valid(cmd):
                self.avoid_cmd = cmd
                self.last_avoid_reason = "mujoco_validated_safe_patch"
            elif nearest <= self.emergency_stop_radius:
                self.avoid_cmd = self._twist_from_velocity(stop_velocity())
                self.last_avoid_reason = "mujoco_emergency_stop_radius"
            else:
                self.avoid_cmd = self._twist_from_velocity(stop_velocity())
                self.last_avoid_reason = "mujoco_trajectory_risk_stop"
        else:
            self.avoid_cmd = self._best_escape_cmd(cmd, local_risk_points, urgency, has_radius_risk)
            self.last_avoid_reason = "mujoco_trajectory_risk"
        self.avoid_until = now + Duration(seconds=self.hold_time)

    def _planned_trajectory(self) -> list[tuple[float, float]]:
        cmd = self.latest_cmd
        speed = math.hypot(float(cmd.linear.x), float(cmd.linear.y))
        horizon = max(0.0, self.trajectory_horizon)
        dt = max(0.05, self.trajectory_dt)
        steps = max(1, int(horizon / dt))
        x = 0.0
        y = 0.0
        heading = 0.0
        points = [(x, y)]
        for _ in range(steps):
            vx = float(cmd.linear.x)
            vy = float(cmd.linear.y)
            x += (math.cos(heading) * vx - math.sin(heading) * vy) * dt
            y += (math.sin(heading) * vx + math.cos(heading) * vy) * dt
            heading += float(cmd.angular.z) * dt
            points.append((x, y))

        length = self._trajectory_length(points)
        if length >= self.corridor_min_length or speed < 1e-3:
            return points
        end_x = math.cos(heading) * self.corridor_min_length
        end_y = math.sin(heading) * self.corridor_min_length
        return [(0.0, 0.0), (end_x, end_y)]

    @staticmethod
    def _trajectory_length(points: list[tuple[float, float]]) -> float:
        return sum(
            math.hypot(bx - ax, by - ay)
            for (ax, ay), (bx, by) in zip(points, points[1:])
        )

    @staticmethod
    def _distance_to_trajectory(
        x: float, y: float, points: list[tuple[float, float]]
    ) -> float:
        if len(points) < 2:
            return math.hypot(x, y)
        closest = math.inf
        for (ax, ay), (bx, by) in zip(points, points[1:]):
            abx = bx - ax
            aby = by - ay
            length_sq = abx * abx + aby * aby
            if length_sq <= 1e-9:
                distance = math.hypot(x - ax, y - ay)
            else:
                t = ((x - ax) * abx + (y - ay) * aby) / length_sq
                t = max(0.0, min(1.0, t))
                px = ax + t * abx
                py = ay + t * aby
                distance = math.hypot(x - px, y - py)
            closest = min(closest, distance)
        return closest

    def _best_escape_cmd(
        self,
        base_cmd: Twist,
        risk_points: list[tuple[float, float, str]],
        urgency: float,
        has_radius_risk: bool,
    ) -> Twist:
        if not risk_points:
            return base_cmd
        if not self._scan_fresh():
            return Twist()

        base_x = float(base_cmd.linear.x)
        base_y = float(base_cmd.linear.y)
        base_yaw = float(base_cmd.angular.z)
        mean_x = sum(point[0] for point in risk_points) / len(risk_points)
        min_clearance = min(math.hypot(point[0], point[1]) for point in risk_points)
        weighted_y = 0.0
        total_weight = 0.0
        for risk_x, risk_y, _ in risk_points:
            weight = 1.0 / max(0.15, math.hypot(risk_x, risk_y))
            weighted_y += risk_y * weight
            total_weight += weight
        mean_y = weighted_y / max(total_weight, 1e-6)
        now = self.get_clock().now()
        if now < self.avoid_side_until and self.avoid_side != 0.0:
            obstacle_side = self.avoid_side
        else:
            if abs(mean_y) < 0.18 and self.avoid_side != 0.0:
                obstacle_side = self.avoid_side
            else:
                obstacle_side = 1.0 if mean_y >= 0.0 else -1.0
            self.avoid_side = obstacle_side
            self.avoid_side_until = now + Duration(seconds=max(0.6, min(1.4, self.hold_time)))

        angular = min(self.max_avoid_angular, 0.22 + urgency * 0.28)
        away_yaw = -obstacle_side * angular
        front_or_side_risk = mean_x > -0.25

        lateral = min(self.max_lateral_linear, 0.08 + urgency * 0.12)
        away_lateral = -obstacle_side * lateral
        reverse_escape = -min(self.max_reverse_linear, 0.08 + 0.10 * urgency)
        forward_escape = min(self.max_avoid_linear, 0.12 + 0.14 * urgency)
        preferred_x = reverse_escape if front_or_side_risk else forward_escape
        preferred = (preferred_x, away_lateral, away_yaw)

        linear_options = [
            0.0,
            max(forward_escape, self._clamp(base_x, 0.0, self.max_avoid_linear * 0.8)) if not front_or_side_risk else self._clamp(base_x, 0.0, self.max_avoid_linear * 0.6),
            reverse_escape,
        ]

        candidates = []
        candidates.append(preferred)
        candidates.append((0.0, away_lateral, 0.0))
        candidates.append((reverse_escape, away_lateral * 0.5, away_yaw * 0.5))
        for vx in linear_options:
            for vy in (away_lateral, 0.0, 0.5 * away_lateral):
                for wz in (away_yaw, 0.5 * away_yaw, 0.0):
                    candidates.append((vx, self._clamp(base_y + vy, -self.max_lateral_linear, self.max_lateral_linear), wz))
        candidates.append((base_x, base_y, base_yaw))

        best_score = -math.inf
        best = None
        for vx, vy, wz in candidates:
            if self._escape_hits_static(vx, vy, wz):
                continue
            score = self._escape_score(vx, vy, wz, risk_points)
            score += self.escape_progress_weight * max(0.0, vx)
            score += 0.35 * max(0.0, -vy * obstacle_side)
            score += 0.25 * max(0.0, -wz * obstacle_side)
            score -= self.escape_reverse_penalty * max(0.0, -vx)
            if score > best_score:
                best_score = score
                best = (vx, vy, wz)

        if best is None:
            return Twist()
        forced = None
        if has_radius_risk and mean_x > -0.35 and best[0] > reverse_escape * 0.5:
            forced = (reverse_escape, best[1] if abs(best[1]) > 1e-3 else away_lateral, best[2] if abs(best[2]) > 1e-3 else away_yaw)
        elif has_radius_risk and mean_x <= -0.35 and min_clearance < self.emergency_stop_radius and best[0] < forward_escape * 0.5:
            forced = (forward_escape, best[1] if abs(best[1]) > 1e-3 else away_lateral, best[2] if abs(best[2]) > 1e-3 else away_yaw)
        if forced is not None and not self._escape_hits_static(*forced):
            best = forced

        cmd = Twist()
        vx, vy, wz = clamp_velocity(
            best[0], best[1], best[2], self.max_avoid_linear,
            self.max_reverse_linear, self.max_lateral_linear, self.max_avoid_angular,
        )
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        return cmd

    def _scan_fresh(self) -> bool:
        stamp = None if self.wall_force_time is None else self.wall_force_time.nanoseconds * 1e-9
        return scan_is_fresh(stamp, self.get_clock().now().nanoseconds * 1e-9, self.scan_timeout)

    @staticmethod
    def _twist_from_velocity(velocity: tuple[float, float, float]) -> Twist:
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = velocity
        return cmd

    def _escape_hits_static(self, vx: float, vy: float, wz: float) -> bool:
        dt = max(0.05, self.escape_search_dt)
        steps = max(1, int(max(0.2, self.escape_search_horizon) / dt))
        x = y = heading = 0.0
        trajectory = [(x, y, heading)]
        for _ in range(steps):
            x += (math.cos(heading) * vx - math.sin(heading) * vy) * dt
            y += (math.sin(heading) * vx + math.cos(heading) * vy) * dt
            heading += wz * dt
            trajectory.append((x, y, heading))
        return trajectory_static_cost(
            trajectory, self.scan_obstacles, self.static_escape_clearance,
            max(self.static_escape_clearance, self.wall_repulsion_range),
        )[1]

    def _turn_only_escape_cmd(self, risk_points: list[tuple[float, float, str]], urgency: float) -> Twist:
        weighted_y = 0.0
        total_weight = 0.0
        for risk_x, risk_y, _ in risk_points:
            weight = 1.0 / max(0.15, math.hypot(risk_x, risk_y))
            weighted_y += risk_y * weight
            total_weight += weight
        mean_y = weighted_y / max(total_weight, 1e-6)
        now = self.get_clock().now()
        if now < self.avoid_side_until and self.avoid_side != 0.0:
            obstacle_side = self.avoid_side
        else:
            if abs(mean_y) < 0.18 and self.avoid_side != 0.0:
                obstacle_side = self.avoid_side
            else:
                obstacle_side = 1.0 if mean_y >= 0.0 else -1.0
            self.avoid_side = obstacle_side
            self.avoid_side_until = now + Duration(seconds=max(0.6, min(1.4, self.hold_time)))

        cmd = Twist()
        cmd.angular.z = -obstacle_side * min(self.max_avoid_angular, 0.16 + 0.16 * urgency)
        return cmd

    def _escape_score(self, vx: float, vy: float, wz: float, risk_points: list[tuple[float, float, str]]) -> float:
        horizon = max(0.2, self.escape_search_horizon)
        dt = max(0.05, self.escape_search_dt)
        steps = max(1, int(horizon / dt))
        x = 0.0
        y = 0.0
        heading = 0.0
        min_clearance = math.inf
        radius_clearance = math.inf
        for _ in range(steps):
            x += (math.cos(heading) * vx - math.sin(heading) * vy) * dt
            y += (math.sin(heading) * vx + math.cos(heading) * vy) * dt
            heading += wz * dt
            for risk_x, risk_y, risk_type in risk_points:
                clearance = math.hypot(x - risk_x, y - risk_y)
                min_clearance = min(min_clearance, clearance)
                if risk_type == "radius":
                    radius_clearance = min(radius_clearance, clearance)
        if not math.isfinite(radius_clearance):
            radius_clearance = min_clearance
        return min_clearance + 0.35 * radius_clearance

    def _timer_callback(self) -> None:
        started = time.perf_counter()
        self.supervisor_tick_interval_ms = None if self.supervisor_tick_started is None else (started-self.supervisor_tick_started)*1000.
        self.supervisor_tick_started = started
        self.supervisor_search_ms = 0.
        self.async_escape_state = 'idle'
        # Drain finished work even when the nominal command is safe, so a
        # completed stale job cannot occupy the single-flight mailbox until the
        # next encounter. Proposals still require fresh validation before use.
        self._collect_escape_result()
        now = self.get_clock().now()
        self._expire_stale_people_forces(now)
        pose = self._robot_pose()
        if pose is None:
            cmd = Twist()
            self.pub.publish(cmd)
            self._publish_debug("missing_tf_stop", self.latest_cmd, cmd, now)
            return
        if now < self.avoid_until:
            cmd, reason = self._validate_final_command(self.avoid_cmd, pose, now)
            self.pub.publish(cmd)
            self._publish_debug(reason or "override", self.latest_cmd, cmd, now)
            self.get_logger().warn(
                "Human collision risk in trajectory corridor: "
                f"overriding cmd_vel, closest predicted point {self.closest_risk:.2f}m "
                f"source={self.last_risk_source} point={self.last_risk_point} "
                f"cmd=({self.avoid_cmd.linear.x:+.2f}, {self.avoid_cmd.linear.y:+.2f}, {self.avoid_cmd.angular.z:+.2f})",
                throttle_duration_sec=1.0,
            )
            return
        if self.hold_start_location:
            cmd = self._home_hold_cmd()
            self.pub.publish(cmd)
            self._publish_debug("hold_home", self.latest_cmd, cmd, now)
            return
        if self._input_stale():
            cmd = Twist()
            self.pub.publish(cmd)
            self._publish_debug("stale_input", self.latest_cmd, cmd, now)
            return
        if not self._input_active():
            cmd = Twist()
            if self._sequence_parameters((0.,0.,0.),now.nanoseconds*1e-9) is not None:
                cmd, reason = self._validate_final_command(cmd,pose,now)
                self.pub.publish(cmd)
                self._publish_debug(reason or 'planned_sequence',self.latest_cmd,cmd,now)
                return
            if (self.predictive_escape_search and self.planner_state in ("EMERGENCY_STOP", "CLEARANCE_RECOVERY") and
                    self.planner_state_time is not None and
                    0. <= (now-self.planner_state_time).nanoseconds*1e-9 <= self.input_timeout):
                cmd, reason = self._validate_final_command(cmd, pose, now)
                self.pub.publish(cmd)
                self._publish_debug(reason or "inactive_input_hold", self.latest_cmd, cmd, now)
                return
            self.pub.publish(cmd)
            self._publish_debug("inactive_input_hold", self.latest_cmd, cmd, now)
            return
        cmd = self._apply_open_space_bias(self.latest_cmd)
        cmd, reason = self._validate_final_command(cmd, pose, now)
        self.pub.publish(cmd)
        mode = reason or ("open_space_bias" if self.enable_open_space_bias else "pass_through")
        self._publish_debug(mode, self.latest_cmd, cmd, now)

    def _publish_debug(self, mode: str, input_cmd: Twist, output_cmd: Twist, now) -> None:
        wall_x, wall_y = self._active_wall_force()
        people_x, people_y = self._active_people_force()
        payload = {
            "async_escape_search": self.async_predictive_escape_search,
            "async_escape_state": self.async_escape_state,
            "async_escape_rejections": self.async_escape_rejections,
            "escape_worker": None if self.escape_worker is None else self.escape_worker.diagnostics(),
            "tick_to_publication_ms": None if self.supervisor_tick_started is None else (time.perf_counter()-self.supervisor_tick_started)*1000.,
            "tick_interval_ms": self.supervisor_tick_interval_ms,
            "predictive_search_ms": None if self.async_predictive_escape_search else self.supervisor_search_ms,
            "escape_control_path_ms": self.supervisor_search_ms,
            "period_s": self.supervisor_period_s,
            "command_envelope_required": self.require_command_envelope,
            "command_publisher_id": self.command_gate.publisher,
            "command_sequence_id": self.command_gate.sequence,
            "command_envelope_valid": self.require_command_envelope and not self._input_stale(),
            "command_source_stamp": None if self.latest_cmd_time is None else self.latest_cmd_time.nanoseconds*1e-9,
            "command_rejections": dict(self.command_gate.rejections),
            "mode": mode,
            "reason": self.last_avoid_reason if mode == "override" else mode,
            "input_cmd": self._twist_dict(input_cmd),
            "output_cmd": self._twist_dict(output_cmd),
            "closest_risk": round(self.closest_risk, 3) if math.isfinite(self.closest_risk) else None,
            "risk_source": self.last_risk_source,
            "risk_point": self._risk_point_dict(),
            "wall_force": {"x": round(wall_x, 4), "y": round(wall_y, 4)},
            "people_force": {"x": round(people_x, 4), "y": round(people_y, 4)},
            "active_people_sources": sorted(self.people_forces.keys()),
            "input_age_s": self._age_seconds(self.latest_cmd_time, now),
            "planner_state": self.planner_state,
            "planner_state_age_s": self._age_seconds(self.planner_state_time, now),
            "safe_patch": (
                None if self.safe_patch is None
                else {"x": round(self.safe_patch[0], 3), "y": round(self.safe_patch[1], 3)}
            ),
            "safe_patch_age_s": self._age_seconds(self.safe_patch_time, now),
            "scan_fresh": self._scan_fresh(),
            "person_cloud_ages_s": {
                source: self._age_seconds(stamp, now) for source, stamp in self.person_cloud_times.items()
            },
            "avoid_until_remaining_s": max(0.0, round((self.avoid_until - now).nanoseconds * 1e-9, 3)),
            "avoid_side": self.avoid_side,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.debug_pub.publish(msg)
        if self.visual_pub.get_subscription_count() and time.monotonic()-self.last_visual_publish >= .1:
            self.last_visual_publish = time.monotonic()
            self.visual_pub.publish(self._command_markers(mode, output_cmd))

    def _command_markers(self, mode, command):
        array = MarkerArray()
        arrow = Marker()
        arrow.header.frame_id = self.robot_frame
        arrow.ns, arrow.id = "safety_output", 0
        arrow.frame_locked = True
        arrow.pose.orientation.w = 1.
        arrow.lifetime = Duration(seconds=.6).to_msg()
        speed = math.hypot(command.linear.x, command.linear.y)
        if speed < .01:
            arrow.type = Marker.CYLINDER
            arrow.pose.position.z = .05
            arrow.scale.x = arrow.scale.y = .40
            arrow.scale.z = .04
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1., .25, .1, .7
        else:
            arrow.type = Marker.ARROW
            arrow.scale.x, arrow.scale.y, arrow.scale.z = .045, .10, .12
            arrow.points = [Point(x=0., y=0., z=.2), Point(x=command.linear.x*2., y=command.linear.y*2., z=.2)]
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = .05, .9, 1., .95
        array.markers.append(arrow)
        label = Marker()
        label.header.frame_id = self.robot_frame
        label.ns, label.id = "safety_output", 1
        label.type = Marker.TEXT_VIEW_FACING
        label.frame_locked = True
        label.pose.orientation.w = 1.
        label.pose.position.z = .9
        label.pose.position.y = -.55
        label.scale.z = .12
        label.color.r, label.color.g, label.color.b, label.color.a = .2, 1., 1., 1.
        label.text = f"Safety:{mode}\nvx:{command.linear.x:+.2f},vy:{command.linear.y:+.2f}m/s\nw:{command.angular.z:+.2f}rad/s"
        label.lifetime = Duration(seconds=.6).to_msg()
        array.markers.append(label)
        return array

    @staticmethod
    def _twist_dict(msg: Twist) -> dict[str, float]:
        return {
            "x": round(float(msg.linear.x), 4),
            "y": round(float(msg.linear.y), 4),
            "z": round(float(msg.angular.z), 4),
        }

    @staticmethod
    def _age_seconds(stamp, now) -> float | None:
        if stamp is None:
            return None
        return round((now - stamp).nanoseconds * 1e-9, 3)

    def _risk_point_dict(self) -> dict[str, float | str] | None:
        if self.last_risk_point is None:
            return None
        source, x, y, distance = self.last_risk_point
        return {"source": source, "x": round(x, 3), "y": round(y, 3), "distance": round(distance, 3)}

    def _expire_stale_people_forces(self, now) -> None:
        for source, stamp in list(self.person_cloud_times.items()):
            age = (now - stamp).nanoseconds * 1e-9
            if age > self.person_cloud_timeout:
                self.people_forces.pop(source, None)
                del self.person_cloud_times[source]
        if not self.person_cloud_times and now >= self.avoid_until:
            self.closest_risk = math.inf
            self.last_risk_source = "none"
            self.last_risk_point = None

    def _apply_open_space_bias(self, base_cmd: Twist) -> Twist:
        cmd = Twist()
        cmd.linear.x = float(base_cmd.linear.x)
        cmd.linear.y = float(base_cmd.linear.y)
        cmd.linear.z = float(base_cmd.linear.z)
        cmd.angular.x = float(base_cmd.angular.x)
        cmd.angular.y = float(base_cmd.angular.y)
        cmd.angular.z = float(base_cmd.angular.z)
        if not self.enable_open_space_bias:
            return cmd

        wall_x, wall_y = self._active_wall_force()
        people_x, people_y = self._active_people_force()
        wall_x = self._clamp(wall_x * self.wall_repulsion_gain, -self.wall_repulsion_max_linear, self.wall_repulsion_max_linear)
        wall_y = self._clamp(wall_y * self.wall_repulsion_gain, -self.wall_repulsion_max_linear, self.wall_repulsion_max_linear)
        people_x = self._clamp(people_x * self.people_repulsion_gain, -self.people_repulsion_max_linear, self.people_repulsion_max_linear)
        people_y = self._clamp(people_y * self.people_repulsion_gain, -self.people_repulsion_max_linear, self.people_repulsion_max_linear)

        cmd.linear.x += wall_x + people_x
        cmd.linear.y += wall_y + people_y
        turn_bias = wall_y * self.wall_repulsion_max_angular + people_y * self.people_repulsion_max_angular
        cmd.angular.z += turn_bias
        cmd.linear.x, cmd.linear.y, cmd.angular.z = clamp_velocity(
            float(cmd.linear.x), float(cmd.linear.y), float(cmd.angular.z),
            self.max_avoid_linear, self.max_reverse_linear,
            self.max_lateral_linear, self.max_avoid_angular,
        )
        return cmd

    def _active_wall_force(self) -> tuple[float, float]:
        if self.wall_force_time is None:
            return 0.0, 0.0
        age = (self.get_clock().now() - self.wall_force_time).nanoseconds * 1e-9
        if age > self.repulsion_timeout:
            return 0.0, 0.0
        return self.wall_force

    def _active_people_force(self) -> tuple[float, float]:
        now = self.get_clock().now()
        force_x = 0.0
        force_y = 0.0
        active = 0
        for source, (source_x, source_y, stamp) in list(self.people_forces.items()):
            age = (now - stamp).nanoseconds * 1e-9
            if age > self.repulsion_timeout:
                del self.people_forces[source]
                continue
            force_x += source_x
            force_y += source_y
            active += 1
        if active <= 0:
            return 0.0, 0.0
        return force_x / active, force_y / active

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def _input_stale(self) -> bool:
        if self.latest_cmd_time is None:
            return True
        now = self.get_clock().now()
        if self.require_command_envelope and (self.mission_state_time is None or
                not 0. <= (now-self.mission_state_time).nanoseconds*1e-9 <= 1.):
            return True
        limit = min(.25,self.input_timeout) if self.require_command_envelope else self.input_timeout
        return not 0. <= (now-self.latest_cmd_time).nanoseconds*1e-9 <= limit

    def _input_active(self) -> bool:
        if self._input_stale():
            return False
        speed = math.hypot(float(self.latest_cmd.linear.x), float(self.latest_cmd.linear.y))
        speed += 0.25 * abs(float(self.latest_cmd.angular.z))
        return speed >= self.min_input_speed_for_avoidance

    def _home_hold_cmd(self) -> Twist:
        pose = self._robot_pose()
        if pose is None:
            return Twist()
        robot_x, robot_y, yaw, tilt = pose
        if tilt > self.return_max_tilt:
            return Twist()
        if self.home_pose is None:
            self.home_pose = (robot_x, robot_y, yaw)
            self.get_logger().info(
                f"Holding start location at x={robot_x:.2f}, y={robot_y:.2f}, yaw={yaw:.2f} unless collision avoidance is needed"
            )
            return Twist()

        home_x, home_y, home_yaw = self.home_pose
        dx = home_x - robot_x
        dy = home_y - robot_y
        distance = math.hypot(dx, dy)
        yaw_error = self._wrap_pi(home_yaw - yaw)
        cmd = Twist()
        if distance > self.return_position_tolerance:
            target_heading = math.atan2(dy, dx)
            heading_error = self._wrap_pi(target_heading - yaw)
            cmd.angular.z = max(-self.return_angular_speed, min(self.return_angular_speed, 1.2 * heading_error))
            if abs(heading_error) < self.return_linear_heading_tolerance:
                alignment = max(0.0, math.cos(heading_error))
                cmd.linear.x = min(self.return_linear_speed, 0.45 * distance) * alignment
        elif abs(yaw_error) > self.return_yaw_tolerance:
            cmd.angular.z = max(-self.return_angular_speed, min(self.return_angular_speed, 1.2 * yaw_error))
        return cmd

    def _robot_pose(self):
        stamped_pose = getattr(self, "odom_pose", None)
        if stamped_pose is not None and self.odom_stamp is not None:
            age = self.get_clock().now().nanoseconds*1e-9 - self.odom_stamp
            if 0. <= age <= self.odom_timeout:
                return (*advance_planar_pose(stamped_pose[:3], self.measured_velocity, age), stamped_pose[3])
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.robot_frame, rclpy.time.Time())
        except TransformException:
            return None
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            self._yaw_from_quat(transform.transform.rotation),
            self._tilt_from_quat(transform.transform.rotation),
        )

    @staticmethod
    def _yaw_from_quat(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _tilt_from_quat(q) -> float:
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
        return max(abs(roll), abs(pitch))

    @staticmethod
    def _wrap_pi(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def destroy_node(self):
        # Request a balanced stop before waiting for any finite background job.
        # This is best-effort publication, not a motor-stop acknowledgement.
        if self.context.ok():
            try:
                self.pub.publish(Twist())
            except Exception as exc:
                self.get_logger().warning(f'Unable to publish shutdown stop: {exc}')
        if self.escape_worker is not None:
            self.escape_worker.shutdown()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = HumanCollisionAvoidance()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
