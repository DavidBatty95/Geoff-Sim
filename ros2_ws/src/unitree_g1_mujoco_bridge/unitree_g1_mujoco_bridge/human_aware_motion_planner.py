import json
import math
import uuid
import time
import numpy as np
from .observed_route import route_is_free, people_clear_corridor
from .forward_reference import forward_reference,loaded_turn_reference,NominalMotionReference
from .command_envelope import goal_signature
from .avoidance_candidates import crossing_escape_directions, velocity_obstacle_command
from dataclasses import dataclass,replace,asdict
from .policy_response import NATIVE_CONTACT_PROFILE,calibrated_turn_reference

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import ColorRGBA, String, Bool
from unitree_hg.msg import LowState
from unitree_social_navigation_msgs.msg import PredictedTrajectories, TrackedPerson, TrackedPersons
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .bridge_safety import (
    being_overtaken,
    blocked_motion_state,
    no_unrequested_forward,
    relative_motion_metrics,
    scan_is_fresh,
    trajectory_static_cost,
    ttc_is_relevant,
)
from .pointcloud import iter_xyz
from .motion_envelope import load_motion_envelope, advance_planar_pose
from .static_distance_field import StaticDistanceField, StaticFieldCache, observed_free_mask
from .kinodynamic_search import remaining_schedule, rollout_schedule, search_sequence


LATEST_SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


@dataclass
class HumanTrack:
    track_id: int
    state: str
    x: float
    y: float
    vx: float
    vy: float
    confidence: float
    person_score: float
    missed_count: int
    cov_xx: float
    cov_yy: float
    stamp: float
    body_radius: float = 0.35
    observation_age: float = 0.0


def align_track_to_time(track: HumanTrack, query_stamp: float) -> HumanTrack:
    """Advance a track state from its observation stamp to a planning epoch."""
    age = max(0.0, query_stamp - track.stamp)
    return HumanTrack(
        track_id=track.track_id,
        state=track.state,
        x=track.x + track.vx * age,
        y=track.y + track.vy * age,
        vx=track.vx,
        vy=track.vy,
        confidence=track.confidence,
        person_score=track.person_score,
        missed_count=track.missed_count,
        cov_xx=track.cov_xx,
        cov_yy=track.cov_yy,
        stamp=query_stamp,
        body_radius=track.body_radius,
        observation_age=track.observation_age + age,
    )


@dataclass(frozen=True)
class PredictionSample:
    time_offset: float
    x: float
    y: float
    vx: float
    vy: float
    cov_xx: float
    cov_yy: float
    confidence: float


def _ros_time_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def interpolate_prediction(
    points: list[PredictionSample], time_offset: float
) -> PredictionSample | None:
    """Interpolate a prediction at an offset from its message epoch."""
    if not points or time_offset < points[0].time_offset or time_offset > points[-1].time_offset:
        return None
    for right_index, right in enumerate(points):
        if time_offset > right.time_offset:
            continue
        if right_index == 0 or time_offset == right.time_offset:
            return right
        left = points[right_index - 1]
        span = right.time_offset - left.time_offset
        if span <= 0.0:
            return None
        fraction = (time_offset - left.time_offset) / span
        return PredictionSample(
            time_offset=time_offset,
            x=left.x + fraction * (right.x - left.x),
            y=left.y + fraction * (right.y - left.y),
            vx=left.vx + fraction * (right.vx - left.vx),
            vy=left.vy + fraction * (right.vy - left.vy),
            cov_xx=left.cov_xx + fraction * (right.cov_xx - left.cov_xx),
            cov_yy=left.cov_yy + fraction * (right.cov_yy - left.cov_yy),
            confidence=left.confidence + fraction * (right.confidence - left.confidence),
        )
    return None


def continuous_prediction(points,time_offset):
    """Continue the typed forecast's endpoint instead of jumping to an old CV path."""
    sample=interpolate_prediction(points,time_offset)
    if sample is not None or not points or time_offset<points[0].time_offset:
        return sample
    last=points[-1]
    elapsed=time_offset-last.time_offset
    return replace(last,time_offset=time_offset,x=last.x+last.vx*elapsed,y=last.y+last.vy*elapsed,
                   cov_xx=last.cov_xx+.04*elapsed**3/3.,cov_yy=last.cov_yy+.04*elapsed**3/3.)


def nearest_clear_patch(
    origin: tuple[float, float],
    danger_points: list[tuple[float, float, float]],
    static_points: list[tuple[float, float]],
    search_radius: float,
    spacing: float,
    clearance: float,
    preferred_heading: float,
) -> tuple[float, float] | None:
    """Return the closest straight-line reachable patch outside all danger regions."""
    ox, oy = origin
    spacing = max(0.1, spacing)
    ring_count = max(1, math.ceil(search_radius / spacing))

    def clear(x: float, y: float) -> bool:
        if any(math.hypot(x - px, y - py) < radius + clearance for px, py, radius in danger_points):
            return False
        return not any(math.hypot(x - px, y - py) < clearance for px, py in static_points)

    def static_clear(x: float, y: float) -> bool:
        return not any(math.hypot(x - px, y - py) < clearance for px, py in static_points)

    for ring in range(1, ring_count + 1):
        radius = min(search_radius, ring * spacing)
        candidates = []
        sample_count = max(16, math.ceil(2.0 * math.pi * radius / spacing))
        for index in range(sample_count):
            angle = 2.0 * math.pi * index / sample_count
            x = ox + radius * math.cos(angle)
            y = oy + radius * math.sin(angle)
            if not clear(x, y):
                continue
            segment_clear = True
            for step in range(1, ring + 1):
                fraction = step / ring
                if not static_clear(ox + (x - ox) * fraction, oy + (y - oy) * fraction):
                    segment_clear = False
                    break
            if segment_clear:
                heading_error = abs(math.atan2(math.sin(angle - preferred_heading), math.cos(angle - preferred_heading)))
                candidates.append((heading_error, x, y))
        if candidates:
            _, x, y = min(candidates)
            return x, y
    return None


def incoming_collision_imminent(
    pose: tuple[float, float, float],
    tracks: list[HumanTrack],
    ttc_limit: float,
    dcpa_limit: float,
    minimum_person_speed: float,
) -> bool:
    robot_x, robot_y, _ = pose
    for track in tracks:
        if track.state not in ("CONFIRMED", "OCCLUDED"):
            continue
        if math.hypot(track.vx, track.vy) < minimum_person_speed:
            continue
        metrics = relative_motion_metrics(
            robot_x,
            robot_y,
            0.0,
            0.0,
            track.x,
            track.y,
            track.vx,
            track.vy,
        )
        if ttc_is_relevant(metrics, ttc_limit, dcpa_limit):
            return True
    return False


@dataclass
class Candidate:
    name: str
    vx: float
    vy: float
    wz: float
    preferred_state: str = ""
    follow_vx: float | None = None
    switch_time: float = 1.2
    follow_vy: float = 0.0
    follow_wz: float = 0.0
    schedule: tuple | None = None


@dataclass
class CandidateResult:
    candidate: Candidate
    score: float
    rejected: bool
    reason: str
    trajectory: list[tuple[float, float, float, float]]
    min_human_distance: float
    min_tcpa: float
    front_cut_cost: float
    score_terms: dict[str, float]


class HumanAwareMotionPlanner(Node):
    """Sampled human-aware velocity planner above the hard safety filter."""

    def __init__(self) -> None:
        super().__init__("human_aware_motion_planner")
        self.declare_parameter("input_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("output_cmd_vel_topic", "/cmd_vel_human_aware")
        self.declare_parameter("tracked_cloud_topic", "/tracked_dynamic_obstacles")
        self.declare_parameter("predicted_cloud_topic", "/predicted_dynamic_obstacles")
        self.declare_parameter("tentative_cloud_topic", "/tentative_dynamic_obstacles")
        self.declare_parameter("debug_tracks_topic", "/dynamic_obstacle_tracker/debug_tracks")
        self.declare_parameter("tracked_persons_topic", "/tracked_persons")
        self.declare_parameter("predicted_trajectories_topic", "/predicted_person_trajectories")
        self.declare_parameter("use_legacy_person_inputs", False)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("rate_hz", 12.0)
        self.declare_parameter("prediction_horizon", 5.0)
        self.declare_parameter("simulation_dt", 0.2)
        self.declare_parameter("input_timeout", 0.6)
        self.declare_parameter("max_person_cloud_age", 0.8)
        self.declare_parameter("max_scan_age", 0.6)
        self.declare_parameter("robot_radius", 0.38)
        self.declare_parameter("human_body_radius", 0.35)
        self.declare_parameter("body_prediction_margin", 0.05)
        self.declare_parameter("prediction_activation_clearance", 0.0)
        self.declare_parameter("hard_safety_margin", 0.40)
        self.declare_parameter("comfort_radius", 1.40)
        self.declare_parameter("front_comfort_scale", 1.7)
        self.declare_parameter("front_arc_risk_enabled", True)
        self.declare_parameter("front_arc_angle_deg", 30.0)
        self.declare_parameter("front_arc_distance", 1.5)
        self.declare_parameter("front_arc_radius_scale", 1.15)
        self.declare_parameter("front_arc_weight", 0.55)
        self.declare_parameter("uncertainty_growth_rate", 0.18)
        self.declare_parameter("low_conf_uncertainty_gain", 0.35)
        self.declare_parameter("occluded_risk_scale", 0.5)
        self.declare_parameter("tentative_caution_radius", 1.0)
        self.declare_parameter("ttc_warning_threshold", 3.0)
        self.declare_parameter("ttc_hard_threshold", 1.0)
        self.declare_parameter("dcpa_comfort_threshold", 1.4)
        self.declare_parameter("crossing_angle_min_deg", 35.0)
        self.declare_parameter("side_commitment_timeout", 2.0)
        self.declare_parameter("stale_track_decay_time", 0.8)
        self.declare_parameter("yield_timeout", 4.0)
        self.declare_parameter("deadlock_timeout", 6.0)
        self.declare_parameter("max_forward_near_human", 0.35)
        self.declare_parameter("max_lateral_near_human", 0.16)
        self.declare_parameter("max_reverse_velocity", 0.10)
        self.declare_parameter("max_angular_near_human", 0.55)
        self.declare_parameter("static_clearance", 0.48)
        self.declare_parameter("static_influence_range", 1.25)
        self.declare_parameter("w_goal", 1.0)
        self.declare_parameter("w_path", 0.8)
        self.declare_parameter("w_static", 8.0)
        self.declare_parameter("w_personal_space", 12.0)
        self.declare_parameter("w_ttc", 10.0)
        self.declare_parameter("w_pass_front", 18.0)
        self.declare_parameter("w_yield", 8.0)
        self.declare_parameter("w_smooth", 4.0)
        self.declare_parameter("w_jerk", 3.0)
        self.declare_parameter("w_legibility", 6.0)
        self.declare_parameter("w_deadlock", 4.0)
        self.declare_parameter("w_enclosure", 0.0)
        self.declare_parameter("publish_risk_grid", True)
        self.declare_parameter("risk_grid_size", 8.0)
        self.declare_parameter("risk_grid_resolution", 0.20)
        self.declare_parameter("local_safe_space_enabled", False)
        self.declare_parameter("response_delay", 0.10)
        self.declare_parameter("command_duration", 1.7)
        self.declare_parameter("hard_collision_horizon", 6.0)
        self.declare_parameter("social_cost_decay", 0.35)
        self.declare_parameter("odom_timeout", 0.4)
        self.declare_parameter("require_fresh_tracks", True)
        self.declare_parameter("visualization_rate_hz", 2.0)
        self.declare_parameter("holonomic_avoidance", False)
        self.declare_parameter("prefer_forward_travel", False)
        self.declare_parameter('prefer_safe_route_alignment',False)
        self.prefer_safe_route_alignment = bool(self.get_parameter('prefer_safe_route_alignment').value)
        self.declare_parameter("max_clockwise_angular_near_human", 0.0)
        self.declare_parameter("static_memory_seconds", 10.0)
        self.declare_parameter("use_low_obstacle_cloud", False)
        self.declare_parameter("use_static_scene_map", False)
        self.declare_parameter("nav2_path_reference", False)
        self.declare_parameter("retain_observed_routes", False)
        self.declare_parameter("planning_budget_ms", 0.0)
        self.planning_budget_ms = float(self.get_parameter("planning_budget_ms").value)
        self.declare_parameter("overtaking_terminal_horizon", 0.0)
        self.overtaking_terminal_horizon = max(0.,float(self.get_parameter("overtaking_terminal_horizon").value))
        self.declare_parameter('priority_candidate_limit',0)
        self.priority_candidate_limit = max(0,int(self.get_parameter('priority_candidate_limit').value))
        self.priority_search_phase = 0
        self.search_phase = 0
        self.escape_plan = None
        self.warm_start_plan = None
        self.search_budget_exhausted = False
        self.observed_map = None
        self.observed_map_time = None
        self.observed_route_valid = None
        self.declare_parameter("local_escape_enabled", False)
        self.declare_parameter("path_reference_speed", 0.35)
        self.declare_parameter("path_reference_lookahead", 0.8)
        self.declare_parameter("path_reference_holonomic_distance", 1.0)
        self.declare_parameter("stationary_person_clearance", 0.0)
        self.declare_parameter("stationary_clearance_weight", 20.0)
        self.declare_parameter("human_intent_min_speed", 0.08)
        self.declare_parameter("selection_hysteresis", 0.0)
        self.declare_parameter("holonomic_heading_alignment", False)
        self.declare_parameter("goal_urgency_max", 1.0)
        self.declare_parameter("goal_progress_timeout", 6.0)
        self.declare_parameter("body_uncertainty_scale", 0.0)
        self.declare_parameter("body_uncertainty_horizon", 1.0)
        self.declare_parameter("kinodynamic_search_enabled", False)
        self.declare_parameter("kinodynamic_budget_ms", 30.0)
        self.declare_parameter('strategic_avoidance_enabled',False)
        self.declare_parameter('record_invalidation_audit',False)
        self.declare_parameter('forecast_change_guard_enabled',False)
        self.declare_parameter('heading_history_enabled',False)
        self.declare_parameter("continuation_prediction_margin", 0.30)
        self.declare_parameter("translation_refinement_enabled", False)
        self.declare_parameter("goal_capture_enabled", False)
        self.declare_parameter("wait_candidates_enabled", False)
        self.declare_parameter("goal_cost_radius", 0.0)
        self.declare_parameter("local_safe_radius", 1.35)
        self.declare_parameter("local_danger_radius", 4.0)
        self.declare_parameter("predicted_danger_radius", 1.25)
        self.declare_parameter("safe_space_max_speed", 0.22)
        self.declare_parameter("safe_space_max_lateral", 0.18)
        self.declare_parameter("safe_space_max_yaw", 0.45)
        self.declare_parameter("safe_space_max_tilt", 0.45)
        self.declare_parameter("safe_patch_search_radius", 3.0)
        self.declare_parameter("safe_patch_spacing", 0.30)
        self.declare_parameter("safe_patch_clearance", 0.55)
        self.declare_parameter("collision_reflex_ttc", 6.0)
        self.declare_parameter("collision_reflex_dcpa", 1.10)
        self.declare_parameter("collision_reflex_min_person_speed", 0.12)

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.horizon = float(self.get_parameter("prediction_horizon").value)
        self.dt = float(self.get_parameter("simulation_dt").value)
        self.input_timeout = float(self.get_parameter("input_timeout").value)
        self.max_person_cloud_age = float(self.get_parameter("max_person_cloud_age").value)
        self.max_scan_age = float(self.get_parameter("max_scan_age").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.human_body_radius = float(self.get_parameter("human_body_radius").value)
        self.body_prediction_margin = float(self.get_parameter("body_prediction_margin").value)
        self.prediction_activation_clearance = float(self.get_parameter("prediction_activation_clearance").value)
        self.hard_safety_margin = float(self.get_parameter("hard_safety_margin").value)
        self.comfort_radius = float(self.get_parameter("comfort_radius").value)
        self.front_comfort_scale = float(self.get_parameter("front_comfort_scale").value)
        self.front_arc_risk_enabled = bool(self.get_parameter("front_arc_risk_enabled").value)
        self.front_arc_angle = math.radians(float(self.get_parameter("front_arc_angle_deg").value))
        self.front_arc_distance = float(self.get_parameter("front_arc_distance").value)
        self.front_arc_radius_scale = float(self.get_parameter("front_arc_radius_scale").value)
        self.front_arc_weight = float(self.get_parameter("front_arc_weight").value)
        self.uncertainty_growth_rate = float(self.get_parameter("uncertainty_growth_rate").value)
        self.low_conf_uncertainty_gain = float(self.get_parameter("low_conf_uncertainty_gain").value)
        self.occluded_risk_scale = float(self.get_parameter("occluded_risk_scale").value)
        self.tentative_caution_radius = float(self.get_parameter("tentative_caution_radius").value)
        self.ttc_warning_threshold = float(self.get_parameter("ttc_warning_threshold").value)
        self.ttc_hard_threshold = float(self.get_parameter("ttc_hard_threshold").value)
        self.dcpa_comfort_threshold = float(self.get_parameter("dcpa_comfort_threshold").value)
        self.crossing_angle_min = math.radians(float(self.get_parameter("crossing_angle_min_deg").value))
        self.side_commitment_timeout = float(self.get_parameter("side_commitment_timeout").value)
        self.stale_track_decay_time = float(self.get_parameter("stale_track_decay_time").value)
        self.yield_timeout = float(self.get_parameter("yield_timeout").value)
        self.deadlock_timeout = float(self.get_parameter("deadlock_timeout").value)
        self.max_forward_near_human = float(self.get_parameter("max_forward_near_human").value)
        self.max_lateral_near_human = float(self.get_parameter("max_lateral_near_human").value)
        self.max_reverse_velocity = float(self.get_parameter("max_reverse_velocity").value)
        self.max_angular_near_human = float(self.get_parameter("max_angular_near_human").value)
        self.static_clearance = float(self.get_parameter("static_clearance").value)
        self.navigation_static_clearance = self.static_clearance
        self.navigation_mode = 'navigate'
        self.payload_active = False
        self.static_influence_range = float(self.get_parameter("static_influence_range").value)
        self.weights = {
            name: float(self.get_parameter(name).value)
            for name in (
                "w_goal",
                "w_path",
                "w_static",
                "w_personal_space",
                "w_ttc",
                "w_pass_front",
                "w_yield",
                "w_smooth",
                "w_jerk",
                "w_legibility",
                "w_deadlock",
                "w_enclosure",
            )
        }
        self.publish_risk_grid = bool(self.get_parameter("publish_risk_grid").value)
        self.risk_grid_size = float(self.get_parameter("risk_grid_size").value)
        self.risk_grid_resolution = float(self.get_parameter("risk_grid_resolution").value)
        self.local_safe_space_enabled = bool(self.get_parameter("local_safe_space_enabled").value)
        self.local_safe_radius = float(self.get_parameter("local_safe_radius").value)
        self.local_danger_radius = float(self.get_parameter("local_danger_radius").value)
        self.predicted_danger_radius = float(self.get_parameter("predicted_danger_radius").value)
        self.safe_space_max_speed = float(self.get_parameter("safe_space_max_speed").value)
        self.safe_space_max_lateral = float(self.get_parameter("safe_space_max_lateral").value)
        self.safe_space_max_yaw = float(self.get_parameter("safe_space_max_yaw").value)
        self.safe_space_max_tilt = float(self.get_parameter("safe_space_max_tilt").value)
        self.safe_patch_search_radius = float(self.get_parameter("safe_patch_search_radius").value)
        self.safe_patch_spacing = float(self.get_parameter("safe_patch_spacing").value)
        self.safe_patch_clearance = float(self.get_parameter("safe_patch_clearance").value)
        self.collision_reflex_ttc = float(self.get_parameter("collision_reflex_ttc").value)
        self.collision_reflex_dcpa = float(self.get_parameter("collision_reflex_dcpa").value)
        self.collision_reflex_min_person_speed = float(
            self.get_parameter("collision_reflex_min_person_speed").value
        )
        self.use_legacy_person_inputs = bool(self.get_parameter("use_legacy_person_inputs").value)
        self.motion_envelope = load_motion_envelope(self)
        self.response_delay = float(self.get_parameter("response_delay").value)
        self.command_duration = float(self.get_parameter("command_duration").value)
        self.hard_collision_horizon = float(self.get_parameter("hard_collision_horizon").value)
        self.social_cost_decay = float(self.get_parameter("social_cost_decay").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.require_fresh_tracks = bool(self.get_parameter("require_fresh_tracks").value)
        self.visualization_period = 1.0 / max(0.1, float(self.get_parameter("visualization_rate_hz").value))
        self.holonomic_avoidance = bool(self.get_parameter("holonomic_avoidance").value)
        self.prefer_forward_travel = bool(self.get_parameter("prefer_forward_travel").value)
        self.max_clockwise_angular_near_human = float(self.get_parameter("max_clockwise_angular_near_human").value)
        self.static_memory_seconds = float(self.get_parameter("static_memory_seconds").value)
        self.static_memory = {}
        self.low_obstacle_memory = {}
        self.static_scene_points = None
        self.use_static_scene_map = bool(self.get_parameter("use_static_scene_map").value)
        self.static_scan_stamp = None
        self.static_error = ""
        self.nav2_path_reference = bool(self.get_parameter("nav2_path_reference").value)
        self.local_escape_enabled = bool(self.get_parameter("local_escape_enabled").value)
        self.path_reference_speed = float(self.get_parameter("path_reference_speed").value)
        self.path_reference_lookahead = float(self.get_parameter("path_reference_lookahead").value)
        self.path_reference_holonomic_distance = max(0., float(self.get_parameter("path_reference_holonomic_distance").value))
        self.stationary_person_clearance = max(0., float(self.get_parameter("stationary_person_clearance").value))
        self.stationary_clearance_weight = max(0., float(self.get_parameter("stationary_clearance_weight").value))
        self.human_intent_min_speed = max(0., float(self.get_parameter("human_intent_min_speed").value))
        self.selection_hysteresis = max(0., float(self.get_parameter("selection_hysteresis").value))
        self.holonomic_heading_alignment = bool(self.get_parameter("holonomic_heading_alignment").value)
        self.goal_urgency_max = max(1.,float(self.get_parameter("goal_urgency_max").value))
        self.goal_progress_timeout = max(.5,float(self.get_parameter("goal_progress_timeout").value))
        self.body_uncertainty_scale = max(0.,float(self.get_parameter("body_uncertainty_scale").value))
        self.body_uncertainty_horizon = max(0.,float(self.get_parameter("body_uncertainty_horizon").value))
        self.kinodynamic_search_enabled = bool(self.get_parameter("kinodynamic_search_enabled").value)
        self.kinodynamic_budget_ms = max(1.,float(self.get_parameter("kinodynamic_budget_ms").value))
        self.continuation_prediction_margin = max(.1,float(self.get_parameter("continuation_prediction_margin").value))
        self.translation_refinement_enabled = bool(self.get_parameter("translation_refinement_enabled").value)
        self.goal_capture_enabled = bool(self.get_parameter("goal_capture_enabled").value)
        self.wait_candidates_enabled = bool(self.get_parameter("wait_candidates_enabled").value)
        self.goal_cost_radius = max(0.,min(.4,float(self.get_parameter("goal_cost_radius").value)))
        self.kinodynamic_status = 'not_requested'
        self.strategic_avoidance_enabled = bool(self.get_parameter('strategic_avoidance_enabled').value)
        self.strategic_status = dict(active=False)
        self.strategic_goal = None
        self.record_invalidation_audit=bool(self.get_parameter('record_invalidation_audit').value)
        self.forecast_change_guard_enabled=bool(self.get_parameter('forecast_change_guard_enabled').value)
        self.heading_history_enabled=bool(self.get_parameter('heading_history_enabled').value)
        from .heading_history import HeadingHistory
        self.heading_history=HeadingHistory()
        self.heading_history_details={}
        self.source_odom_velocity=(0.,0.,0.)
        from .forecast_change import ForecastChangeReserve
        self.forecast_change_reserve=ForecastChangeReserve()
        self.forecast_source='unavailable'
        self.invalidation_audits_remaining=4
        self.primitive_last_validation=None
        self.primitive_validation_audit=None
        self.primitive_worker = None
        if self.strategic_avoidance_enabled:
            from .escape_primitives import process_worker
            self.primitive_worker=process_worker()
        self.goal_urgency = 1.
        self.goal_best_distance = math.inf
        self.goal_progress_stamp = None
        self.goal_holonomic = False
        self.reference_path = None
        self.reference_goal = None
        self.reference_goal_world = None
        self.reference_progress_target = None
        self.reference_path_time = None
        self.path_reference_status = "not_requested"
        self.mission_active = False
        self.mission_state_time = None
        self.upstream_cmd = Twist()
        self.last_visualization = -math.inf
        self.measured_velocity = (0.0, 0.0, 0.0)
        self.continuous_reference = bool(self.declare_parameter('continuous_motion_reference',False).value)
        self.nominal_reference = NominalMotionReference()
        self.nominal_reference_stamp = None
        self.policy_command = None
        self.policy_command_stamp = None
        self.odom_stamp = None
        self.odom_pose = None
        self.odom_pose_history = {}
        self.planning_started = time.perf_counter()
        self.search_budget_exhausted = False
        self.planning_cpu_started = time.thread_time()
        self.planning_phases = {}
        self.prediction_cache = {}
        self.scoring_phases = {}
        self.command_publisher_id = uuid.uuid4().hex
        self.command_sequence_id = 0
        self.static_cache = {}
        self.static_points_cache = None
        self.static_field_cache = None
        self.static_frame_valid = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_cmd = Twist()
        self.latest_cmd_time = None
        self.latest_scan: LaserScan | None = None
        self.latest_scan_time = None
        self.tracks: dict[int, HumanTrack] = {}
        self.tracks_epoch: float | None = None
        self.legacy_tracks: dict[int, HumanTrack] = {}
        self.predicted_points: list[tuple[float, float, float]] = []
        self.predicted_time = None
        self.predictions: dict[int, list[PredictionSample]] = {}
        self.prediction_epoch: float | None = None
        self.tentative_points: list[tuple[float, float, float]] = []
        self.tentative_time = None
        self.state = "NORMAL_NAVIGATION"
        self.state_since = self.get_clock().now()
        self.side_commitment = 0.0
        self.side_commitment_until = self.get_clock().now()
        self.last_cmd = Twist()
        self.last_tilt = 0.0
        self.last_progress_time = self.get_clock().now()
        self.last_debug = {}
        self.selected_safe_patch: tuple[float, float] | None = None

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("output_cmd_vel_topic").value), 1)
        self.state_pub = self.create_publisher(String, "/human_avoidance_state", 1)
        self.debug_pub = self.create_publisher(String, "/human_avoidance_debug", 10)
        self.sequence_pub = self.create_publisher(String, "/human_avoidance_command_sequence", 1)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/human_social_risk_grid", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/human_avoidance_markers", 10)
        self.safe_patch_pub = self.create_publisher(PoseStamped, "/human_avoidance_safe_patch", 10)
        self.create_subscription(Twist, str(self.get_parameter("input_cmd_vel_topic").value), self._cmd_callback, 1)
        self.create_subscription(
            TrackedPersons, str(self.get_parameter("tracked_persons_topic").value), self._tracked_persons_callback, 1
        )
        self.create_subscription(
            PredictedTrajectories,
            str(self.get_parameter("predicted_trajectories_topic").value),
            self._predicted_trajectories_callback,
            1,
        )
        if self.use_legacy_person_inputs:
            self.create_subscription(String, str(self.get_parameter("debug_tracks_topic").value), self._debug_tracks_callback, 10)
            self.create_subscription(PointCloud2, str(self.get_parameter("predicted_cloud_topic").value), self._predicted_callback, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, str(self.get_parameter("tentative_cloud_topic").value), self._tentative_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value), self._scan_callback, LATEST_SENSOR_QOS)
        self.create_subscription(LowState, "/lowstate", self._lowstate_callback, LATEST_SENSOR_QOS)
        self.create_subscription(Odometry, "/odom", self._odom_callback, LATEST_SENSOR_QOS)
        if bool(self.get_parameter("use_low_obstacle_cloud").value):
            self.create_subscription(PointCloud2, "/livox/lidar_world", self._low_obstacle_callback, qos_profile_sensor_data)
        if self.use_static_scene_map:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, "/simulation/static_map", self._scene_map_callback, qos)
        self.create_subscription(TwistStamped, "/g1/policy_cmd_vel", self._policy_command_callback, LATEST_SENSOR_QOS)
        self.create_subscription(String,'/navigation/goal_mode',self._mode_callback,1)
        self.create_subscription(Bool,'/navigation/carrying',self._payload_callback,1)
        if self.nav2_path_reference:
            if bool(self.get_parameter("retain_observed_routes").value):
                qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
                self.create_subscription(OccupancyGrid, "/map", self._observed_map_callback, qos)
            self.create_subscription(Path, "/plan", self._path_callback, 1)
            self.create_subscription(PoseStamped, "/benchmark/current_goal", self._goal_callback, 1)
            self.create_subscription(Bool, "/benchmark/goal_active", self._mission_callback, 1)
        self.planner_period_s = 1.0 / max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(self.planner_period_s, self._timer_callback)
        self.get_logger().info("Human-aware sampled motion planner ready")

    def _cmd_callback(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.upstream_cmd = msg
        self.latest_cmd_time = self.get_clock().now()

    def _path_callback(self, msg: Path) -> None:
        if not msg.poses:
            return
        if not all(math.isfinite(p.pose.position.x) and math.isfinite(p.pose.position.y) for p in msg.poses):
            return
        goal = self.reference_goal
        endpoint = msg.poses[-1].pose.position
        if goal is not None and (msg.header.frame_id != goal.header.frame_id or
                math.hypot(endpoint.x - goal.pose.position.x, endpoint.y - goal.pose.position.y) > 0.30):
            return
        if (self.reference_path is not None and msg.header.frame_id == self.reference_path.header.frame_id
                and _ros_time_seconds(msg.header.stamp) < _ros_time_seconds(self.reference_path.header.stamp)):
            return  # A late result must not replace a newer route to the same goal.
        self.reference_path = msg
        self.observed_route_valid = None
        self.reference_path_time = self.get_clock().now()

    def _observed_map_callback(self, msg):
        self.observed_map = msg
        self.observed_map_time = self.get_clock().now()
        self.observed_route_valid = None

    def _retained_route_observed(self, path):
        if getattr(self, "observed_map_time", None) is None or (self.get_clock().now() - self.observed_map_time).nanoseconds * 1e-9 > 2.5:
            return False
        if self.observed_route_valid is None:
            self.observed_route_valid = route_is_free(self.observed_map, path, self.robot_radius)
        return self.observed_route_valid

    def destroy_node(self):
        worker=getattr(self,'primitive_worker',None)
        if worker is not None:
            worker.shutdown()
        return super().destroy_node()

    def _mission_callback(self, msg: Bool) -> None:
        was_active = self.mission_active
        self.mission_active = bool(msg.data)
        if not self.mission_active:
            if hasattr(self,'nominal_reference'):
                self.nominal_reference.reset()
                self.nominal_reference_stamp = None
            if was_active:
                self.escape_plan = None
                self.warm_start_plan = None
            self.goal_progress_stamp = None
            self.goal_best_distance = math.inf
            self.goal_urgency = 1.
            self.goal_holonomic = False
        self.mission_state_time = self.get_clock().now()

    def _mode_callback(self,msg):
        mode = msg.data if msg.data in ('navigate','dock','scan','manipulate','brake','reserve_recovery') else 'navigate'
        if mode=='brake' and self.navigation_mode!='brake':
            self.braking_reference = [self.last_cmd.linear.x,self.last_cmd.linear.y,self.last_cmd.angular.z]
            self.braking_stamp = self.get_clock().now().nanoseconds*1e-9
        self.navigation_mode = mode
        self.static_clearance = .26 if self.navigation_mode in ('dock','scan','manipulate') else self.navigation_static_clearance+(.16 if self.payload_active else 0.)

    def _payload_callback(self,msg):
        if bool(msg.data) != self.payload_active:
            self.robot_radius += .16 if msg.data else -.16
            self.payload_active = bool(msg.data)
        self.static_clearance = .26 if self.navigation_mode in ('dock','scan','manipulate') else self.navigation_static_clearance+(.16 if self.payload_active else 0.)
        self.motion_envelope = replace(self.motion_envelope,payload_active=self.payload_active)

    def _dock_reference(self,pose):
        if self.navigation_mode=='brake':
            stamp = self.get_clock().now().nanoseconds*1e-9
            dt = max(0.,min(.1,stamp-self.braking_stamp))
            self.braking_stamp = stamp
            cmd = Twist()
            for index,deceleration in enumerate((.8,.8,.7)):
                value = self.braking_reference[index]
                self.braking_reference[index] = math.copysign(max(0.,abs(value)-deceleration*dt),value)
            cmd.linear.x,cmd.linear.y,cmd.angular.z = self.braking_reference
            self.path_reference_status = 'nominal_braking'
            return cmd
        goal = self.reference_goal
        if goal is None or goal.header.frame_id != self.target_frame:
            return None
        self.reference_goal_world = (goal.pose.position.x,goal.pose.position.y)
        self.reference_progress_target = self.reference_goal_world
        q = goal.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        error = self._wrap_pi(yaw-pose[2])
        cmd = Twist()
        if self.navigation_mode=='manipulate':
            from .manipulation_heading import heading_hold_reference
            cmd.angular.z = heading_hold_reference(error,self.measured_velocity[2],self.max_angular_near_human)
            self.path_reference_status = 'manipulation_heading_hold'
            return cmd
        if abs(error) > .12:
            if self.motion_envelope.response_profile==NATIVE_CONTACT_PROFILE:
                cmd.angular.z = calibrated_turn_reference(error,self.measured_velocity[2],self.payload_active)
            elif getattr(self,'payload_active',False):
                cmd.angular.z = loaded_turn_reference(error)
            else:
                cmd.angular.z = min(.6,max(.55,1.5*abs(error))) if error > 0. else -min(.9,max(.65,1.5*abs(error)))
        elif self.navigation_mode != 'scan':
            dx,dy = goal.pose.position.x-pose[0],goal.pose.position.y-pose[1]
            distance = math.hypot(dx,dy)
            if distance > .025:
                # Live docking traces show stalls at 0.12 and 0.20 m/s.
                # Retain a walking command until the positional stop threshold;
                # the normal swept-clearance and slew checks still apply.
                forward = math.cos(pose[2])*dx+math.sin(pose[2])*dy
                lateral = -math.sin(pose[2])*dx+math.cos(pose[2])*dy
                # Correct one axis at a time so diagonal components do not
                # both fall back into the policy's low-command stand region.
                if abs(forward) >= abs(lateral):
                    cmd.linear.x = min(.45,self.max_forward_near_human)
                    # Dex3 trials acknowledge 0.30 yet stand still, including
                    # after release while the arm remains in its carry posture.
                    # Keep the executable reverse envelope to the stop band;
                    # dropping to -0.30 near the endpoint stalls before arrival.
                    # rollout, clearance and slew checks still score this command.
                    if forward < 0.:
                        cmd.linear.x = -min(.45,self.max_reverse_velocity)
                else:
                    cmd.linear.y = math.copysign(.30,lateral)
        self.path_reference_status = 'precision_docking'
        return cmd

    def _goal_callback(self, msg: PoseStamped) -> None:
        if self._goal_key(msg) != self._goal_key(self.reference_goal):
            self.escape_plan = None
            self.warm_start_plan = None
            self.goal_urgency = 1.
            self.goal_best_distance = math.inf
            self.goal_progress_stamp = None
            self.goal_holonomic = False
        self.reference_goal = msg
        self.reference_goal_world = None
        if self.reference_path is not None:
            endpoint = self.reference_path.poses[-1].pose.position
            if self.reference_path.header.frame_id != msg.header.frame_id or math.hypot(endpoint.x - msg.pose.position.x, endpoint.y - msg.pose.position.y) > 0.30:
                self.reference_path = None
                self.reference_path_time = None

    def _path_reference(self, pose) -> Twist | None:
        self.reference_progress_target = None
        path, goal = self.reference_path, self.reference_goal
        self.path_reference_status = "missing_path_or_goal"
        if path is None or goal is None or not path.poses or path.header.frame_id != goal.header.frame_id:
            return None
        if self.reference_path_time is None or (not self.use_static_scene_map and (self.get_clock().now() - self.reference_path_time).nanoseconds * 1e-9 > 2.5 and not self._retained_route_observed(path)):
            self.path_reference_status = "stale_path"
            return None
        endpoint = path.poses[-1].pose.position
        if math.hypot(endpoint.x - goal.pose.position.x, endpoint.y - goal.pose.position.y) > 0.30:
            self.path_reference_status = "path_goal_mismatch"
            return None  # ComputePathToPose also publishes paths for rejected samples.
        points = np.asarray([(p.pose.position.x, p.pose.position.y) for p in path.poses])
        goal_point = np.asarray((goal.pose.position.x, goal.pose.position.y), dtype=float)
        robot_in_path_frame = pose[:2]
        if path.header.frame_id != self.target_frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.target_frame, path.header.frame_id, rclpy.time.Time())
            except TransformException:
                self.path_reference_status = "missing_path_tf"
                return None
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            c, s = math.cos(yaw), math.sin(yaw)
            rx, ry = pose[0]-tf.transform.translation.x, pose[1]-tf.transform.translation.y
            robot_in_path_frame = (c*rx+s*ry, -s*rx+c*ry)
            points = points @ np.asarray(((c, s), (-s, c)))
            points += (tf.transform.translation.x, tf.transform.translation.y)
            goal_point = goal_point @ np.asarray(((c, s), (-s, c)))
            goal_point += (tf.transform.translation.x, tf.transform.translation.y)
        # Smac may end a valid path within its endpoint tolerance. That endpoint
        # is not the requested goal: stopping 0.4 m from an endpoint displaced
        # by 0.25 m could leave the robot permanently outside the 0.5 m goal.
        if np.linalg.norm(points[-1]-goal_point) > 1e-6:
            points = np.vstack((points, goal_point))
        goal_distance = float(np.linalg.norm(goal_point - pose[:2]))
        # Lateral finishing is a local exception, not a latch for the whole
        # mission. Resume facing the route if avoidance pushes us away again.
        finishing_radius = self.path_reference_holonomic_distance
        if finishing_radius <= 0. or goal_distance > finishing_radius+.25:
            self.goal_holonomic = False
        elif goal_distance <= finishing_radius:
            self.goal_holonomic = True
        self.reference_goal_world = tuple(goal_point)
        command = Twist()
        self.path_reference_status = "active"
        if goal_distance <= 0.4:
            self.path_reference_status = "at_goal"
            return command
        nearest = int(np.argmin(np.sum((points - pose[:2]) ** 2, axis=1)))
        index, distance = nearest, 0.0
        while index + 1 < len(points) and distance < self.path_reference_lookahead:
            distance += float(np.linalg.norm(points[index + 1] - points[index]))
            index += 1
        if (getattr(self, "prefer_forward_travel", False) and goal_distance <= 3. and
                getattr(self, "observed_map_time", None) is not None and
                (self.get_clock().now()-self.observed_map_time).nanoseconds*1e-9 <= 2.5):
            shortcut = Path()
            shortcut.header.frame_id = path.header.frame_id
            current = PoseStamped()
            current.pose.position.x, current.pose.position.y = robot_in_path_frame
            shortcut.poses = [current, goal]
            people = [(t.x,t.y,t.vx,t.vy,t.body_radius) for t in self._fresh_tracks(self.get_clock().now())]
            if (people_clear_corridor(pose, goal_point, people, self.robot_radius) and
                    route_is_free(self.observed_map, shortcut, self.robot_radius+.10)):
                # Smooth a short route only through known-free footprint cells.
                # Dynamic people are still checked by every candidate rollout.
                index = len(points)-1
                self.path_reference_status = "observed_goal_shortcut"
        self.reference_progress_target = tuple(points[index])
        dx, dy = points[index] - pose[:2]
        norm = max(1e-6, math.hypot(dx, dy))
        # The official policy stalls at tiny requested forward speeds. Keep a
        # useful gait until the arrival region instead of asymptotically parking
        # just outside the benchmark's 0.5 m handoff radius.
        minimum_speed = .30 if getattr(self, "prefer_forward_travel", False) else .08
        speed = min(self.path_reference_speed, max(minimum_speed, 0.6 * (goal_distance - 0.35)))
        command.linear.x = speed * (math.cos(pose[2]) * dx + math.sin(pose[2]) * dy) / norm
        command.linear.y = speed * (-math.sin(pose[2]) * dx + math.cos(pose[2]) * dy) / norm
        if getattr(self, "prefer_forward_travel", False) and not self.goal_holonomic:
            command.linear.x, command.linear.y, command.angular.z = forward_reference(
                math.cos(pose[2])*dx + math.sin(pose[2])*dy,
                -math.sin(pose[2])*dx + math.cos(pose[2])*dy,
                speed, self.max_angular_near_human,
                getattr(self, "max_clockwise_angular_near_human", 0.) or None,
                smooth_loaded=getattr(self,'payload_active',False))
            if self.motion_envelope.response_profile==NATIVE_CONTACT_PROFILE and abs(command.linear.x)<.1:
                heading = self._wrap_pi(math.atan2(dy,dx)-pose[2])
                command.angular.z = calibrated_turn_reference(heading,self.measured_velocity[2],self.payload_active)
        elif getattr(self, "prefer_forward_travel", False) and goal_distance <= 1.0:
            # Finish short adjustments holonomically without circling the goal.
            scale = min(1., .30 / max(1e-6, math.hypot(command.linear.x, command.linear.y)))
            command.linear.x *= scale
            command.linear.y *= scale
        elif self.holonomic_heading_alignment and goal_distance > 1.0:
            nearby = self._fresh_tracks(self.get_clock().now())
            if all(math.hypot(t.x-pose[0],t.y-pose[1]) >= 2.5 for t in nearby):
                heading = math.atan2(command.linear.y,command.linear.x)
                limit = .35 if command.linear.x < 0. else .6
                command.angular.z = 0. if abs(heading) < .15 else self._clamp(.8*heading,-limit,limit)
        if getattr(self,'continuous_reference',False):
            stamp = self.get_clock().now().nanoseconds*1e-9
            dt = .05 if self.nominal_reference_stamp is None else stamp-self.nominal_reference_stamp
            if dt>.3:
                self.nominal_reference.reset()
            self.nominal_reference_stamp = stamp
            heading = self._wrap_pi(math.atan2(dy,dx)-pose[2]) if not self.goal_holonomic else 0.
            target = (command.linear.x,command.linear.y,command.angular.z)
            command.linear.x,command.linear.y,command.angular.z = self.nominal_reference.update(
                target,heading,self.measured_velocity,dt,self.payload_active)
        return command

    def _scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_scan_time = rclpy.time.Time.from_msg(msg.header.stamp)

    def _odom_callback(self, msg: Odometry) -> None:
        velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
        if all(math.isfinite(value) for value in velocity):
            self.measured_velocity = velocity
            self.source_odom_velocity = velocity
            self.odom_stamp = _ros_time_seconds(msg.header.stamp)
            if msg.header.frame_id == self.target_frame:
                q = msg.pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
                self.odom_pose_history[self.odom_stamp] = self.odom_pose
                self.odom_pose_history = {stamp: pose for stamp, pose in self.odom_pose_history.items() if self.odom_stamp - stamp <= 2.0}

    def _scene_map_callback(self, msg: OccupancyGrid) -> None:
        if msg.header.frame_id != self.target_frame or len(msg.data) != msg.info.width * msg.info.height:
            return
        ys, xs = np.nonzero(np.asarray(msg.data).reshape((msg.info.height, msg.info.width)) >= 50)
        points = np.column_stack(((xs + 0.5) * msg.info.resolution, (ys + 0.5) * msg.info.resolution))
        q = msg.info.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        self.static_scene_points = points @ np.asarray(((c, s), (-s, c))) + (msg.info.origin.position.x, msg.info.origin.position.y)

    def _low_obstacle_callback(self, msg: PointCloud2) -> None:
        if msg.header.frame_id != self.target_frame:
            return
        stamp = _ros_time_seconds(msg.header.stamp)
        if not 0.0 <= self.get_clock().now().nanoseconds * 1e-9 - stamp <= self.max_scan_age:
            return
        people = [(t.x + t.vx * (stamp - t.stamp), t.y + t.vy * (stamp - t.stamp), t.body_radius + 0.15) for t in self.tracks.values()]
        def person_return(x, y):
            return any(math.hypot(x - px, y - py) <= radius for px, py, radius in people)
        for x, y, z in iter_xyz(msg):
            if 0.08 <= z <= 0.55 and not person_return(x, y):
                self.low_obstacle_memory[(round(x / 0.10), round(y / 0.10))] = (x, y, stamp)
        self.low_obstacle_memory = {key: value for key, value in self.low_obstacle_memory.items()
                                    if 0.0 <= stamp - value[2] <= 60.0 and not person_return(value[0], value[1])}

    def _policy_command_callback(self, msg: TwistStamped) -> None:
        values = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        if all(math.isfinite(value) for value in values):
            self.policy_command = values
            self.policy_command_stamp = _ros_time_seconds(msg.header.stamp)

    def _lowstate_callback(self, msg: LowState, info=None) -> None:
        rpy = msg.imu_state.rpy
        roll = math.atan2(math.sin(float(rpy[0])), math.cos(float(rpy[0])))
        pitch = math.atan2(math.sin(float(rpy[1])), math.cos(float(rpy[1])))
        self.last_tilt = max(abs(roll), abs(pitch))
        if getattr(self,'heading_history_enabled',False):
            stamp = (info.get('source_timestamp',0) if isinstance(info,dict)
                     else getattr(info,'source_timestamp',0))
            self.heading_history.observe(float(stamp or 0)*1e-9,
                self.get_clock().now().nanoseconds*1e-9, int(msg.tick),
                tuple(map(float,rpy)), tuple(map(float,msg.imu_state.gyroscope)))

    def _tentative_callback(self, msg: PointCloud2) -> None:
        self.tentative_points = list(iter_xyz(msg))[:64]
        self.tentative_time = _ros_time_seconds(msg.header.stamp)

    def _predicted_callback(self, msg: PointCloud2) -> None:
        self.predicted_points = list(iter_xyz(msg))[:256]
        self.predicted_time = _ros_time_seconds(msg.header.stamp)

    def _tracked_persons_callback(self, msg: TrackedPersons) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            self.get_logger().warn(
                f"Ignoring tracked persons in {msg.header.frame_id}; expected {self.target_frame}",
                throttle_duration_sec=2.0,
            )
            return
        stamp = _ros_time_seconds(msg.header.stamp)
        tracks = {}
        if (self.forecast_change_guard_enabled and self.tracks_epoch is not None and stamp<=self.tracks_epoch):
            return
        for person in msg.persons:
            if person.track_state not in (
                TrackedPerson.TRACK_STATE_TENTATIVE,
                TrackedPerson.TRACK_STATE_CONFIRMED,
                TrackedPerson.TRACK_STATE_LOST,
            ):
                continue
            tracks[int(person.track_id)] = HumanTrack(
                track_id=int(person.track_id),
                state={TrackedPerson.TRACK_STATE_CONFIRMED: "CONFIRMED", TrackedPerson.TRACK_STATE_LOST: "OCCLUDED", TrackedPerson.TRACK_STATE_TENTATIVE: "TENTATIVE"}[person.track_state],
                x=float(person.pose.pose.position.x),
                y=float(person.pose.pose.position.y),
                vx=float(person.velocity.twist.linear.x),
                vy=float(person.velocity.twist.linear.y),
                confidence=max(0.0, min(1.0, float(person.confidence))),
                person_score=max(0.0, min(1.0, float(person.confidence))),
                missed_count=1 if person.track_state == TrackedPerson.TRACK_STATE_LOST else 0,
                cov_xx=max(0.0, float(person.pose.covariance[0])),
                cov_yy=max(0.0, float(person.pose.covariance[7])),
                stamp=stamp,
                body_radius=max(0.0, float(person.body_radius)),
                observation_age=_ros_time_seconds(person.observation_age),
            )
        if self.forecast_change_guard_enabled:
            self.forecast_change_reserve.observe(stamp,tracks.values())
        self.tracks = tracks
        self.tracks_epoch = stamp

    def _predicted_trajectories_callback(self, msg: PredictedTrajectories) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            self.get_logger().warn(
                f"Ignoring person predictions in {msg.header.frame_id}; expected {self.target_frame}",
                throttle_duration_sec=2.0,
            )
            return
        epoch=_ros_time_seconds(msg.header.stamp)
        if self.forecast_change_guard_enabled and self.prediction_epoch is not None and epoch<=self.prediction_epoch:
            return
        predictions = {}
        for trajectory in msg.trajectories:
            points = []
            previous_offset = -1.0
            for point in trajectory.points:
                offset = _ros_time_seconds(point.time_offset)
                values = (point.position.x, point.position.y, point.velocity.x, point.velocity.y)
                if offset < 0.0 or offset <= previous_offset or not all(math.isfinite(float(value)) for value in values):
                    continue
                points.append(PredictionSample(
                    time_offset=offset,
                    x=float(point.position.x),
                    y=float(point.position.y),
                    vx=float(point.velocity.x),
                    vy=float(point.velocity.y),
                    cov_xx=max(0.0, float(point.position_covariance[0])),
                    cov_yy=max(0.0, float(point.position_covariance[4])),
                    confidence=max(0.0, min(1.0, float(point.confidence))),
                ))
                previous_offset = offset
            if points:
                predictions[int(trajectory.track_id)] = points
        self.predictions = predictions
        self.prediction_epoch = _ros_time_seconds(msg.header.stamp)

    def _debug_tracks_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        stamp = float(payload.get("stamp", self.get_clock().now().nanoseconds * 1e-9))
        tracks = {}
        for item in payload.get("tracks", []):
            state = str(item.get("state", ""))
            if state not in ("CONFIRMED", "OCCLUDED"):
                continue
            try:
                track_id = int(item["track_id"])
                tracks[track_id] = HumanTrack(
                    track_id=track_id,
                    state=state,
                    x=float(item.get("x", 0.0)),
                    y=float(item.get("y", 0.0)),
                    vx=float(item.get("vx", 0.0)),
                    vy=float(item.get("vy", 0.0)),
                    confidence=float(item.get("confidence", 0.0)),
                    person_score=float(item.get("person_score", 0.0)),
                    missed_count=int(item.get("missed_count", 0)),
                    cov_xx=float(item.get("cov_xx", 0.20)),
                    cov_yy=float(item.get("cov_yy", 0.20)),
                    stamp=stamp,
                    body_radius=self.human_body_radius,
                )
            except (TypeError, ValueError, KeyError):
                continue
        self.legacy_tracks = tracks

    def _timer_callback(self) -> None:
        if self.nav2_path_reference:
            self.latest_cmd = self.upstream_cmd
        self.planning_started = time.perf_counter()
        self.planning_cpu_started = time.thread_time()
        self.planning_phases = {}
        self.scoring_phases = {}
        self.kinodynamic_status = 'not_requested'
        self.kinodynamic_details = {}
        self.primitive_warm_start = None
        self.primitive_continuation_plan = None
        self.primitive_validation_audit=None
        self.strategic_status = dict(active=False)
        self.prediction_cache = {}
        self.static_cache = {}
        self.static_points_cache = None
        self.static_field_cache = None
        self.static_frame_valid = None
        now = self.get_clock().now()
        self.selected_safe_patch = None
        pose = self._robot_pose()
        tracks = self._fresh_tracks(now)
        tentative_points = self._fresh_tentative(now)
        predictions, prediction_age, typed_predictions_fresh = self._fresh_predictions(now)
        predicted_points = [] if typed_predictions_fresh else self._fresh_predicted(now)
        if self.nav2_path_reference:
            if self.mission_state_time is None or (now - self.mission_state_time).nanoseconds * 1e-9 > 1.0:
                self._publish(Twist(), "MISSION_STALE_STOP", [], None)
                return
            if not self.mission_active:
                self.latest_cmd = Twist()
        if self._input_stale(now):
            autonomous_risk = (self.nav2_path_reference and self.reference_goal is not None and pose is not None and
                               ((self.mission_active and self.reference_path is not None) or
                                incoming_collision_imminent(pose, tracks, self.horizon, self.robot_radius + self.human_body_radius + 0.5, 0.12)))
            if not autonomous_risk:
                self._publish(Twist(), "NORMAL_NAVIGATION", [], None)
                return
            self.latest_cmd = Twist()
        if pose is None:
            self._publish(Twist(), "EMERGENCY_STOP", [], None)
            return
        now_seconds = now.nanoseconds * 1e-9
        if self.require_fresh_tracks and (self.tracks_epoch is None or not 0.0 <= now_seconds - self.tracks_epoch <= self.max_person_cloud_age):
            self.last_cmd = Twist()
            self._publish(Twist(), "PERCEPTION_STALE_STOP", [], None)
            return
        if self.odom_stamp is None or not 0.0 <= now_seconds - self.odom_stamp <= self.odom_timeout:
            self.last_cmd = Twist()
            self._publish(Twist(), "ODOMETRY_STALE_STOP", [], None)
            return
        if self.nav2_path_reference and self.mission_active:
            reference = self._dock_reference(pose) if self.navigation_mode in ('dock','scan','manipulate','brake','reserve_recovery') else self._path_reference(pose)
            if reference is not None:
                self.latest_cmd = reference
        self._update_goal_urgency(pose,now_seconds)
        self.strategic_status = dict(active=False)
        if (getattr(self,'strategic_avoidance_enabled',False) and len(tracks)>=2 and
                self.navigation_mode!='manipulate' and self.policy_command is not None and
                self.policy_command_stamp is not None and 0.<=now_seconds-self.policy_command_stamp<=.4):
            from .strategic_avoidance import crowd_forecast,predict_strategic_risk
            times=np.linspace(0.,7.,36)
            people,radii=crowd_forecast(tracks,times,self.robot_radius,self.body_prediction_margin)
            if self.forecast_change_guard_enabled:
                radii+=np.max(self.forecast_change_reserve.reserve(tracks,times,now_seconds),axis=0)
            command=(self.latest_cmd.linear.x,self.latest_cmd.linear.y,self.latest_cmd.angular.z)
            nominal=np.asarray(self.motion_envelope.rollout(pose,command,self.measured_velocity,
                7.,.2,self.response_delay,filtered_command=self.policy_command))
            self.strategic_status=predict_strategic_risk(nominal[:,:2],people,radii,times)
            self.strategic_status['track_pairs']=[[tracks[i].track_id,tracks[j].track_id]
                                                 for i,j in self.strategic_status['pairs']]
            self.strategic_status['watch_tracks']=[tracks[i].track_id for i in self.strategic_status['watch_indices']]
        committed=self._primitive_commitment_active(now_seconds)
        if committed:
            self.strategic_status['continuation_active']=True
            candidate,origin,_=self.escape_plan
            self.strategic_status['commitment_end_ros_s']=origin+sum(part[0] for part in candidate.schedule)
            if not self.strategic_status['active']:
                self.strategic_status.update(active=True,reason='committed_escape')
        pincer=self.strategic_status['active']
        if not pincer:
            self.strategic_goal=None
        if (self.prefer_safe_route_alignment and self.nav2_path_reference and self.mission_active and
                self.navigation_mode in ('scan','manipulate','brake') and not pincer):
            # A scan/brake is an explicit request to stop translating, not a
            # new route-progress objective. Recheck its full stopping/turning
            # rollout before continuing an old escape or optimizing comfort.
            # Unsafe requests still fall through to the normal escape search.
            nominal = self._executable_candidates(
                self._candidate_commands(self.latest_cmd,'NORMAL_NAVIGATION',pose,tracks))[0]
            checked = self._score_candidate(nominal,pose,tracks,tentative_points,
                                             predicted_points,predictions,prediction_age)
            if self._has_nominal_clearance(checked,tracks):
                self.escape_plan = self.warm_start_plan = None
                cmd = self._twist_from_candidate(nominal)
                self.last_cmd = cmd
                self.path_reference_status = 'safe_precision_reference'
                self._publish(cmd,'NORMAL_NAVIGATION',[checked],checked)
                return
        pending = self._pending_escape(now_seconds)
        if pending is not None:
            result = self._checked_continuation(pending, pose, tracks, tentative_points,
                                               predicted_points, predictions, prediction_age)
            if not result.rejected or result.reason == 'human_margin':
                cmd = self._twist_from_candidate(pending)
                self.last_cmd = cmd
                self._publish(cmd, "CLEARANCE_RECOVERY" if result.rejected else "HUMAN_CAUTION", [result], result)
                return
            self.escape_plan = None
        if self.local_safe_space_enabled and self.last_tilt <= self.safe_space_max_tilt:
            safe_selected, safe_results = self._select_local_safe_space_command(
                pose, tracks, tentative_points, predicted_points, predictions, prediction_age
            )
            if safe_selected is not None and safe_selected.candidate.name == "nearest_clear_patch":
                cmd = self._twist_from_candidate(safe_selected.candidate)
                self.last_cmd = cmd
                self._publish(cmd, "LOCAL_SAFE_SPACE", safe_results, safe_selected)
                return
        state = self._update_state(now, pose, tracks, tentative_points)
        if pincer and state=='NORMAL_NAVIGATION':
            state='HUMAN_CAUTION'
            self.state=state
            self.state_since=now
        # Full-horizon feasibility must activate search before the short CPA
        # state thresholds. Otherwise a rejected raw candidate means a late stop.
        if state == "NORMAL_NAVIGATION" and tracks:
            raw = self._candidate_commands(self.latest_cmd, state, pose, tracks)[0]
            raw_result = self._score_candidate(raw, pose, tracks, tentative_points, predicted_points, predictions, prediction_age)
            if self._can_prioritize_alignment(raw_result,tracks):
                # A broad caution activation radius is not itself a reason to
                # abandon a fully checked alignment turn for faster sideways
                # progress. Genuine forecast conflicts still activate search.
                cmd = self._twist_from_candidate(raw)
                self.last_cmd = cmd
                self.path_reference_status = 'safe_route_alignment'
                self._publish(cmd,'NORMAL_NAVIGATION',[raw_result],raw_result)
                return
            warning_distance = max(self.robot_radius + track.body_radius + self.hard_safety_margin for track in tracks)
            warning_distance = max(warning_distance, getattr(self, "prediction_activation_clearance", 0.))
            holding_risk = (getattr(self, "prefer_forward_travel", False) and
                incoming_collision_imminent(pose, tracks, 3., warning_distance+.2, .12))
            if holding_risk or raw_result.rejected or raw_result.min_human_distance < warning_distance or raw_result.score_terms.get("terminal_conflict", 0.0) > 0.0:
                state = "HUMAN_CAUTION"
                self.state = state
                self.state_since = now
                if self.side_commitment == 0.0 or now > self.side_commitment_until:
                    self.side_commitment = self._preferred_side(pose, tracks)
                    self.side_commitment_until = now + Duration(seconds=self.side_commitment_timeout)
        candidates = self._candidate_commands(self.latest_cmd, state, pose, tracks)
        if self.warm_start_plan is not None:
            schedule,goal_key = self.warm_start_plan
            self.warm_start_plan = None
            if goal_key == (self._goal_key(self.reference_goal) if self.mission_active else None):
                candidates.insert(0,self._schedule_candidate('beam_warm_start',schedule))
        survey_refuge = self.navigation_mode in ('scan','brake')
        if ((self.kinodynamic_search_enabled or survey_refuge or pincer) and state != 'NORMAL_NAVIGATION' and (tracks or committed) and
                (self.mission_active or any(math.hypot(t.vx,t.vy)>.25 for t in tracks))):
            proposed = self._kinodynamic_candidate(pose,tracks,tentative_points,predicted_points,predictions,prediction_age)
            if proposed is not None:
                proposed = self._executable_candidates([proposed])[0]
                checked = self._score_candidate(proposed,pose,tracks,tentative_points,predicted_points,predictions,prediction_age)
                if not checked.rejected or checked.reason == 'human_margin':
                    cmd = self._twist_from_candidate(proposed)
                    self.last_cmd = cmd
                    if proposed.name=='escape_primitive' and self.primitive_validation_audit is not None:
                        self.primitive_last_validation=self.primitive_validation_audit
                    continuation=getattr(self,'primitive_continuation_plan',None)
                    self.escape_plan = (continuation if proposed.name=='escape_primitive' and continuation is not None else
                        (proposed,self.get_clock().now().nanoseconds*1e-9,
                         self._goal_key(self.reference_goal) if self.mission_active else None))
                    self._publish(cmd,'CLEARANCE_RECOVERY' if checked.rejected else 'HUMAN_CAUTION',[checked],checked)
                    return
                candidates.insert(0,proposed)
        candidates = self._executable_candidates(candidates)
        scoring_started = time.perf_counter()
        self.planning_phases["prepare_ms"] = (scoring_started - self.planning_started) * 1000.0
        results = self._evaluate_candidates(candidates, pose, tracks, tentative_points,
                                            predicted_points, predictions, prediction_age)
        feasible = [result for result in results if not result.rejected or
                    (result.candidate.name == 'beam_warm_start' and result.reason == 'human_margin' and
                     'continuation_margin' in result.score_terms)]
        self.planning_phases["scoring_ms"] = (time.perf_counter() - scoring_started) * 1000.0
        if not feasible:
            # A comfort-margin breach is preferable to a predicted body
            # collision while standing still. Never relax body/static geometry.
            recovery = [result for result in results if result.reason == "human_margin"]
            if recovery:
                feasible = [max(recovery, key=lambda result: result.min_human_distance)]
        if not feasible:
            if self.local_escape_enabled:
                escape = self._short_horizon_escape(results, tracks)
                if escape is not None:
                    feasible = [escape]
        if not feasible:
            feasible = [result for result in results if result.reason == "terminal_conflict"]
        selected = self._stable_selection(feasible)
        if selected is None:
            cmd = Twist()
            state = "EMERGENCY_STOP"
        else:
            cmd = self._twist_from_candidate(selected.candidate)
            if selected.rejected:
                state = "CLEARANCE_RECOVERY"
            elif selected.candidate.name == "yield_stop" and tracks:
                state = "YIELD_TO_HUMAN"
            elif selected.candidate.preferred_state:
                state = selected.candidate.preferred_state
        self.last_cmd = cmd
        if (selected is not None and (not selected.rejected or selected.reason == 'human_margin') and
                (selected.candidate.schedule is not None or selected.candidate.name in
                 ("crossing_then_route", "escape_then_route", "wait_then_route", "wait_then_escape"))):
            self.escape_plan = (selected.candidate, self.get_clock().now().nanoseconds*1e-9,
                                self._goal_key(self.reference_goal) if self.mission_active else None)
        self._publish(cmd, state, results, selected)

    def _can_prioritize_alignment(self,result,tracks):
        candidate = result.candidate
        return (self.prefer_safe_route_alignment and self.prefer_forward_travel and self.nav2_path_reference and
                 self.mission_active and self.navigation_mode=='navigate' and not self.goal_holonomic and
                not self.payload_active and self._has_nominal_clearance(result,tracks) and
                 abs(candidate.vx)+abs(candidate.vy)<1e-6 and abs(candidate.wz)>.05)

    def _has_nominal_clearance(self,result,tracks):
        reserve = max([self.comfort_radius]+[self.robot_radius+t.body_radius+self.body_prediction_margin+.25 for t in tracks])
        return not result.rejected and result.min_human_distance>=reserve

    def _update_goal_urgency(self,pose,seconds):
        if not self.mission_active or self.reference_goal_world is None:
            self.goal_urgency = 1.
            return
        distance = math.hypot(pose[0]-self.reference_goal_world[0],pose[1]-self.reference_goal_world[1])
        if (self.goal_progress_stamp is None or seconds < self.goal_progress_stamp or
                distance < self.goal_best_distance-.1):
            self.goal_best_distance = distance
            self.goal_progress_stamp = seconds
        fraction = self._clamp((seconds-self.goal_progress_stamp-self.goal_progress_timeout)/self.goal_progress_timeout,0.,1.)
        self.goal_urgency = 1.+(self.goal_urgency_max-1.)*fraction

    def _stable_selection(self, feasible):
        if not feasible:
            return None
        selected = min(feasible, key=lambda result: result.score)
        previous = next((result for result in feasible if result.candidate.name == 'previous_command'), None)
        if (self.selection_hysteresis > 0. and previous is not None and not previous.rejected and
                not selected.rejected and math.hypot(previous.candidate.vx, previous.candidate.vy) > .1 and
                previous.score <= selected.score+self.selection_hysteresis and
                previous.min_human_distance >= selected.min_human_distance-.1):
            return previous
        return selected

    @staticmethod
    def _goal_key(goal):
        if goal is None:
            return None
        p,q = goal.pose.position,goal.pose.orientation
        return (goal.header.frame_id,p.x,p.y,p.z,q.x,q.y,q.z,q.w)

    @staticmethod
    def _schedule_candidate(name,schedule):
        first = schedule[0]
        following = schedule[1][1:] if len(schedule)>1 else (0.,0.,0.)
        return Candidate(name,*first[1:],'',following[0],first[0],following[1],following[2],schedule)

    def _checked_continuation(self,candidate,pose,tracks,tentative,predicted,predictions,age):
        result = self._score_candidate(candidate,pose,tracks,tentative,predicted,predictions,age)
        margin = min(self.body_prediction_margin,self.continuation_prediction_margin)
        if result.reason == 'hard_human_collision' and margin < self.body_prediction_margin:
            recovery = self._score_candidate(candidate,pose,tracks,tentative,predicted,predictions,age,
                                             prediction_margin=margin)
            if not recovery.rejected or recovery.reason == 'human_margin':
                recovery.score_terms['continuation_margin'] = margin
                return recovery
        return result

    def _primitive_audit(self,pose,tracks,predictions,prediction_age,people,radii,goal,horizon,program):
        return dict(forecast_epoch=tracks[0].stamp if tracks else self.get_clock().now().nanoseconds*1e-9,
            odom_stamp=self.odom_stamp,tracks_epoch=self.tracks_epoch,prediction_epoch=self.prediction_epoch,
            prediction_age=prediction_age,pose=list(pose),velocity=list(self.measured_velocity),
            filtered=list(self.policy_command),program=program,envelope=asdict(self.motion_envelope),
            people=people.tolist(),radii=radii.tolist(),tracks=[asdict(track) for track in tracks],
            typed_predictions={str(identity):[[p.time_offset,p.x,p.y,p.vx,p.vy] for p in samples]
                               for identity,samples in predictions.items()},
            goal=list(goal),horizon=horizon,dt=self.dt,delay=self.response_delay,
            static_clearance=self.static_clearance,forecast_source=self.forecast_source,
            heading_history=dict(getattr(self,'heading_history_details',{})),
            source_odom_pose=self.odom_pose,source_odom_velocity=self.source_odom_velocity,
            forecast_change_guard=self.forecast_change_guard_enabled)

    def _kinodynamic_candidate(self,pose,tracks,tentative,predicted,predictions,prediction_age):
        started = time.perf_counter()
        self.primitive_continuation_plan=None
        seconds = self.get_clock().now().nanoseconds*1e-9
        goal = self.reference_progress_target or self.reference_goal_world
        strategic=getattr(self,'strategic_status',{}).get('active',False)
        guarded=strategic and self.forecast_change_guard_enabled
        if guarded and (self.odom_stamp is None or not 0.<=seconds-self.odom_stamp<=.3):
            self.kinodynamic_status='primitive_pose_too_old'
            return None
        refuge = strategic or not self.mission_active or self.navigation_mode in ('scan','brake')
        self.kinodynamic_details={}
        proposals=[]
        if strategic:
            goal=pose[:2]  # Replaced below after static-aware all-person ranking.
            from .escape_primitives import templates,check_programs,snapshot,process_worker
            if self.primitive_worker is None:
                self.primitive_worker=process_worker()
            if not self.primitive_worker.warmup.done():
                self.kinodynamic_status='primitive_worker_warming'
                return None
            context=(self._goal_key(self.reference_goal),self.mission_active,self.navigation_mode,
                     self.payload_active,self.motion_envelope)
            ready=self.primitive_worker.poll(.2)
            warm=getattr(self,'primitive_warm_start',None)
            if warm:
                proposals.append(warm)
            if ready is not None:
                self.kinodynamic_details['worker_result']=dict(ready.snapshot.diagnostics)
                self.kinodynamic_details['result_age_s']=time.monotonic()-ready.submitted_at
                if ready.snapshot.context==context and ready.command is not None:
                    proposals.append(ready.command)
                    goal=ready.snapshot.inputs['goal']
                elif ready.snapshot.context!=context:
                    self.kinodynamic_details['reason']='context_changed'
            if proposals and self.strategic_goal is not None:
                goal=self.strategic_goal
        if refuge and not strategic:
            moving = [track for track in tracks if math.hypot(track.vx,track.vy)>.25]
            if not moving:
                return None
            track = min(moving,key=lambda t:math.hypot(t.x-pose[0],t.y-pose[1]))
            speed = math.hypot(track.vx,track.vy)
            nx,ny = -track.vy/speed,track.vx/speed
            side = 1. if (pose[0]-track.x)*nx+(pose[1]-track.y)*ny>=0. else -1.
            # The search can retreat before sidestepping around a jamb. Its
            # target is separation from the walking lane, never the old mission.
            goal = (pose[0]+side*1.5*nx,pose[1]+side*1.5*ny)
        if (goal is None or self.policy_command is None or self.policy_command_stamp is None or
                not 0. <= seconds-self.policy_command_stamp <= .4):
            self.kinodynamic_status = 'missing_goal_or_feedback'
            return None
        if strategic and proposals:
            # A returned plan needs fresh static geometry, not another complete
            # human-risk score for a discarded zero-command anchor. Its full
            # human/static trajectory is checked below with the new forecasts.
            self._static_obstacle_cost([(*pose,0.)])
        else:
            anchor = self._score_candidate(Candidate('beam_anchor',0.,0.,0.),pose,tracks,tentative,predicted,predictions,prediction_age)
            self._static_obstacle_cost(anchor.trajectory)
        if self.static_frame_valid is False:
            self.kinodynamic_status = 'static_frame_unavailable'
            return None
        if strategic and not proposals:
            from .strategic_avoidance import crowd_forecast,refuge_candidates
            forecast_times=np.linspace(0.,7.,36)
            crowd,radii=crowd_forecast(tracks,forecast_times,self.robot_radius,self.body_prediction_margin)
            if guarded:
                radii+=np.max(self.forecast_change_reserve.reserve(tracks,forecast_times,seconds),axis=0)
            refuges=refuge_candidates(pose,crowd,radii,forecast_times,
                static_query=None if self.static_field_cache is None else self.static_field_cache.query,
                static_clearance=self.static_clearance,previous=self.strategic_goal)
            self.strategic_status['refuges']=refuges[:3]
            if not refuges:
                self.kinodynamic_status='no_static_refuge'
                return None
            goal=tuple(refuges[0]['goal'])
            self.strategic_goal=goal
        horizon = max(4.5,self.horizon) if refuge else self.horizon
        times = np.linspace(0.,horizon,max(1,math.ceil(horizon/max(.02,self.dt)))+1)
        people,radii = [],[]
        sample_at=continuous_prediction if self.forecast_change_guard_enabled else interpolate_prediction
        for track in tracks:
            samples = predictions.get(track.track_id)
            positions,sigmas = [],[]
            for t in times:
                sample = sample_at(samples,prediction_age+t) if samples else None
                uncertainty_time = min(t,self.body_uncertainty_horizon)
                uncertainty = sample_at(samples,prediction_age+uncertainty_time) if samples else None
                positions.append((track.x+track.vx*t,track.y+track.vy*t) if sample is None else (sample.x,sample.y))
                sigmas.append(math.sqrt(max(0.,uncertainty.cov_xx,uncertainty.cov_yy)) if uncertainty else
                              math.sqrt(max(0.,track.cov_xx,track.cov_yy))+.25*uncertainty_time)
            people.append(np.asarray(positions))
            inflated = self.robot_radius+track.body_radius+self.body_prediction_margin+self.body_uncertainty_scale*np.asarray(sigmas)
            radii.append(inflated if refuge else np.where(times<=self.hard_collision_horizon,inflated,
                                                         self.robot_radius+track.body_radius+.05))
        controls = [(vx,vy,0.) for vx in (-self.max_reverse_velocity,0.,.5*self.max_forward_near_human,self.max_forward_near_human)
                    for vy in (-self.max_lateral_near_human,0.,self.max_lateral_near_human)]
        controls.append((self._clamp(self.latest_cmd.linear.x,-self.max_reverse_velocity,self.max_forward_near_human),
                         self._clamp(self.latest_cmd.linear.y,-self.max_lateral_near_human,self.max_lateral_near_human),0.))
        unique = {}
        for command in controls:
            command = self.motion_envelope.normalize(command)
            unique[tuple(round(v,4) for v in command)] = command
        radius = .4 if self.reference_goal_world is not None and math.hypot(goal[0]-self.reference_goal_world[0],goal[1]-self.reference_goal_world[1]) < .01 else 0.
        people=(np.stack(people,axis=1) if people else np.empty((len(times),0,2)))
        radii=(np.stack(radii,axis=1) if radii else np.empty((len(times),0)))
        if guarded:
            reserve=self.forecast_change_reserve.reserve(tracks,times,seconds)
            radii+=reserve
            self.kinodynamic_details['forecast_change_reserve_m']={str(track.track_id):float(np.max(reserve[:,i]))
                                                                  for i,track in enumerate(tracks)}
        static_query=None if self.static_field_cache is None else self.static_field_cache.query
        budget=min(20.,self.kinodynamic_budget_ms) if refuge else self.kinodynamic_budget_ms
        self.kinodynamic_details['preparation_ms']=(time.perf_counter()-started)*1000.
        schedule=None
        name='kinodynamic'
        remaining=budget
        if strategic:
            remaining=budget-(time.perf_counter()-started)*1000.
            if proposals:
                # Spend the existing planner budget on mandatory fresh-plan
                # admission before falling back to another expensive search.
                # Keep the 12 ms cap and reserve 10 ms for final scoring/publish.
                global_budget=getattr(self,'planning_budget_ms',0.)
                if global_budget>0.:
                    elapsed=(time.perf_counter()-getattr(self,'planning_started',started))*1000.
                    remaining=min(12.,global_budget-elapsed-10.)
                self.kinodynamic_details['admission_budget_ms']=max(0.,remaining)
            if proposals and remaining>0.:
                details={}
                schedule=check_programs(self.motion_envelope,pose,self.measured_velocity,self.policy_command,
                    proposals,people,radii,goal,horizon,self.dt,self.response_delay,
                    static_query=static_query,static_clearance=self.static_clearance,
                    budget_ms=min(12.,remaining),diagnostics=details)
                self.kinodynamic_details['revalidation']=details
                if schedule is not None:
                    name='escape_primitive'
                elif warm and details.get('reason')!='deadline':
                    if self.record_invalidation_audit and self.primitive_last_validation is not None and self.invalidation_audits_remaining>0:
                        self.kinodynamic_details['invalidation_audit']=dict(
                            previous=self.primitive_last_validation,
                            current=self._primitive_audit(pose,tracks,predictions,prediction_age,people,radii,goal,horizon,warm),
                            rejection=dict(details))
                        self.invalidation_audits_remaining-=1
                    # Keep elapsed phase timing across a missed computation,
                    # never across a known geometric/terminal-state rejection.
                    self.escape_plan=None
                    self.primitive_warm_start=None
                    self.kinodynamic_details['continuation']='invalidated_by_fresh_check'
                elif warm:
                    self.kinodynamic_details['continuation']='retained_pending_fresh_check'
            elif remaining<=0.:
                self.kinodynamic_details['reason']='preparation_deadline'
            if schedule is None and self.primitive_worker.future is None:
                programs=templates(self.motion_envelope,horizon,self.max_forward_near_human,
                                   self.max_reverse_velocity,self.max_lateral_near_human)
                request=snapshot(dict(envelope=self.motion_envelope,pose=tuple(pose),
                    velocity=tuple(self.measured_velocity),filtered=tuple(self.policy_command),
                    programs=programs,people=people,radii=radii,goal=tuple(goal),horizon=horizon,
                    dt=self.dt,delay=self.response_delay,static_query=static_query,
                    static_clearance=self.static_clearance,budget_ms=80.,
                    admission_delays=(0.,.1,.2),minimum_margin=.10),context)
                self.primitive_worker.submit(request)
            self.kinodynamic_details['worker']=self.primitive_worker.diagnostics()
        if not strategic and schedule is None and remaining>1.:
            details={}
            schedule = search_sequence(self.motion_envelope,pose,self.measured_velocity,self.policy_command,
                list(unique.values()),people,radii,goal,horizon,self.dt,self.response_delay,
                static_query=static_query,static_clearance=self.static_clearance,goal_radius=radius,
                goal_weight=2. if refuge else self.weights['w_goal']*self.goal_urgency,
                budget_ms=remaining,beam_width=16 if refuge else 28,
                stationary=[math.hypot(t.vx,t.vy)<.5 for t in tracks],
                onset_radius=self.stationary_person_clearance,onset_weight=self.stationary_clearance_weight,
                enclosure_weight=max(8.,self.weights['w_enclosure']) if strategic else self.weights['w_enclosure'],
                diagnostics=details)
            self.kinodynamic_details['beam']=details
        current=self.get_clock().now().nanoseconds*1e-9
        if schedule is not None and (not 0.<=current-self.policy_command_stamp<=.4 or
                self.odom_stamp is None or not 0.<=current-self.odom_stamp<=self.odom_timeout or
                (guarded and current-self.odom_stamp>.3) or
                (self.require_fresh_tracks and (self.tracks_epoch is None or
                 not 0.<=current-self.tracks_epoch<=self.max_person_cloud_age))):
            schedule=None
            self.kinodynamic_details['reason']='state_expired_during_search'
        if schedule is not None and strategic and warm and schedule==warm:
            self.primitive_continuation_plan=self.escape_plan
            self.kinodynamic_details['continuation']='revalidated_original_clock'
        if schedule is not None and strategic and self.record_invalidation_audit:
            self.primitive_validation_audit=self._primitive_audit(
                pose,tracks,predictions,prediction_age,people,radii,goal,horizon,schedule)
        self.planning_phases['kinodynamic_ms'] = (time.perf_counter()-started)*1000.
        self.kinodynamic_status = 'proposed' if schedule is not None else ('pending_primitives' if strategic else 'no_complete_plan')
        return None if schedule is None else self._schedule_candidate(name,schedule)

    def _primitive_commitment_active(self,seconds):
        plan=getattr(self,'escape_plan',None)
        if not getattr(self,'strategic_avoidance_enabled',False) or plan is None:
            return False
        candidate,started,goal=plan
        return bool(candidate.name=='escape_primitive' and candidate.schedule and
            goal==(self._goal_key(self.reference_goal) if self.mission_active else None) and
            0.<=seconds-started<sum(part[0] for part in candidate.schedule))

    def _pending_escape(self, seconds):
        """Advance a selected two-phase escape instead of restarting its clock.

        The caller must revalidate this continuation against fresh observations
        on every tick, and discard it if it is no longer feasible.
        """
        if self.escape_plan is None:
            return None
        candidate, started, goal = self.escape_plan
        if goal != (self._goal_key(self.reference_goal) if self.mission_active else None) or seconds < started:
            self.escape_plan = None
            return None
        if candidate.schedule is not None:
            schedule = remaining_schedule(candidate.schedule,seconds-started)
            if not schedule or self.policy_command_stamp is None or not 0. <= seconds-self.policy_command_stamp <= .4:
                self.escape_plan = None
                return None
            if candidate.name=='escape_primitive':
                # Never continue a cached primitive based only on its previous
                # admission. Roll the remaining program out against fresh state,
                # all current people and static geometry on this very tick.
                self.primitive_warm_start=schedule
                # Retain timing if this tick's check misses its budget. This
                # method returns no command: fresh validation is still required
                # before publication, and the original end time keeps advancing.
                return None
            if seconds-started >= candidate.schedule[0][0]:
                self.warm_start_plan = (schedule,goal)
                self.escape_plan = None
                return None
            return self._schedule_candidate('beam_pending',schedule)
        remaining = candidate.switch_time-(seconds-started)
        if remaining <= 0.:
            self.escape_plan = None
            return Candidate('escape_route_phase', candidate.follow_vx, candidate.follow_vy, candidate.follow_wz)
        return Candidate('escape_pending', candidate.vx, candidate.vy, candidate.wz, '',
                         candidate.follow_vx, remaining, candidate.follow_vy, candidate.follow_wz)

    def _evaluate_candidates(self, candidates, pose, tracks, tentative_points, predicted_points, predictions, prediction_age):
        """Bound search latency; every selectable command is still fully checked.

        Always recheck stop, the previous command, and the route command first.
        Rotate additional candidates across ticks so a deadline cannot permanently
        starve a direction or a step-and-hold refinement.
        """
        budget = self.planning_budget_ms
        ordered = candidates
        mandatory = 3
        priority_count = 0
        remaining = []
        if budget > 0 and len(candidates) > 3:
            names = ("yield_stop", "beam_warm_start", "kinodynamic", "velocity_obstacle", "early_crossing_escape", "previous_command", "raw_reference", "goal_capture", "wait_then_route",
                     "escape_translation", "social_translation", "crossing_then_route")
            priority = set(names)
            first = [c for name in names for c in candidates if c.name == name]
            remaining = [c for c in candidates if c.name not in priority]
            deferred = []
            if self.priority_candidate_limit:
                core_names = ('yield_stop','previous_command','raw_reference')
                core = [c for name in core_names for c in first if c.name == name]
                extra = [c for c in first if c.name not in core_names]
                if extra:
                    offset = self.priority_search_phase % len(extra)
                    extra = extra[offset:]+extra[:offset]
                reserved = extra[:self.priority_candidate_limit]
                deferred = extra[self.priority_candidate_limit:]
                self.priority_search_phase += len(reserved)
                first = core+reserved
            priority_count = len(first)
            # The budget is a soft deadline: reserve one rotating proposal even
            # when mandatory checks consume it, otherwise search can freeze.
            mandatory = max(3, priority_count + bool(remaining))
            if remaining:
                offset = self.search_phase % len(remaining)
                remaining = remaining[offset:] + remaining[:offset]
            ordered = first + remaining + deferred
        results = []
        checked_original = 0
        self.search_budget_exhausted = False
        for candidate in ordered:
            if budget > 0 and checked_original >= mandatory and (time.perf_counter() - self.planning_started) * 1000 >= budget:
                self.search_budget_exhausted = True
                break
            scorer = self._checked_continuation if candidate.name == 'beam_warm_start' else self._score_candidate
            results.append(scorer(candidate, pose, tracks, tentative_points,
                                  predicted_points, predictions, prediction_age))
            checked_original += 1
            if (self.translation_refinement_enabled and tracks and priority_count and checked_original == priority_count and
                    not any(not result.rejected for result in results)):
                for refined in self._escape_refinements(results):
                    if budget > 0 and (time.perf_counter()-self.planning_started)*1000 >= budget:
                        break
                    results.append(self._score_candidate(refined, pose, tracks, tentative_points,
                                                        predicted_points, predictions, prediction_age))
        if remaining:
            # Advance by work actually checked. A fixed stride can share a
            # divisor with the candidate count and permanently skip directions.
            self.search_phase += max(1, checked_original - priority_count)
        return results

    def _escape_refinements(self, results):
        """Refine near-feasible translations instead of jumping straight to stop."""
        seeds = sorted((r for r in results if r.candidate.follow_vx is None and
                        abs(r.candidate.wz) < .01 and math.hypot(r.candidate.vx,r.candidate.vy) > .1),
                       key=lambda r: r.min_human_distance, reverse=True)[:3]
        seen = {tuple(round(v,4) for v in (r.candidate.vx,r.candidate.vy,r.candidate.wz)) for r in results
                if r.candidate.follow_vx is None}
        for step in (.12,.06):
            for result in seeds:
                for dx,dy in ((-step,0.),(step,0.),(0.,-step),(0.,step)):
                    c = result.candidate
                    command = self.motion_envelope.normalize((
                        self._clamp(c.vx+dx,-self.max_reverse_velocity,self.max_forward_near_human),
                        self._clamp(c.vy+dy,-self.max_lateral_near_human,self.max_lateral_near_human),0.))
                    key = tuple(round(v,4) for v in command)
                    if key not in seen:
                        seen.add(key)
                        yield Candidate('escape_refinement',*command)

    def _short_horizon_escape(self, results, tracks):
        """Time-checked one-second escape when holding predicts body contact.

        This is an explicitly labelled receding-horizon recovery, not a claim
        that a constant command remains safe for the entire prediction horizon.
        """
        stop = next((r for r in results if r.candidate.name == "yield_stop"), None)
        if stop is None or stop.reason != "hard_human_collision" or not tracks:
            return None
        best = None
        best_rank = None
        stop_points = np.asarray(stop.trajectory)
        for result in results:
            if (result is stop or result.reason == 'stale_policy_feedback' or
                    result.candidate.follow_vx is not None or result.candidate.schedule is not None):
                # Short recovery publishes one command, not an unverified
                # future phase. Timed plans require full-horizon admission.
                continue
            robot = np.asarray(result.trajectory)
            mask = robot[:, 3] <= 1.0 + 1e-6
            short = robot[mask]
            if not len(short) or self._static_obstacle_cost([tuple(p) for p in short])[1]:
                continue
            minimum = math.inf
            final_clearance = math.inf
            stop_clearance = math.inf
            forecast_clearance = math.inf
            stop_forecast_clearance = math.inf
            violation_time = math.inf
            safe = True
            for track in tracks:
                human, _, _ = self.prediction_cache[(track.track_id, tuple(robot[:, 3]))]
                h = human[mask, :2]
                radius = self.robot_radius + track.body_radius
                separation = np.linalg.norm(short[:, :2] - h, axis=1)
                relative = robot[:, :2] - human[:, :2]
                delta = np.diff(relative, axis=0)
                fraction = np.clip(-np.sum(relative[:-1]*delta, axis=1) /
                                   np.maximum(1e-12, np.sum(delta*delta, axis=1)), 0., 1.)
                swept = np.linalg.norm(relative[:-1] + fraction[:, None]*delta, axis=1)
                swept_times = robot[:-1, 3] + fraction*np.diff(robot[:, 3])
                short_minimum = float(np.min(separation))
                short_swept = swept[swept_times <= short[-1, 3]]
                if len(short_swept):
                    short_minimum = min(short_minimum, float(np.min(short_swept)))
                if short_minimum < radius + 0.10:
                    safe = False
                    break
                minimum = min(minimum, short_minimum)
                forecast_minimum = min(short_minimum, float(np.min(swept))) if len(swept) else short_minimum
                unsafe = swept < radius+.10
                if np.any(unsafe):
                    violation_time = min(violation_time,float(np.min(robot[:-1,3][unsafe])))
                if len(robot) > 1:
                    interval = max(1e-6, robot[-1, 3]-robot[-2, 3])
                    terminal = relative_motion_metrics(
                        robot[-1, 0], robot[-1, 1],
                        (robot[-1, 0]-robot[-2, 0])/interval, (robot[-1, 1]-robot[-2, 1])/interval,
                        human[-1, 0], human[-1, 1], human[-1, 2], human[-1, 3])
                    if terminal['closing'] and terminal['tcpa'] <= 2.*self.horizon:
                        forecast_minimum = min(forecast_minimum, terminal['dcpa'])
                        if terminal['dcpa'] < radius+.10:
                            violation_time = min(violation_time,float(robot[-1,3])+terminal['tcpa'])
                forecast_clearance = min(forecast_clearance, forecast_minimum-radius)
                stop_forecast_clearance = min(stop_forecast_clearance,
                    float(np.min(np.linalg.norm(stop_points[:, :2]-human[:, :2], axis=1)))-radius)
                final_clearance = min(final_clearance, float(separation[-1]) - radius)
                stop_clearance = min(stop_clearance, float(np.linalg.norm(stop_points[mask][-1, :2] - h[-1])) - radius)
            gain = final_clearance - stop_clearance
            if safe and (gain > .05 or forecast_clearance > stop_forecast_clearance + .05):
                # A faster person can catch a retreat that looks best at one
                # second. Prefer sustained separation across the full forecast
                # among short-safe recoveries, rather than short-term gain alone.
                full_static_clear = not self._static_obstacle_cost(result.trajectory)[1]
                if not full_static_clear and self.static_field_cache is not None:
                    violation_time = min(violation_time,self.static_field_cache.first_violation_time(
                        robot[:,:2],robot[:,3],self.static_clearance,recovery_radius=self.robot_radius))
                # Do not prefer an imminent person collision merely because an
                # alternative needs another turn before a much later wall.
                rank = (violation_time, forecast_clearance, gain)
                if best_rank is None or rank > best_rank:
                    best_rank = rank
                    best = CandidateResult(result.candidate, -forecast_clearance, True, "local_escape", [tuple(p) for p in short], minimum, 1.0, 0.0,
                                           {"clearance_gain": gain, "forecast_clearance": forecast_clearance,
                                            "violation_time": violation_time if math.isfinite(violation_time) else self.horizon*3.})
        return best

    def _fresh_tracks(self, now) -> list[HumanTrack]:
        now_seconds = now.nanoseconds * 1e-9
        if self.tracks_epoch is not None and 0.0 <= now_seconds - self.tracks_epoch <= self.max_person_cloud_age:
            return [align_track_to_time(track, now_seconds) for track in self.tracks.values()]
        if not self.use_legacy_person_inputs:
            return []
        return [
            align_track_to_time(track, now_seconds) for track in self.legacy_tracks.values()
            if 0.0 <= now_seconds - track.stamp <= self.max_person_cloud_age
        ]

    def _fresh_tentative(self, now) -> list[tuple[float, float, float]]:
        if not self.use_legacy_person_inputs or self.tentative_time is None:
            return []
        age = now.nanoseconds * 1e-9 - self.tentative_time
        if age < 0.0 or age > self.max_person_cloud_age:
            return []
        return self.tentative_points

    def _fresh_predicted(self, now) -> list[tuple[float, float, float]]:
        if not self.use_legacy_person_inputs or self.predicted_time is None:
            return []
        age = now.nanoseconds * 1e-9 - self.predicted_time
        if age < 0.0 or age > self.max_person_cloud_age:
            return []
        return self.predicted_points

    def _fresh_predictions(self, now) -> tuple[dict[int, list[PredictionSample]], float, bool]:
        self.forecast_source='track_cv_unavailable_prediction'
        if self.prediction_epoch is None:
            return {}, 0.0, False
        age = now.nanoseconds * 1e-9 - self.prediction_epoch
        if age < 0.0 or age > self.max_person_cloud_age:
            return {}, 0.0, False
        if (getattr(self,'forecast_change_guard_enabled',False) and
                (self.tracks_epoch is None or abs(self.prediction_epoch-self.tracks_epoch)>1e-6)):
            self.forecast_source='track_cv_unpaired_epochs'
            return {},0.,False
        self.forecast_source='typed_matched' if getattr(self,'forecast_change_guard_enabled',False) else 'typed'
        return self.predictions, age, True

    def _update_state(self, now, pose: tuple[float, float, float], tracks: list[HumanTrack], tentative_points) -> str:
        min_distance = math.inf
        hard_ttc = False
        hard_distance = False
        warning_ttc = False
        crossing_front = False
        for track in tracks:
            if track.state == "TENTATIVE":
                if math.hypot(track.x - pose[0], track.y - pose[1]) < self.comfort_radius + 1.0:
                    warning_ttc = True
            metrics = self._relative_metrics_for_raw_cmd(pose, track)
            min_distance = min(min_distance, metrics["distance"])
            hard_radius = self.robot_radius + track.body_radius + self.hard_safety_margin
            hard_distance = hard_distance or metrics["distance"] <= hard_radius
            hard_ttc = hard_ttc or ttc_is_relevant(metrics, self.ttc_hard_threshold, hard_radius)
            warning_ttc = warning_ttc or ttc_is_relevant(metrics, self.ttc_warning_threshold, self.dcpa_comfort_threshold)
            crossing_front = crossing_front or metrics["front_cut"]
        next_state = self.state
        forward_blocked = self._human_blocks_forward_path(pose, tracks)
        state_age = (now - self.state_since).nanoseconds * 1e-9
        blocked_age = (now - self.last_progress_time).nanoseconds * 1e-9
        if hard_distance or hard_ttc:
            next_state = "EMERGENCY_STOP"
        elif crossing_front:
            next_state = "PASS_BEHIND"
        elif min_distance <= self.comfort_radius or warning_ttc:
            if forward_blocked:
                next_state = blocked_motion_state(
                    self.state, state_age, blocked_age, self.yield_timeout, self.deadlock_timeout
                )
            else:
                next_state = "HUMAN_CAUTION"
        elif tentative_points:
            next_state = "HUMAN_CAUTION"
        else:
            next_state = "NORMAL_NAVIGATION"
            self.last_progress_time = now
        if next_state != self.state:
            self.state_since = now
        self.state = next_state
        if self.state in ("HUMAN_CAUTION", "PASS_BEHIND") and now > self.side_commitment_until:
            side = self._preferred_side(pose, tracks)
            if side != 0.0:
                self.side_commitment = side
                self.side_commitment_until = now + Duration(seconds=self.side_commitment_timeout)
        return self.state

    def _candidate_commands(
        self,
        raw: Twist,
        state: str,
        pose: tuple[float, float, float],
        tracks: list[HumanTrack],
    ) -> list[Candidate]:
        raw_x = self._clamp(float(raw.linear.x), -self.max_reverse_velocity, self.max_forward_near_human)
        raw_y = self._clamp(float(raw.linear.y), -self.max_lateral_near_human, self.max_lateral_near_human)
        clockwise = getattr(self, "max_clockwise_angular_near_human", 0.) or self.max_angular_near_human
        raw_w = self._clamp(float(raw.angular.z), -clockwise, self.max_angular_near_human)
        bounded_route = getattr(self, "prefer_forward_travel", False) and self.nav2_path_reference
        route_follow = 0.0 if bounded_route else None
        if state == "NORMAL_NAVIGATION":
            return [Candidate("raw_limited", raw_x, raw_y, raw_w, "", route_follow, .8)]
        if self.holonomic_avoidance and self.max_lateral_near_human > 0.0:
            # Keep the torso heading stable during close encounters. Translating
            # out of a crossing corridor is more predictable than a slow G1 yaw
            # transient. Nav2 heading tracking resumes once the conflict clears.
            incoming = incoming_collision_imminent(pose, tracks, self.horizon, self.robot_radius + self.human_body_radius + 0.5, 0.12)
            early = []
            if getattr(self, "prefer_forward_travel", False):
                people = [(t.x,t.y,t.vx,t.vy,self.robot_radius+t.body_radius+self.body_prediction_margin) for t in tracks]
                for bias in (0., -.25, .25):
                    command = velocity_obstacle_command(pose,self.measured_velocity,(raw_x,raw_y+bias),people,
                        (self.max_forward_near_human,self.max_reverse_velocity,self.max_lateral_near_human),
                        (self.motion_envelope.forward_gain,self.motion_envelope.lateral_gain,
                         self.motion_envelope.lateral_response_gain(-1.)),
                        horizon=self.horizon, side_hint=self.side_commitment)
                    if command is not None:
                        early.append(Candidate("velocity_obstacle",command[0],command[1],0.,"HUMAN_CAUTION"))
                for x, y in crossing_escape_directions(pose, [(t.x,t.y,t.vx,t.vy) for t in tracks],
                                                       clearance=self.robot_radius+self.human_body_radius+.6):
                    # Saturate along the perpendicular ray. Scaling each axis
                    # independently both slows diagonal escape and rotates it
                    # away from perpendicular when forward/lateral bounds differ.
                    x_limit = self.max_forward_near_human if x >= 0 else self.max_reverse_velocity
                    x /= max(.1,self.motion_envelope.forward_gain)
                    y /= max(.1,self.motion_envelope.lateral_response_gain(y))
                    scale = min(x_limit / max(abs(x), 1e-9),
                                self.max_lateral_near_human / max(abs(y), 1e-9))
                    early.append(Candidate("early_crossing_escape",
                        x*scale, y*scale, 0.0))
            active_request = max(abs(raw_x), abs(raw_y), abs(raw_w)) > 0.03
            forward = self.max_forward_near_human if incoming or active_request else 0.0
            route_yaw = raw_w if getattr(self, "prefer_forward_travel", False) else 0.0
            translations = [Candidate("raw_reference", raw_x, raw_y, route_yaw, "", route_follow, .8),
                            Candidate("previous_command", self.last_cmd.linear.x, self.last_cmd.linear.y,
                                      self.last_cmd.angular.z if getattr(self, "prefer_forward_travel", False) else 0.0)] + [
                Candidate("escape_translation" if incoming else "social_translation", vx, vy, 0.0)
                for vx in (-self.max_reverse_velocity, 0.0, 0.5 * forward, forward)
                for vy in (-self.max_lateral_near_human, 0.0, self.max_lateral_near_human)
                if abs(vx) + abs(vy) > 1e-6
            ]
            if bounded_route:
                translations.append(Candidate("route_continuation", raw_x, raw_y, route_yaw))
            # Search how long to translate, not just the direction. A six-second
            # constant retreat can hit someone that a short step-and-hold avoids.
            refinements = [Candidate("fine_translation", vx, vy, 0.0)
                           for vx in (-0.5 * self.max_reverse_velocity, 0.5 * forward, forward)
                            for vy in (-0.5 * self.max_lateral_near_human, 0.5 * self.max_lateral_near_human)]
            # A constant retreat or a step followed by stopping cannot represent
            # walking around a crossing person and then resuming the route.
            # Both phases are response-aware and checked over the full horizon.
            sequences = []
            if bounded_route and active_request:
                sequences = [Candidate("crossing_then_route", c.vx, c.vy, 0., "",
                                       raw_x, duration, raw_y, route_yaw)
                             for c in early if c.name == "early_crossing_escape"
                             for duration in (.6, 1.2)]
                sequences += [Candidate("escape_then_route", c.vx, c.vy, 0., "",
                                        raw_x, duration, raw_y, route_yaw)
                              for c in translations if c.name in ("escape_translation", "social_translation")
                              for duration in (.6, 1.2)]
                if self.wait_candidates_enabled:
                    sequences += [Candidate('wait_then_route',0.,0.,0.,'',raw_x,duration,raw_y,route_yaw)
                                  for duration in (.3,.6,1.2)]
                    sequences += [Candidate('wait_then_escape',0.,0.,0.,'',c.vx,duration,c.vy,0.)
                                  for c in translations if c.name in ('escape_translation','social_translation')
                                  for duration in (.6,1.2)]
            capture = self._goal_capture_candidates(pose, tracks) if bounded_route and self.goal_capture_enabled else []
            return [Candidate("yield_stop", 0.0, 0.0, 0.0, "YIELD_TO_HUMAN")] + early + translations + capture + sequences + refinements + [
                Candidate("step_and_hold", c.vx, c.vy, 0.0, "", 0.0, duration)
                for c in translations for duration in (.6, 1.2, 2.0)
            ]
        side = self.side_commitment if self.side_commitment != 0.0 else 1.0
        candidates = [
            Candidate("raw_limited", raw_x, raw_y, raw_w),
            Candidate("half_speed", raw_x * 0.5, raw_y * 0.5, raw_w * 0.5),
            Candidate("quarter_speed", raw_x * 0.25, raw_y * 0.25, raw_w * 0.25),
            Candidate("slow_forward", max(0.0, min(max(raw_x * 0.65, 0.18), self.max_forward_near_human)), 0.0, raw_w * 0.5),
            Candidate("creep", min(0.18, self.max_forward_near_human), 0.0, 0.0),
            Candidate("yield_stop", 0.0, 0.0, 0.0, "YIELD_TO_HUMAN"),
            Candidate("sidestep_left", 0.10, self.max_lateral_near_human, 0.0, "SIDESTEP_LEFT"),
            Candidate("sidestep_right", 0.10, -self.max_lateral_near_human, 0.0, "SIDESTEP_RIGHT"),
            Candidate("lateral_left", 0.0, self.max_lateral_near_human, 0.0, "SIDESTEP_LEFT"),
            Candidate("lateral_right", 0.0, -self.max_lateral_near_human, 0.0, "SIDESTEP_RIGHT"),
            Candidate("arc_left", 0.20, 0.0, self.max_angular_near_human * 0.65),
            Candidate("arc_right", 0.20, 0.0, -self.max_angular_near_human * 0.65),
            Candidate("turn_left", 0.0, 0.0, self.max_angular_near_human * 0.55),
            Candidate("turn_right", 0.0, 0.0, -self.max_angular_near_human * 0.55),
            Candidate("retreat", -self.max_reverse_velocity, 0.0, 0.0, "RETREAT_OR_BACKUP"),
        ]
        for speed in (raw_x, min(raw_x, 0.25)):
            for yaw in (-self.max_angular_near_human, -0.5 * self.max_angular_near_human, 0.5 * self.max_angular_near_human, self.max_angular_near_human):
                candidates.append(Candidate(f"sample_{speed:.2f}_{yaw:.2f}", speed, 0.0, yaw))
        if self.max_lateral_near_human > 0.0:
            for speed in (0.0, 0.5 * raw_x, raw_x):
                for lateral in (-self.max_lateral_near_human, self.max_lateral_near_human):
                    for yaw in (0.0, raw_w):
                        candidates.append(Candidate("velocity_grid", speed, lateral, yaw))
        away = self._move_away_candidate(pose, tracks)
        if away is not None:
            candidates.insert(1, away)
        if state in ("PASS_BEHIND", "HUMAN_CAUTION"):
            candidates.insert(1, Candidate("committed_side", 0.14, side * self.max_lateral_near_human, -side * self.max_angular_near_human * 0.25))
        if state == "DEADLOCK_RECOVERY":
            lateral = side * self.max_lateral_near_human
            candidates = [
                Candidate("deadlock_sidestep", 0.0, lateral, -side * self.max_angular_near_human * 0.25),
                Candidate("deadlock_retreat", -self.max_reverse_velocity, 0.5 * lateral, 0.0, "RETREAT_OR_BACKUP"),
                Candidate("deadlock_turn", 0.0, 0.0, -side * self.max_angular_near_human * 0.55),
                Candidate("yield_stop", 0.0, 0.0, 0.0, "YIELD_TO_HUMAN"),
            ]
        # A stopped robot is not safe if a person is walking into it. Keep
        # executable escape candidates available even in emergency/yield mode.
        for candidate in candidates:
            candidate.vx = no_unrequested_forward(candidate.vx, raw_x)
            if raw_x >= 0.0:
                candidate.vx = min(candidate.vx, raw_x)
        if tracks and incoming_collision_imminent(pose, tracks, self.horizon, self.robot_radius + self.human_body_radius + 0.5, 0.12):
            # Active avoidance may need more speed than a blocked Nav2 command.
            # Every escape is still checked against people and static geometry.
            for yaw in (-self.max_angular_near_human, 0.0, self.max_angular_near_human):
                candidates.append(Candidate("escape_arc", self.max_forward_near_human, 0.0, yaw, "CLEARANCE_RECOVERY"))
                for lateral in (-self.max_lateral_near_human, self.max_lateral_near_human):
                    candidates.append(Candidate("escape_diagonal", self.max_forward_near_human, lateral, yaw, "CLEARANCE_RECOVERY"))
                for duration in (1.0, 2.0):
                    candidates.append(Candidate("escape_turn_then_walk", 0.0, 0.0, yaw, "CLEARANCE_RECOVERY", self.max_forward_near_human, duration))
        return candidates

    def _goal_capture_candidates(self, pose, tracks):
        """Stop inside the goal region on its clear side, not necessarily its centre.

        Goal identity and acceptance are unchanged. The full simulated approach
        and stopping tail are scored normally against people and static geometry.
        """
        goal = self.reference_goal_world
        if goal is None or not .4 < math.hypot(goal[0]-pose[0],goal[1]-pose[1]) <= 1.5:
            return []
        standing = [t for t in tracks if math.hypot(t.vx,t.vy) < .5 and
                    math.hypot(t.x-goal[0],t.y-goal[1]) < self.stationary_person_clearance+.5]
        if not standing:
            return []
        targets = [(goal[0]+.42*math.cos(angle),goal[1]+.42*math.sin(angle))
                   for angle in np.linspace(0.,2.*math.pi,16,endpoint=False)]
        targets.sort(key=lambda p: (min(math.hypot(p[0]-t.x,p[1]-t.y) for t in standing),
                                    -math.hypot(p[0]-pose[0],p[1]-pose[1])), reverse=True)
        c,s = math.cos(pose[2]),math.sin(pose[2])
        candidates = []
        for x,y in targets[:3]:
            dx,dy = x-pose[0],y-pose[1]
            distance = math.hypot(dx,dy)
            if distance < .05:
                continue
            vx = self.path_reference_speed*(c*dx+s*dy)/distance/max(.1,self.motion_envelope.forward_gain)
            local_y = -s*dx+c*dy
            vy = self.path_reference_speed*local_y/distance/max(.1,self.motion_envelope.lateral_response_gain(local_y))
            scale = min(1., (self.max_forward_near_human if vx >= 0. else self.max_reverse_velocity)/max(1e-9,abs(vx)),
                        self.max_lateral_near_human/max(1e-9,abs(vy)))
            vx,vy,_ = self.motion_envelope.normalize((vx*scale,vy*scale,0.))
            duration = min(max(.1,self.horizon-.5), math.ceil(
                (distance/max(.1,self.path_reference_speed)+self.response_delay+.25)/self.dt)*self.dt)
            low,high = 0.,1.
            for _ in range(8):
                scale = (low+high)*.5
                candidate = Candidate('goal_capture',vx*scale,vy*scale,0.,'',0.,duration)
                end = self._simulate(pose,candidate)[-1]
                progress = ((end[0]-pose[0])*dx+(end[1]-pose[1])*dy)/distance
                if progress < distance:
                    low = scale
                else:
                    high = scale
            candidates.append(Candidate('goal_capture',vx*low,vy*low,0.,'',0.,duration))
        return candidates

    def _executable_candidates(self, candidates: list[Candidate]) -> list[Candidate]:
        result = []
        seen = set()
        for candidate in candidates:
            command = self.motion_envelope.normalize((candidate.vx, candidate.vy, candidate.wz), stopped=max(abs(self.last_cmd.linear.x), abs(self.last_cmd.linear.y), abs(self.last_cmd.angular.z)) <= 1e-6)
            schedule = candidate.schedule
            if schedule is not None:
                schedule = ((schedule[0][0],*command),*schedule[1:])
            # A requested turn may require the configured forward arc floor.
            # Score that executable arc rather than discarding Nav2's turn.
            # An all-zero stop remains zero in MotionEnvelope.normalize().
            # Numerically identical projections can differ by roundoff. Avoid
            # spending separate rollout budgets on indistinguishable commands.
            key = (tuple(None if value is None else round(value,4) for value in
                         (*command,candidate.follow_vx,candidate.follow_vy,candidate.follow_wz,candidate.switch_time)),
                   None if schedule is None else tuple(tuple(round(v,4) for v in step) for step in schedule))
            if key in seen:
                continue
            seen.add(key)
            result.append(Candidate(candidate.name, *command, candidate.preferred_state, candidate.follow_vx,
                                    candidate.switch_time, candidate.follow_vy, candidate.follow_wz,schedule))
        return result

    def _move_away_candidate(self, pose: tuple[float, float, float], tracks: list[HumanTrack]) -> Candidate | None:
        if not tracks:
            return None
        robot_x, robot_y, yaw = pose
        nearest = None
        nearest_distance = math.inf
        nearest_local_y = 0.0
        for track in tracks:
            dx = track.x - robot_x
            dy = track.y - robot_y
            local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
            local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            distance = math.hypot(dx, dy)
            if -0.4 <= local_x <= 3.0 and distance < nearest_distance:
                nearest = track
                nearest_distance = distance
                nearest_local_y = local_y
        if nearest is None or nearest_distance > max(2.8, self.comfort_radius * 2.0):
            return None
        if abs(nearest_local_y) < 0.18 and self.side_commitment != 0.0:
            side = self.side_commitment
        else:
            side = -1.0 if nearest_local_y >= 0.0 else 1.0
        return Candidate(
            "move_away_from_person",
            min(0.18, self.max_forward_near_human),
            side * self.max_lateral_near_human,
            -side * self.max_angular_near_human * 0.25,
            "SIDESTEP_LEFT" if side > 0.0 else "SIDESTEP_RIGHT",
        )

    def _select_local_safe_space_command(
        self,
        pose: tuple[float, float, float],
        tracks: list[HumanTrack],
        tentative_points: list[tuple[float, float, float]],
        predicted_points: list[tuple[float, float, float]],
        predictions: dict[int, list[PredictionSample]],
        prediction_age: float,
    ) -> tuple[CandidateResult | None, list[CandidateResult]]:
        danger_points = self._danger_zone_points(
            tracks, tentative_points, predicted_points, predictions, prediction_age
        )
        if not self._local_danger_present(pose, danger_points):
            return None, []

        speed = max(0.05, min(self.safe_space_max_speed, self.max_forward_near_human))
        lateral = max(0.0, min(self.safe_space_max_lateral, self.max_lateral_near_human))
        yaw = max(0.10, min(self.safe_space_max_yaw, self.max_angular_near_human))
        candidates = [
            Candidate("safe_hold", 0.0, 0.0, 0.0),
            Candidate("safe_retreat", -min(speed, self.max_reverse_velocity), 0.0, 0.0, "RETREAT_OR_BACKUP"),
            Candidate("safe_left", 0.0, lateral, 0.0, "SIDESTEP_LEFT"),
            Candidate("safe_right", 0.0, -lateral, 0.0, "SIDESTEP_RIGHT"),
            Candidate("safe_back_left", -min(speed, self.max_reverse_velocity), lateral, yaw * 0.35, "RETREAT_OR_BACKUP"),
            Candidate("safe_back_right", -min(speed, self.max_reverse_velocity), -lateral, -yaw * 0.35, "RETREAT_OR_BACKUP"),
            Candidate("safe_forward_left", speed, lateral, yaw * 0.45, "SIDESTEP_LEFT"),
            Candidate("safe_forward_right", speed, -lateral, -yaw * 0.45, "SIDESTEP_RIGHT"),
            Candidate("safe_arc_left", speed * 0.7, 0.0, yaw, "SIDESTEP_LEFT"),
            Candidate("safe_arc_right", speed * 0.7, 0.0, -yaw, "SIDESTEP_RIGHT"),
        ]
        patch = self._nearest_clear_patch(pose, danger_points)
        if patch is not None:
            self.selected_safe_patch = patch
            dx = patch[0] - pose[0]
            dy = patch[1] - pose[1]
            local_x = math.cos(pose[2]) * dx + math.sin(pose[2]) * dy
            local_y = -math.sin(pose[2]) * dx + math.cos(pose[2]) * dy
            distance = max(1e-6, math.hypot(local_x, local_y))
            heading = math.atan2(local_y, local_x)
            candidates.insert(
                0,
                Candidate(
                    "nearest_clear_patch",
                    speed * local_x / distance,
                    lateral * local_y / distance,
                    self._clamp(heading, -yaw, yaw),
                    "LOCAL_SAFE_SPACE",
                ),
            )
        results = [self._score_safe_space_candidate(candidate, pose, danger_points) for candidate in candidates]
        patch_result = next(
            (result for result in results if result.candidate.name == "nearest_clear_patch" and not result.rejected),
            None,
        )
        if patch_result is not None:
            return patch_result, results
        feasible = [result for result in results if not result.rejected]
        if feasible:
            return min(feasible, key=lambda result: result.score), results
        return None, results

    def _danger_zone_points(
        self,
        tracks: list[HumanTrack],
        tentative_points: list[tuple[float, float, float]],
        predicted_points: list[tuple[float, float, float]],
        predictions: dict[int, list[PredictionSample]],
        prediction_age: float,
    ) -> list[tuple[float, float, float]]:
        points = [(x, y, self.predicted_danger_radius) for x, y, _ in predicted_points]
        points.extend((x, y, self.tentative_caution_radius) for x, y, _ in tentative_points)
        for samples in predictions.values():
            points.extend(
                (sample.x, sample.y, self.predicted_danger_radius)
                for sample in samples
                if prediction_age <= sample.time_offset <= prediction_age + self.horizon
            )
        steps = max(1, int(self.horizon / max(0.05, self.dt)))
        for track in tracks:
            if track.track_id in predictions:
                continue
            base_radius = self._comfort_radius(track, 0.0)
            for step in range(steps + 1):
                t = step * self.dt
                radius = min(self.local_safe_radius, base_radius + self.uncertainty_growth_rate * t)
                points.append((track.x + track.vx * t, track.y + track.vy * t, radius))
        return points

    def _nearest_clear_patch(
        self,
        pose: tuple[float, float, float],
        danger_points: list[tuple[float, float, float]],
    ) -> tuple[float, float] | None:
        static_points = []
        now = self.get_clock().now()
        scan_time = None if self.latest_scan_time is None else self.latest_scan_time.nanoseconds * 1e-9
        if self.latest_scan is None or not scan_is_fresh(scan_time, now.nanoseconds * 1e-9, self.max_scan_age):
            return None
        for index, distance in enumerate(self.latest_scan.ranges):
            if not math.isfinite(distance) or distance <= self.latest_scan.range_min:
                continue
            angle = pose[2] + self.latest_scan.angle_min + index * self.latest_scan.angle_increment
            static_points.append((pose[0] + distance * math.cos(angle), pose[1] + distance * math.sin(angle)))
        raw_x = float(self.latest_cmd.linear.x)
        raw_y = float(self.latest_cmd.linear.y)
        preferred_heading = pose[2]
        if math.hypot(raw_x, raw_y) > 1e-3:
            preferred_heading += math.atan2(raw_y, raw_x)
        return nearest_clear_patch(
            (pose[0], pose[1]),
            danger_points,
            static_points,
            self.safe_patch_search_radius,
            self.safe_patch_spacing,
            self.safe_patch_clearance,
            preferred_heading,
        )

    def _local_danger_present(self, pose: tuple[float, float, float], danger_points: list[tuple[float, float, float]]) -> bool:
        if not danger_points:
            return False
        robot_x, robot_y, _ = pose
        for x, y, radius in danger_points:
            if math.hypot(robot_x - x, robot_y - y) <= max(self.local_danger_radius, radius):
                return True
        return False

    def _score_safe_space_candidate(
        self,
        candidate: Candidate,
        pose: tuple[float, float, float],
        danger_points: list[tuple[float, float, float]],
    ) -> CandidateResult:
        trajectory = self._simulate(pose, candidate)
        min_clearance = math.inf
        final_clearance = -math.inf
        initial_clearance = -math.inf
        first_safe_time = math.inf
        path_length = 0.0
        last_xy = None
        for x, y, _, t in trajectory:
            if last_xy is not None:
                path_length += math.hypot(x - last_xy[0], y - last_xy[1])
            last_xy = (x, y)
            clearance = min(math.hypot(x - px, y - py) - radius for px, py, radius in danger_points)
            if not math.isfinite(initial_clearance):
                initial_clearance = clearance
            min_clearance = min(min_clearance, clearance)
            final_clearance = clearance
            if not math.isfinite(first_safe_time) and clearance >= self.local_safe_radius:
                first_safe_time = t
        static_cost, static_reject = self._static_obstacle_cost(trajectory)
        if static_reject:
            return CandidateResult(candidate, math.inf, True, "static_obstacle", trajectory, max(0.0, min_clearance), math.inf, 0.0, {})
        if not math.isfinite(first_safe_time):
            if candidate.name == "nearest_clear_patch" and final_clearance >= initial_clearance + 0.10:
                score_terms = {
                    "clearance_progress": -(final_clearance - initial_clearance),
                    "static": self.weights["w_static"] * static_cost,
                    "path_length": 0.35 * path_length,
                }
                return CandidateResult(
                    candidate,
                    sum(score_terms.values()),
                    False,
                    "",
                    trajectory,
                    max(0.0, min_clearance),
                    math.inf,
                    0.0,
                    score_terms,
                )
            speed_cost = 0.15 * (abs(candidate.vx) + abs(candidate.vy))
            yaw_cost = 0.08 * abs(candidate.wz)
            score = 100.0 - final_clearance + 0.25 * path_length + speed_cost + yaw_cost
            return CandidateResult(
                candidate,
                score,
                True,
                "no_safe_space_reached",
                trajectory,
                final_clearance,
                math.inf,
                0.0,
                {"final_clearance": final_clearance, "min_clearance": min_clearance},
            )
        speed_cost = 0.15 * (abs(candidate.vx) + abs(candidate.vy))
        yaw_cost = 0.08 * abs(candidate.wz)
        score_terms = {
            "time_to_safe": first_safe_time,
            "path_length": 0.35 * path_length,
            "static": self.weights["w_static"] * static_cost,
            "speed": speed_cost,
            "yaw": yaw_cost,
            "clearance": -0.25 * min_clearance,
        }
        score = sum(score_terms.values())
        return CandidateResult(candidate, score, False, "", trajectory, max(0.0, min_clearance), math.inf, 0.0, score_terms)

    def _score_candidate(
        self,candidate,pose,tracks,tentative_points,predicted_points,predictions,prediction_age,prediction_margin=None,
    ):
        return self._timed_scoring('candidate_total_ms',self._score_candidate_impl,
            candidate,pose,tracks,tentative_points,predicted_points,predictions,prediction_age,prediction_margin)

    def _timed_scoring(self,label,callback,*args):
        started = time.perf_counter()
        try:
            return callback(*args)
        finally:
            self.scoring_phases[label] = self.scoring_phases.get(label,0.)+(time.perf_counter()-started)*1000.

    def _score_candidate_impl(
        self,
        candidate: Candidate,
        pose: tuple[float, float, float],
        tracks: list[HumanTrack],
        tentative_points: list[tuple[float, float, float]],
        predicted_points: list[tuple[float, float, float]],
        predictions: dict[int, list[PredictionSample]],
        prediction_age: float,
        prediction_margin: float | None = None,
    ) -> CandidateResult:
        trajectory = self._timed_scoring('rollout_ms',self._simulate,pose,candidate)
        if candidate.schedule is not None and (self.policy_command_stamp is None or
                not 0. <= self.get_clock().now().nanoseconds*1e-9-self.policy_command_stamp <= .4):
            return CandidateResult(candidate,math.inf,True,'stale_policy_feedback',trajectory,math.inf,math.inf,0.,{})
        min_human_distance = math.inf
        min_tcpa = math.inf
        personal_space_cost = 0.0
        onset_cost = 0.0
        enclosure_people = []
        ttc_cost = 0.0
        front_cut_cost = 0.0
        yield_cost = 0.0
        margin_violation = False
        body_collision_time = math.inf
        terminal_conflict = 0.0
        overtaking_conflict = 0.0
        robot_points = np.asarray(trajectory)
        times = robot_points[:, 3]
        time_key = tuple(times)
        sample_at=continuous_prediction if getattr(self,'forecast_change_guard_enabled',False) else interpolate_prediction
        for track in tracks:
            track_hard_radius = self.robot_radius + track.body_radius + self.hard_safety_margin
            margin = getattr(self,"body_prediction_margin",.05) if prediction_margin is None else max(.1,prediction_margin)
            body_distance = self.robot_radius + track.body_radius + margin
            initial_distance = math.hypot(pose[0] - track.x, pose[1] - track.y)
            samples = predictions.get(track.track_id)
            cache_key = (track.track_id, time_key)
            if cache_key not in self.prediction_cache:
                values = []
                for t in times:
                    sample = sample_at(samples, prediction_age + t) if samples else None
                    uncertainty_time = min(t,self.body_uncertainty_horizon)
                    uncertainty_sample = sample_at(samples,prediction_age+uncertainty_time) if samples else None
                    sigma = (math.sqrt(max(0.,uncertainty_sample.cov_xx,uncertainty_sample.cov_yy)) if uncertainty_sample else
                             math.sqrt(max(0.,track.cov_xx,track.cov_yy))+.25*uncertainty_time)
                    values.append((track.x + track.vx * t, track.y + track.vy * t, track.vx, track.vy, self._comfort_radius(track, t),sigma) if sample is None else
                                  (sample.x, sample.y, sample.vx, sample.vy, self._prediction_comfort_radius(track, sample),sigma))
                human = np.asarray(values)
                angles = np.arctan2(human[:, 3], human[:, 2])[:, None] + np.asarray((-1.0, -0.5, 0.0, 0.5, 1.0)) * self.front_arc_angle
                arc_x = human[:, 0, None] + max(0.4, self.front_arc_distance) * np.cos(angles)
                arc_y = human[:, 1, None] + max(0.4, self.front_arc_distance) * np.sin(angles)
                self.prediction_cache[cache_key] = human, arc_x, arc_y
            human, arc_x, arc_y = self.prediction_cache[cache_key]
            body_distances = body_distance+self.body_uncertainty_scale*human[:,5]
            distances = np.hypot(robot_points[:, 0] - human[:, 0], robot_points[:, 1] - human[:, 1])
            relevant = times <= 1.5 if track.state == "TENTATIVE" else np.ones(len(times), dtype=bool)
            track_min_distance = float(np.min(distances[relevant]))
            min_human_distance = min(min_human_distance, track_min_distance)
            required_distance = np.maximum(body_distances, min(track_hard_radius, initial_distance - 0.02))
            hard_window = relevant & (times <= self.hard_collision_horizon)
            physical_distance = self.robot_radius+track.body_radius+.05
            hits = (hard_window & (distances < body_distances)) | (relevant & (distances < physical_distance))
            if np.any(hits):
                body_collision_time = min(body_collision_time, float(times[np.flatnonzero(hits)[0]]))
            margin_violation = margin_violation or bool(np.any(hard_window & (distances < required_distance)))
            # Swept relative segments avoid missing a crossing between samples.
            relative = robot_points[:, :2] - human[:, :2]
            if self.weights['w_enclosure'] > 0.:
                influence = np.clip((3.-distances)/max(.1,3.-body_distance),0.,1.) * relevant * track.confidence
                enclosure_people.append((relative/np.maximum(1e-6,distances)[:,None],influence))
            delta = np.diff(relative, axis=0)
            fraction = np.clip(-np.sum(relative[:-1] * delta, axis=1) / np.maximum(1e-12, np.sum(delta * delta, axis=1)), 0.0, 1.0)
            swept = np.linalg.norm(relative[:-1] + fraction[:, None] * delta, axis=1)
            swept_times = times[:-1] + fraction * np.diff(times)
            swept_relevant = swept_times <= (1.5 if track.state == "TENTATIVE" else self.horizon)
            if np.any(swept_relevant):
                track_min_distance = min(track_min_distance, float(np.min(swept[swept_relevant])))
                min_human_distance = min(min_human_distance, track_min_distance)
            swept_window = swept_relevant & (swept_times <= self.hard_collision_horizon)
            # Use the larger endpoint reserve on each swept segment, so growth
            # of the uncertainty envelope cannot hide a between-sample conflict.
            swept_body_distances = np.maximum(body_distances[:-1],body_distances[1:])
            swept_hits = (swept_window & (swept < swept_body_distances)) | (swept_relevant & (swept < physical_distance))
            if np.any(swept_hits):
                body_collision_time = min(body_collision_time, float(np.min(swept_times[swept_hits])))
            swept_required = np.maximum(required_distance[:-1],required_distance[1:])
            margin_violation = margin_violation or bool(np.any(swept_window & (swept < swept_required)))
            weights = self.dt * np.exp(-self.social_cost_decay * times) * relevant
            if track.state == "TENTATIVE":
                weights *= track.confidence
            if math.hypot(track.vx, track.vy) < .5 and self.stationary_person_clearance > body_distance:
                # Unknown walking intent warrants extra room, but this preference
                # must not make an otherwise safe goal region unreachable.
                onset_cost += float(np.sum(weights * (np.maximum(0., self.stationary_person_clearance-distances) /
                                     (self.stationary_person_clearance-body_distance)) ** 2))
            personal_space_cost += float(np.sum(weights * np.maximum(0.0, np.exp(-0.5 * (distances / np.maximum(0.1, human[:, 4])) ** 2) - math.exp(-0.5))))
            if self.front_arc_risk_enabled:
                arc_distances = np.hypot(robot_points[:, 0, None] - arc_x, robot_points[:, 1, None] - arc_y)
                arc_cost = self.front_arc_weight * np.sum(np.exp(-0.5 * (arc_distances / np.maximum(0.1, human[:, 4, None] * 0.55)) ** 2), axis=1)
                personal_space_cost += float(np.sum(weights * arc_cost * (np.hypot(human[:, 2], human[:, 3]) >= self.human_intent_min_speed)))
            future = relevant & (times > self.hard_collision_horizon)
            personal_space_cost += float(np.sum(weights[future] * 2.0 * np.maximum(0.0, track_hard_radius - distances[future]) ** 2))
            metrics = self._relative_metrics_for_candidate(pose, candidate, track)
            if math.hypot(track.vx, track.vy) > max(.12, self.human_intent_min_speed) and len(trajectory) > 1:
                end, before = trajectory[-1], trajectory[-2]
                interval = max(1e-6, end[3] - before[3])
                terminal = relative_motion_metrics(end[0], end[1], (end[0] - before[0]) / interval, (end[1] - before[1]) / interval,
                                                   human[-1, 0], human[-1, 1], human[-1, 2], human[-1, 3])
                terminal_horizon = 2.0 * self.horizon if candidate.vx <= 0.0 and abs(candidate.vy) < 0.1 else 2.0
                base_terminal_horizon = terminal_horizon
                if (track.state=='CONFIRMED' and track.confidence>=.8 and
                        being_overtaken(end[:2],((end[0]-before[0])/interval,(end[1]-before[1])/interval),
                                        human[-1,:2],human[-1,2:4])):
                    # A forward command can be a losing retreat too. Preserve
                    # escape space before a faster walker catches the robot,
                    # rather than limiting this longer lookahead to reverse vx.
                    terminal_horizon = max(terminal_horizon,self.overtaking_terminal_horizon)
                if terminal["closing"] and terminal["dcpa"] < self.robot_radius + track.body_radius + 0.05 and terminal["tcpa"] < terminal_horizon:
                    if terminal['tcpa']>=base_terminal_horizon:
                        overtaking_conflict += 50.0
                    else:
                        terminal_conflict += 50.0
            min_tcpa = min(min_tcpa, metrics["tcpa"])
            # Instantaneous-command CPA is only a directional hint: it assumes
            # immediate acceleration and never brakes. Weight its soft penalties
            # by the clearance of the executable, swept rollout so safe short
            # steps do not lose to indefinite waiting on a phantom collision.
            proximity = max(0., min(1., (self.dcpa_comfort_threshold - track_min_distance) /
                                   max(.1, self.dcpa_comfort_threshold - body_distance))) ** 2
            if metrics["dcpa"] < self.dcpa_comfort_threshold and metrics["tcpa"] < self.ttc_warning_threshold:
                ttc_cost += proximity * ((self.ttc_warning_threshold - metrics["tcpa"]) / self.ttc_warning_threshold) ** 2
            if metrics["front_cut"] and metrics["tcpa"] <= self.horizon:
                front_cut_cost += proximity * math.exp(-metrics["dcpa"] / max(0.1, self.dcpa_comfort_threshold))
            if (self.state == "YIELD_TO_HUMAN" and candidate.vx > 0.03 and
                    not (self.nav2_path_reference and getattr(self, "prefer_forward_travel", False))):
                yield_cost += 1.0
        for tx, ty, _ in tentative_points:
            for x, y, _, _ in trajectory[:4]:
                if math.hypot(x - tx, y - ty) < self.tentative_caution_radius:
                    personal_space_cost += 0.35
        for px, py, _ in predicted_points:
            for x, y, _, t in trajectory:
                distance = math.hypot(x - px, y - py)
                hard_radius = self.robot_radius + self.human_body_radius + self.hard_safety_margin
                if distance < hard_radius:
                    return CandidateResult(candidate, math.inf, True, "predicted_danger_zone", trajectory, distance, min_tcpa, front_cut_cost, {})
                if distance < self.predicted_danger_radius:
                    personal_space_cost += math.exp(-0.5 * (distance / max(0.1, self.predicted_danger_radius)) ** 2) * (1.0 + 0.2 * t)
        if math.isfinite(body_collision_time):
            return CandidateResult(candidate, math.inf, True, "hard_human_collision", trajectory, min_human_distance, body_collision_time, front_cut_cost, {})
        static_cost, static_reject = self._timed_scoring('static_ms',self._static_obstacle_cost,trajectory)
        if static_reject:
            return CandidateResult(candidate, math.inf, True, "static_obstacle", trajectory, min_human_distance, min_tcpa, front_cut_cost, {})
        # Preserve room to escape: individually clear trajectories can still
        # enter a squeeze between people approaching from opposite sides.
        enclosure = np.zeros(len(times))
        for index,(direction,influence) in enumerate(enclosure_people):
            for other,other_influence in enclosure_people[index+1:]:
                enclosure += influence*other_influence*np.maximum(0.,-np.sum(direction*other,axis=1))
        enclosure_cost = float(np.sum(enclosure*self.dt*np.exp(-.25*times)))
        progress = self._goal_progress_cost(candidate)
        if self.nav2_path_reference and self.reference_goal_world is not None:
            endpoint = min(trajectory, key=lambda point: abs(point[3] - self.command_duration))
            # Progress must follow the admitted route around people. Measuring
            # only straight-line distance to the final goal penalized necessary
            # sidesteps and made a stopped command a persistent local minimum.
            gx, gy = getattr(self, "reference_progress_target", None) or self.reference_goal_world
            target_distance = math.hypot(endpoint[0] - gx, endpoint[1] - gy)
            starting_distance = math.hypot(pose[0]-gx,pose[1]-gy)
            if math.hypot(gx-self.reference_goal_world[0], gy-self.reference_goal_world[1]) < .01:
                # Arrival is an event, not an obligation to remain at the goal
                # for the rest of a rollout. Credit a safe trajectory that
                # reaches it and keeps moving clear of an approaching person.
                target_distance = min(math.hypot(x-gx, y-gy) for x,y,_,t in trajectory if t <= endpoint[3])
                # Reaching the internal arrival region earns full progress;
                # penetrating farther toward its centre earns no extra reward.
                target_distance = max(0.,target_distance-self.goal_cost_radius)
                starting_distance = max(0.,starting_distance-self.goal_cost_radius)
            progress = (target_distance - starting_distance) / max(0.1, endpoint[3])
        path = self._path_alignment_cost(candidate)
        smooth = (candidate.vx - float(self.last_cmd.linear.x)) ** 2 + (candidate.vy - float(self.last_cmd.linear.y)) ** 2
        jerk = smooth + 0.25 * (candidate.wz - float(self.last_cmd.angular.z)) ** 2
        legibility = self._side_switch_cost(candidate)
        deadlock = 1.0 if self.state == "DEADLOCK_RECOVERY" and candidate.name == "yield_stop" else 0.0
        score_terms = {
            "goal": self.weights["w_goal"] * self.goal_urgency * progress,
            "path": self.weights["w_path"] * path,
            "static": self.weights["w_static"] * static_cost,
            "personal_space": self.weights["w_personal_space"] * personal_space_cost,
            "onset": self.stationary_clearance_weight * onset_cost,
            "enclosure": self.weights['w_enclosure'] * enclosure_cost,
            "ttc": self.weights["w_ttc"] * ttc_cost,
            "pass_front": self.weights["w_pass_front"] * front_cut_cost,
            "yield": self.weights["w_yield"] * yield_cost,
            "smooth": self.weights["w_smooth"] * smooth,
            "jerk": self.weights["w_jerk"] * jerk,
            "legibility": self.weights["w_legibility"] * legibility,
            "deadlock": self.weights["w_deadlock"] * deadlock,
            "terminal_conflict": self.weights["w_deadlock"] * terminal_conflict,
            "overtaking_conflict": self.weights["w_deadlock"] * overtaking_conflict,
        }
        score = (
            score_terms["goal"]
            + score_terms["path"]
            + score_terms["static"]
            + score_terms["personal_space"]
            + score_terms["onset"]
            + score_terms["enclosure"]
            + score_terms["ttc"]
            + score_terms["pass_front"]
            + score_terms["yield"]
            + score_terms["smooth"]
            + score_terms["jerk"]
            + score_terms["legibility"]
            + score_terms["deadlock"]
            + score_terms["terminal_conflict"]
            + score_terms["overtaking_conflict"]
        )
        reason = "terminal_conflict" if terminal_conflict or overtaking_conflict else ("human_margin" if margin_violation else "")
        return CandidateResult(candidate, score, bool(reason), reason, trajectory, min_human_distance, min_tcpa, front_cut_cost, score_terms)

    def _simulate(self, pose: tuple[float, float, float], candidate: Candidate) -> list[tuple[float, float, float, float]]:
        next_command = None if candidate.follow_vx is None else (candidate.follow_vx, candidate.follow_vy, candidate.follow_wz)
        duration = math.inf if next_command is None else candidate.switch_time
        filtered = self.policy_command if self.policy_command_stamp is not None and 0.0 <= self.get_clock().now().nanoseconds * 1e-9 - self.policy_command_stamp <= 0.4 else None
        if candidate.schedule is not None:
            initial = filtered if filtered is not None else self.motion_envelope.normalize(self.measured_velocity)
            # A freshly checked primitive includes its braking phase. Cutting
            # that phase off at the nominal horizon invents a moving endpoint
            # and can falsely reject its already-checked stopped refuge.
            horizon=max(self.horizon,sum(part[0] for part in candidate.schedule)) if candidate.name=='escape_primitive' else self.horizon
            return rollout_schedule(self.motion_envelope,pose,self.measured_velocity,initial,
                                    candidate.schedule,horizon,self.dt,self.response_delay)
        return self.motion_envelope.rollout(pose, (candidate.vx, candidate.vy, candidate.wz), self.measured_velocity, self.horizon, self.dt, self.response_delay, duration, next_command, filtered)

    def _static_obstacle_cost(self, trajectory: list[tuple[float, float, float, float]]) -> tuple[float, bool]:
        if self.static_frame_valid is False:
            return math.inf, True
        # Nav2 replans curved static routes continuously. Check a local command
        # interval plus braking, rather than an unchanged five-second arc that
        # Nav2 never requested. Human occupancy remains checked over the horizon.
        braking = max(abs(self.measured_velocity[0]) / max(0.01, self.motion_envelope.accel_x), abs(self.measured_velocity[1]) / max(0.01, self.motion_envelope.accel_y))
        static_horizon = (self.horizon if getattr(self, "prefer_forward_travel", False) else
                          max(self.command_duration, self.response_delay + braking + 0.3))
        trajectory = [point for point in trajectory if point[3] <= static_horizon]
        now = self.get_clock().now()
        scan_time = None if self.latest_scan_time is None else self.latest_scan_time.nanoseconds * 1e-9
        if self.latest_scan is None or not scan_is_fresh(scan_time, now.nanoseconds * 1e-9, self.max_scan_age):
            self.static_frame_valid = False
            moving = any(math.hypot(point[0] - trajectory[0][0], point[1] - trajectory[0][1]) > 1e-4 for point in trajectory[1:])
            return (math.inf, True) if moving else (0.0, False)
        key = tuple(trajectory)
        if key in self.static_cache:
            return self.static_cache[key]
        if self.use_static_scene_map and self.static_points_cache is None:
            if self.static_scene_points is None:
                self.static_frame_valid = False
                return math.inf, True
            extent = self.static_influence_range + self.horizon * self.motion_envelope.max_x
            nearby = np.linalg.norm(self.static_scene_points - trajectory[0][:2], axis=1) < extent
            self.static_points_cache = self.static_scene_points[nearby]
            self.static_frame_valid = True
        if self.static_points_cache is None:
            ranges = np.asarray(self.latest_scan.ranges, dtype=float)
            angles = self.latest_scan.angle_min + np.arange(len(ranges)) * self.latest_scan.angle_increment
            valid = np.isfinite(ranges) & (ranges > self.latest_scan.range_min) & (ranges < self.static_influence_range + self.horizon * self.motion_envelope.max_x)
            points = np.column_stack((ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid])))
            stamp = _ros_time_seconds(self.latest_scan.header.stamp)
            scan_pose = self.odom_pose_history.get(stamp) if self.latest_scan.header.frame_id == self.robot_frame else None
            if scan_pose is None and self.latest_scan.header.frame_id == self.robot_frame and self.odom_pose_history:
                nearest_stamp = min(self.odom_pose_history, key=lambda value: abs(value - stamp))
                offset = stamp - nearest_stamp
                if abs(offset) <= 0.15:
                    x, y, yaw = self.odom_pose_history[nearest_stamp]
                    vx, vy, wz = self.measured_velocity
                    scan_pose = (x + (math.cos(yaw) * vx - math.sin(yaw) * vy) * offset,
                                 y + (math.sin(yaw) * vx + math.cos(yaw) * vy) * offset, yaw + wz * offset)
            if scan_pose is None:
                try:
                    transform = self.tf_buffer.lookup_transform(self.target_frame, self.latest_scan.header.frame_id, rclpy.time.Time.from_msg(self.latest_scan.header.stamp))
                    q = transform.transform.rotation
                    scan_pose = (transform.transform.translation.x, transform.transform.translation.y,
                                 math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
                except TransformException as exc:
                    self.static_error = str(exc)
                    if self.static_scan_stamp is None or now.nanoseconds * 1e-9 - self.static_scan_stamp > self.max_scan_age:
                        self.static_frame_valid = False
                        return math.inf, True
            if scan_pose is not None:
                self.static_error = ""
                yaw = scan_pose[2]
                c, s = math.cos(yaw), math.sin(yaw)
                points = points @ np.asarray(((c, s), (-s, c)))
                points += scan_pose[:2]
                if self.static_scan_stamp != stamp:
                    keys = list(self.static_memory)
                    if keys:
                        old_points = np.asarray([self.static_memory[key][:2] for key in keys])
                        cleared = observed_free_mask(old_points, scan_pose, ranges, self.latest_scan.angle_min, self.latest_scan.angle_increment)
                        for key, free in zip(keys, cleared):
                            if free:
                                del self.static_memory[key]
                    for x, y in points:
                        if not any(math.hypot(x - track.x, y - track.y) <= track.body_radius + 0.10 for track in self.tracks.values()):
                            self.static_memory[(round(x / 0.10), round(y / 0.10))] = (x, y, stamp)
                    self.static_scan_stamp = stamp
                    self.static_memory = {key: value for key, value in self.static_memory.items() if 0.0 <= stamp - value[2] <= self.static_memory_seconds
                        and not any(math.hypot(value[0] - track.x, value[1] - track.y) <= track.body_radius + 0.10 for track in self.tracks.values())}
            # Maintain recently observed low obstacles when they move below the
            # sensor's vertical field of view. All points are in world, not the
            # offset/tilted LiDAR frame previously mistaken for base_footprint.
            origin_x, origin_y = trajectory[0][:2]
            remembered = list(self.static_memory.values()) + list(self.low_obstacle_memory.values())
            self.static_points_cache = np.asarray([(x, y) for x, y, _ in remembered
                if math.hypot(x - origin_x, y - origin_y) < self.static_influence_range + self.horizon * self.motion_envelope.max_x]).reshape((-1, 2))
            self.static_frame_valid = True
        obstacles = self.static_points_cache
        if len(obstacles) == 0:
            result = (0.0, False)
        else:
            positions = np.asarray(trajectory)[:, :2]
            if self.static_field_cache is None:
                if not hasattr(self,'static_field_reuse'):
                    self.static_field_reuse = StaticFieldCache()
                self.static_field_cache = self.static_field_reuse.get(obstacles,trajectory[0][:2],
                    self.static_influence_range+self.horizon*self.motion_envelope.max_x,self.static_influence_range)
            nearest = self.static_field_cache.query(positions)
            rejected = self.static_field_cache.trajectory_is_blocked(positions, self.static_clearance, nearest,
                                                                    recovery_radius=self.robot_radius)
            cost = float(np.sum(np.maximum(0.0, (self.static_influence_range - nearest) / self.static_influence_range) ** 2)) * self.dt
            result = (math.inf, True) if rejected else (cost, False)
        self.static_cache[key] = result
        return result

    def _relative_metrics_for_raw_cmd(self, pose: tuple[float, float, float], track: HumanTrack) -> dict[str, float | bool]:
        return self._relative_metrics_for_candidate(
            pose,
            Candidate("raw", float(self.latest_cmd.linear.x), float(self.latest_cmd.linear.y), float(self.latest_cmd.angular.z)),
            track,
        )

    def _relative_metrics_for_candidate(self, pose: tuple[float, float, float], candidate: Candidate, track: HumanTrack) -> dict[str, float | bool]:
        robot_x, robot_y, yaw = pose
        rvx = math.cos(yaw) * candidate.vx - math.sin(yaw) * candidate.vy
        rvy = math.sin(yaw) * candidate.vx + math.cos(yaw) * candidate.vy
        metrics = relative_motion_metrics(robot_x, robot_y, rvx, rvy, track.x, track.y, track.vx, track.vy)
        tcpa = float(metrics["tcpa"])
        dcpa = float(metrics["dcpa"])
        human_speed = math.hypot(track.vx, track.vy)
        robot_speed = math.hypot(rvx, rvy)
        crossing = False
        front_cut = False
        if bool(metrics["closing"]) and human_speed > self.human_intent_min_speed and robot_speed > 0.03:
            dot = (track.vx * rvx + track.vy * rvy) / max(1e-6, human_speed * robot_speed)
            angle = abs(math.acos(max(-1.0, min(1.0, dot))))
            crossing = self.crossing_angle_min < angle < math.pi - self.crossing_angle_min
            human_future_x = track.x + track.vx * tcpa
            human_future_y = track.y + track.vy * tcpa
            robot_future_x = robot_x + rvx * tcpa
            robot_future_y = robot_y + rvy * tcpa
            ahead = ((robot_future_x - human_future_x) * track.vx + (robot_future_y - human_future_y) * track.vy) / human_speed
            front_cut = crossing and ahead > 0.0 and dcpa < self.dcpa_comfort_threshold
        return {**metrics, "front_cut": front_cut}

    def _comfort_radius(self, track: HumanTrack, t: float) -> float:
        uncertainty = math.sqrt(max(0.0, track.cov_xx + track.cov_yy))
        uncertainty += self.uncertainty_growth_rate * t
        uncertainty += self.low_conf_uncertainty_gain * max(0.0, 1.0 - track.confidence)
        uncertainty += 0.08 * max(0, track.missed_count)
        if track.state == "OCCLUDED":
            uncertainty += self.occluded_risk_scale
        return self.comfort_radius + uncertainty

    def _prediction_comfort_radius(self, track: HumanTrack, sample: PredictionSample) -> float:
        uncertainty = math.sqrt(max(0.0, sample.cov_xx + sample.cov_yy))
        uncertainty += self.low_conf_uncertainty_gain * max(0.0, 1.0 - sample.confidence)
        uncertainty += 0.08 * max(0, track.missed_count)
        if track.state == "OCCLUDED":
            uncertainty += self.occluded_risk_scale
        return self.comfort_radius + uncertainty

    def _front_arc_risk_cost(self, robot_x: float, robot_y: float, track: HumanTrack, t: float, radius: float) -> float:
        return self._front_arc_risk_at(
            robot_x,
            robot_y,
            track.x + track.vx * t,
            track.y + track.vy * t,
            track.vx,
            track.vy,
            radius,
        )

    def _front_arc_risk_at(
        self, robot_x: float, robot_y: float, hx: float, hy: float,
        vx: float, vy: float, radius: float,
    ) -> float:
        if not self.front_arc_risk_enabled:
            return 0.0
        speed = math.hypot(vx, vy)
        if speed < 0.08:
            return 0.0
        heading = math.atan2(vy, vx)
        arc_radius = max(0.4, self.front_arc_distance)
        cost = 0.0
        for fraction in (-1.0, -0.5, 0.0, 0.5, 1.0):
            angle = heading + fraction * self.front_arc_angle
            ax = hx + arc_radius * math.cos(angle)
            ay = hy + arc_radius * math.sin(angle)
            distance = math.hypot(robot_x - ax, robot_y - ay)
            cost += self.front_arc_weight * math.exp(-0.5 * (distance / max(0.1, radius * 0.55)) ** 2)
        return cost

    def _goal_progress_cost(self, candidate: Candidate) -> float:
        raw_x = float(self.latest_cmd.linear.x)
        raw_y = float(self.latest_cmd.linear.y)
        norm = math.hypot(raw_x, raw_y)
        if norm < 1e-3:
            return 0.0
        progress = (candidate.vx * raw_x + candidate.vy * raw_y) / norm
        return -progress

    def _path_alignment_cost(self, candidate: Candidate) -> float:
        if (getattr(self, "prefer_forward_travel", False) and
                math.hypot(self.latest_cmd.linear.x, self.latest_cmd.linear.y) < .03):
            # A turn request has no translational direction. atan2(0, 0) used
            # to penalize a necessary reverse/side escape as a 180-degree path
            # violation, trapping the robot against room boundaries.
            return abs(candidate.wz-self.latest_cmd.angular.z) + .2*abs(candidate.vy)
        raw_angle = math.atan2(float(self.latest_cmd.linear.y), float(self.latest_cmd.linear.x))
        cand_angle = math.atan2(candidate.vy, candidate.vx) if math.hypot(candidate.vx, candidate.vy) > 1e-3 else raw_angle
        return abs(self._wrap_pi(cand_angle - raw_angle)) + abs(candidate.wz - float(self.latest_cmd.angular.z))

    def _side_switch_cost(self, candidate: Candidate) -> float:
        # Turning in place does not change which side of a person we pass.
        # Penalizing yaw sign could make an essential alignment turn costlier
        # than remaining stopped indefinitely behind an obstacle.
        side = candidate.vy
        if self.side_commitment == 0.0 or abs(side) < 1e-3:
            return 0.0
        return 1.0 if side * self.side_commitment < 0.0 else 0.0

    def _human_blocks_forward_path(self, pose: tuple[float, float, float], tracks: list[HumanTrack]) -> bool:
        robot_x, robot_y, yaw = pose
        for track in tracks:
            dx = track.x - robot_x
            dy = track.y - robot_y
            local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
            local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            if 0.0 < local_x < self.comfort_radius * 1.4 and abs(local_y) < self.comfort_radius * 0.55:
                return True
        return False

    def _preferred_side(self, pose: tuple[float, float, float], tracks: list[HumanTrack]) -> float:
        if not tracks:
            return 0.0
        robot_x, robot_y, yaw = pose
        target = getattr(self, "reference_progress_target", None)
        if getattr(self, "prefer_forward_travel", False) and target is not None:
            lateral = -math.sin(yaw)*(target[0]-robot_x)+math.cos(yaw)*(target[1]-robot_y)
            if abs(lateral) > .2:
                return math.copysign(1., lateral)
        nearest = min(tracks, key=lambda track: math.hypot(track.x - robot_x, track.y - robot_y))
        dx = nearest.x - robot_x
        dy = nearest.y - robot_y
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return -1.0 if local_y >= 0.0 else 1.0

    def _publish(self, cmd: Twist, state: str, results: list[CandidateResult], selected: CandidateResult | None) -> None:
        publishing_started = time.perf_counter()
        now = self.get_clock().now()
        tracks = self._fresh_tracks(now)
        closest_track = self._closest_track_debug(tracks)
        state_msg = String()
        state_msg.data = state
        self.state_pub.publish(state_msg)
        self.command_sequence_id += 1
        sequence = {'version': 1, 'publisher_id': self.command_publisher_id,
                    'sequence_id': self.command_sequence_id, 'state': state,
                    'stamp': now.nanoseconds*1e-9, 'frame': self.robot_frame,
                    'active': self.mission_active, 'first': [cmd.linear.x,cmd.linear.y,cmd.angular.z],
                    'duration': 0., 'then': None, 'goal': None}
        if self.reference_goal is not None:
            sequence['goal'] = goal_signature(self.reference_goal)
        if selected is not None and (not selected.rejected or selected.reason == 'human_margin') and selected.candidate.follow_vx is not None:
            candidate = selected.candidate
            remaining = candidate.switch_time
            if candidate.name in ('escape_pending','beam_pending') and self.escape_plan is not None:
                original,started,_ = self.escape_plan
                remaining = max(0.,started+original.switch_time-now.nanoseconds*1e-9)
            sequence.update(duration=remaining, then=[candidate.follow_vx,candidate.follow_vy,candidate.follow_wz])
        sequence_msg = String()
        sequence_msg.data = json.dumps(sequence,separators=(',',':'))
        self.sequence_pub.publish(sequence_msg)
        if self.selected_safe_patch is not None:
            patch_msg = PoseStamped()
            patch_msg.header.frame_id = self.target_frame
            patch_msg.header.stamp = now.to_msg()
            patch_msg.pose.position.x = self.selected_safe_patch[0]
            patch_msg.pose.position.y = self.selected_safe_patch[1]
            patch_msg.pose.orientation.w = 1.0
            self.safe_patch_pub.publish(patch_msg)
        self.cmd_pub.publish(cmd)
        command_published = self.get_clock().now()
        self.planning_phases["control_publish_ms"] = (time.perf_counter() - publishing_started) * 1000.0
        debug_pose = self._robot_pose()
        debug = {
            "command_published_ros_s": command_published.nanoseconds*1e-9,
            "planner_period_s": self.planner_period_s,
            "planning_budget_ms": self.planning_budget_ms,
            "scan_age_at_command_publish_s": self._age_seconds(self.latest_scan_time,command_published),
            "tracks_age_at_command_publish_s": None if self.tracks_epoch is None else command_published.nanoseconds*1e-9-self.tracks_epoch,
            "prediction_age_at_command_publish_s": None if self.prediction_epoch is None else command_published.nanoseconds*1e-9-self.prediction_epoch,
            "static_error": self.static_error,
            "static_memory_points": len(self.static_memory),
            "low_obstacle_points": len(self.low_obstacle_memory),
            "nearest_static": sorted(self.static_memory.values(), key=lambda p: math.hypot(p[0] - debug_pose[0], p[1] - debug_pose[1]))[:3] if self.static_memory and debug_pose is not None else [],
            "planning_ms": round((time.perf_counter() - self.planning_started) * 1000.0, 3),
            "search_budget_exhausted": self.search_budget_exhausted,
            "kinodynamic_status": self.kinodynamic_status,
            "forecast_source": getattr(self,'forecast_source','unavailable'),
            "heading_history": getattr(self,'heading_history_details',{}),
            "kinodynamic_details": getattr(self,'kinodynamic_details',{}),
            "strategic_avoidance": getattr(self,'strategic_status',dict(active=False)),
            "evaluated_candidates": len(results),
            "planning_thread_ms": round((time.thread_time() - self.planning_cpu_started) * 1000.0, 3),
            "planning_phases_ms": {key: round(value, 3) for key, value in self.planning_phases.items()},
            "scoring_phases_ms": {key: round(value, 3) for key, value in self.scoring_phases.items()},
            "tracks_age_s": None if self.tracks_epoch is None else round(now.nanoseconds * 1e-9 - self.tracks_epoch, 4),
            "prediction_age_s": None if self.prediction_epoch is None else round(now.nanoseconds * 1e-9 - self.prediction_epoch, 4),
            "measured_velocity": list(self.measured_velocity),
            "policy_command": self.policy_command,
            "state": state,
            "state_age_s": round((now - self.state_since).nanoseconds * 1e-9, 3),
            "selected": selected.candidate.name if selected else "none",
            "selected_reason": selected.reason if selected else "no_feasible_candidate",
            "raw_cmd": self._twist_dict(self.upstream_cmd if self.nav2_path_reference else self.latest_cmd),
            "reference_cmd": self._twist_dict(self.latest_cmd),
            "path_reference_status": self.path_reference_status,
            "mission_active": self.mission_active,
            "navigation_mode": self.navigation_mode,
            "payload_active": self.payload_active,
            "goal_urgency": self.goal_urgency,
            "goal_holonomic": self.goal_holonomic,
            "selected_cmd": self._twist_dict(cmd),
            "command_sequence": sequence,
            "score": round(selected.score, 3) if selected and math.isfinite(selected.score) else None,
            "rejected": sum(1 for result in results if result.rejected),
            "min_human_distance": round(selected.min_human_distance, 3) if selected and math.isfinite(selected.min_human_distance) else None,
            "min_tcpa": round(selected.min_tcpa, 3) if selected and math.isfinite(selected.min_tcpa) else None,
            "side_commitment": self.side_commitment,
            "input_age_s": self._age_seconds(self.latest_cmd_time, now),
            "scan_age_s": self._age_seconds(self.latest_scan_time, now),
            "track_count": len(tracks),
            "tentative_count": len(self._fresh_tentative(now)),
            "closest_track": closest_track,
            "candidate_results": [self._candidate_result_debug(result) for result in results],
        }
        debug_msg = String()
        debug_msg.data = json.dumps(debug, separators=(",", ":"), sort_keys=True)
        self.debug_pub.publish(debug_msg)
        if time.perf_counter() - self.last_visualization >= self.visualization_period:
            self.last_visualization = time.perf_counter()
            if self.marker_pub.get_subscription_count():
                self.marker_pub.publish(self._markers(results, selected, state))
            if self.publish_risk_grid and self.grid_pub.get_subscription_count():
                self.grid_pub.publish(self._risk_grid())

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

    def _closest_track_debug(self, tracks: list[HumanTrack]) -> dict[str, float | int | str] | None:
        pose = self._robot_pose()
        if pose is None or not tracks:
            return None
        robot_x, robot_y, _ = pose
        track = min(tracks, key=lambda item: math.hypot(item.x - robot_x, item.y - robot_y))
        metrics = self._relative_metrics_for_raw_cmd(pose, track)
        return {
            "track_id": track.track_id,
            "state": track.state,
            "x": round(track.x, 3),
            "y": round(track.y, 3),
            "vx": round(track.vx, 3),
            "vy": round(track.vy, 3),
            "confidence": round(track.confidence, 3),
            "person_score": round(track.person_score, 3),
            "distance": round(float(metrics["distance"]), 3),
            "tcpa": round(float(metrics["tcpa"]), 3) if math.isfinite(float(metrics["tcpa"])) else None,
            "dcpa": round(float(metrics["dcpa"]), 3),
            "front_cut": bool(metrics["front_cut"]),
        }

    def _candidate_result_debug(self, result: CandidateResult) -> dict[str, object]:
        data = {
            "name": result.candidate.name,
            "cmd": {"x": round(result.candidate.vx, 4), "y": round(result.candidate.vy, 4), "z": round(result.candidate.wz, 4)},
            "score": round(result.score, 3) if math.isfinite(result.score) else None,
            "rejected": result.rejected,
            "reason": result.reason,
            "min_human_distance": round(result.min_human_distance, 3) if math.isfinite(result.min_human_distance) else None,
            "min_tcpa": round(result.min_tcpa, 3) if math.isfinite(result.min_tcpa) else None,
            "front_cut_cost": round(result.front_cut_cost, 3),
            "command_duration_s": result.candidate.switch_time if result.candidate.follow_vx is not None else None,
            "schedule": result.candidate.schedule,
            "following_cmd": None if result.candidate.follow_vx is None else {
                "x": result.candidate.follow_vx, "y": result.candidate.follow_vy,
                "z": result.candidate.follow_wz},
        }
        if result.score_terms:
            data["score_terms"] = {key: round(value, 3) for key, value in result.score_terms.items()}
        return data

    def _markers(self, results: list[CandidateResult], selected: CandidateResult | None, state: str) -> MarkerArray:
        markers = MarkerArray()
        now = rclpy.time.Time().to_msg()  # Display with latest TF, not a future ego transform.
        alternatives = [result for result in results if result is not selected][:12]
        for index, result in enumerate(alternatives):
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = now
            marker.ns = "human_aware_candidates"
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.
            marker.scale.x = 0.015
            if result.rejected:
                marker.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.18)
            else:
                marker.color = ColorRGBA(r=0.25, g=0.55, b=1.0, a=0.22)
            for x, y, _, _ in result.trajectory:
                marker.points.append(Point(x=x, y=y, z=0.08))
            marker.lifetime = Duration(seconds=0.4).to_msg()
            markers.markers.append(marker)
        for index in range(len(alternatives), getattr(self, "_last_candidate_marker_count", 0)):
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.ns, marker.id, marker.action = "human_aware_candidates", index, Marker.DELETE
            markers.markers.append(marker)
        self._last_candidate_marker_count = len(alternatives)
        chosen = Marker()
        chosen.header.frame_id = self.target_frame
        chosen.ns, chosen.id = "human_aware_selected", 0
        chosen.pose.orientation.w = 1.
        chosen.type = Marker.LINE_STRIP
        chosen.scale.x = .075
        chosen.lifetime = Duration(seconds=.6).to_msg()
        if selected is None:
            chosen.action = Marker.DELETE
        else:
            chosen.action = Marker.ADD
            chosen.color = ColorRGBA(r=1., g=.65, b=.1, a=1.) if selected.rejected else ColorRGBA(r=.1, g=1., b=.35, a=1.)
            chosen.points = [Point(x=x, y=y, z=.12) for x, y, _, _ in selected.trajectory]
        markers.markers.append(chosen)
        label = Marker()
        label.header.frame_id = self.robot_frame
        label.header.stamp = now
        label.ns = "human_aware_state"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.z = 1.85
        label.pose.orientation.w = 1.
        label.frame_locked = True
        label.scale.z = 0.14
        label.color = ColorRGBA(r=1.0, g=0.9, b=0.2, a=1.0)
        reasons = {}
        for result in results:
            if result.rejected:
                reasons[result.reason] = reasons.get(result.reason, 0) + 1
        top = sorted(reasons.items(), key=lambda item: item[1], reverse=True)[:2]
        checks = ",".join(f"{reason}:{count}" for reason, count in top) or "checks:clear"
        label.text = (f"{state}\n"
                      f"Choose:{selected.candidate.name if selected else 'hold'}\n"
                      f"Route:{self.path_reference_status};tracks:{len(self.tracks)}\n{checks}")
        label.lifetime = Duration(seconds=0.5).to_msg()
        markers.markers.append(label)
        if self.selected_safe_patch is not None:
            patch = Marker()
            patch.header.frame_id = self.target_frame
            patch.header.stamp = now
            patch.ns = "nearest_clear_ground_patch"
            patch.id = 1
            patch.type = Marker.CYLINDER
            patch.action = Marker.ADD
            patch.pose.position.x = self.selected_safe_patch[0]
            patch.pose.position.y = self.selected_safe_patch[1]
            patch.pose.position.z = 0.025
            patch.pose.orientation.w = 1.0
            patch.scale.x = self.safe_patch_clearance * 2.0
            patch.scale.y = self.safe_patch_clearance * 2.0
            patch.scale.z = 0.05
            patch.color = ColorRGBA(r=0.05, g=1.0, b=0.25, a=0.65)
            patch.lifetime = Duration(seconds=0.5).to_msg()
            markers.markers.append(patch)
        return markers

    def _risk_grid(self) -> OccupancyGrid:
        grid = OccupancyGrid()
        grid.header.frame_id = self.target_frame
        grid.header.stamp = self.get_clock().now().to_msg()
        pose = self._robot_pose()
        center_x, center_y = (pose[0], pose[1]) if pose else (0.0, 0.0)
        resolution = max(0.05, self.risk_grid_resolution)
        width = max(1, int(self.risk_grid_size / resolution))
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = width
        grid.info.origin.position.x = center_x - self.risk_grid_size * 0.5
        grid.info.origin.position.y = center_y - self.risk_grid_size * 0.5
        grid.info.origin.orientation.w = 1.0
        tracks = self._fresh_tracks(self.get_clock().now())
        now = self.get_clock().now()
        predictions, prediction_age, typed_predictions_fresh = self._fresh_predictions(now)
        predicted_points = [] if typed_predictions_fresh else self._fresh_predicted(now)
        data = []
        for row in range(width):
            y = grid.info.origin.position.y + (row + 0.5) * resolution
            for col in range(width):
                x = grid.info.origin.position.x + (col + 0.5) * resolution
                risk = 0.0
                for track in tracks:
                    radius = self._comfort_radius(track, 0.0)
                    risk = max(risk, math.exp(-0.5 * (math.hypot(x - track.x, y - track.y) / max(0.1, radius)) ** 2))
                for px, py, _ in predicted_points:
                    risk = max(risk, math.exp(-0.5 * (math.hypot(x - px, y - py) / max(0.1, self.predicted_danger_radius)) ** 2))
                for samples in predictions.values():
                    for sample in samples:
                        if prediction_age <= sample.time_offset <= prediction_age + self.horizon:
                            risk = max(
                                risk,
                                math.exp(-0.5 * (math.hypot(x - sample.x, y - sample.y) / max(0.1, self.predicted_danger_radius)) ** 2),
                            )
                data.append(int(max(0.0, min(1.0, risk)) * 100.0))
        grid.data = data
        return grid

    def _twist_from_candidate(self, candidate: Candidate) -> Twist:
        msg = Twist()
        vx, vy, wz = self.motion_envelope.normalize((candidate.vx, candidate.vy, candidate.wz))
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        return msg

    def _robot_pose(self) -> tuple[float, float, float] | None:
        if self.odom_pose is not None and self.odom_stamp is not None:
            age = self.get_clock().now().nanoseconds * 1e-9 - self.odom_stamp
            if 0.0 <= age <= self.odom_timeout:
                if getattr(self,'heading_history_enabled',False):
                    state,self.heading_history_details=self.heading_history.propagate(
                        self.odom_pose,self.odom_stamp,self.odom_stamp+age)
                    if state is None:
                        return None
                    # Keep the existing XY projection separate from current heading/rate.
                    projected=advance_planar_pose(self.odom_pose,self.source_odom_velocity,age)
                    self.measured_velocity=(*self.source_odom_velocity[:2],state[1])
                    return projected[0],projected[1],state[0]
                return advance_planar_pose(self.odom_pose, self.measured_velocity, age)
        if getattr(self,'heading_history_enabled',False):
            self.heading_history_details=dict(valid=False,reason='stale_odometry')
            return None
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.robot_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(f"No transform {self.target_frame} <- {self.robot_frame}: {exc}", throttle_duration_sec=2.0)
            return None
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return transform.transform.translation.x, transform.transform.translation.y, yaw

    def _input_stale(self, now) -> bool:
        if self.latest_cmd_time is None:
            return True
        return (now - self.latest_cmd_time).nanoseconds * 1e-9 > self.input_timeout

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    @staticmethod
    def _wrap_pi(value: float) -> float:
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value


def main() -> None:
    rclpy.init()
    node = HumanAwareMotionPlanner()
    from .dds_metadata_executor import DDSMetadataExecutor
    executor = DDSMetadataExecutor(metadata_callbacks=(node._lowstate_callback,)) if node.heading_history_enabled else None
    if executor is not None:
        executor.add_node(node)
    try:
        executor.spin() if executor is not None else rclpy.spin(node)
    finally:
        if executor is not None:
            executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
