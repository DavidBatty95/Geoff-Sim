import math
import json
import struct
import time
import copy
from array import array
from collections import deque
from dataclasses import dataclass, field
from typing import Iterable

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from unitree_social_navigation_msgs.msg import (
    PredictedTrajectories,
    PredictedTrajectory,
    PredictedTrajectoryPoint,
    TrackedPerson,
    TrackedPersons,
)

from .pointcloud import iter_xyz
from .point_components import connected_components
from .wall_evidence import WallEvidence


@dataclass
class Cluster:
    x: float
    y: float
    z: float
    radius: float
    count: int
    width: float
    depth: float
    height: float
    vertical_layers: int
    cells: set[tuple[int, int]]
    range: float = 0.0
    density: float = 0.0
    bbox_volume: float = 0.0
    compactness: float = 0.0
    aspect_ratio: float = 0.0
    static_fraction: float = 0.0
    noise_fraction: float = 0.0
    rejection_reason: str = ""
    measured_vx: float | None = None
    measured_vy: float | None = None
    partial_observation: bool = False
    z_max: float | None = None


@dataclass
class Track:
    x: float
    y: float
    stamp: float
    first_x: float
    first_y: float
    speed: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    age: int = 1
    dynamic_observations: int = 0
    person_observations: int = 0
    static_observations: int = 0
    missed: int = 0
    radius: float = 0.35
    width: float = 0.35
    depth: float = 0.35
    height: float = 0.0
    confirmed: bool = False
    confidence: float = 0.25
    confidence_stamp: float = 0.0
    person_score: float = 0.0
    detection_confidence: float = 0.0
    state: str = "TENTATIVE"
    hit_history: deque[bool] = field(default_factory=lambda: deque(maxlen=5))
    first_stamp: float = 0.0
    last_association_distance: float = math.inf
    rejection_reason: str = ""
    kf_p: list[list[float]] = field(
        default_factory=lambda: [
            [0.20, 0.0, 0.0, 0.0],
            [0.0, 0.20, 0.0, 0.0],
            [0.0, 0.0, 0.60, 0.0],
            [0.0, 0.0, 0.0, 0.60],
        ]
    )
    history: deque[tuple[float, float]] = field(default_factory=lambda: deque(maxlen=120))
    measurements: deque[tuple[float, float, float]] = field(default_factory=lambda: deque(maxlen=16))
    observation_age: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    last_full_stamp: float | None = None


@dataclass
class PersonDetection:
    x: float
    y: float
    z: float
    confidence: float
    radius: float
    stamp: float
    vx: float | None = None
    vy: float | None = None


def _iter_detections(msg: PointCloud2) -> Iterable[tuple[float, float, float, float, float]]:
    offsets = {field.name: field.offset for field in msg.fields}
    if not {"x", "y", "z"}.issubset(offsets):
        return
    confidence_offset = offsets.get("confidence")
    radius_offset = offsets.get("radius")
    for base in range(0, len(msg.data), msg.point_step):
        x = struct.unpack_from("<f", msg.data, base + offsets["x"])[0]
        y = struct.unpack_from("<f", msg.data, base + offsets["y"])[0]
        z = struct.unpack_from("<f", msg.data, base + offsets["z"])[0]
        confidence = 1.0 if confidence_offset is None else struct.unpack_from("<f", msg.data, base + confidence_offset)[0]
        radius = 0.45 if radius_offset is None else struct.unpack_from("<f", msg.data, base + radius_offset)[0]
        if all(math.isfinite(value) for value in (x, y, z, confidence, radius)):
            yield x, y, z, confidence, radius


def _split_seconds(value: float) -> tuple[int, int]:
    """Return a normalized ROS seconds/nanoseconds pair."""
    sec = math.floor(value)
    nanosec = round((value - sec) * 1e9)
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return int(sec), int(nanosec)


def _position_covariance_at(kf_p: list[list[float]], offset: float) -> tuple[float, float, float, float]:
    """Propagate the x/y covariance of a [x, y, vx, vy] constant-velocity state."""
    t = max(0.0, offset)
    xx = kf_p[0][0] + t * (kf_p[0][2] + kf_p[2][0]) + t * t * kf_p[2][2]
    xy = kf_p[0][1] + t * (kf_p[0][3] + kf_p[2][1]) + t * t * kf_p[2][3]
    yx = kf_p[1][0] + t * (kf_p[1][2] + kf_p[3][0]) + t * t * kf_p[3][2]
    yy = kf_p[1][1] + t * (kf_p[1][3] + kf_p[3][1]) + t * t * kf_p[3][3]
    return max(0.0, xx), xy, yx, max(0.0, yy)


def _constant_velocity_covariance(kf_p: list[list[float]], offset: float) -> list[list[float]]:
    t = max(0.0, offset)
    result = [row[:] for row in kf_p]
    # Exploit the sparse CV transition instead of 256 Python multiplications
    # per association. This is on the high-frequency tracking critical path.
    for row in range(2):
        for col in range(2):
            result[row][col] += t * (kf_p[row + 2][col] + kf_p[row][col + 2]) + t * t * kf_p[row + 2][col + 2]
            result[row][col + 2] += t * kf_p[row + 2][col + 2]
            result[row + 2][col] += t * kf_p[row + 2][col + 2]
    return result


def _motion_covariance(kf_p: list[list[float]], offset: float, noise: float = 0.35) -> list[list[float]]:
    """Continuous white-acceleration model; uncertainty grows with elapsed time."""
    t = max(0.0, offset)
    p = _constant_velocity_covariance(kf_p, t)
    for position, velocity in ((0, 2), (1, 3)):
        p[position][position] += noise * t ** 3 / 3.0
        p[position][velocity] += noise * t * t / 2.0
        p[velocity][position] += noise * t * t / 2.0
        p[velocity][velocity] += noise * t
    return p


def predict_track_motion(track: Track, horizon: float, dt: float, max_speed: float, acceleration_horizon: float = 0.5):
    """Short constant-acceleration transition followed by constant velocity.

    Anticipates walking onset/braking without extrapolating acceleration for the
    entire horizon, and never predicts a reversal merely because a person stops.
    """
    x, y, vx, vy = track.x, track.y, track.vx, track.vy
    ax, ay = track.ax, track.ay
    if math.hypot(vx, vy) < 0.03:
        vx = vy = ax = ay = 0.0
    result = [(0.0, x, y, vx, vy)]
    t = 0.0
    while t < horizon - 1e-9:
        step = min(max(0.02, dt), horizon - t)
        active = min(step, max(0.0, acceleration_horizon - t))
        nx, ny = vx + ax * active, vy + ay * active
        if vx * nx + vy * ny < 0.0 or (math.hypot(vx, vy) > 1e-6 and math.hypot(nx, ny) <= 1e-6):
            nx = ny = ax = ay = 0.0
        speed = math.hypot(nx, ny)
        if speed > max_speed:
            nx *= max_speed / speed
            ny *= max_speed / speed
        x += 0.5 * (vx + nx) * active + nx * (step - active)
        y += 0.5 * (vy + ny) * active + ny * (step - active)
        vx, vy = nx, ny
        t += step
        result.append((t, x, y, vx, vy))
    return result


def _typed_track_state(state: str) -> int:
    if state == "CONFIRMED":
        return TrackedPerson.TRACK_STATE_CONFIRMED
    if state == "OCCLUDED":
        return TrackedPerson.TRACK_STATE_LOST
    if state == "TENTATIVE":
        return TrackedPerson.TRACK_STATE_TENTATIVE
    return TrackedPerson.TRACK_STATE_UNKNOWN


def _track_should_expire(
    track: Track,
    age: float,
    drop_confidence_threshold: float,
    tentative_timeout: float,
    confirmed_timeout: float,
) -> bool:
    if track.confirmed:
        return track.confidence < drop_confidence_threshold or age > confirmed_timeout
    return age > tentative_timeout


class DynamicObstacleTracker(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_tracker")
        self.declare_parameter("cloud_topic", "/livox/lidar_world")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("use_sensor_input", True)
        self.declare_parameter("use_scan", True)
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("marker_topic", "/dynamic_obstacles")
        self.declare_parameter("prediction_cloud_topic", "/predicted_dynamic_obstacles")
        self.declare_parameter("tracked_cloud_topic", "/tracked_dynamic_obstacles")
        self.declare_parameter("tentative_prediction_cloud_topic", "/tentative_dynamic_obstacles")
        self.declare_parameter("debug_tracks_topic", "/dynamic_obstacle_tracker/debug_tracks")
        self.declare_parameter("tracked_persons_topic", "/tracked_persons")
        self.declare_parameter("predicted_trajectories_topic", "/predicted_person_trajectories")
        self.declare_parameter("safety_grid_topic", "/human_safety_grid")
        self.declare_parameter("use_ml_person_detections", True)
        self.declare_parameter("person_detection_topic", "/lidar_person_detections")
        self.declare_parameter("use_mujoco_people_detections", True)
        self.declare_parameter("mujoco_people_authoritative", True)
        self.declare_parameter("mujoco_people_topic", "/mujoco/pedestrians")
        self.declare_parameter("mujoco_people_timeout", 0.35)
        self.declare_parameter("mujoco_person_height", 1.55)
        self.declare_parameter("publish_safety_grid", True)
        self.declare_parameter("min_height", 0.05)
        self.declare_parameter("max_height", 2.2)
        self.declare_parameter("min_range", 0.25)
        self.declare_parameter("max_range", 12.0)
        self.declare_parameter("ground_rejection_height", -0.03)
        self.declare_parameter("cell_size", 0.35)
        self.declare_parameter("cluster_distance", 0.45)
        self.declare_parameter("cluster_z_distance", 0.90)
        self.declare_parameter("person_cluster_distance", 0.30)
        self.declare_parameter("min_cluster_points", 3)
        self.declare_parameter("max_cluster_points", 160)
        self.declare_parameter("enable_point_noise_filter", True)
        self.declare_parameter("noise_filter_radius", 0.38)
        self.declare_parameter("min_point_neighbors", 2)
        self.declare_parameter("min_person_cluster_points", 6)
        self.declare_parameter("require_person_vertical_structure", False)
        self.declare_parameter("min_person_vertical_layers", 2)
        self.declare_parameter("person_vertical_layer_height", 0.30)
        self.declare_parameter("reject_livox_pattern_artifacts", True)
        self.declare_parameter("livox_artifact_max_height", 0.28)
        self.declare_parameter("livox_artifact_max_layers", 1)
        self.declare_parameter("livox_artifact_min_extent", 0.55)
        self.declare_parameter("livox_artifact_max_points", 18)
        self.declare_parameter("max_person_aspect_ratio", 3.2)
        self.declare_parameter("min_vertical_layers", 2)
        self.declare_parameter("candidate_person_score_threshold", 0.72)
        self.declare_parameter("confirmed_person_score_threshold", 0.62)
        self.declare_parameter("min_person_radius", 0.12)
        self.declare_parameter("max_person_radius", 0.62)
        self.declare_parameter("max_person_extent", 1.10)
        self.declare_parameter("min_person_height", 0.35)
        self.declare_parameter("max_person_height", 2.15)
        self.declare_parameter("min_person_centroid_height", 0.35)
        self.declare_parameter("min_person_top_height", 0.0)
        self.declare_parameter("use_torso_centroid", False)
        self.declare_parameter("max_person_centroid_height", 1.65)
        self.declare_parameter("person_fragment_merge_distance", 0.72)
        self.declare_parameter("max_cluster_radius", 1.2)
        self.declare_parameter("track_match_distance", 1.4)
        self.declare_parameter("min_dynamic_speed", 0.12)
        self.declare_parameter("min_confirm_displacement", 0.20)
        self.declare_parameter("require_motion_to_confirm", True)
        self.declare_parameter("max_person_speed", 2.2)
        self.declare_parameter("max_person_acceleration", 2.8)
        self.declare_parameter("tracking_process_noise", 0.35)
        self.declare_parameter("near_accumulation_radius", 0.0)
        self.declare_parameter("near_accumulation_time", 0.05)
        self.declare_parameter("measurement_noise_variance", 0.02)
        self.declare_parameter("tracked_body_radius_floor", 0.0)
        self.tracked_body_radius_floor = float(self.get_parameter("tracked_body_radius_floor").value)
        self.declare_parameter("use_track_conditioned_crops", False)
        self.declare_parameter("partial_observation_timeout", 2.5)
        self.use_track_conditioned_crops = bool(self.get_parameter("use_track_conditioned_crops").value)
        self.partial_observation_timeout = max(.1,float(self.get_parameter("partial_observation_timeout").value))
        self.declare_parameter("max_confirmed_measurement_jump", 0.85)
        self.declare_parameter("min_track_age", 3)
        self.declare_parameter("min_dynamic_observations", 2)
        self.declare_parameter("min_person_observations", 2)
        self.declare_parameter("min_static_person_observations", 6)
        self.declare_parameter("max_static_speed", 0.06)
        self.declare_parameter("max_human_speed", 2.5)
        self.declare_parameter("max_human_accel", 3.5)
        self.declare_parameter("static_cell_observations", 8)
        self.declare_parameter("static_suppression_radius", 0.45)
        self.declare_parameter("coast_confirmed_tracks", True)
        self.declare_parameter("max_coast_time", 0.8)
        self.declare_parameter("max_lost_persistence", 4.0)
        self.declare_parameter("max_visible_confirmed_tracks", 4)
        self.declare_parameter("confirm_confidence_threshold", 0.70)
        self.declare_parameter("drop_confidence_threshold", 0.22)
        self.declare_parameter("confidence_decay_per_second", 0.22)
        self.declare_parameter("immediate_person_risk", True)
        self.declare_parameter("candidate_person_min_age", 2)
        self.declare_parameter("candidate_person_min_confidence", 0.45)
        self.declare_parameter("candidate_person_robot_radius", 2.8)
        self.declare_parameter("candidate_person_risk_radius", 0.75)
        self.declare_parameter("robot_self_filter_radius", 0.55)
        self.declare_parameter("personal_space_radius", 0.85)
        self.declare_parameter("robot_avoidance_radius", 1.35)
        self.declare_parameter("robot_risk_inflation_radius", 1.15)
        self.declare_parameter("prediction_horizon", 5.0)
        self.declare_parameter("prediction_dt", 0.25)
        self.declare_parameter("publish_candidate_predictions", True)
        self.declare_parameter("publish_tentative_predictions", False)
        self.declare_parameter("publish_legacy_prediction_cloud", True)
        self.declare_parameter("tentative_prediction_radius_scale", 0.45)
        self.declare_parameter("prediction_min_confidence", 0.55)
        self.declare_parameter("prediction_min_person_score", 0.55)
        self.declare_parameter("prediction_min_dynamic_observations", 1)
        self.declare_parameter("max_prediction_coast_time", 1.0)
        self.declare_parameter("min_prediction_speed", 0.10)
        self.declare_parameter("point_accumulation_time", 0.80)
        self.declare_parameter("accumulation_motion_compensation", True)
        self.declare_parameter("max_accumulation_robot_yaw_rate", 0.8)
        self.declare_parameter("max_accumulated_points", 9000)
        self.declare_parameter("voxel_leaf_size", 0.05)
        self.declare_parameter("use_livox_tag_filter", True)
        self.declare_parameter("max_noise_tag_fraction", 0.35)
        self.declare_parameter("range_adaptive_neighbor_filter", True)
        self.declare_parameter("near_noise_radius", 0.15)
        self.declare_parameter("far_noise_radius", 0.35)
        self.declare_parameter("min_neighbors_near", 3)
        self.declare_parameter("min_neighbors_far", 4)
        self.declare_parameter("range_adaptive_cluster_distance", True)
        self.declare_parameter("cluster_epsilon_base", 0.16)
        self.declare_parameter("cluster_epsilon_per_meter", 0.025)
        self.declare_parameter("cluster_epsilon_min", 0.14)
        self.declare_parameter("cluster_epsilon_max", 0.38)
        self.declare_parameter("min_cluster_points_by_range", True)
        self.declare_parameter("max_static_cell_fraction_new_track", 0.35)
        self.declare_parameter("tentative_confirm_min_hits", 3)
        self.declare_parameter("tentative_confirm_window", 5)
        self.declare_parameter("min_confirm_age", 0.50)
        self.declare_parameter("confirmed_keep_missed_time", 0.50)
        self.declare_parameter("tentative_delete_missed_time", 0.25)
        self.declare_parameter("stale_track_timeout", 1.0)
        self.declare_parameter("mahalanobis_gate", 7.8)
        self.declare_parameter("debug_markers_enabled", True)
        self.declare_parameter("marker_lifetime", 60.0)
        self.declare_parameter("candidate_marker_lifetime", 15.0)
        self.declare_parameter("marker_display_hold_time", 3.0)
        self.declare_parameter("safety_grid_size", 10.0)
        self.declare_parameter("safety_grid_resolution", 0.25)
        self.declare_parameter("safety_grid_danger_radius", 0.65)
        self.declare_parameter("safety_grid_caution_radius", 1.20)
        self.declare_parameter("person_detection_timeout", 0.6)
        self.declare_parameter("person_detection_association_radius", 0.70)
        self.declare_parameter("person_detection_min_confidence", 0.45)
        self.declare_parameter("person_detection_confirm_confidence", 0.65)
        self.declare_parameter("person_detection_score_boost", 0.35)
        self.declare_parameter("use_object_motion_detection", False)
        self.declare_parameter("object_score_threshold", 0.45)
        self.declare_parameter("object_min_points", 3)
        self.declare_parameter("object_max_points", 320)
        self.declare_parameter("object_min_radius", 0.06)
        self.declare_parameter("object_max_radius", 0.90)
        self.declare_parameter("object_max_extent", 1.50)
        self.declare_parameter("object_max_height", 2.30)
        self.declare_parameter("object_min_height", 0.12)
        self.declare_parameter("object_min_cells", 2)
        self.declare_parameter("object_near_range", 4.0)
        self.declare_parameter("object_near_min_points", 6)
        self.declare_parameter("require_person_shape_for_object_detection", True)
        self.declare_parameter("reject_wall_supported_clusters", True)
        self.reject_wall_supported_clusters = bool(self.get_parameter("reject_wall_supported_clusters").value)
        self.declare_parameter("local_tracking_radius", 7.0)

        self.min_height = float(self.get_parameter("min_height").value)
        self.max_height = float(self.get_parameter("max_height").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.ground_rejection_height = float(self.get_parameter("ground_rejection_height").value)
        self.cell_size = float(self.get_parameter("cell_size").value)
        self.cluster_distance = float(self.get_parameter("cluster_distance").value)
        self.cluster_z_distance = float(self.get_parameter("cluster_z_distance").value)
        self.person_cluster_distance = float(self.get_parameter("person_cluster_distance").value)
        self.min_cluster_points = int(self.get_parameter("min_cluster_points").value)
        self.max_cluster_points = int(self.get_parameter("max_cluster_points").value)
        self.enable_point_noise_filter = bool(self.get_parameter("enable_point_noise_filter").value)
        self.noise_filter_radius = float(self.get_parameter("noise_filter_radius").value)
        self.min_point_neighbors = int(self.get_parameter("min_point_neighbors").value)
        self.min_person_cluster_points = int(self.get_parameter("min_person_cluster_points").value)
        self.require_person_vertical_structure = bool(self.get_parameter("require_person_vertical_structure").value)
        self.min_person_vertical_layers = int(self.get_parameter("min_person_vertical_layers").value)
        self.person_vertical_layer_height = float(self.get_parameter("person_vertical_layer_height").value)
        self.reject_livox_pattern_artifacts = bool(self.get_parameter("reject_livox_pattern_artifacts").value)
        self.livox_artifact_max_height = float(self.get_parameter("livox_artifact_max_height").value)
        self.livox_artifact_max_layers = int(self.get_parameter("livox_artifact_max_layers").value)
        self.livox_artifact_min_extent = float(self.get_parameter("livox_artifact_min_extent").value)
        self.livox_artifact_max_points = int(self.get_parameter("livox_artifact_max_points").value)
        self.max_person_aspect_ratio = float(self.get_parameter("max_person_aspect_ratio").value)
        self.min_vertical_layers = int(self.get_parameter("min_vertical_layers").value)
        self.candidate_person_score_threshold = float(self.get_parameter("candidate_person_score_threshold").value)
        self.confirmed_person_score_threshold = float(self.get_parameter("confirmed_person_score_threshold").value)
        self.min_person_radius = float(self.get_parameter("min_person_radius").value)
        self.max_person_radius = float(self.get_parameter("max_person_radius").value)
        self.max_person_extent = float(self.get_parameter("max_person_extent").value)
        self.min_person_height = float(self.get_parameter("min_person_height").value)
        self.max_person_height = float(self.get_parameter("max_person_height").value)
        self.min_person_centroid_height = float(self.get_parameter("min_person_centroid_height").value)
        self.min_person_top_height = float(self.get_parameter("min_person_top_height").value)
        self.use_torso_centroid = bool(self.get_parameter("use_torso_centroid").value)
        self.max_person_centroid_height = float(self.get_parameter("max_person_centroid_height").value)
        self.person_fragment_merge_distance = float(self.get_parameter("person_fragment_merge_distance").value)
        self.max_cluster_radius = float(self.get_parameter("max_cluster_radius").value)
        self.track_match_distance = float(self.get_parameter("track_match_distance").value)
        self.min_dynamic_speed = float(self.get_parameter("min_dynamic_speed").value)
        self.min_confirm_displacement = float(self.get_parameter("min_confirm_displacement").value)
        self.require_motion_to_confirm = bool(self.get_parameter("require_motion_to_confirm").value)
        self.max_person_speed = float(self.get_parameter("max_person_speed").value)
        self.max_person_acceleration = float(self.get_parameter("max_person_acceleration").value)
        self.tracking_process_noise = max(.01, float(self.get_parameter("tracking_process_noise").value))
        self.near_accumulation_radius = max(0., float(self.get_parameter("near_accumulation_radius").value))
        self.near_accumulation_time = max(0., float(self.get_parameter("near_accumulation_time").value))
        self.measurement_noise_variance = max(.0001, float(self.get_parameter("measurement_noise_variance").value))
        self.max_human_speed = float(self.get_parameter("max_human_speed").value)
        self.max_human_accel = float(self.get_parameter("max_human_accel").value)
        self.max_person_speed = min(self.max_person_speed, self.max_human_speed)
        self.max_person_acceleration = min(self.max_person_acceleration, self.max_human_accel)
        self.max_confirmed_measurement_jump = float(self.get_parameter("max_confirmed_measurement_jump").value)
        self.max_static_speed = float(self.get_parameter("max_static_speed").value)
        self.min_track_age = int(self.get_parameter("min_track_age").value)
        self.min_dynamic_observations = int(self.get_parameter("min_dynamic_observations").value)
        self.min_person_observations = int(self.get_parameter("min_person_observations").value)
        self.min_static_person_observations = int(self.get_parameter("min_static_person_observations").value)
        self.static_cell_observations = int(self.get_parameter("static_cell_observations").value)
        self.static_suppression_radius = float(self.get_parameter("static_suppression_radius").value)
        self.coast_confirmed_tracks = bool(self.get_parameter("coast_confirmed_tracks").value)
        self.max_coast_time = float(self.get_parameter("max_coast_time").value)
        self.max_lost_persistence = float(self.get_parameter("max_lost_persistence").value)
        self.max_visible_confirmed_tracks = int(self.get_parameter("max_visible_confirmed_tracks").value)
        self.confirm_confidence_threshold = float(self.get_parameter("confirm_confidence_threshold").value)
        self.drop_confidence_threshold = float(self.get_parameter("drop_confidence_threshold").value)
        self.confidence_decay_per_second = float(self.get_parameter("confidence_decay_per_second").value)
        self.immediate_person_risk = bool(self.get_parameter("immediate_person_risk").value)
        self.candidate_person_min_age = int(self.get_parameter("candidate_person_min_age").value)
        self.candidate_person_min_confidence = float(self.get_parameter("candidate_person_min_confidence").value)
        self.candidate_person_robot_radius = float(self.get_parameter("candidate_person_robot_radius").value)
        self.candidate_person_risk_radius = float(self.get_parameter("candidate_person_risk_radius").value)
        self.robot_self_filter_radius = float(self.get_parameter("robot_self_filter_radius").value)
        self.personal_space_radius = float(self.get_parameter("personal_space_radius").value)
        self.robot_avoidance_radius = float(self.get_parameter("robot_avoidance_radius").value)
        self.robot_risk_inflation_radius = float(self.get_parameter("robot_risk_inflation_radius").value)
        self.prediction_horizon = float(self.get_parameter("prediction_horizon").value)
        self.prediction_dt = float(self.get_parameter("prediction_dt").value)
        self.publish_candidate_predictions = bool(self.get_parameter("publish_candidate_predictions").value)
        self.publish_tentative_predictions = bool(self.get_parameter("publish_tentative_predictions").value)
        self.publish_legacy_prediction_cloud = bool(self.get_parameter("publish_legacy_prediction_cloud").value)
        self.render_markers = False
        self.last_marker_time = -math.inf
        self.tentative_prediction_radius_scale = float(self.get_parameter("tentative_prediction_radius_scale").value)
        self.prediction_min_confidence = float(self.get_parameter("prediction_min_confidence").value)
        self.prediction_min_person_score = float(self.get_parameter("prediction_min_person_score").value)
        self.prediction_min_dynamic_observations = int(self.get_parameter("prediction_min_dynamic_observations").value)
        self.max_prediction_coast_time = float(self.get_parameter("max_prediction_coast_time").value)
        self.min_prediction_speed = float(self.get_parameter("min_prediction_speed").value)
        self.point_accumulation_time = float(self.get_parameter("point_accumulation_time").value)
        self.accumulation_motion_compensation = bool(self.get_parameter("accumulation_motion_compensation").value)
        self.max_accumulation_robot_yaw_rate = float(self.get_parameter("max_accumulation_robot_yaw_rate").value)
        self.max_accumulated_points = int(self.get_parameter("max_accumulated_points").value)
        self.voxel_leaf_size = float(self.get_parameter("voxel_leaf_size").value)
        self.use_livox_tag_filter = bool(self.get_parameter("use_livox_tag_filter").value)
        self.max_noise_tag_fraction = float(self.get_parameter("max_noise_tag_fraction").value)
        self.range_adaptive_neighbor_filter = bool(self.get_parameter("range_adaptive_neighbor_filter").value)
        self.near_noise_radius = float(self.get_parameter("near_noise_radius").value)
        self.far_noise_radius = float(self.get_parameter("far_noise_radius").value)
        self.min_neighbors_near = int(self.get_parameter("min_neighbors_near").value)
        self.min_neighbors_far = int(self.get_parameter("min_neighbors_far").value)
        self.range_adaptive_cluster_distance = bool(self.get_parameter("range_adaptive_cluster_distance").value)
        self.cluster_epsilon_base = float(self.get_parameter("cluster_epsilon_base").value)
        self.cluster_epsilon_per_meter = float(self.get_parameter("cluster_epsilon_per_meter").value)
        self.cluster_epsilon_min = float(self.get_parameter("cluster_epsilon_min").value)
        self.cluster_epsilon_max = float(self.get_parameter("cluster_epsilon_max").value)
        self.min_cluster_points_by_range = bool(self.get_parameter("min_cluster_points_by_range").value)
        self.max_static_cell_fraction_new_track = float(self.get_parameter("max_static_cell_fraction_new_track").value)
        self.tentative_confirm_min_hits = int(self.get_parameter("tentative_confirm_min_hits").value)
        self.tentative_confirm_window = int(self.get_parameter("tentative_confirm_window").value)
        self.min_confirm_age = float(self.get_parameter("min_confirm_age").value)
        self.confirmed_keep_missed_time = float(self.get_parameter("confirmed_keep_missed_time").value)
        self.tentative_delete_missed_time = float(self.get_parameter("tentative_delete_missed_time").value)
        self.stale_track_timeout = float(self.get_parameter("stale_track_timeout").value)
        self.mahalanobis_gate = float(self.get_parameter("mahalanobis_gate").value)
        self.debug_markers_enabled = bool(self.get_parameter("debug_markers_enabled").value)
        self.marker_lifetime = float(self.get_parameter("marker_lifetime").value)
        self.candidate_marker_lifetime = float(self.get_parameter("candidate_marker_lifetime").value)
        self.marker_display_hold_time = float(self.get_parameter("marker_display_hold_time").value)
        self.publish_safety_grid = bool(self.get_parameter("publish_safety_grid").value)
        self.safety_grid_size = float(self.get_parameter("safety_grid_size").value)
        self.safety_grid_resolution = float(self.get_parameter("safety_grid_resolution").value)
        self.safety_grid_danger_radius = float(self.get_parameter("safety_grid_danger_radius").value)
        self.safety_grid_caution_radius = float(self.get_parameter("safety_grid_caution_radius").value)
        self.use_ml_person_detections = bool(self.get_parameter("use_ml_person_detections").value)
        self.use_mujoco_people_detections = bool(self.get_parameter("use_mujoco_people_detections").value)
        self.use_sensor_input = bool(self.get_parameter("use_sensor_input").value)
        self.mujoco_people_authoritative = bool(self.get_parameter("mujoco_people_authoritative").value)
        self.mujoco_people_timeout = float(self.get_parameter("mujoco_people_timeout").value)
        self.mujoco_person_height = float(self.get_parameter("mujoco_person_height").value)
        self.person_detection_timeout = float(self.get_parameter("person_detection_timeout").value)
        self.person_detection_association_radius = float(self.get_parameter("person_detection_association_radius").value)
        self.person_detection_min_confidence = float(self.get_parameter("person_detection_min_confidence").value)
        self.person_detection_confirm_confidence = float(self.get_parameter("person_detection_confirm_confidence").value)
        self.person_detection_score_boost = float(self.get_parameter("person_detection_score_boost").value)
        self.use_object_motion_detection = bool(self.get_parameter("use_object_motion_detection").value)
        self.object_score_threshold = float(self.get_parameter("object_score_threshold").value)
        self.object_min_points = int(self.get_parameter("object_min_points").value)
        self.object_max_points = int(self.get_parameter("object_max_points").value)
        self.object_min_radius = float(self.get_parameter("object_min_radius").value)
        self.object_max_radius = float(self.get_parameter("object_max_radius").value)
        self.object_max_extent = float(self.get_parameter("object_max_extent").value)
        self.object_max_height = float(self.get_parameter("object_max_height").value)
        self.object_min_height = float(self.get_parameter("object_min_height").value)
        self.object_min_cells = int(self.get_parameter("object_min_cells").value)
        self.object_near_range = float(self.get_parameter("object_near_range").value)
        self.object_near_min_points = int(self.get_parameter("object_near_min_points").value)
        self.require_person_shape_for_object_detection = bool(self.get_parameter("require_person_shape_for_object_detection").value)
        self.local_tracking_radius = float(self.get_parameter("local_tracking_radius").value)
        self.next_track_id = 1
        self.tracks: dict[int, Track] = {}
        self.last_cluster_count = 0
        self.effective_people_source = "not_observed"
        self.last_candidate_count = 0
        self.last_confirmed_count = 0
        self.last_predicted_point_count = 0
        self.display_markers: dict[tuple[str, int], tuple[Marker, float]] = {}
        self.static_cells: dict[tuple[int, int], int] = {}
        self.recent_points: deque[tuple[float, list[tuple[float, float, float]]]] = deque()
        self.last_processed_stamp = -math.inf
        self.processing_ms = 0.0
        self.source_age_s = 0.0
        self.debug_events: list[dict[str, object]] = []
        self.person_detections: list[PersonDetection] = []
        self.mujoco_detections: list[PersonDetection] = []
        self.visible_track_ids: set[int] = set()
        self._angle_cache_key: tuple[float, float, int] | None = None
        self._angle_cache: list[tuple[float, float]] = []
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_odometry = None
        self.create_subscription(Odometry, "/odom", self._robot_odometry_callback,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        latest_sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.pub = self.create_publisher(MarkerArray, str(self.get_parameter("marker_topic").value), 10)
        self.prediction_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("prediction_cloud_topic").value), 10
        )
        self.tracked_pub = self.create_publisher(PointCloud2, str(self.get_parameter("tracked_cloud_topic").value), 10)
        self.tentative_prediction_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("tentative_prediction_cloud_topic").value), 10
        )
        self.debug_pub = self.create_publisher(String, str(self.get_parameter("debug_tracks_topic").value), 10)
        self.tracked_persons_pub = self.create_publisher(
            TrackedPersons, str(self.get_parameter("tracked_persons_topic").value), 10
        )
        self.predicted_trajectories_pub = self.create_publisher(
            PredictedTrajectories, str(self.get_parameter("predicted_trajectories_topic").value), 10
        )
        self.safety_grid_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("safety_grid_topic").value), 10
        )
        if self.use_ml_person_detections:
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("person_detection_topic").value),
                self._person_detection_callback,
                qos_profile_sensor_data,
            )
        if self.use_mujoco_people_detections:
            self.create_subscription(
                String,
                str(self.get_parameter("mujoco_people_topic").value),
                self._mujoco_people_callback,
                1,
            )
        if self.use_sensor_input and bool(self.get_parameter("use_scan").value):
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("scan_topic").value),
                self._scan_callback,
                latest_sensor_qos,
            )
        elif self.use_sensor_input:
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("cloud_topic").value),
                self._cloud_callback,
                latest_sensor_qos,
            )
        self.get_logger().info(
            "Tracking moving compact objects from latest raw LiDAR samples"
        )
        self.create_timer(2.0, self._diagnostics)
        self.create_timer(.1, self._refresh_person_markers)

    def _person_detection_callback(self, msg: PointCloud2) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        query_time = rclpy.time.Time.from_msg(msg.header.stamp)
        transform = None
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            try:
                transform = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, query_time)
            except TransformException as exc:
                self.get_logger().warn(
                    f"No transform {self.target_frame} <- {msg.header.frame_id}: {exc}",
                    throttle_duration_sec=2.0,
                )
                return
        detections = []
        for x, y, z, confidence, radius in _iter_detections(msg):
            if confidence < self.person_detection_min_confidence:
                continue
            if transform is not None:
                x, y, z = self._transform_point(transform, x, y, z)
            detections.append(PersonDetection(x, y, z, confidence, radius, stamp))
        self.person_detections = detections

    def _mujoco_people_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.mujoco_detections = []
            return
        now = self.get_clock().now()
        now_seconds = now.nanoseconds * 1e-9
        stamp = float(data.get("stamp", now_seconds) or now_seconds)
        if now_seconds - stamp > self.mujoco_people_timeout:
            self.mujoco_detections = []
            return
        robot_xy = self._lookup_robot_xy()
        detections: list[PersonDetection] = []
        synthetic_points: list[tuple[float, float, float]] = []
        for person in data.get("people", []):
            try:
                x = float(person.get("x", 0.0))
                y = float(person.get("y", 0.0))
                vx = float(person.get("vx", 0.0))
                vy = float(person.get("vy", 0.0))
                radius = max(0.25, float(person.get("radius_m", 0.42)))
            except (TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in (x, y, vx, vy, radius)):
                continue
            if self._outside_local_tracking_radius(x, y, robot_xy):
                continue
            detections.append(PersonDetection(x, y, self.mujoco_person_height * 0.5, 1.0, radius, stamp, vx, vy))
            synthetic_points.extend(self._synthetic_person_points(x, y, radius))
        self.mujoco_detections = detections
        self._publish_points(synthetic_points, stamp, self.target_frame)

    def _synthetic_person_points(self, x: float, y: float, radius: float) -> list[tuple[float, float, float]]:
        points = []
        ring_radius = min(max(0.12, radius * 0.45), 0.24)
        for z in (0.25, 0.75, 1.20, self.mujoco_person_height):
            points.append((x, y, z))
            for angle in (0.0, math.pi * 0.5, math.pi, math.pi * 1.5):
                points.append((x + ring_radius * math.cos(angle), y + ring_radius * math.sin(angle), z))
        return points

    def _scan_callback(self, msg: LaserScan) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        query_time = rclpy.time.Time.from_msg(msg.header.stamp)
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, query_time)
        except TransformException as exc:
            self.get_logger().warn(f"No transform {self.target_frame} <- {msg.header.frame_id}: {exc}", throttle_duration_sec=2.0)
            return

        points = []
        angles = self._scan_angles(msg)
        robot_xy = self._lookup_robot_xy(query_time)
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or distance < msg.range_min or distance > msg.range_max:
                continue
            if distance < self.min_range or distance > self.max_range:
                continue
            cos_angle, sin_angle = angles[index]
            lx = float(distance) * cos_angle
            ly = float(distance) * sin_angle
            x, y, z = self._transform_point(transform, lx, ly, 0.0)
            if self._outside_local_tracking_radius(x, y, robot_xy):
                continue
            points.append((x, y, z))
        self._publish_points(points, stamp, self.target_frame)

    def _scan_angles(self, msg: LaserScan) -> list[tuple[float, float]]:
        key = (msg.angle_min, msg.angle_increment, len(msg.ranges))
        if self._angle_cache_key != key:
            self._angle_cache_key = key
            self._angle_cache = [
                (math.cos(msg.angle_min + index * msg.angle_increment), math.sin(msg.angle_min + index * msg.angle_increment))
                for index in range(len(msg.ranges))
            ]
        return self._angle_cache

    def _cloud_callback(self, msg: PointCloud2) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not math.isfinite(stamp) or stamp<=getattr(self,'last_processed_stamp',-math.inf):
            return
        query_time = rclpy.time.Time.from_msg(msg.header.stamp)
        points = []
        wall_points = []
        transform = None
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            try:
                transform = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, query_time)
            except TransformException as exc:
                self.get_logger().warn(
                    f"No transform {self.target_frame} <- {msg.header.frame_id}: {exc}",
                    throttle_duration_sec=2.0,
                )
                return
        robot_xy = self._lookup_robot_xy(query_time)
        for x, y, z, is_noise in self._iter_cloud_points(msg):
            if transform is not None:
                x, y, z = self._transform_point(transform, x, y, z)
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            if robot_xy is not None:
                point_range = math.hypot(x - robot_xy[0], y - robot_xy[1])
                if point_range < self.min_range or point_range > self.max_range:
                    continue
            if is_noise:
                self.debug_events.append({"kind": "rejected_point", "reason": "livox_noise_tag"})
                continue
            if z < self.ground_rejection_height:
                continue
            if (getattr(self,'reject_wall_supported_clusters',False) and
                    self.max_person_height+.15<z<=self.max_height+1.5 and
                    not self._outside_local_tracking_radius(x,y,robot_xy) and
                    len(wall_points)<self.max_accumulated_points):
                wall_points.append((x,y,z))
            if z < self.min_height or z > self.max_height:
                continue
            if self._outside_local_tracking_radius(x, y, robot_xy):
                continue
            points.append((x, y, z))

        frame = self.target_frame if transform is not None else msg.header.frame_id
        if not hasattr(self,'wall_history'):
            self.wall_history = deque(maxlen=8)
        self.wall_history.append((stamp,frame,wall_points))
        while self.wall_history and (stamp-self.wall_history[0][0]>.25 or self.wall_history[0][1]!=frame):
            self.wall_history.popleft()
        # Static structural returns can be accumulated longer than close moving
        # bodies. Keep the history independently bounded and in one frame.
        high = [point for _,source,points_in_frame in self.wall_history if source==frame for point in points_in_frame]
        self.wall_observation = stamp,frame,high[-getattr(self,'max_accumulated_points',20000):]
        self._publish_points(points, stamp, frame)

    def _iter_cloud_points(self, msg: PointCloud2) -> Iterable[tuple[float, float, float, bool]]:
        offsets = {field.name: field.offset for field in msg.fields}
        if not {"x", "y", "z"}.issubset(offsets) or msg.point_step <= 0:
            return
        max_offset = max(offsets["x"], offsets["y"], offsets["z"]) + 4
        if msg.point_step < max_offset:
            return
        tag_name = "tag" if "tag" in offsets else "livox_tag" if "livox_tag" in offsets else ""
        tag_offset = offsets.get(tag_name) if tag_name else None
        fields_by_name = {field.name: field for field in msg.fields}
        for base in range(0, len(msg.data) - max_offset + 1, msg.point_step):
            x = struct.unpack_from("<f", msg.data, base + offsets["x"])[0]
            y = struct.unpack_from("<f", msg.data, base + offsets["y"])[0]
            z = struct.unpack_from("<f", msg.data, base + offsets["z"])[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            is_noise = False
            if self.use_livox_tag_filter and tag_offset is not None:
                field = fields_by_name[tag_name]
                tag = self._read_numeric_field(msg.data, base + tag_offset, field.datatype)
                # Livox tag layouts differ by driver; non-zero high bits are treated as weak/noisy returns.
                is_noise = int(tag) not in (0, 1) and (int(tag) & 0b11111100) != 0
            yield x, y, z, is_noise

    @staticmethod
    def _read_numeric_field(data: bytes, offset: int, datatype: int) -> float:
        if datatype == PointField.FLOAT32:
            return float(struct.unpack_from("<f", data, offset)[0])
        if datatype == PointField.FLOAT64:
            return float(struct.unpack_from("<d", data, offset)[0])
        if datatype == PointField.UINT8:
            return float(struct.unpack_from("<B", data, offset)[0])
        if datatype == PointField.INT8:
            return float(struct.unpack_from("<b", data, offset)[0])
        if datatype == PointField.UINT16:
            return float(struct.unpack_from("<H", data, offset)[0])
        if datatype == PointField.INT16:
            return float(struct.unpack_from("<h", data, offset)[0])
        if datatype == PointField.UINT32:
            return float(struct.unpack_from("<I", data, offset)[0])
        if datatype == PointField.INT32:
            return float(struct.unpack_from("<i", data, offset)[0])
        return 0.0

    def _publish_points(self, points: list[tuple[float, float, float]], stamp: float, frame_id: str) -> None:
        # Repeated or reordered samples are not independent evidence of a person.
        if not math.isfinite(stamp) or stamp <= self.last_processed_stamp:
            return
        started = time.perf_counter()
        self.processing_phases_ms = {}
        self.render_markers = self.pub.get_subscription_count() > 0 and started - self.last_marker_time >= 0.2
        if self.render_markers:
            self.last_marker_time = started
        self.last_processed_stamp = stamp
        self.source_age_s = max(0.0, self.get_clock().now().nanoseconds * 1e-9 - stamp)
        if self.mujoco_people_authoritative and self._active_mujoco_detections(stamp):
            # Oracle positions are already detections; do not reconstruct them
            # through three geometric clustering passes and synthetic point grids.
            self._publish_clusters([], stamp, frame_id)
            self.processing_ms = (time.perf_counter() - started) * 1000.0
            return
        self.recent_points.append((stamp, points))
        min_stamp = stamp - self.point_accumulation_time
        while self.recent_points and self.recent_points[0][0] < min_stamp:
            self.recent_points.popleft()

        accumulated_count = sum(len(entry_points) for _, entry_points in self.recent_points)
        while self.recent_points and accumulated_count > self.max_accumulated_points:
            _, removed_points = self.recent_points.popleft()
            accumulated_count -= len(removed_points)

        accumulated_points = []
        robot_xy = self._lookup_robot_xy(rclpy.time.Time(nanoseconds=max(0,int(stamp*1e9))))
        for entry_stamp, entry_points in self.recent_points:
            for x, y, z in entry_points:
                if (self.near_accumulation_radius > 0 and robot_xy is not None and
                        stamp-entry_stamp > self.near_accumulation_time and
                        math.hypot(x-robot_xy[0],y-robot_xy[1]) < self.near_accumulation_radius):
                    continue  # Old near-person returns otherwise add unlabelled motion lag.
                accumulated_points.append((x, y, z))
        accumulated_points = self._voxel_downsample(accumulated_points)
        if self.enable_point_noise_filter:
            accumulated_points = self._filter_isolated_points(accumulated_points)
        self.processing_phases_ms["prepare"] = 1000.*(time.perf_counter()-started)
        self._publish_clusters(accumulated_points, stamp, frame_id)
        self.processing_ms = (time.perf_counter() - started) * 1000.0

    def _voxel_downsample(self, points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        leaf = max(0.0, self.voxel_leaf_size)
        if leaf <= 1e-4 or not points:
            return points
        voxels: dict[tuple[int, int, int], tuple[float, float, float, int]] = {}
        for x, y, z in points:
            key = (math.floor(x / leaf), math.floor(y / leaf), math.floor(z / leaf))
            if key in voxels:
                sx, sy, sz, count = voxels[key]
                voxels[key] = (sx + x, sy + y, sz + z, count + 1)
            else:
                voxels[key] = (x, y, z, 1)
        return [(sx / count, sy / count, sz / count) for sx, sy, sz, count in voxels.values()]

    def _filter_isolated_points(self, points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        if len(points) <= self.min_cluster_points or self.min_point_neighbors <= 0:
            return points
        robot_xy = self._lookup_robot_xy()
        max_radius = max(1e-3, self.far_noise_radius if self.range_adaptive_neighbor_filter else self.noise_filter_radius)
        grid: dict[tuple[int, int], list[int]] = {}
        for index, (x, y, _) in enumerate(points):
            grid.setdefault((math.floor(x / max_radius), math.floor(y / max_radius)), []).append(index)

        filtered = []
        for index, (x, y, z) in enumerate(points):
            point_range = math.hypot(x - robot_xy[0], y - robot_xy[1]) if robot_xy is not None else math.hypot(x, y)
            radius, min_neighbors = self._noise_filter_params(point_range)
            radius_sq = radius * radius
            cx = math.floor(x / max_radius)
            cy = math.floor(y / max_radius)
            neighbors = 0
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for other_index in grid.get((cx + dx, cy + dy), ()):
                        if other_index == index:
                            continue
                        ox, oy, _ = points[other_index]
                        if (ox - x) * (ox - x) + (oy - y) * (oy - y) <= radius_sq:
                            neighbors += 1
                            if neighbors >= min_neighbors:
                                filtered.append((x, y, z))
                                break
                    if neighbors >= min_neighbors:
                        break
                if neighbors >= min_neighbors:
                    break
        return filtered

    def _noise_filter_params(self, point_range: float) -> tuple[float, int]:
        if not self.range_adaptive_neighbor_filter:
            return max(1e-3, self.noise_filter_radius), max(1, self.min_point_neighbors)
        t = max(0.0, min(1.0, (point_range - 1.5) / 6.0))
        radius = self.near_noise_radius + t * (self.far_noise_radius - self.near_noise_radius)
        min_neighbors = round(self.min_neighbors_near + t * (self.min_neighbors_far - self.min_neighbors_near))
        return max(1e-3, radius), max(1, int(min_neighbors))

    def _publish_clusters(
        self,
        points: list[tuple[float, float, float]],
        stamp: float,
        frame_id: str,
    ) -> None:
        phase_started = time.perf_counter()
        robot_xy = self._lookup_robot_xy(rclpy.time.Time(nanoseconds=max(0, int(stamp * 1e9))))
        raw_clusters = self._clusters(points, robot_xy=robot_xy)
        self.last_cluster_count = len(raw_clusters)
        if self.use_object_motion_detection:
            clusters = self._dedupe_clusters(raw_clusters)
        else:
            person_clusters = self._person_focused_clusters(points, robot_xy)
            self._update_static_background(raw_clusters)
            foreground_points = [
                point
                for point in points
                if self.static_cells.get((math.floor(point[0] / self.cell_size), math.floor(point[1] / self.cell_size)), 0)
                < self.static_cell_observations
            ]
            clusters = self._dedupe_clusters(raw_clusters + self._clusters(foreground_points, robot_xy=robot_xy) + person_clusters)
        clusters, self.effective_people_source = self._effective_people_clusters(clusters, stamp)
        evidence = getattr(self,'wall_observation',None)
        wall = (WallEvidence(evidence[2],self.max_person_height+.15)
                if getattr(self,'reject_wall_supported_clusters',False) and evidence is not None and
                evidence[0]==stamp and evidence[1]==frame_id else None)
        def wall_supported(cluster):
            return (wall is not None and wall.supports(cluster.x,cluster.y) and
                    self._cluster_detection_confidence(cluster,stamp)<self.person_detection_confirm_confidence)
        eligible_clusters = []
        for cluster in clusters:
            self._annotate_cluster_quality(cluster, robot_xy)
            if self._is_robot_self_cluster(cluster, robot_xy):
                self.debug_events.append(self._cluster_debug_event(cluster, "rejected", "robot_self"))
                continue
            if self.use_object_motion_detection and not self._is_object_candidate_cluster(cluster, robot_xy):
                if self._partial_confirmed_observation(cluster, stamp):
                    cluster.partial_observation = True
                else:
                    self.debug_events.append(self._cluster_debug_event(cluster, "rejected", "object_gate"))
                    continue
            # Run the more expensive structural check only after the cheap
            # size/shape gate; rejected full wall clusters need no line fitting.
            if wall_supported(cluster):
                self.debug_events.append(self._cluster_debug_event(cluster,"rejected","wall_continuation"))
                continue
            if not self.use_object_motion_detection and self._is_livox_pattern_artifact(cluster):
                self.debug_events.append(self._cluster_debug_event(cluster, "rejected", "livox_pattern_artifact"))
                continue
            if (
                not self.use_object_motion_detection
                and self._is_static_background(cluster)
                and not self._near_confirmed_track(cluster, stamp)
                and self._cluster_detection_confidence(cluster, stamp) <= 0.0
            ):
                self.debug_events.append(self._cluster_debug_event(cluster, "static_suppressed", "static_background"))
                continue
            eligible_clusters.append(cluster)
        clusters = eligible_clusters
        if self.use_track_conditioned_crops:
            for cluster in self._track_conditioned_crops(points, clusters, stamp):
                if wall_supported(cluster):
                    self.debug_events.append(self._cluster_debug_event(cluster,"rejected","wall_continuation"))
                else:
                    clusters.append(cluster)
        if not hasattr(self, "processing_phases_ms"):
            self.processing_phases_ms = {}
        self.processing_phases_ms["cluster"] = 1000.*(time.perf_counter()-phase_started)
        phase_started = time.perf_counter()
        markers = MarkerArray()
        predicted_points: list[tuple[float, float, float]] = []
        tentative_points: list[tuple[float, float, float]] = []
        tracked_points: list[tuple[float, float, float]] = []
        typed_tracks: list[tuple[int, Track]] = []
        typed_predictions: list[tuple[int, Track]] = []
        seen_ids = set()
        visible_ids = set()
        deleted_ids = set()
        candidate_count = 0
        confirmed_count = 0
        assignments = self._global_track_assignments(clusters, stamp)
        for cluster_index, cluster in enumerate(clusters):
            if cluster.partial_observation and (cluster_index not in assignments or not self.tracks[assignments[cluster_index]].confirmed):
                continue  # A partial body return cannot create a new identity.
            track_id, track = self._match_track(cluster, stamp, seen_ids, assignments.get(cluster_index))
            seen_ids.add(track_id)
            if not track.confirmed or track.person_observations < max(1, self.min_person_observations - 1):
                if self._is_candidate_person_risk(cluster, track, robot_xy):
                    candidate_count += 1
                    typed_tracks.append((track_id, track))
                    if self.publish_tentative_predictions:
                        tentative_points.extend(
                            self._prediction_points(
                                cluster.x,
                                cluster.y,
                                0.6,
                                track.vx,
                                track.vy,
                                max(cluster.radius, self.candidate_person_risk_radius - 0.35) * self.tentative_prediction_radius_scale,
                                robot_xy,
                            )
                        )
                    self._append_candidate_marker(markers, track_id, cluster, frame_id)
                else:
                    self.debug_events.append(self._cluster_debug_event(cluster, "tentative", cluster.rejection_reason or "not_confirmed"))
                continue
            confirmed_count += 1
            visible_ids.add(track_id)
            self._append_track_outputs(markers, predicted_points, tracked_points, track_id, track, frame_id, robot_xy, coasting=False,
                render_marker=self.max_visible_confirmed_tracks <= 0 or confirmed_count <= self.max_visible_confirmed_tracks)
            typed_tracks.append((track_id, track))
            if self._should_publish_prediction(track, False, 0.0):
                typed_predictions.append((track_id, track))

        for track_id, track in list(self.tracks.items()):
            if track_id not in seen_ids:
                track.missed += 1
                track.hit_history.append(False)
                coast_time = stamp - track.stamp
                track.state = "OCCLUDED" if track.confirmed else "STALE"
                if (
                    self.coast_confirmed_tracks
                    and track.confirmed
                    and track.person_observations > 0
                    and coast_time <= min(self.max_lost_persistence, self.confirmed_keep_missed_time)
                ):
                    decay_dt = max(0.0, stamp - track.confidence_stamp)
                    track.confidence = max(0.0, track.confidence - self.confidence_decay_per_second * decay_dt)
                    track.confidence_stamp = stamp
                    coasted = Track(
                        x=track.x + track.vx * coast_time,
                        y=track.y + track.vy * coast_time,
                        stamp=stamp,
                        first_x=track.first_x,
                        first_y=track.first_y,
                        speed=track.speed,
                        vx=track.vx,
                        vy=track.vy,
                        age=track.age,
                        dynamic_observations=track.dynamic_observations,
                        person_observations=track.person_observations,
                        static_observations=track.static_observations,
                        missed=track.missed,
                        radius=track.radius,
                        width=track.width,
                        depth=track.depth,
                        height=track.height,
                        confirmed=track.confirmed,
                        confidence=track.confidence,
                        confidence_stamp=track.confidence_stamp,
                        person_score=track.person_score,
                        detection_confidence=track.detection_confidence,
                        state="OCCLUDED",
                        kf_p=_motion_covariance(track.kf_p, coast_time, getattr(self, "tracking_process_noise", .35)),
                        observation_age=coast_time,
                        history=track.history,
                    )
                    if coasted.confidence >= self.drop_confidence_threshold:
                        visible_ids.add(track_id)
                        self._append_track_outputs(
                            markers,
                            predicted_points,
                            tracked_points,
                            track_id,
                            coasted,
                            frame_id,
                            robot_xy,
                            coasting=True,
                            coast_time=coast_time,
                        )
                        typed_tracks.append((track_id, coasted))
                        if self._should_publish_prediction(coasted, True, coast_time):
                            typed_predictions.append((track_id, coasted))
            track_age = stamp - track.stamp
            if _track_should_expire(
                track,
                track_age,
                self.drop_confidence_threshold,
                min(self.tentative_delete_missed_time, self.stale_track_timeout),
                self.max_lost_persistence,
            ):
                deleted_ids.add(track_id)
                track.state = "DELETED"
                self.debug_events.append(self._track_debug_event(track_id, track, "deleted"))
                markers.markers.extend(self._delete_track_markers(track_id, frame_id))
                del self.tracks[track_id]
        self.visible_track_ids = (self.visible_track_ids | visible_ids) - deleted_ids
        self.last_candidate_count = candidate_count
        self.last_confirmed_count = confirmed_count
        self.last_predicted_point_count = len(predicted_points)
        self.processing_phases_ms["track"] = 1000.*(time.perf_counter()-phase_started)
        phase_started = time.perf_counter()
        if self.render_markers:
            self.pub.publish(self._stable_marker_array(markers, stamp))
        self.prediction_pub.publish(self._prediction_cloud(predicted_points, frame_id, stamp))
        self.tracked_pub.publish(self._prediction_cloud(tracked_points, frame_id, stamp))
        self.tentative_prediction_pub.publish(self._prediction_cloud(tentative_points, frame_id, stamp))
        self.tracked_persons_pub.publish(self._tracked_persons_message(typed_tracks, frame_id, stamp))
        self.predicted_trajectories_pub.publish(
            self._predicted_trajectories_message(typed_predictions, frame_id, stamp)
        )
        self.processing_phases_ms["publish"] = 1000.*(time.perf_counter()-phase_started)
        self._publish_debug_tracks(stamp)
        if self.publish_safety_grid:
            self.safety_grid_pub.publish(
                self._safety_grid(predicted_points, tracked_points, frame_id, robot_xy)
            )

    def _stable_marker_array(self, current: MarkerArray, stamp: float) -> MarkerArray:
        for marker in current.markers:
            marker.header.stamp = rclpy.time.Time().to_msg()
        if self.marker_display_hold_time <= 0.0:
            return current

        delete_markers = []
        for marker in current.markers:
            key = (marker.ns, marker.id)
            if marker.action == Marker.DELETE:
                self.display_markers.pop(key, None)
                delete_markers.append(marker)
                continue
            if marker.action == Marker.ADD:
                old = self.display_markers.get(key)
                if old is not None and marker.ns == "tracked_people":
                    marker.pose.position.x = .7*marker.pose.position.x + .3*old[0].pose.position.x
                    marker.pose.position.y = .7*marker.pose.position.y + .3*old[0].pose.position.y
                self.display_markers[key] = (marker, stamp + self.marker_display_hold_time)

        stable = MarkerArray()
        stable.markers.extend(delete_markers)
        for key, (marker, expires_at) in list(self.display_markers.items()):
            if stamp > expires_at:
                deletion = Marker()
                deletion.header.frame_id = marker.header.frame_id
                deletion.ns, deletion.id, deletion.action = marker.ns, marker.id, Marker.DELETE
                stable.markers.append(deletion)
                del self.display_markers[key]
                continue
            # Visualization uses latest TF. Stamping these with 'now' asks RViz
            # for an ego transform newer than the most recent LiDAR odometry.
            marker.header.stamp = rclpy.time.Time().to_msg()
            stable.markers.append(marker)
        return stable

    def _refresh_person_markers(self):
        """Keep display orbs steady between detections, without refreshing tracks."""
        if not self.display_markers or self.pub.get_subscription_count() == 0:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        markers = MarkerArray()
        for key, (cached, expiry) in list(self.display_markers.items()):
            if now > expiry:
                deletion = Marker()
                deletion.header.frame_id = cached.header.frame_id
                deletion.ns, deletion.id, deletion.action = cached.ns, cached.id, Marker.DELETE
                markers.markers.append(deletion)
                del self.display_markers[key]
                continue
            if cached.ns not in ("tracked_people", "dynamic_obstacle_labels"):
                continue
            marker = copy.deepcopy(cached)
            marker.header.stamp = rclpy.time.Time().to_msg()
            age = now - (expiry-self.marker_display_hold_time)
            if age > .5:
                marker.color.r, marker.color.g, marker.color.b = 1., .65, .1
                marker.color.a = max(.25, .7*(expiry-now)/max(.1, self.marker_display_hold_time-.5))
                if marker.ns == "dynamic_obstacle_labels":
                    marker.text = marker.text.split("\n")[0] + "\nSTALE/OBSERVATION"
            markers.markers.append(marker)
        if markers.markers:
            self.pub.publish(markers)

    def _diagnostics(self) -> None:
        self.get_logger().info(
            "pedestrian tracker: "
            f"clusters={self.last_cluster_count} candidates={self.last_candidate_count} "
            f"confirmed={self.last_confirmed_count} tracks={len(self.tracks)} "
            f"ml_detections={len(self.person_detections)} mujoco_detections={len(self.mujoco_detections)} "
            f"prediction_points={self.last_predicted_point_count}",
            throttle_duration_sec=2.0,
        )

    def _publish_debug_tracks(self, stamp: float) -> None:
        tracks = []
        for track_id, track in self.tracks.items():
            tracks.append(self._track_debug_event(track_id, track, "active"))
        payload = {
            "stamp": stamp,
            "previous_processing_ms": round(self.processing_ms, 3),
            "processing_phases_ms": {key: round(value, 3) for key, value in getattr(self, "processing_phases_ms", {}).items()},
            "source_age_s": round(self.source_age_s, 4),
            "clusters": self.last_cluster_count,
            "candidates": self.last_candidate_count,
            "confirmed": self.last_confirmed_count,
            "ml_detections": len(self.person_detections),
            "mujoco_detections": len(self.mujoco_detections),
            "effective_people_source": self.effective_people_source,
            "prediction_points": self.last_predicted_point_count,
            "tracks": tracks,
            "events": self.debug_events[-40:],
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.debug_pub.publish(msg)
        self.debug_events.clear()

    def _track_debug_event(self, track_id: int, track: Track, event: str) -> dict[str, object]:
        return {
            "event": event,
            "track_id": track_id,
            "state": track.state,
            "age": track.age,
            "age_sec": max(0.0, track.stamp - track.first_stamp) if track.first_stamp else 0.0,
            "confidence": round(track.confidence, 3),
            "person_score": round(track.person_score, 3),
            "detection_count": track.person_observations,
            "dynamic_count": track.dynamic_observations,
            "missed_count": track.missed,
            "association_distance": round(track.last_association_distance, 3) if math.isfinite(track.last_association_distance) else None,
            "speed": round(track.speed, 3),
            "body_radius": round(max(getattr(self, "tracked_body_radius_floor", 0.), track.radius), 3),
            "width": round(track.width, 3),
            "height": round(track.height, 3),
            "last_full_observation_stamp": track.last_full_stamp,
            "depth": round(track.depth, 3),
            "vx": round(track.vx, 3),
            "vy": round(track.vy, 3),
            "x": round(track.x, 3),
            "y": round(track.y, 3),
            "cov_xx": round(track.kf_p[0][0], 4),
            "cov_yy": round(track.kf_p[1][1], 4),
            "rejection_reason": track.rejection_reason,
        }

    @staticmethod
    def _cluster_debug_event(cluster: Cluster, state: str, reason: str) -> dict[str, object]:
        return {
            "event": "cluster",
            "state": state,
            "reason": reason,
            "x": round(cluster.x, 3),
            "y": round(cluster.y, 3),
            "range": round(cluster.range, 3),
            "points": cluster.count,
            "height": round(cluster.height, 3),
            "top_height": None if cluster.z_max is None else round(cluster.z_max, 3),
            "z_bins": cluster.vertical_layers,
            "width": round(cluster.width, 3),
            "depth": round(cluster.depth, 3),
            "density": round(cluster.density, 3),
            "static_fraction": round(cluster.static_fraction, 3),
            "noise_fraction": round(cluster.noise_fraction, 3),
        }

    def _dedupe_clusters(self, clusters: list[Cluster]) -> list[Cluster]:
        ordered = sorted(
            clusters,
            key=lambda cluster: (
                not self._is_person_sized(cluster, 0.0),
                cluster.radius,
                -cluster.count,
            ),
        )
        result = []
        for cluster in ordered:
            duplicate = False
            for kept in result:
                center_distance = math.hypot(cluster.x - kept.x, cluster.y - kept.y)
                if center_distance <= max(0.20, min(cluster.radius, kept.radius, 0.45)):
                    duplicate = True
                    break
            if not duplicate:
                result.append(cluster)
        return result

    def _lookup_robot_xy(self, query_time=None) -> tuple[float, float] | None:
        sample = getattr(self, "robot_odometry", None)
        if sample is not None:
            stamp, x, y, vx, vy = sample
            now = self.get_clock().now().nanoseconds * 1e-9
            requested = now if query_time is None else query_time.nanoseconds * 1e-9
            dt = requested - stamp
            if 0.0 <= now - stamp <= .6 and abs(dt) <= .6:
                return x + vx * dt, y + vy * dt
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.robot_frame,
                query_time if query_time is not None else rclpy.time.Time(),
            )
        except TransformException:
            return None
        return transform.transform.translation.x, transform.transform.translation.y

    def _robot_odometry_callback(self, msg):
        if msg.header.frame_id != self.target_frame:
            return
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        velocity = msg.twist.twist.linear
        self.robot_odometry = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            c * velocity.x - s * velocity.y, s * velocity.x + c * velocity.y)

    def _outside_local_tracking_radius(self, x: float, y: float, robot_xy: tuple[float, float] | None) -> bool:
        if robot_xy is None or self.local_tracking_radius <= 0.0:
            return False
        return math.hypot(x - robot_xy[0], y - robot_xy[1]) > self.local_tracking_radius

    def _kf_predict(self, track: Track, dt: float) -> tuple[float, float, float, float, list[list[float]]]:
        dt = max(0.0, dt)
        x = track.x + track.vx * dt
        y = track.y + track.vy * dt
        vx = track.vx
        vy = track.vy
        p = _motion_covariance(track.kf_p, dt, getattr(self, "tracking_process_noise", .35))
        return x, y, vx, vy, p

    def _kf_update(self, track: Track, measured_x: float, measured_y: float, dt: float, noise_scale: float = 1.) -> None:
        pred_x, pred_y, pred_vx, pred_vy, p = self._kf_predict(track, dt)
        variance = getattr(self, "measurement_noise_variance", .02) * noise_scale
        measurement_noise = variance * (1. + min(4., .5 * max(0., track.radius / max(.05, track.width+track.depth))))
        s00 = p[0][0] + measurement_noise
        s01 = p[0][1]
        s10 = p[1][0]
        s11 = p[1][1] + measurement_noise
        det = s00 * s11 - s01 * s10
        if abs(det) < 1e-9:
            return
        inv00 = s11 / det
        inv01 = -s01 / det
        inv10 = -s10 / det
        inv11 = s00 / det
        innovation_x = measured_x - pred_x
        innovation_y = measured_y - pred_y
        gains = []
        for row in range(4):
            gains.append((p[row][0] * inv00 + p[row][1] * inv10, p[row][0] * inv01 + p[row][1] * inv11))
        state = [pred_x, pred_y, pred_vx, pred_vy]
        for row in range(4):
            state[row] += gains[row][0] * innovation_x + gains[row][1] * innovation_y
        i_kh = [[float(row == col) for col in range(4)] for row in range(4)]
        for row in range(4):
            i_kh[row][0] -= gains[row][0]
            i_kh[row][1] -= gains[row][1]
        track.kf_p = [[sum(i_kh[row][k] * p[k][col] for k in range(4)) for col in range(4)] for row in range(4)]
        track.x, track.y, track.vx, track.vy = state

    @staticmethod
    def _mahalanobis_xy(
        x: float,
        y: float,
        predicted_x: float,
        predicted_y: float,
        p: list[list[float]],
    ) -> float:
        s00 = p[0][0] + 0.12
        s01 = p[0][1]
        s10 = p[1][0]
        s11 = p[1][1] + 0.12
        det = s00 * s11 - s01 * s10
        if abs(det) < 1e-9:
            return math.inf
        dx = x - predicted_x
        dy = y - predicted_y
        inv00 = s11 / det
        inv01 = -s01 / det
        inv10 = -s10 / det
        inv11 = s00 / det
        return math.sqrt(max(0.0, dx * (inv00 * dx + inv01 * dy) + dy * (inv10 * dx + inv11 * dy)))

    def _is_robot_self_cluster(self, cluster: Cluster, robot_xy: tuple[float, float] | None) -> bool:
        if robot_xy is None or self.robot_self_filter_radius <= 0.0:
            return False
        return math.hypot(cluster.x - robot_xy[0], cluster.y - robot_xy[1]) <= self.robot_self_filter_radius

    @staticmethod
    def _transform_point(transform, x: float, y: float, z: float) -> tuple[float, float, float]:
        q = transform.transform.rotation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        # Quaternion rotation, expanded to avoid an extra dependency.
        uvx = q.y * z - q.z * y
        uvy = q.z * x - q.x * z
        uvz = q.x * y - q.y * x
        uuvx = q.y * uvz - q.z * uvy
        uuvy = q.z * uvx - q.x * uvz
        uuvz = q.x * uvy - q.y * uvx
        rx = x + 2.0 * (q.w * uvx + uuvx)
        ry = y + 2.0 * (q.w * uvy + uuvy)
        rz = z + 2.0 * (q.w * uvz + uuvz)
        return rx + tx, ry + ty, rz + tz

    def _clusters(
        self,
        points: list[tuple[float, float, float]],
        cluster_distance: float | None = None,
        robot_xy: tuple[float, float] | None = None,
    ) -> list[Cluster]:
        if not points:
            return []
        base_distance = self.cluster_distance if cluster_distance is None else cluster_distance
        grid_size = max(self.cluster_epsilon_max if self.range_adaptive_cluster_distance else base_distance, base_distance, 1e-3)
        radii = [self._cluster_epsilon(x, y, robot_xy, base_distance) for x, y, _ in points]
        result = []
        for cluster_indexes in connected_components(points, radii, grid_size, self.cluster_z_distance):
            count = 0
            sum_x = 0.0
            sum_y = 0.0
            sum_z = 0.0
            min_x = math.inf
            max_x = -math.inf
            min_y = math.inf
            max_y = -math.inf
            min_z = math.inf
            max_z = -math.inf
            z_layers = set()
            cells = set()
            torso_x = torso_y = 0.
            torso_count = 0
            for point_index in cluster_indexes:
                px, py, pz = points[point_index]
                cells.add((math.floor(px / self.cell_size), math.floor(py / self.cell_size)))
                z_layers.add(math.floor(pz / max(1e-3, self.person_vertical_layer_height)))
                count += 1
                sum_x += px
                sum_y += py
                sum_z += pz
                min_x = min(min_x, px)
                max_x = max(max_x, px)
                min_y = min(min_y, py)
                max_y = max(max_y, py)
                min_z = min(min_z, pz)
                max_z = max(max_z, pz)
                if self.use_torso_centroid and .8 <= pz <= 1.5:
                    torso_x += px
                    torso_y += py
                    torso_count += 1
            if count < self.min_cluster_points:
                continue
            mean_x = sum_x / count
            mean_y = sum_y / count
            mean_z = sum_z / count
            if torso_count >= 6:
                # Keep shape gates/radius based on the complete cluster, but
                # do not let changing foot/head sampling shift the tracked XY.
                mean_x,mean_y = torso_x/torso_count,torso_y/torso_count
            radius = 0.0
            for point_index in cluster_indexes:
                px, py, _ = points[point_index]
                radius = max(radius, math.hypot(px - mean_x, py - mean_y))
            width = max_x - min_x
            depth = max_y - min_y
            height = max_z - min_z
            if radius <= self.max_cluster_radius and count >= self._min_cluster_points_for_range(mean_x, mean_y, robot_xy):
                cluster = Cluster(mean_x, mean_y, mean_z, radius, count, width, depth, height, len(z_layers), cells, z_max=max_z)
                self._annotate_cluster_quality(cluster, robot_xy)
                result.append(cluster)
        return self._merge_person_fragments(result)

    def _cluster_epsilon(
        self,
        x: float,
        y: float,
        robot_xy: tuple[float, float] | None,
        fallback: float,
    ) -> float:
        if not self.range_adaptive_cluster_distance:
            return fallback
        point_range = math.hypot(x - robot_xy[0], y - robot_xy[1]) if robot_xy is not None else math.hypot(x, y)
        epsilon = self.cluster_epsilon_base + self.cluster_epsilon_per_meter * point_range
        return max(self.cluster_epsilon_min, min(self.cluster_epsilon_max, epsilon))

    def _min_cluster_points_for_range(
        self, x: float, y: float, robot_xy: tuple[float, float] | None
    ) -> int:
        if not self.min_cluster_points_by_range:
            return self.min_cluster_points
        point_range = math.hypot(x - robot_xy[0], y - robot_xy[1]) if robot_xy is not None else math.hypot(x, y)
        return max(self.min_cluster_points, int(round(self.min_cluster_points + max(0.0, point_range - 3.0) * 0.45)))

    def _annotate_cluster_quality(self, cluster: Cluster, robot_xy: tuple[float, float] | None) -> None:
        cluster.range = math.hypot(cluster.x - robot_xy[0], cluster.y - robot_xy[1]) if robot_xy is not None else math.hypot(cluster.x, cluster.y)
        cluster.bbox_volume = max(1e-4, cluster.width * cluster.depth * max(cluster.height, 0.05))
        cluster.density = cluster.count / cluster.bbox_volume
        xy_area = max(1e-4, cluster.width * cluster.depth)
        cluster.compactness = min(1.0, cluster.count / (xy_area * 120.0))
        if min(cluster.width, cluster.depth) > 1e-3:
            cluster.aspect_ratio = max(cluster.width, cluster.depth) / min(cluster.width, cluster.depth)
        else:
            cluster.aspect_ratio = math.inf
        if cluster.cells:
            static_hits = sum(1 for cell in cluster.cells if self.static_cells.get(cell, 0) >= self.static_cell_observations)
            cluster.static_fraction = static_hits / len(cluster.cells)
        else:
            cluster.static_fraction = 0.0

    def _person_focused_clusters(self, points: list[tuple[float, float, float]], robot_xy: tuple[float, float] | None) -> list[Cluster]:
        """Second pass tuned for separating pedestrians from nearby static geometry."""
        tight_clusters = self._clusters(points, max(0.12, self.person_cluster_distance), robot_xy=robot_xy)
        result = []
        for cluster in tight_clusters:
            if cluster.count < self.min_cluster_points:
                continue
            if cluster.count > self.max_cluster_points:
                continue
            if max(cluster.width, cluster.depth, cluster.radius * 2.0) > self.max_person_extent:
                continue
            if cluster.radius > self.max_person_radius * 1.10:
                continue
            if cluster.height > self.max_person_height:
                continue
            if cluster.vertical_layers < 1:
                continue
            if self._is_livox_pattern_artifact(cluster):
                continue
            score = self._person_score(cluster, 0.0)
            if score >= max(0.45, self.candidate_person_score_threshold - 0.12):
                result.append(cluster)
        return self._merge_person_fragments(result)

    def _merge_person_fragments(self, clusters: list[Cluster]) -> list[Cluster]:
        merged = []
        used = set()
        for index, cluster in enumerate(clusters):
            if index in used:
                continue
            used.add(index)
            current = cluster
            changed = True
            while changed:
                changed = False
                for other_index, other in enumerate(clusters):
                    if other_index in used:
                        continue
                    if not self._can_merge_person_fragments(current, other):
                        continue
                    used.add(other_index)
                    current = self._combined_cluster(current, other)
                    changed = True
            merged.append(current)
        return merged

    def _can_merge_person_fragments(self, first: Cluster, second: Cluster) -> bool:
        distance = math.hypot(first.x - second.x, first.y - second.y)
        if distance > self.person_fragment_merge_distance:
            return False
        combined = self._combined_cluster(first, second)
        combined_extent = max(combined.width, combined.depth, combined.radius * 2.0)
        return (
            combined.radius <= self.max_person_radius
            and combined_extent <= self.max_person_extent
            and combined.height <= self.max_person_height
            and combined.count <= self.max_cluster_points
        )

    def _combined_cluster(self, first: Cluster, second: Cluster) -> Cluster:
        count = first.count + second.count
        x = (first.x * first.count + second.x * second.count) / count
        y = (first.y * first.count + second.y * second.count) / count
        z = (first.z * first.count + second.z * second.count) / count
        min_x = min(first.x - first.width * 0.5, second.x - second.width * 0.5)
        max_x = max(first.x + first.width * 0.5, second.x + second.width * 0.5)
        min_y = min(first.y - first.depth * 0.5, second.y - second.depth * 0.5)
        max_y = max(first.y + first.depth * 0.5, second.y + second.depth * 0.5)
        min_z = min(first.z - first.height * 0.5, second.z - second.height * 0.5)
        max_z = max(first.z + first.height * 0.5, second.z + second.height * 0.5)
        width = max_x - min_x
        depth = max_y - min_y
        height = max_z - min_z
        radius = max(
            math.hypot(min_x - x, min_y - y),
            math.hypot(min_x - x, max_y - y),
            math.hypot(max_x - x, min_y - y),
            math.hypot(max_x - x, max_y - y),
        )
        return Cluster(
            x,
            y,
            z,
            radius,
            count,
            width,
            depth,
            height,
            max(first.vertical_layers, second.vertical_layers),
            first.cells | second.cells,
            z_max=max(first.z_max if first.z_max is not None else first.z+first.height*.5,
                      second.z_max if second.z_max is not None else second.z+second.height*.5),
        )

    def _global_track_assignments(self, clusters: list[Cluster], stamp: float) -> dict[int, int]:
        pairs: list[tuple[float, int, int]] = []
        for cluster_index, cluster in enumerate(clusters):
            for track_id, track in self.tracks.items():
                cost = self._association_cost(cluster, track, stamp)
                if math.isfinite(cost):
                    pairs.append((cost, cluster_index, track_id))
        pairs.sort(key=lambda item: item[0])
        assignments: dict[int, int] = {}
        used_tracks = set()
        for _, cluster_index, track_id in pairs:
            if cluster_index in assignments or track_id in used_tracks:
                continue
            assignments[cluster_index] = track_id
            used_tracks.add(track_id)
        return assignments

    def _association_cost(self, cluster: Cluster, track: Track, stamp: float) -> float:
        if cluster.partial_observation and not self._partial_track_gate(cluster,track,stamp):
            return math.inf
        if stamp <= track.stamp:
            return math.inf
        dt = max(0.0, stamp - track.stamp)
        predicted_x, predicted_y, _, _, predicted_p = self._kf_predict(track, min(dt, self.prediction_horizon))
        distance = math.hypot(cluster.x - predicted_x, cluster.y - predicted_y)
        mahalanobis = self._mahalanobis_xy(cluster.x, cluster.y, predicted_x, predicted_y, predicted_p)
        if mahalanobis > self.mahalanobis_gate:
            return math.inf
        gate = self.track_match_distance + min(0.9, 0.35 * track.speed + 0.12 * track.missed)
        if track.confirmed:
            gate = min(gate, self.max_confirmed_measurement_jump + min(0.35, 0.15 * track.missed))
        if distance > gate:
            return math.inf
        # Bound displacement from the last *measurement*, not filtered-position
        # error divided by dt twice. The latter rejects normal walking at high Hz.
        if track.measurements:
            observed_stamp, observed_x, observed_y = track.measurements[-1]
            allowance = self.max_person_speed * (stamp - observed_stamp) + 0.30
            if math.hypot(cluster.x - observed_x, cluster.y - observed_y) > allowance:
                return math.inf
        shape_penalty = 0.35 * abs(cluster.radius - track.radius) + 0.15 * abs(cluster.width - track.width) + 0.15 * abs(cluster.depth - track.depth)
        quality_penalty = 0.4 * cluster.static_fraction + 0.5 * cluster.noise_fraction
        continuity_bonus = -0.25 if track.confirmed else 0.0
        return 0.65 * distance + 0.18 * mahalanobis + shape_penalty + quality_penalty + continuity_bonus

    def _match_track(
        self,
        cluster: Cluster,
        stamp: float,
        already_matched: set[int],
        preferred_track_id: int | None = None,
    ) -> tuple[int, Track]:
        previous_velocity = None
        best_id = None
        best_score = math.inf
        best_distance = math.inf
        if preferred_track_id is not None and preferred_track_id not in already_matched and preferred_track_id in self.tracks:
            preferred = self.tracks[preferred_track_id]
            best_score = self._association_cost(cluster, preferred, stamp)
            if math.isfinite(best_score):
                pred_x, pred_y, _, _, _ = self._kf_predict(preferred, max(0.0, stamp - preferred.stamp))
                best_distance = math.hypot(cluster.x - pred_x, cluster.y - pred_y)
                best_id = preferred_track_id
        if best_id is None:
            best_id = self.next_track_id
            self.next_track_id += 1
            track = Track(
                cluster.x,
                cluster.y,
                stamp,
                first_x=cluster.x,
                first_y=cluster.y,
                radius=cluster.radius,
                width=cluster.width,
                depth=cluster.depth,
                height=cluster.height,
                first_stamp=stamp,
                confidence_stamp=stamp,
            )
            track.hit_history = deque(maxlen=max(1, self.tentative_confirm_window))
            track.history.append((cluster.x, cluster.y))
            track.measurements.append((stamp, cluster.x, cluster.y))
            best_distance = 0.0
        else:
            track = self.tracks[best_id]
            previous_velocity = (track.vx, track.vy)
            dt = max(1e-3, stamp - track.stamp)
            track.measurements.append((stamp, cluster.x, cluster.y))
            # Bootstrap using a short least-squares window. Subsequent updates
            # use the CV filter, which can respond to stops and turns directly.
            window = [sample for sample in track.measurements if stamp - sample[0] <= 0.40]
            if stamp - track.first_stamp <= 0.50 and len(window) >= 2 and window[-1][0] - window[0][0] >= 0.09:
                mean_t = sum(sample[0] - stamp for sample in window) / len(window)
                mean_x = sum(sample[1] for sample in window) / len(window)
                mean_y = sum(sample[2] for sample in window) / len(window)
                denominator = sum((sample[0] - stamp - mean_t) ** 2 for sample in window)
                track.vx = sum((t - stamp - mean_t) * (x - mean_x) for t, x, _ in window) / denominator
                track.vy = sum((t - stamp - mean_t) * (y - mean_y) for t, _, y in window) / denominator
            if track.last_full_stamp is None:
                track.last_full_stamp = track.stamp
            self._kf_update(track, cluster.x, cluster.y, dt, noise_scale=4. if cluster.partial_observation else 1.)
            speed = math.hypot(track.vx, track.vy)
            if speed > self.max_person_speed:
                track.vx *= self.max_person_speed / speed
                track.vy *= self.max_person_speed / speed
            track.speed = math.hypot(track.vx, track.vy)
            track.stamp = stamp
            track.age += 1
            track.missed = 0
            if not cluster.partial_observation:
                track.radius = 0.70 * track.radius + 0.30 * cluster.radius
                track.width = 0.70 * track.width + 0.30 * cluster.width
                track.depth = 0.70 * track.depth + 0.30 * cluster.depth
                track.height = 0.70 * track.height + 0.30 * cluster.height
            if not track.history or math.hypot(track.x - track.history[-1][0], track.y - track.history[-1][1]) >= 0.03:
                track.history.append((track.x, track.y))
        if cluster.measured_vx is not None and cluster.measured_vy is not None:
            track.x = cluster.x
            track.y = cluster.y
            track.vx = cluster.measured_vx
            track.vy = cluster.measured_vy
            track.speed = math.hypot(track.vx, track.vy)
            track.kf_p = [
                [0.02, 0.0, 0.0, 0.0],
                [0.0, 0.02, 0.0, 0.0],
                [0.0, 0.0, 0.04, 0.0],
                [0.0, 0.0, 0.0, 0.04],
            ]
        if previous_velocity is not None:
            ax = (track.vx - previous_velocity[0]) / dt
            ay = (track.vy - previous_velocity[1]) / dt
            magnitude = math.hypot(ax, ay)
            if magnitude > self.max_person_acceleration:
                ax *= self.max_person_acceleration / magnitude
                ay *= self.max_person_acceleration / magnitude
            alpha = dt / (0.15 + dt)
            track.ax += alpha * (ax - track.ax)
            track.ay += alpha * (ay - track.ay)
        track.last_association_distance = best_distance if math.isfinite(best_distance) else 0.0

        detection_confidence = self._cluster_detection_confidence(cluster, stamp)
        track.detection_confidence = 0.70 * track.detection_confidence + 0.30 * detection_confidence
        if self.use_object_motion_detection:
            object_score = self._object_score(cluster)
            person_shape_score = self._person_score(cluster, min(track.speed, self.max_person_speed))
            base_score = min(object_score, person_shape_score + 0.10)
        else:
            base_score = self._person_score(cluster, track.speed)
        person_score = min(1.0, base_score + self.person_detection_score_boost * detection_confidence)
        if cluster.partial_observation and track.confirmed:
            # Near the scanner the full body leaves the vertical FOV. Retain the
            # observed identity through a gated position update, without treating
            # its clipped height as evidence that the person ceased to exist.
            person_score = max(person_score, track.person_score)
        if track.age <= 1 and track.person_score <= 0.0:
            track.person_score = person_score
        else:
            track.person_score = 0.65 * track.person_score + 0.35 * person_score
        valid_person_hit = False
        if cluster.partial_observation and track.confirmed:
            valid_person_hit = True
            track.confidence = max(self.drop_confidence_threshold, track.confidence - 0.10 * dt)
        elif detection_confidence >= self.person_detection_confirm_confidence:
            track.person_observations = min(track.person_observations + 2, 24)
            track.confidence = min(1.0, track.confidence + 0.25 * detection_confidence)
            valid_person_hit = True
        elif person_score >= 0.85:
            track.person_observations = min(track.person_observations + 2, 24)
            track.confidence = min(1.0, track.confidence + 0.22)
            valid_person_hit = True
        elif person_score >= self.confirmed_person_score_threshold:
            track.person_observations = min(track.person_observations + 1, 24)
            track.confidence = min(1.0, track.confidence + 0.12)
            valid_person_hit = True
        else:
            track.person_observations = max(0, track.person_observations - 1)
            track.confidence = max(0.0, track.confidence - 0.08)
        if valid_person_hit and not cluster.partial_observation:
            track.last_full_stamp = stamp
        if not cluster.partial_observation and track.speed >= self.min_dynamic_speed:
            track.dynamic_observations = min(track.dynamic_observations + 1, 20)
            track.static_observations = max(0, track.static_observations - 1)
            track.confidence = min(1.0, track.confidence + 0.05)
        elif not cluster.partial_observation and track.speed <= self.max_static_speed:
            track.dynamic_observations = max(0, track.dynamic_observations - 1)
            track.static_observations = min(track.static_observations + 1, 30)
        track.hit_history.append(valid_person_hit)
        track.confirmed = self._is_confirmed_person(track)
        if track.confirmed:
            track.state = "CONFIRMED"
        elif track.missed > 0:
            track.state = "OCCLUDED"
        else:
            track.state = "TENTATIVE"
        track.confidence_stamp = stamp
        self.tracks[best_id] = track
        return best_id, track

    def _is_person_sized(self, cluster: Cluster, speed: float) -> bool:
        if self.use_object_motion_detection:
            return self._object_score(cluster) >= self.object_score_threshold
        return self._person_score(cluster, speed) >= self.confirmed_person_score_threshold

    def _cluster_detection_confidence(self, cluster: Cluster, stamp: float) -> float:
        detections = self._active_person_detections(stamp)
        if not detections:
            return 0.0
        best = 0.0
        gate = max(0.10, self.person_detection_association_radius)
        for detection in detections:
            distance = math.hypot(cluster.x - detection.x, cluster.y - detection.y)
            radius_gate = max(gate, cluster.radius + detection.radius)
            if distance > radius_gate:
                continue
            association = 1.0 - distance / max(1e-3, radius_gate)
            best = max(best, detection.confidence * association)
        return best

    def _active_person_detections(self, stamp: float) -> list[PersonDetection]:
        detections = []
        if self.use_ml_person_detections:
            detections.extend(
                detection
                for detection in self.person_detections
                if stamp - detection.stamp <= self.person_detection_timeout
            )
        if self.use_mujoco_people_detections:
            detections.extend(
                detection
                for detection in self.mujoco_detections
                if stamp - detection.stamp <= self.mujoco_people_timeout
            )
        return detections

    def _active_mujoco_detections(self, stamp: float) -> list[PersonDetection]:
        if not self.use_mujoco_people_detections:
            return []
        return [
            detection
            for detection in self.mujoco_detections
            if stamp - detection.stamp <= self.mujoco_people_timeout
        ]

    def _mujoco_authoritative_clusters(self, clusters: list[Cluster], stamp: float) -> list[Cluster]:
        detections = self._active_mujoco_detections(stamp)
        if not detections:
            return clusters
        return [self._cluster_from_detection(detection) for detection in detections]

    def _effective_people_clusters(
        self, clusters: list[Cluster], stamp: float
    ) -> tuple[list[Cluster], str]:
        if not self.mujoco_people_authoritative:
            return clusters, "sensor_or_fused"
        detections = self._active_mujoco_detections(stamp)
        if not detections:
            return clusters, "sensor_or_fused"
        return [self._cluster_from_detection(detection) for detection in detections], "mujoco_truth_oracle"

    def _cluster_from_detection(self, detection: PersonDetection) -> Cluster:
        radius = max(0.25, detection.radius)
        diameter = radius * 2.0
        cells = {
            (math.floor((detection.x + dx) / self.cell_size), math.floor((detection.y + dy) / self.cell_size))
            for dx in (-radius * 0.5, radius * 0.5)
            for dy in (-radius * 0.5, radius * 0.5)
        }
        return Cluster(
            x=detection.x,
            y=detection.y,
            z=self.mujoco_person_height * 0.5,
            radius=radius,
            count=20,
            width=diameter,
            depth=diameter,
            height=self.mujoco_person_height,
            vertical_layers=4,
            cells=cells,
            range=math.hypot(detection.x, detection.y),
            density=20.0 / max(1e-3, diameter * diameter * self.mujoco_person_height),
            bbox_volume=max(1e-3, diameter * diameter * self.mujoco_person_height),
            compactness=0.5,
            aspect_ratio=1.0,
            measured_vx=detection.vx,
            measured_vy=detection.vy,
        )

    def _object_score(self, cluster: Cluster) -> float:
        extent = max(cluster.width, cluster.depth, cluster.radius * 2.0)
        if self._is_livox_pattern_artifact(cluster):
            return 0.0
        if cluster.count < self.object_min_points or cluster.count > self.object_max_points:
            return 0.0
        if len(cluster.cells) < self.object_min_cells:
            return 0.0
        if cluster.radius > self.object_max_radius or extent > self.object_max_extent:
            return 0.0
        if cluster.height < self.object_min_height or cluster.height > self.object_max_height:
            return 0.0

        score = 0.20
        if self.object_min_radius <= cluster.radius <= self.object_max_radius:
            score += 0.22
        if extent <= self.object_max_extent:
            score += 0.18
        if cluster.count >= self.object_min_points:
            score += min(0.18, 0.025 * cluster.count)
        if cluster.vertical_layers >= 1:
            score += 0.08
        if cluster.height >= 0.15:
            score += 0.10
        if min(cluster.width, cluster.depth) > 0.04:
            aspect = max(cluster.width, cluster.depth) / min(cluster.width, cluster.depth)
            if aspect <= 4.0:
                score += 0.12
            elif aspect <= 6.0:
                score += 0.04
        return max(0.0, min(1.0, score))

    def _is_object_candidate_cluster(self, cluster: Cluster, robot_xy: tuple[float, float] | None) -> bool:
        if self._object_score(cluster) < self.object_score_threshold:
            return False
        if self.require_person_shape_for_object_detection:
            if self._person_score(cluster, 0.0) < max(0.45, self.candidate_person_score_threshold - 0.08):
                return False
        if robot_xy is not None:
            distance = math.hypot(cluster.x - robot_xy[0], cluster.y - robot_xy[1])
            if distance <= self.object_near_range and cluster.count < self.object_near_min_points:
                return False
        return True

    def _person_score(self, cluster: Cluster, speed: float) -> float:
        reason = self._person_rejection_reason(cluster, speed, existing_confirmed=False)
        if reason:
            cluster.rejection_reason = reason
            return 0.0
        extent = max(cluster.width, cluster.depth, cluster.radius * 2.0)
        score = 0.0
        if self.min_person_radius <= cluster.radius <= self.max_person_radius:
            score += 0.18
        elif cluster.radius <= self.max_person_radius * 1.25:
            score += 0.05
        if extent <= self.max_person_extent:
            score += 0.16
        elif extent <= self.max_person_extent * 1.20:
            score += 0.04
        if self.min_person_height <= cluster.height <= self.max_person_height:
            score += 0.22
        if self.min_person_centroid_height <= cluster.z <= self.max_person_centroid_height:
            score += 0.12
        if cluster.count >= self.min_person_cluster_points:
            score += 0.08
        if cluster.vertical_layers >= max(self.min_vertical_layers, self.min_person_vertical_layers):
            score += 0.12
        if min(cluster.width, cluster.depth) > 0.05:
            aspect_ratio = max(cluster.width, cluster.depth) / min(cluster.width, cluster.depth)
            if aspect_ratio <= self.max_person_aspect_ratio:
                score += 0.10
            elif aspect_ratio <= self.max_person_aspect_ratio * 1.4:
                score += 0.03
        if cluster.static_fraction <= self.max_static_cell_fraction_new_track:
            score += 0.08
        if cluster.compactness >= 0.12:
            score += 0.04
        return max(0.0, min(1.0, score))

    def _person_rejection_reason(self, cluster: Cluster, speed: float, existing_confirmed: bool) -> str:
        extent = max(cluster.width, cluster.depth, cluster.radius * 2.0)
        top = cluster.z_max if cluster.z_max is not None else cluster.z + cluster.height*.5
        if not existing_confirmed and top < self.min_person_top_height:
            return "top_height_gate"
        if cluster.count < self._min_cluster_points_for_range(cluster.x, cluster.y, self._lookup_robot_xy()):
            return "too_few_points"
        if cluster.count > self.max_cluster_points:
            return "too_many_points"
        if self._is_livox_pattern_artifact(cluster):
            return "livox_pattern_artifact"
        if speed > self.max_person_speed:
            return "impossible_speed"
        if cluster.noise_fraction > self.max_noise_tag_fraction:
            return "noise_tag_fraction"
        if extent > self.max_person_extent * (1.25 if existing_confirmed else 1.0):
            return "xy_extent_too_large"
        if cluster.height < self.min_person_height or cluster.height > self.max_person_height:
            return "height_gate"
        if cluster.vertical_layers < max(self.min_vertical_layers, self.min_person_vertical_layers):
            return "vertical_layers"
        if cluster.z < self.min_person_centroid_height or cluster.z > self.max_person_centroid_height:
            return "centroid_height"
        if cluster.static_fraction > self.max_static_cell_fraction_new_track and not existing_confirmed:
            return "static_suppressed"
        if math.isfinite(cluster.aspect_ratio) and cluster.aspect_ratio > self.max_person_aspect_ratio * 1.4:
            return "aspect_ratio"
        return ""

    def _is_livox_pattern_artifact(self, cluster: Cluster) -> bool:
        if not self.reject_livox_pattern_artifacts:
            return False
        extent = max(cluster.width, cluster.depth, cluster.radius * 2.0)
        thin_height = cluster.height <= self.livox_artifact_max_height
        few_layers = cluster.vertical_layers <= self.livox_artifact_max_layers
        sparse = cluster.count <= self.livox_artifact_max_points
        elongated = extent >= self.livox_artifact_min_extent
        if thin_height and few_layers and (sparse or elongated):
            return True
        if sparse and few_layers and extent >= self.max_person_extent * 0.85:
            return True
        return False

    def _is_candidate_person_risk(
        self,
        cluster: Cluster,
        track: Track,
        robot_xy: tuple[float, float] | None,
    ) -> bool:
        if not self.immediate_person_risk:
            return False
        if track.age < self.candidate_person_min_age or track.confidence < self.candidate_person_min_confidence:
            return False
        person_shape_score = self._person_score(cluster, min(track.speed, self.max_person_speed))
        if self.use_object_motion_detection:
            score = min(self._object_score(cluster), person_shape_score + 0.10)
            threshold = max(self.object_score_threshold, self.candidate_person_score_threshold - 0.05)
        else:
            score = person_shape_score
            threshold = self.candidate_person_score_threshold
        if score < threshold:
            return False
        if robot_xy is None:
            return True
        return math.hypot(cluster.x - robot_xy[0], cluster.y - robot_xy[1]) <= self.candidate_person_robot_radius

    def _is_confirmed_person(self, track: Track) -> bool:
        net_displacement = math.hypot(track.x - track.first_x, track.y - track.first_y)
        if track.confirmed:
            return (
                track.person_observations >= max(1, self.min_person_observations - 1)
                and track.confidence >= self.drop_confidence_threshold
                and track.speed <= self.max_person_speed * 1.5
            )
        hit_count = sum(1 for hit in track.hit_history if hit)
        if hit_count < self.tentative_confirm_min_hits:
            return False
        if track.first_stamp > 0.0 and track.stamp - track.first_stamp < self.min_confirm_age:
            return False
        person_like = track.person_observations >= self.min_person_observations
        moving_like = track.dynamic_observations >= self.min_dynamic_observations and net_displacement >= self.min_confirm_displacement
        persistent_person_like = (
            track.person_observations >= self.min_static_person_observations
            and sum(1 for hit in track.hit_history if hit) >= self.tentative_confirm_min_hits
            and track.person_score >= max(self.confirmed_person_score_threshold, 0.78)
        )
        ml_person_like = track.detection_confidence >= self.person_detection_confirm_confidence
        return (
            track.age >= self.min_track_age
            and person_like
            and (moving_like or persistent_person_like or ml_person_like)
            and track.confidence >= self.confirm_confidence_threshold
            and track.person_score >= self.confirmed_person_score_threshold
            and track.speed <= self.max_person_speed
        )

    def _near_confirmed_track(self, cluster: Cluster, stamp: float) -> bool:
        for track in self.tracks.values():
            if not track.confirmed:
                continue
            dt = max(0.0, stamp - track.stamp)
            predicted_x = track.x + track.vx * min(dt, self.prediction_horizon)
            predicted_y = track.y + track.vy * min(dt, self.prediction_horizon)
            if math.hypot(cluster.x - predicted_x, cluster.y - predicted_y) <= self.track_match_distance:
                return True
        return False

    def _partial_track_gate(self,cluster,track,stamp):
        full_stamp = track.stamp if track.last_full_stamp is None else track.last_full_stamp
        if not track.confirmed or not 0. < stamp-full_stamp <= self.partial_observation_timeout:
            return False
        extent = max(cluster.width,cluster.depth)
        if extent > max(.4,1.6*max(track.width,track.depth)+.1):
            return False
        if extent > .6 and extent/max(.02,min(cluster.width,cluster.depth)) > self.max_person_aspect_ratio*1.4:
            return False
        if track.height > .1 and cluster.height > max(track.height+.25,1.4*track.height):
            return False
        return True

    def _partial_confirmed_observation(self, cluster: Cluster, stamp: float) -> bool:
        if not (cluster.count >= 4 and 0.15 <= cluster.height <= self.max_person_height
                and max(cluster.width, cluster.depth) <= self.max_person_extent * 1.25
                and cluster.noise_fraction <= self.max_noise_tag_fraction):
            return False
        return any(self._partial_track_gate(cluster,track,stamp) and 0.0 < stamp - track.stamp <= self.confirmed_keep_missed_time
                   and math.isfinite(self._association_cost(cluster, track, stamp))
                   for track in self.tracks.values())

    def _track_conditioned_crops(self, points, clusters, stamp):
        """Recover visible body fragments merged into another object/occluder.

        Only recently confirmed tracks may propose crops, and the normal global
        association still decides whether each crop can update an existing ID.
        Empty/occluded regions supply no evidence and cannot refresh a track.
        """
        crops = []
        claimed = set()
        occupied_cells = {cell for cluster in clusters for cell in cluster.cells}
        point_cells = [(math.floor(x/self.cell_size),math.floor(y/self.cell_size)) for x,y,_ in points]
        ordered = sorted(self.tracks.values(),key=lambda t: (t.last_full_stamp if t.last_full_stamp is not None else t.stamp,t.confidence),reverse=True)
        for track in ordered:
            dt = stamp - track.stamp
            full_stamp = track.stamp if track.last_full_stamp is None else track.last_full_stamp
            if not track.confirmed or not 0 < dt <= .8 or stamp-full_stamp > self.partial_observation_timeout:
                continue
            if any(math.isfinite(self._association_cost(c, track, stamp)) for c in clusters):
                continue
            x, y = track.x + track.vx * dt, track.y + track.vy * dt
            radius = min(.55,max(.25,track.radius+.15))
            indexes = [index for index,(px,py,pz) in enumerate(points)
                       if index not in claimed and point_cells[index] not in occupied_cells and
                       (px-x)**2+(py-y)**2 <= radius**2 and .2 <= pz <= 2.1]
            selected = [points[index] for index in indexes]
            if len(selected) < 6:
                continue
            xs, ys, zs = zip(*selected)
            mx, my, mz = sum(xs)/len(xs), sum(ys)/len(ys), sum(zs)/len(zs)
            cluster = Cluster(mx, my, mz,
                max(math.hypot(px-mx, py-my) for px, py, _ in selected), len(selected),
                max(xs)-min(xs), max(ys)-min(ys), max(zs)-min(zs),
                len({math.floor(z / self.person_vertical_layer_height) for z in zs}),
                {(math.floor(px/self.cell_size), math.floor(py/self.cell_size)) for px, py, _ in selected},
                partial_observation=True, z_max=max(zs))
            if self._partial_track_gate(cluster,track,stamp) and self._partial_confirmed_observation(cluster, stamp):
                crops.append(cluster)
                claimed.update(indexes)
                self.debug_events.append(self._cluster_debug_event(cluster, "partial", "track_conditioned_crop"))
        return crops

    def _update_static_background(self, clusters: list[Cluster]) -> None:
        dynamic_cells = set()
        observed_cells = set()
        for track in self.tracks.values():
            if track.speed >= self.min_dynamic_speed:
                cx = math.floor(track.x / self.cell_size)
                cy = math.floor(track.y / self.cell_size)
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        dynamic_cells.add((cx + dx, cy + dy))
        for cluster in clusters:
            observed_cells.update(cluster.cells)
            if self._near_confirmed_track(cluster, 0.0):
                continue
            if self.use_object_motion_detection and self._object_score(cluster) >= self.object_score_threshold:
                continue
            if cluster.radius > self.max_person_radius or cluster.count > self.max_cluster_points:
                for cell in cluster.cells:
                    self.static_cells[cell] = min(self.static_cell_observations * 3, self.static_cells.get(cell, 0) + 1)
            elif not cluster.cells & dynamic_cells:
                for cell in cluster.cells:
                    self.static_cells[cell] = min(self.static_cell_observations * 3, self.static_cells.get(cell, 0) + 1)
        for cell in list(self.static_cells):
            if cell in observed_cells:
                continue
            self.static_cells[cell] -= 1
            if self.static_cells[cell] <= 0:
                del self.static_cells[cell]

    def _is_static_background(self, cluster: Cluster) -> bool:
        if self._is_person_sized(cluster, 0.0):
            return False
        if cluster.radius > self.max_person_radius or max(cluster.width, cluster.depth) > self.max_person_extent:
            return True
        static_hits = sum(1 for cell in cluster.cells if self.static_cells.get(cell, 0) >= self.static_cell_observations)
        return static_hits >= max(2, len(cluster.cells) // 2) and cluster.radius <= self.static_suppression_radius

    def _prediction_points(
        self,
        x: float,
        y: float,
        z: float,
        vx: float,
        vy: float,
        radius: float,
        robot_xy: tuple[float, float] | None,
    ) -> list[tuple[float, float, float]]:
        points = []
        steps = max(1, int(self.prediction_horizon / max(self.prediction_dt, 1e-3)))
        base_radius = max(self.personal_space_radius, min(1.1, radius + 0.35))
        risk_radius = base_radius
        if robot_xy is not None and self._trajectory_near_robot(x, y, vx, vy, robot_xy):
            risk_radius = max(risk_radius, self.robot_risk_inflation_radius)
        for ox in (-risk_radius, -0.5 * risk_radius, 0.0, 0.5 * risk_radius, risk_radius):
            for oy in (-risk_radius, -0.5 * risk_radius, 0.0, 0.5 * risk_radius, risk_radius):
                if math.hypot(ox, oy) <= risk_radius:
                    points.append((x + ox, y + oy, z))
        speed = math.hypot(vx, vy)
        if speed < 1e-6:
            return points
        nx = -vy / speed
        ny = vx / speed
        for step in range(1, steps + 1):
            t = step * self.prediction_dt
            px = x + vx * t
            py = y + vy * t
            forward_uncertainty = min(1.2, 0.12 + 0.10 * step)
            lateral_scale = min(1.45, 1.0 + 0.025 * step)
            for lateral in (
                -risk_radius * lateral_scale,
                -0.5 * risk_radius * lateral_scale,
                0.0,
                0.5 * risk_radius * lateral_scale,
                risk_radius * lateral_scale,
            ):
                for forward in (-forward_uncertainty, -0.5 * forward_uncertainty, 0.0, 0.5 * forward_uncertainty, forward_uncertainty):
                    points.append((px + nx * lateral + vx / speed * forward, py + ny * lateral + vy / speed * forward, z))
        return points

    @staticmethod
    def _footprint_points(x: float, y: float, z: float, radius: float) -> list[tuple[float, float, float]]:
        footprint_radius = max(0.45, min(0.85, radius + 0.25))
        points = [(x, y, z)]
        for angle in (0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0, math.pi, 5.0 * math.pi / 4.0, 3.0 * math.pi / 2.0, 7.0 * math.pi / 4.0):
            points.append((x + footprint_radius * math.cos(angle), y + footprint_radius * math.sin(angle), z))
        return points

    def _trajectory_near_robot(self, x: float, y: float, vx: float, vy: float, robot_xy: tuple[float, float]) -> bool:
        rx, ry = robot_xy
        if math.hypot(x - rx, y - ry) <= self.robot_avoidance_radius:
            return True
        speed_sq = vx * vx + vy * vy
        if speed_sq < 1e-6:
            return False
        t = ((rx - x) * vx + (ry - y) * vy) / speed_sq
        t = max(0.0, min(self.prediction_horizon, t))
        closest_x = x + vx * t
        closest_y = y + vy * t
        return math.hypot(closest_x - rx, closest_y - ry) <= self.robot_avoidance_radius

    def _trajectory_marker(self, track_id: int, track: Track, frame_id: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "tracked_people_predictions"
        marker.id = track_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.06
        marker.color = ColorRGBA(r=1.0, g=0.75, b=0.05, a=0.9)
        marker.pose.orientation.w = 1.0
        marker.lifetime = Duration(seconds=self.marker_lifetime).to_msg()
        marker.points.append(Point(x=track.x, y=track.y, z=0.08))
        for _, x, y, _, _ in predict_track_motion(track, self.prediction_horizon, self.prediction_dt, self.max_person_speed)[1:]:
            marker.points.append(Point(x=x, y=y, z=0.08))
        return marker

    def _trajectory_corridor_marker(self, track_id: int, track: Track, frame_id: str) -> Marker:
        marker = self._trajectory_marker(track_id, track, frame_id)
        marker.ns = "tracked_people_danger_corridors"
        marker.id = track_id
        marker.scale.x = 2.0 * max(self.personal_space_radius, track.radius + 0.35)
        marker.color = ColorRGBA(r=1.0, g=0.08, b=0.02, a=0.22)
        for point in marker.points:
            point.z = 0.035
        return marker

    def _path_marker(self, track_id: int, track: Track, frame_id: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "tracked_object_paths"
        marker.id = track_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.95)
        marker.pose.orientation.w = 1.0
        marker.lifetime = Duration(seconds=max(self.marker_lifetime, 30.0)).to_msg()
        for x, y in track.history:
            marker.points.append(Point(x=x, y=y, z=0.12))
        marker.points.append(Point(x=track.x, y=track.y, z=0.12))
        return marker

    def _append_track_outputs(
        self,
        markers: MarkerArray,
        predicted_points: list[tuple[float, float, float]],
        tracked_points: list[tuple[float, float, float]],
        track_id: int,
        track: Track,
        frame_id: str,
        robot_xy: tuple[float, float] | None,
        coasting: bool,
        coast_time: float = 0.0,
        render_marker: bool = True,
    ) -> None:
        predict = self._should_publish_prediction(track, coasting, coast_time)
        if predict and self.publish_legacy_prediction_cloud:
            vx, vy = self._prediction_velocity(track)
            predicted_points.extend(self._prediction_points(track.x, track.y, 0.6, vx, vy, track.radius, robot_xy))
        if not coasting and track.state == "CONFIRMED":
            tracked_points.extend(self._footprint_points(track.x, track.y, 0.6, track.radius))
        if not self.render_markers or not render_marker:
            return
        color = ColorRGBA(r=0.10, g=1.0, b=0.20, a=.95) if not coasting else ColorRGBA(r=1., g=.65, b=.10, a=.70)
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "tracked_people"
        marker.id = track_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = track.x
        marker.pose.position.y = track.y
        marker.pose.position.z = 0.28
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.42
        marker.scale.y = marker.scale.x
        marker.scale.z = marker.scale.x
        marker.color = color
        marker.lifetime = Duration(seconds=self.marker_lifetime).to_msg()
        markers.markers.append(marker)

        label = Marker()
        label.header = marker.header
        label.ns = "dynamic_obstacle_labels"
        label.id = track_id
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = track.x
        label.pose.position.y = track.y
        label.pose.position.z = 0.65
        label.pose.orientation.w = 1.0
        label.scale.z = 0.14
        label.color = ColorRGBA(r=0.55, g=1.0, b=0.55, a=1.0)
        label.text = f"P{track_id}\nv={track.speed:.2f}m/s" + ("\nLOST/PREDICTED" if coasting else "")
        label.lifetime = Duration(seconds=self.marker_lifetime).to_msg()
        markers.markers.append(label)
        if len(track.history) >= 2:
            markers.markers.append(self._path_marker(track_id, track, frame_id))

        if predict:
            markers.markers.append(self._trajectory_corridor_marker(track_id, track, frame_id))
            markers.markers.append(self._trajectory_marker(track_id, track, frame_id))

    def _prediction_velocity(self, track: Track) -> tuple[float, float]:
        speed = math.hypot(track.vx, track.vy)
        if speed >= self.min_prediction_speed:
            return track.vx, track.vy
        return 0.0, 0.0

    def _should_publish_prediction(self, track: Track, coasting: bool, coast_time: float) -> bool:
        # A confirmed lost track remains safety-relevant until the lower drop
        # threshold expires it; do not retract its trajectory on the stricter
        # new-prediction confidence threshold during a brief occlusion.
        if not coasting and track.confidence < self.prediction_min_confidence:
            return False
        if track.person_score < self.prediction_min_person_score:
            return False
        if track.person_observations < self.min_person_observations:
            return False
        if (
            track.dynamic_observations < self.prediction_min_dynamic_observations
            and track.detection_confidence < self.person_detection_confirm_confidence
            and track.static_observations < self.min_static_person_observations
        ):
            return False
        if coasting and coast_time > self.max_prediction_coast_time:
            return False
        return track.speed <= self.max_person_speed

    @staticmethod
    def _set_header(message, frame_id: str, stamp: float) -> None:
        message.header.frame_id = frame_id
        message.header.stamp.sec, message.header.stamp.nanosec = _split_seconds(stamp)

    def _tracked_persons_message(
        self, tracks: list[tuple[int, Track]], frame_id: str, stamp: float
    ) -> TrackedPersons:
        message = TrackedPersons()
        self._set_header(message, frame_id, stamp)
        for track_id, track in tracks:
            person = TrackedPerson()
            person.track_id = track_id
            person.pose.pose.position.x = track.x
            person.pose.pose.position.y = track.y
            person.pose.pose.orientation.w = 1.0
            person.pose.covariance[0] = max(0.0, track.kf_p[0][0])
            person.pose.covariance[1] = track.kf_p[0][1]
            person.pose.covariance[6] = track.kf_p[1][0]
            person.pose.covariance[7] = max(0.0, track.kf_p[1][1])
            person.velocity.twist.linear.x = track.vx
            person.velocity.twist.linear.y = track.vy
            person.velocity.covariance[0] = max(0.0, track.kf_p[2][2])
            person.velocity.covariance[1] = track.kf_p[2][3]
            person.velocity.covariance[6] = track.kf_p[3][2]
            person.velocity.covariance[7] = max(0.0, track.kf_p[3][3])
            person.confidence = max(0.0, min(1.0, track.confidence))
            person.body_radius = max(getattr(self, "tracked_body_radius_floor", 0.0), track.radius)
            person.track_state = _typed_track_state(track.state)
            person.observation_age.sec, person.observation_age.nanosec = _split_seconds(track.observation_age)
            message.persons.append(person)
        return message

    def _predicted_trajectories_message(
        self, tracks: list[tuple[int, Track]], frame_id: str, stamp: float
    ) -> PredictedTrajectories:
        message = PredictedTrajectories()
        self._set_header(message, frame_id, stamp)
        for track_id, track in tracks:
            trajectory = PredictedTrajectory()
            trajectory.track_id = track_id
            for offset, x, y, vx, vy in predict_track_motion(track, self.prediction_horizon, self.prediction_dt, self.max_person_speed):
                point = PredictedTrajectoryPoint()
                point.time_offset.sec, point.time_offset.nanosec = _split_seconds(offset)
                point.position.x = x
                point.position.y = y
                point.velocity.x = vx
                point.velocity.y = vy
                xx, xy, yx, yy = _position_covariance_at(track.kf_p, offset)
                # Future changes in walking velocity are not represented by FPF'.
                process_variance = 0.04 * offset ** 3 / 3.0
                xx += process_variance
                yy += process_variance
                point.position_covariance[0] = xx
                point.position_covariance[1] = xy
                point.position_covariance[3] = yx
                point.position_covariance[4] = yy
                point.confidence = max(0.0, min(1.0, track.confidence))
                trajectory.points.append(point)
            message.trajectories.append(trajectory)
        return message

    def _append_candidate_marker(self, markers: MarkerArray, track_id: int, cluster: Cluster, frame_id: str) -> None:
        if not self.render_markers:
            return
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "candidate_people"
        marker.id = track_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = cluster.x
        marker.pose.position.y = cluster.y
        marker.pose.position.z = 0.24
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.20, min(0.36, cluster.radius * 1.15))
        marker.scale.y = marker.scale.x
        marker.scale.z = marker.scale.x
        marker.color = ColorRGBA(r=0.10, g=0.35, b=1.0, a=0.78)
        marker.lifetime = Duration(seconds=self.candidate_marker_lifetime).to_msg()
        markers.markers.append(marker)

        label = Marker()
        label.header = marker.header
        label.ns = "candidate_people_labels"
        label.id = track_id
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = cluster.x
        label.pose.position.y = cluster.y
        label.pose.position.z = 0.75
        label.pose.orientation.w = 1.0
        label.scale.z = 0.28
        label.color = ColorRGBA(r=0.45, g=0.70, b=1.0, a=1.0)
        label.text = (
            f"id={track_id} TENTATIVE\n"
            f"pts={cluster.count} zbin={cluster.vertical_layers} h={cluster.height:.2f}\n"
            f"score={self._person_score(cluster, 0.0):.2f} static={cluster.static_fraction:.2f}\n"
            f"reason={cluster.rejection_reason or 'candidate'}"
        )
        label.lifetime = Duration(seconds=self.candidate_marker_lifetime).to_msg()
        markers.markers.append(label)

    def _delete_track_markers(self, track_id: int, frame_id: str) -> list[Marker]:
        now = self.get_clock().now().to_msg()
        markers = []
        for namespace in (
            "tracked_people",
            "dynamic_obstacle_labels",
            "tracked_people_predictions",
            "tracked_object_paths",
            "candidate_people",
            "candidate_people_labels",
        ):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = now
            marker.ns = namespace
            marker.id = track_id
            marker.action = Marker.DELETE
            markers.append(marker)
        return markers

    def _safety_grid(
        self,
        predicted_points: list[tuple[float, float, float]],
        tracked_points: list[tuple[float, float, float]],
        frame_id: str,
        robot_xy: tuple[float, float] | None,
    ) -> OccupancyGrid:
        grid = OccupancyGrid()
        grid.header.frame_id = frame_id
        grid.header.stamp = self.get_clock().now().to_msg()
        resolution = max(0.05, self.safety_grid_resolution)
        width = max(1, int(self.safety_grid_size / resolution))
        height = width
        origin_x = -self.safety_grid_size * 0.5
        origin_y = -self.safety_grid_size * 0.5
        if robot_xy is not None:
            origin_x += robot_xy[0]
            origin_y += robot_xy[1]
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.orientation.w = 1.0

        risk_points = [(x, y) for x, y, _ in predicted_points]
        person_points = [(x, y) for x, y, _ in tracked_points]
        data = []
        danger_sq = self.safety_grid_danger_radius * self.safety_grid_danger_radius
        caution_sq = self.safety_grid_caution_radius * self.safety_grid_caution_radius
        for row in range(height):
            y = origin_y + (row + 0.5) * resolution
            for col in range(width):
                x = origin_x + (col + 0.5) * resolution
                occupied = self._near_any_point(x, y, person_points, danger_sq)
                risky = occupied or self._near_any_point(x, y, risk_points, danger_sq)
                caution = risky or self._near_any_point(x, y, risk_points, caution_sq)
                if risky:
                    data.append(100)
                elif caution:
                    data.append(50)
                else:
                    data.append(0)
        grid.data = data
        return grid

    @staticmethod
    def _near_any_point(
        x: float, y: float, points: list[tuple[float, float]], radius_sq: float
    ) -> bool:
        return any((px - x) * (px - x) + (py - y) * (py - y) <= radius_sq for px, py in points)

    def _prediction_cloud(
        self, points: list[tuple[float, float, float]], frame_id: str, stamp: float
    ) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header.frame_id = frame_id
        cloud.header.stamp.sec, cloud.header.stamp.nanosec = _split_seconds(stamp)
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        data = bytearray(cloud.row_step)
        for index, point in enumerate(points):
            struct.pack_into("<fff", data, index * cloud.point_step, *point)
        cloud.data = array("B", data)
        return cloud


def main() -> None:
    rclpy.init()
    node = DynamicObstacleTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
