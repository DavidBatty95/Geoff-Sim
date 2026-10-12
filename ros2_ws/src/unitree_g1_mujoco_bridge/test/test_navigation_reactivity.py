"""Behavioral regressions for perception and executable social planning."""

import math
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.duration import Duration
from tf2_ros import TransformException

from unitree_g1_mujoco_bridge.dynamic_obstacle_tracker import (
    Cluster, DynamicObstacleTracker, PersonDetection, Track, _motion_covariance, predict_track_motion,
)
from unitree_g1_mujoco_bridge.human_aware_motion_planner import (
    Candidate, HumanAwareMotionPlanner, HumanTrack, PredictionSample,
)
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope
from unitree_g1_mujoco_bridge.human_collision_avoidance import HumanCollisionAvoidance
from unitree_g1_mujoco_bridge.livox_to_scan import LivoxToScan
from unitree_g1_mujoco_bridge.g1_sim_sensors import G1SimSensors
from unitree_g1_mujoco_bridge.static_distance_field import StaticDistanceField, observed_free_mask
from unitree_g1_mujoco_bridge.scan_dynamic_filter import ScanDynamicFilter
from unitree_social_navigation_msgs.msg import TrackedPerson


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def tracker():
    node = DynamicObstacleTracker()
    node.use_mujoco_people_detections = False
    node.use_ml_person_detections = False
    node.min_confirm_age = 0.15
    node.confirm_confidence_threshold = 0.5
    node._lookup_robot_xy = lambda *args: (0.0, 0.0)
    yield node
    node.destroy_node()


def observe(node, stamp, x, y=0.0):
    cluster = node._cluster_from_detection(PersonDetection(x, y, 0.8, 0.9, 0.35, stamp))
    assignments = node._global_track_assignments([cluster], stamp)
    return node._match_track(cluster, stamp, set(), assignments.get(0))


def test_wall_crops_cannot_refresh_person_tracks(tracker):
    tracker.tracks[1] = Track(2.,0.,10.,2.,0.,confirmed=True,confidence=1.,height=1.6)
    wall = [(2.,float(y),z) for y in np.linspace(-.54,.54,19) for z in (.2,.6,1.,1.4,1.8)]
    assert tracker._track_conditioned_crops(wall,[],10.1) == []


def test_partial_crops_do_not_reuse_one_body_for_multiple_identities(tracker):
    tracker.tracks[1] = Track(2.,0.,10.,2.,0.,confirmed=True,confidence=1.,height=1.6)
    tracker.tracks[2] = Track(2.01,0.,10.,2.01,0.,confirmed=True,confidence=1.,height=1.6)
    points = [(2.+x,y,z) for x in (-.1,.1) for y in (-.1,.1) for z in (.5,.7,.9)]
    assert len(tracker._track_conditioned_crops(points,[],10.1)) == 1


def test_partial_updates_cannot_extend_full_observation_age_indefinitely(tracker):
    from dataclasses import replace
    for index in range(20):
        track_id,track = observe(tracker,10.+index*.1,2.)
    assert track.confirmed
    full_stamp = track.last_full_stamp
    cluster = replace(tracker._cluster_from_detection(PersonDetection(2.,0.,.8,.9,.35,12.)),
                      partial_observation=True)
    for elapsed in (.2,.4,.6,.8):
        stamp = full_stamp+elapsed
        assert tracker._partial_confirmed_observation(cluster,stamp)
        _,track = tracker._match_track(cluster,stamp,set(),track_id)
        assert track.last_full_stamp == full_stamp
    assert not tracker._partial_confirmed_observation(cluster,full_stamp+tracker.partial_observation_timeout+.1)
    assert track.confidence < 1.


def test_low_furniture_cannot_birth_a_person_but_partial_known_people_remain(tracker):
    from dataclasses import replace
    tracker.min_person_top_height = 1.
    tracker.require_person_shape_for_object_detection = True
    table = Cluster(2.,0.,.6,.35,40,.6,.5,.8,4,{(2,0),(2,1),(3,0),(3,1)},z_max=.9)
    assert not tracker._is_object_candidate_cluster(table,(0.,0.))
    assert table.rejection_reason == 'top_height_gate'
    person = replace(table,z=1.,height=1.6,z_max=1.8)
    assert tracker._is_object_candidate_cluster(person,(0.,0.))
    tracker.tracks[1] = Track(2.,0.,10.,2.,0.,confirmed=True,confidence=1.,person_score=1.)
    points = [(2.+.03*i,.02*(i%2),.3+.05*i) for i in range(8)]
    crops = tracker._track_conditioned_crops(points,[],10.1)
    assert len(crops) == 1 and crops[0].partial_observation
    assert crops[0].z_max < tracker.min_person_top_height


def test_torso_centroid_is_stable_when_lower_body_sampling_changes(tracker):
    tracker.use_torso_centroid = True
    torso = [(2.+x,y,z) for x in (-.05,.05) for y in (-.05,.05) for z in (.9,1.1,1.3)]
    lower = [(2.2,0.,z) for z in (.3,.5,.7)]
    sparse = tracker._clusters(torso+lower,robot_xy=(0.,0.))
    dense = tracker._clusters(torso+lower*8,robot_xy=(0.,0.))
    assert len(sparse) == len(dense) == 1
    assert sparse[0].x == pytest.approx(2.)
    assert dense[0].x == pytest.approx(sparse[0].x)
    assert dense[0].count > sparse[0].count
    assert dense[0].height == pytest.approx(sparse[0].height)


def test_static_grid_ambiguity_does_not_trap_a_clear_robot_beside_a_wall():
    field = StaticDistanceField([(0.,.58)],(0.,0.),3.2,1.25)
    assert field.query([(0.,0.)])[0] < .48
    assert not field.trajectory_is_blocked([(0.,0.),(0.,-.3)],.48)
    assert field.trajectory_is_blocked([(0.,0.),(0.,.2)],.48)


def test_static_margin_recovery_only_moves_clear_of_existing_reserve_overlap():
    field = StaticDistanceField([(0.,.43)],(0.,0.),3.2,1.25)
    assert field.trajectory_is_blocked([(0.,0.),(0.,-.3)],.48)
    assert not field.trajectory_is_blocked([(0.,0.),(0.,-.3)],.48,recovery_radius=.38)
    assert field.trajectory_is_blocked([(0.,0.),(0.,.02),(0.,-.3)],.48,recovery_radius=.38)
    physical_overlap = StaticDistanceField([(0.,.3)],(0.,0.),3.2,1.25)
    assert physical_overlap.trajectory_is_blocked([(0.,0.),(0.,-.3)],.48,recovery_radius=.38)


def test_exact_static_refinement_checks_segments_and_keeps_unknown_window_blocked():
    field = StaticDistanceField([(0.,.4)],(0.,0.),3.2,1.25)
    assert field.trajectory_is_blocked([(-1.,0.),(1.,0.)],.48)
    assert field.trajectory_is_blocked([(0.,0.),(4.,0.)],.48)


def test_nearby_motion_uses_fresh_points_while_sparse_far_returns_accumulate(tracker):
    tracker.near_accumulation_radius = 4.
    tracker.near_accumulation_time = .05
    tracker.point_accumulation_time = .2
    tracker.enable_point_noise_filter = False
    tracker._voxel_downsample = lambda points: points
    captured = []
    tracker._publish_clusters = lambda points, stamp, frame: captured.append(points)
    tracker._publish_points([(1.,0.,1.),(6.,0.,1.)],100.,'world')
    tracker._publish_points([(1.1,0.,1.),(6.1,0.,1.)],100.1,'world')
    assert (1.,0.,1.) not in captured[-1]
    assert set(captured[-1]) == {(1.1,0.,1.),(6.,0.,1.),(6.1,0.,1.)}


@pytest.mark.parametrize("rate", [5, 10, 20, 30])
@pytest.mark.parametrize("speed", [0.5, 1.0, 1.8])
def test_walking_initializes_velocity_without_identity_reset(tracker, rate, speed):
    identities = []
    for index in range(rate * 2):
        t = index / rate
        track_id, track = observe(tracker, 10.0 + t, 3.0 + speed * t)
        identities.append(track_id)
    assert len(set(identities)) == 1
    assert track.confirmed
    assert track.vx == pytest.approx(speed, abs=0.15)
    assert track.x == pytest.approx(3.0 + speed * t, abs=0.12)


def test_noisy_centroids_do_not_trigger_acceleration_rejection(tracker):
    identities = []
    for index in range(40):
        t = index * 0.05
        track_id, track = observe(tracker, 10.0 + t, 3.0 + t + 0.025 * math.sin(index * 2.0))
        identities.append(track_id)
    assert len(set(identities)) == 1
    assert track.vx == pytest.approx(1.0, abs=0.2)


def test_track_handles_turn_and_stop_without_lifetime_direction_prediction(tracker):
    ids = []
    for index in range(90):
        t = index * 0.1
        x = 3.0 + min(t, 2.0)
        y = max(0.0, min(t - 2.0, 2.0))
        track_id, track = observe(tracker, 10.0 + t, x, y)
        ids.append(track_id)
    assert len(set(ids)) == 1
    assert track.speed < 0.1
    assert tracker._prediction_velocity(track) == (0.0, 0.0)


def test_outlier_does_not_modify_existing_measurement_epoch(tracker):
    for index in range(10):
        _, track = observe(tracker, 10.0 + index * 0.1, 3.0 + index * 0.1)
    stamp, x = track.stamp, track.x
    outlier_id, _ = observe(tracker, stamp + 0.1, x + 5.0)
    assert track.stamp == stamp
    assert track.x == x
    assert tracker.tracks[outlier_id] is not track


def test_person_orb_refresh_uses_latest_tf_without_refreshing_control_tracks(tracker):
    from visualization_msgs.msg import Marker, MarkerArray
    published = []
    tracker.pub = SimpleNamespace(get_subscription_count=lambda: 1, publish=published.append)
    tracker.marker_display_hold_time = .9
    stamp = tracker.get_clock().now().nanoseconds * 1e-9
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = tracker.get_clock().now().to_msg()
    marker.ns, marker.id, marker.action = "tracked_people", 7, Marker.ADD
    marker.color.a = 1.
    tracker.tracks[7] = Track(3., 0., stamp, 3., 0.)
    message = MarkerArray(markers=[marker])
    stable = tracker._stable_marker_array(message, stamp)
    assert stable.markers[0].header.stamp.sec == 0
    tracker._refresh_person_markers()
    assert published[-1].markers[0].action == Marker.ADD
    assert tracker.tracks[7].stamp == stamp
    tracker.display_markers[("tracked_people", 7)] = (marker, stamp-1.)
    tracker._refresh_person_markers()
    assert published[-1].markers[0].action == Marker.DELETE
    assert tracker.tracks[7].stamp == stamp


def test_partial_body_keeps_confirmed_identity_prediction_and_footprint(tracker):
    for index in range(20):
        track_id, track = observe(tracker, 10.0 + index * .1, 3.0 + index * .05)
    assert track.confirmed
    radius, height, score = track.radius, track.height, track.person_score
    for index in range(20):
        stamp = 12.0 + index * .1
        cluster = tracker._cluster_from_detection(PersonDetection(4.0 + index * .05, 0., .4, .9, .15, stamp))
        cluster.height = .25
        assert tracker._partial_confirmed_observation(cluster, stamp)
        cluster.partial_observation = True
        identity, updated = tracker._match_track(cluster, stamp, set(), track_id)
        assert identity == track_id and updated.confirmed
        assert updated.person_score >= score
        assert updated.radius == radius and updated.height == height
        assert tracker._should_publish_prediction(updated, False, 0.)
    cluster.x += 5.
    assert not tracker._partial_confirmed_observation(cluster, stamp + .1)


def test_track_crop_requires_returns_and_cannot_refresh_unseen_people(tracker):
    for index in range(20):
        track_id, track = observe(tracker, 10.0 + index * .1, 3.0 + index * .05)
    assert track.confirmed
    points = [(4.0 + dx, dy, z) for dx in (-.15, .0, .15)
              for dy in (-.15, .15) for z in (.3, .6, .9)]
    # A distant wall contributes no points to the track-local measured crop.
    points += [(6.0, float(y), .8) for y in range(-5, 6)]
    crops = tracker._track_conditioned_crops(points, [], 12.0)
    assert len(crops) == 1 and crops[0].partial_observation
    assert crops[0].x == pytest.approx(4.0)
    assert tracker._track_conditioned_crops([], [], 12.0) == []
    assert tracker._track_conditioned_crops(points, [], 15.0) == []


@pytest.mark.parametrize('vx,vy', [(.65,.45), (-.45,.45), (.65,-.45)])
def test_combined_translation_limit_preserves_heading_and_rollout_bound(vx, vy):
    envelope = MotionEnvelope(max_x=.65, max_y=.45, accel_x=1.2, accel_y=1.2,
                              max_planar_speed=.65)
    x, y, _ = envelope.normalize((vx, vy, 0.))
    assert math.hypot(x, y) <= .65 + 1e-9
    assert x*vy == pytest.approx(y*vx)
    trajectory = envelope.rollout((0.,0.,0.), (vx,vy,0.), (0.,0.,0.), 3., .1,
                                  filtered_command=(0.,0.,0.))
    for first, second in zip(trajectory, trajectory[1:]):
        assert math.hypot(second[0]-first[0], second[1]-first[1]) / (second[3]-first[3]) <= .65+1e-9


@pytest.mark.parametrize('filtered', [None, (0.,0.,0.)])
def test_lateral_response_includes_measured_signed_heading_drift(filtered):
    envelope = MotionEnvelope(max_y=.45, accel_y=1.2, lateral_gain=.8, lateral_yaw_gain=.1)
    left = envelope.rollout((0.,0.,0.),(0.,.45,0.),(0.,0.,0.),6.,.05,.3,filtered_command=filtered)
    right = envelope.rollout((0.,0.,0.),(0.,-.45,0.),(0.,0.,0.),6.,.05,.3,filtered_command=filtered)
    assert .20 < left[-1][2] < .28
    assert right[-1][2] == pytest.approx(-left[-1][2])
    assert right[-1][1] == pytest.approx(-left[-1][1])


def test_signed_lateral_response_matches_distinct_stop_distances():
    envelope = MotionEnvelope(max_y=.45,accel_y=2.4,lateral_gain=.8,negative_lateral_gain=.9,
                              linear_time_constant=.4,positive_lateral_time_constant=.3)
    left = envelope.rollout((0.,0.,0.),(0.,0.,0.),(0.,.36,0.),3.,.15,.1,filtered_command=(0.,.45,0.))
    right = envelope.rollout((0.,0.,0.),(0.,0.,0.),(0.,-.405,0.),3.,.15,.1,filtered_command=(0.,-.45,0.))
    assert .10 < left[-1][1] < .13
    assert .14 < -right[-1][1] < .18


def test_turn_response_distinguishes_walking_from_a_stationary_pivot():
    envelope = MotionEnvelope(max_x=.65,max_yaw=.6,accel_x=2.4,accel_yaw=.9,
                              yaw_gain=.32,walking_yaw_gain=.55)
    walking = envelope.rollout((0.,0.,0.),(.3,0.,.35),(0.,0.,0.),6.,.05,
                                filtered_command=(0.,0.,0.))
    pivot = envelope.rollout((0.,0.,0.),(0.,0.,.35),(0.,0.,0.),6.,.05,
                              filtered_command=(0.,0.,0.))
    assert .9 < walking[-1][2] < 1.15
    assert walking[-1][2] > 1.5*pivot[-1][2]


def test_policy_slew_respects_combined_speed_without_exceeding_axis_slew():
    from unitree_g1_mujoco_bridge.g1_policy_cmd_vel_controller import G1PolicyCmdVelController
    params = dict(max_lin_vel_x=.65,max_lin_vel_y=.45,max_planar_speed=.65,
                  cmd_accel_x=2.4,cmd_accel_y=2.4,axis_exclusive_cmd_vel=False)
    node = SimpleNamespace(command_stopped=False,filtered_command=[.6,.2,0.],
        has_parameter=lambda key: key in params,
        declare_parameter=lambda key,value: params.setdefault(key,value),
        get_parameter=lambda key: SimpleNamespace(value=params[key]))
    command = G1PolicyCmdVelController._filtered_active_command(node,(.64,0.,0.),1/60)
    assert math.hypot(*command[:2]) <= .65+1e-9
    assert abs(command[0]-.6) <= .04+1e-9
    assert abs(command[1]-.2) <= .04+1e-9


def test_profile_command_cap_is_shared_with_policy_execution():
    from pathlib import Path as FilePath
    import yaml
    profile = yaml.safe_load((FilePath(__file__).parents[2]/'unitree_g1_sim_bringup/config/social_navigation_onboard_v1.yaml').read_text())
    limits = [profile[name]['ros__parameters']['max_planar_speed'] for name in
              ('human_aware_motion_planner','human_collision_avoidance','g1_policy_cmd_vel_controller')]
    assert limits == [.65,.65,.65]


def test_budget_rechecks_stop_and_previous_and_does_not_starve_directions(planner, monkeypatch):
    import unitree_g1_mujoco_bridge.human_aware_motion_planner as module
    clock = [0.0]
    monkeypatch.setattr(module.time, "perf_counter", lambda: clock[0])
    planner.planning_budget_ms = 30.0
    checked = []

    def score(candidate, *args):
        clock[0] += .007
        checked.append(candidate.name)
        return SimpleNamespace(candidate=candidate)

    planner._score_candidate = score
    candidates = [Candidate(name, 0., 0., 0.) for name in
                  ("yield_stop", "previous_command", "raw_reference", *(f"direction_{i}" for i in range(8)))]
    covered = set()
    for _ in range(4):
        planner.planning_started = clock[0]
        results = planner._evaluate_candidates(candidates, None, [], [], [], {}, 0.)
        names = [result.candidate.name for result in results]
        assert names[:3] == ["yield_stop", "previous_command", "raw_reference"]
        assert len(names) == 5 and planner.search_budget_exhausted
        assert names == checked[-5:]
        covered.update(names)
    assert covered == {candidate.name for candidate in candidates}


@pytest.mark.parametrize("budget_ms", [1., 28.])
def test_budget_covers_every_direction_even_with_one_optional_slot(planner, monkeypatch, budget_ms):
    import unitree_g1_mujoco_bridge.human_aware_motion_planner as module
    clock = [0.]
    monkeypatch.setattr(module.time, "perf_counter", lambda: clock[0])
    planner.planning_budget_ms = budget_ms

    def score(candidate, *args):
        clock[0] += .007
        return SimpleNamespace(candidate=candidate)

    planner._score_candidate = score
    priority = ("yield_stop", "previous_command", "raw_reference")
    directions = tuple(f"direction_{i}" for i in range(12))
    candidates = [Candidate(name, 0., 0., 0.) for name in priority + directions]
    covered = set()
    for _ in directions:
        planner.planning_started = clock[0]
        results = planner._evaluate_candidates(candidates, None, [], [], [], {}, 0.)
        names = [result.candidate.name for result in results]
        assert names[:3] == list(priority)
        covered.update(names[3:])
    assert covered == set(directions)


def test_budget_checks_coarse_escape_directions_before_refinements(planner, monkeypatch):
    import unitree_g1_mujoco_bridge.human_aware_motion_planner as module
    monkeypatch.setattr(module.time, 'perf_counter', lambda: 1.)
    planner.planning_started = 0.
    planner.planning_budget_ms = 1.
    planner._score_candidate = lambda candidate, *args: SimpleNamespace(candidate=candidate)
    coarse = [Candidate('escape_translation', vx, vy, 0.)
              for vx in (-.45, 0., .65) for vy in (-.45, 0., .45) if vx or vy]
    candidates = [Candidate('yield_stop', 0., 0., 0.),
                  Candidate('fine_translation', .1, .1, 0.), *coarse]
    results = planner._evaluate_candidates(candidates, None, [], [], [], {}, 0.)
    checked = [result.candidate for result in results]
    assert all(candidate in checked for candidate in coarse)
    assert checked.index(coarse[-1]) < checked.index(candidates[1])


def test_bounded_priority_checks_core_and_fairly_covers_escape_families(planner,monkeypatch):
    import unitree_g1_mujoco_bridge.human_aware_motion_planner as module
    clock = [0.]
    monkeypatch.setattr(module.time,'perf_counter',lambda:clock[0])
    planner.priority_candidate_limit = 4
    planner.planning_budget_ms = 1.
    core = [Candidate(name,0.,0.,0.) for name in ('yield_stop','previous_command','raw_reference')]
    escapes = [Candidate('escape_translation',index*.01,.1,0.) for index in range(20)]
    optional = [Candidate('fine_translation',index*.01,-.1,0.) for index in range(7)]
    def score(candidate,*args):
        clock[0] += .007
        return SimpleNamespace(candidate=candidate)
    planner._score_candidate = score
    seen_escape,seen_optional = set(),set()
    for _ in range(7):
        planner.planning_started = clock[0]
        results = planner._evaluate_candidates(core+escapes+optional,None,[],[],[],{},0.)
        commands = [result.candidate for result in results]
        assert commands[:3] == core
        assert len(commands) == 8  # Three core, four escapes, one rotating refinement.
        assert planner.search_budget_exhausted
        seen_escape.update(c.vx for c in commands if c.name=='escape_translation')
        seen_optional.update(c.vx for c in commands if c.name=='fine_translation')
    assert seen_escape == {c.vx for c in escapes}
    assert seen_optional == {c.vx for c in optional}


def test_bounded_priority_does_not_discard_candidates_when_budget_is_available(planner,monkeypatch):
    import unitree_g1_mujoco_bridge.human_aware_motion_planner as module
    monkeypatch.setattr(module.time,'perf_counter',lambda:0.)
    planner.planning_started = 0.
    planner.planning_budget_ms = 45.
    planner.priority_candidate_limit = 2
    candidates = [Candidate('yield_stop',0.,0.,0.),
                  *[Candidate('escape_translation',index*.01,.1,0.) for index in range(12)],
                  Candidate('fine_translation',.1,-.1,0.)]
    planner._score_candidate = lambda candidate,*args:SimpleNamespace(candidate=candidate)
    results = planner._evaluate_candidates(candidates,None,[],[],[],{},0.)
    assert {id(result.candidate) for result in results} == {id(candidate) for candidate in candidates}
    assert len(results) == len(candidates)


def test_search_refines_a_narrow_safe_velocity_band_before_stopping(planner, monkeypatch):
    import unitree_g1_mujoco_bridge.human_aware_motion_planner as module
    clock = [0.]
    planner.translation_refinement_enabled = True
    monkeypatch.setattr(module.time,'perf_counter',lambda: clock[0])
    planner.motion_envelope = MotionEnvelope(max_x=.65,max_y=.45)
    planner.max_forward_near_human = .65
    planner.max_reverse_velocity = planner.max_lateral_near_human = .45
    planner.planning_budget_ms = 20.
    planner.planning_started = 0.
    def score(candidate,*args):
        clock[0] += .001
        error = abs(candidate.vx-.53)
        return SimpleNamespace(candidate=candidate,rejected=error>.01,min_human_distance=1.1-error,score=error)
    planner._score_candidate = score
    candidates = [Candidate('yield_stop',0.,0.,0.),Candidate('raw_reference',.325,0.,0.),
                  Candidate('escape_translation',.65,0.,0.),Candidate('fine_translation',.2,0.,0.)]
    results = planner._evaluate_candidates(candidates,None,[object()],[],[],{},0.)
    assert any(not result.rejected and result.candidate.name == 'escape_refinement' for result in results)
    assert any(result.candidate.name == 'fine_translation' for result in results)


def test_equivalent_velocity_projections_do_not_consume_duplicate_rollouts(planner):
    planner.motion_envelope = MotionEnvelope(max_x=.65,max_y=.45)
    candidates = [Candidate('first',.3,.2,0.),Candidate('roundoff',.3+1e-12,.2,0.)]
    executable = planner._executable_candidates(candidates)
    assert len(executable) == 1 and executable[0].name == 'first'


def test_coasting_uncertainty_is_elapsed_time_consistent():
    track = Track(0.0, 0.0, 10.0, 0.0, 0.0)
    whole = _motion_covariance(track.kf_p, 1.0)
    split = _motion_covariance(_motion_covariance(track.kf_p, 0.4), 0.6)
    for a, b in zip(whole, split):
        assert a == pytest.approx(b)
    assert whole[0][0] > track.kf_p[0][0]


@pytest.fixture
def planner():
    node = HumanAwareMotionPlanner()
    node.motion_envelope = MotionEnvelope(max_y=0.0, accel_x=0.35)
    node.max_lateral_near_human = 0.0
    node.measured_velocity = (0.35, 0.0, 0.0)
    node._static_obstacle_cost = lambda trajectory: (0.0, False)
    node.latest_cmd = Twist()
    node.latest_cmd.linear.x = 0.35
    node.latest_cmd_time = node.get_clock().now()
    node.odom_stamp = node.latest_cmd_time.nanoseconds * 1e-9
    node._robot_pose = lambda: (0.0, 0.0, 0.0)
    node.published = []
    node._publish = lambda cmd, state, results, selected: node.published.append((cmd, state, results, selected))
    yield node
    node.destroy_node()


def test_tentative_typed_track_reaches_planner(tracker, planner):
    track = Track(2.0, 0.0, 10.0, 2.0, 0.0)
    track.state = "TENTATIVE"
    message = tracker._tracked_persons_message([(1, track)], "world", 10.0)
    planner._tracked_persons_callback(message)
    assert message.persons[0].track_state == TrackedPerson.TRACK_STATE_TENTATIVE
    assert planner.tracks[1].state == "TENTATIVE"


def test_qualified_tentative_risk_is_published_before_confirmation(tracker):
    cluster = tracker._cluster_from_detection(PersonDetection(2.0, 0.0, 0.8, 0.9, 0.35, 10.0))
    tracker._clusters = lambda *args, **kwargs: [cluster]
    tracker._person_focused_clusters = lambda *args: []
    tracker._is_candidate_person_risk = lambda *args: True
    tracker.min_confirm_age = 0.5
    published = []
    tracker.tracked_persons_pub = SimpleNamespace(publish=published.append)
    tracker._publish_clusters([], 10.0, "world")
    assert published[-1].persons
    assert published[-1].persons[0].track_state == TrackedPerson.TRACK_STATE_TENTATIVE


def test_lost_track_observation_age_survives_message(tracker, planner):
    track = Track(2.0, 0.0, 10.0, 2.0, 0.0)
    track.state = "OCCLUDED"
    track.observation_age = 2.5
    message = tracker._tracked_persons_message([(1, track)], "world", 10.0)
    planner._tracked_persons_callback(message)
    assert planner.tracks[1].observation_age == pytest.approx(2.5)


def test_future_conflict_searches_alternatives_before_cpa_warning(planner):
    stamp = planner.get_clock().now().nanoseconds * 1e-9
    # Crossing at t=4s, beyond the 3s state warning; the five-second rollout
    # should find a slowing/yield alternative rather than emergency fallback.
    person = HumanTrack(1, "CONFIRMED", 1.4, -4.0, 0.0, 1.0, 0.9, 0.9, 0, 0.01, 0.01, stamp)
    planner.tracks = {1: person}
    planner.tracks_epoch = stamp
    planner._timer_callback()
    command, state, results, selected = planner.published[-1]
    assert len(results) > 1
    assert selected is not None
    assert state != "EMERGENCY_STOP"
    assert command.linear.x < 0.35


def test_empty_fresh_scene_preserves_command_but_stale_scene_stops(planner):
    planner.tracks_epoch = planner.get_clock().now().nanoseconds * 1e-9
    planner._timer_callback()
    assert planner.published[-1][0].linear.x == pytest.approx(0.35)
    planner.tracks_epoch -= 2.0
    planner._timer_callback()
    assert planner.published[-1][0].linear.x == 0.0
    assert planner.published[-1][1] == "PERCEPTION_STALE_STOP"


def test_candidate_rollout_matches_arc_transform_and_lateral_clamp(planner):
    planner.motion_envelope = MotionEnvelope(max_y=0.0, arc_forward=0.20, arc_yaw=0.12)
    candidates = planner._executable_candidates([Candidate("turn", 0.0, 0.12, 0.11)])
    assert (candidates[0].vx, candidates[0].vy, candidates[0].wz) == pytest.approx((0.20, 0.0, 0.12))
    assert planner._simulate((0.0, 0.0, 0.0), candidates[0])[-1][0] > 0.5


def test_nav2_turn_request_is_not_discarded_by_forward_arc_floor(planner):
    planner.motion_envelope = MotionEnvelope(max_y=0.0, arc_forward=0.06)
    planner.latest_cmd.linear.x = 0.0
    planner.latest_cmd.angular.z = 0.3
    candidates = planner._executable_candidates([Candidate("raw_limited", 0.0, 0.0, 0.3)])
    assert len(candidates) == 1
    assert candidates[0].vx == pytest.approx(0.06)
    stopped = planner._executable_candidates([Candidate("yield_stop", 0.0, 0.0, 0.0)])
    assert stopped[0].vx == 0.0


def test_zero_command_has_physical_braking_distance():
    envelope = MotionEnvelope(accel_x=0.35)
    trajectory = envelope.rollout((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.35, 0.0, 0.0), 2.0, 0.01, 0.15)
    assert trajectory[-1][0] == pytest.approx(0.35 * 0.15 + 0.35 ** 2 / (2 * 0.35), abs=0.005)


def test_arc_floor_cannot_exceed_policy_limits():
    envelope = MotionEnvelope(max_x=0.1, max_yaw=0.1, arc_forward=0.2, arc_yaw=0.2)
    assert envelope.normalize((0.0, 0.0, 0.1)) == pytest.approx((0.1, 0.0, 0.1))


def test_prediction_anticipates_acceleration_and_does_not_reverse_after_braking():
    track = Track(0.0, 0.0, 1.0, 0.0, 0.0, vx=0.2, ax=1.0)
    accelerated = predict_track_motion(track, 2.0, 0.1, 2.0)
    assert accelerated[-1][1] > 0.2 * 2.0 + 0.5
    track.vx, track.ax = 0.4, -2.0
    braking = predict_track_motion(track, 2.0, 0.1, 2.0)
    assert all(p[3] >= 0.0 for p in braking)
    assert braking[-1][3] == 0.0
    assert braking[-1][1] < 0.1
    track.vx = 0.0
    assert predict_track_motion(track, 2.0, 0.1, 2.0)[-1][1] == 0.0


def test_rollout_includes_software_command_slew_before_physical_reversal():
    envelope = MotionEnvelope(max_x=0.65, accel_x=0.6)
    pose = (0.0, 0.0, 0.0)
    command, velocity = (-0.4, 0.0, 0.0), (0.2, 0.0, 0.0)
    simple = envelope.rollout(pose, command, velocity, 0.5, 0.05, 0.1)
    cascaded = envelope.rollout(pose, command, velocity, 0.5, 0.05, 0.1, filtered_command=(0.6, 0.0, 0.0))
    assert cascaded[-1][0] > simple[-1][0] + 0.05


def test_swept_collision_between_prediction_samples(planner):
    planner._simulate = lambda pose, candidate: [(0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)]
    person = HumanTrack(1, "CONFIRMED", 0.0, -1.0, 0.0, 2.0, 1.0, 1.0, 0, 0.01, 0.01, 0.0, 0.42)
    result = planner._score_candidate(Candidate("hold", 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), [person], [], [], {}, 0.0)
    assert result.reason == "hard_human_collision"
    assert result.min_human_distance == pytest.approx(0.0)


def test_ground_referenced_projection_rejects_floor_but_keeps_low_obstacle(tracker):
    node = LivoxToScan()
    try:
        tf = TransformStamped()
        tf.transform.rotation.w = 1.0
        tf.transform.translation.z = 1.0
        node.tf_buffer.lookup_transform = lambda *args: tf
        published = []
        node.pub = SimpleNamespace(publish=published.append)
        cloud = tracker._prediction_cloud([(2.0, 0.0, -1.0), (3.0, 0.0, -0.64)], "lidar", 10.0)
        node._cloud_callback(cloud)
        scan = published[0]
        assert scan.header.frame_id == "base_footprint"
        assert min(scan.ranges) == pytest.approx(3.0)
        assert scan.header.stamp.sec == 10
        assert node._fill_circular_gaps([2.0, math.inf, math.inf, 8.0])[1:3] == [math.inf, math.inf]
        assert all(math.isfinite(v) for v in node._fill_circular_gaps([2.0, math.inf, 2.1]))
    finally:
        node.destroy_node()


def test_static_distance_field_never_overstates_clearance():
    rng = np.random.default_rng(23)
    obstacles = rng.uniform(-2.0, 2.0, (30, 2))
    queries = rng.uniform(-2.5, 2.5, (100, 2))
    field = StaticDistanceField(obstacles, (0.0, 0.0), 3.0, 1.5)
    exact = np.min(np.linalg.norm(queries[:, None, :] - obstacles[None, :, :], axis=2), axis=1)
    assert np.all(field.query(queries) <= exact + 1e-9)
    assert field.query([[10.0, 10.0]])[0] == 0.0


def test_static_memory_requires_measured_free_space_to_clear():
    points = np.asarray([[2.0, 0.0]])
    assert observed_free_mask(points, (0.0, 0.0, 0.0), [3.0, 3.0, 3.0], -0.02, 0.02)[0]
    assert not observed_free_mask(points, (0.0, 0.0, 0.0), [3.0, math.inf, 3.0], -0.02, 0.02)[0]
    assert not observed_free_mask(points, (0.0, 0.0, 0.0), [3.0, 2.0, 3.0], -0.02, 0.02)[0]


def test_typed_person_scan_mask_preserves_background_wall():
    node = object.__new__(ScanDynamicFilter)
    node.min_filter_range = 0.2
    node.filter_radius = 0.75
    node.range_padding = 0.45
    scan = LaserScan()
    scan.range_min, scan.range_max = 0.1, 10.0
    scan.angle_min, scan.angle_max, scan.angle_increment = -0.02, 0.02, 0.02
    scan.ranges = [2.7, 2.0, 2.7]
    node._remove_dynamic_return(scan, 2.0, 0.0, 0.52)
    assert math.isinf(scan.ranges[1])
    assert scan.ranges[0] == pytest.approx(2.7)
    assert scan.ranges[2] == pytest.approx(2.7)


def test_known_scene_map_preserves_fountain_footprint_and_excludes_floor(planner):
    sensor = object.__new__(G1SimSensors)
    sensor.get_clock = planner.get_clock
    sensor.scene_boxes = [((-10.0, -10.0, -0.1), (10.0, 10.0, 0.0)),
                          ((3.15, 1.75, 0.0), (5.05, 3.65, 0.36))]
    grid = sensor._static_scene_map()
    def cell(x, y):
        col = int((x - grid.info.origin.position.x) / grid.info.resolution)
        row = int((y - grid.info.origin.position.y) / grid.info.resolution)
        return grid.data[row * grid.info.width + col]
    assert cell(0.0, 0.0) == 0
    assert cell(3.2, 2.7) == 100
    planner._scene_map_callback(grid)
    assert planner.static_scene_points is not None


def test_active_path_reference_is_frame_correct_and_ignores_sampling_paths(planner):
    path = Path()
    path.header.frame_id = "world"
    for x in (0.0, 1.0, 2.0):
        pose = PoseStamped()
        pose.pose.position.x = x
        path.poses.append(pose)
    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.pose.position.x = 2.0
    planner._goal_callback(goal)
    planner._path_callback(path)
    reference = planner._path_reference((0.0, 0.0, math.pi / 2.0))
    assert reference.linear.x == pytest.approx(0.0, abs=1e-6)
    assert reference.linear.y == pytest.approx(-0.35)
    goal.pose.position.x = 4.0
    planner._goal_callback(goal)
    assert planner._path_reference((0.0, 0.0, 0.0)) is None


def test_valid_static_route_survives_unrelated_planning_and_waits(planner):
    planner.use_static_scene_map = True
    path = Path()
    path.header.frame_id = "world"
    for x in (0.0, 2.0):
        point = PoseStamped()
        point.pose.position.x = x
        path.poses.append(point)
    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.pose.position.x = 2.0
    planner._goal_callback(goal)
    planner._path_callback(path)
    unrelated = Path()
    unrelated.header.frame_id = "world"
    point = PoseStamped()
    point.pose.position.x = 8.0
    unrelated.poses = [point]
    planner._path_callback(unrelated)
    planner.reference_path_time -= Duration(seconds=30.0)
    assert planner.reference_path is path
    assert planner._path_reference((0.0, 0.0, 0.0)).linear.x > 0.0


def test_forward_preference_turns_for_travel_but_finishes_nearby_goals(planner):
    planner.prefer_forward_travel = True
    for distance in (2., .6):
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.pose.position.x = distance
        path = Path()
        path.header.frame_id = "world"
        path.poses = [goal]
        planner._goal_callback(goal)
        planner._path_callback(path)
        reference = planner._path_reference((0., 0., math.pi/2))
        if distance > 1.:
            assert reference.linear.y == 0.
            assert reference.angular.z < 0.
        else:
            assert reference.linear.y < -.25
            assert reference.angular.z == 0.


@pytest.mark.parametrize("heading", [math.pi/2, math.pi])
def test_short_route_can_progress_without_stationary_alignment(planner, heading):
    planner.prefer_forward_travel = True
    planner.path_reference_holonomic_distance = 3.
    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.pose.position.x = 2.
    path = Path()
    path.header.frame_id = "world"
    path.poses = [goal]
    planner._goal_callback(goal)
    planner._path_callback(path)
    reference = planner._path_reference((0., 0., heading))
    world_x = math.cos(heading)*reference.linear.x - math.sin(heading)*reference.linear.y
    assert world_x > .3
    assert reference.angular.z == 0.


def test_avoidance_detour_restores_forward_travel_outside_finishing_region(planner):
    planner.prefer_forward_travel = True
    planner.path_reference_holonomic_distance = 3.
    goal = PoseStamped()
    goal.header.frame_id = 'world'
    goal.pose.position.x = 2.
    path = Path()
    path.header.frame_id = 'world'
    path.poses = [goal]
    planner._goal_callback(goal)
    planner._path_callback(path)
    planner._path_reference((0.,0.,math.pi/2))
    detour = planner._path_reference((-2.,0.,math.pi/2))
    assert detour.linear.y == 0. and detour.angular.z < 0.
    replacement = PoseStamped()
    replacement.header.frame_id = 'world'
    replacement.pose.position.x = 7.
    planner._goal_callback(replacement)
    path.poses = [replacement]
    planner._path_callback(path)
    distant = planner._path_reference((0.,0.,math.pi/2))
    assert distant.linear.y == 0. and distant.angular.z < 0.


def test_onboard_profile_faces_short_goals_but_keeps_near_goal_hysteresis(planner):
    from pathlib import Path as FilePath
    import yaml
    profile = yaml.safe_load((FilePath(__file__).parents[2] / 'unitree_g1_sim_bringup/config/social_navigation_onboard_v1.yaml').read_text())['human_aware_motion_planner']['ros__parameters']
    planner.prefer_forward_travel = profile['prefer_forward_travel']
    planner.path_reference_holonomic_distance = profile['path_reference_holonomic_distance']
    assert profile['holonomic_avoidance']  # Close-person evasions remain available.
    goal = PoseStamped()
    goal.header.frame_id = 'world'
    goal.pose.position.x = 2.
    path = Path()
    path.header.frame_id = 'world'
    path.poses = [goal]
    planner._goal_callback(goal)
    planner._path_callback(path)
    for remaining,finish in ((2.,False),(.7,True),(.85,True),(1.1,False)):
        command = planner._path_reference((2.-remaining,0.,math.pi/2))
        assert planner.goal_holonomic is finish
        if finish:
            assert command.linear.y < 0. and command.angular.z == 0.
        else:
            assert command.linear.x == command.linear.y == 0.
            assert command.angular.z < 0.


def test_safe_alignment_is_not_replaced_by_sideways_goal_progress(planner):
    planner.prefer_safe_route_alignment = planner.prefer_forward_travel = planner.nav2_path_reference = True
    planner.mission_active = True
    planner.mission_state_time = planner.get_clock().now()
    planner.tracks_epoch = planner.mission_state_time.nanoseconds*1e-9
    planner.reference_goal = PoseStamped()
    planner.reference_goal_world = planner.reference_progress_target = (0.,2.)
    planner.reference_path = Path()
    planner.prediction_activation_clearance = 3.2
    planner.local_safe_space_enabled = False
    planner.holonomic_avoidance = True
    planner.motion_envelope = MotionEnvelope(max_x=.65,max_y=.45,max_yaw=.6)
    planner.max_lateral_near_human = .45
    planner.measured_velocity = (0.,0.,0.)
    turn = Twist()
    turn.angular.z = .6
    planner._path_reference = lambda pose:turn
    person = HumanTrack(1,'CONFIRMED',2.8,0.,0.,0.,1.,1.,0,.002,.002,planner.tracks_epoch,.42)
    planner._fresh_tracks = lambda now:[person]
    planner._timer_callback()
    command,_,_,selected = planner.published[-1]
    assert command.linear.x == command.linear.y == 0.
    assert command.angular.z > 0. and not selected.rejected
    assert planner.path_reference_status == 'safe_route_alignment'


def test_alignment_priority_never_bypasses_forecast_or_static_collision(planner):
    planner.prefer_safe_route_alignment = planner.prefer_forward_travel = planner.nav2_path_reference = True
    planner.mission_active = True
    planner.measured_velocity = (0.,0.,0.)
    planner.horizon = planner.hard_collision_horizon = 3.
    candidate = Candidate('raw_limited',0.,0.,.5,'',0.,.8)
    person = HumanTrack(1,'CONFIRMED',2.,0.,-1.,0.,1.,1.,0,.002,.002,0.,.42)
    result = planner._score_candidate(candidate,(0.,0.,0.),[person],[],[],{},0.)
    assert result.reason == 'hard_human_collision'
    assert not planner._can_prioritize_alignment(result,[person])
    planner.prediction_cache = {}
    person.vx = 0.
    person.x = 3.
    planner._static_obstacle_cost = lambda trajectory:(math.inf,True)
    result = planner._score_candidate(candidate,(0.,0.,0.),[person],[],[],{},0.)
    assert result.reason == 'static_obstacle'
    assert not planner._can_prioritize_alignment(result,[person])


def test_heading_alignment_keeps_translating_and_stops_turning_near_people(planner):
    planner.prefer_forward_travel = True
    planner.holonomic_heading_alignment = True
    planner.path_reference_holonomic_distance = 3.
    goal = PoseStamped()
    goal.header.frame_id = 'world'
    goal.pose.position.x = 2.
    path = Path()
    path.header.frame_id = 'world'
    path.poses = [goal]
    planner._goal_callback(goal)
    planner._path_callback(path)
    planner._fresh_tracks = lambda now: []
    command = planner._path_reference((0.,0.,math.pi/2))
    assert command.linear.y < 0. and command.angular.z < 0.
    planner._fresh_tracks = lambda now: [HumanTrack(1,'CONFIRMED',0.,2.,0.,0.,1.,1.,0,.002,.002,0.,.42)]
    command = planner._path_reference((0.,0.,math.pi/2))
    assert command.linear.y < 0. and command.angular.z == 0.


def test_path_endpoint_tolerance_does_not_become_goal_arrival_tolerance(planner):
    planner.prefer_forward_travel = True
    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.pose.position.x = 2.
    endpoint = PoseStamped()
    endpoint.pose.position.x = 1.75
    path = Path()
    path.header.frame_id = "world"
    path.poses = [endpoint]
    planner._goal_callback(goal)
    planner._path_callback(path)
    command = planner._path_reference((1.35, 0., 0.))
    assert command.linear.x >= .29
    assert planner.reference_goal_world == (2., 0.)


def test_onboard_objective_prefers_safe_progress_over_remaining_stopped(planner):
    from pathlib import Path as FilePath
    import yaml
    profile = yaml.safe_load((FilePath(__file__).parents[2] / 'unitree_g1_sim_bringup/config/social_navigation_onboard_v1.yaml').read_text())['human_aware_motion_planner']['ros__parameters']
    for name in planner.weights:
        if name in profile:
            planner.weights[name] = profile[name]
    planner.motion_envelope = MotionEnvelope(max_x=.65, max_y=.45, accel_x=1.2,
                                             accel_y=1.2, linear_time_constant=.4)
    planner.measured_velocity = (0.,0.,0.)
    planner.policy_command = (0.,0.,0.)
    planner.policy_command_stamp = planner.get_clock().now().nanoseconds*1e-9
    planner.last_cmd = Twist()
    planner.nav2_path_reference = True
    planner.prefer_forward_travel = True
    planner.state = 'YIELD_TO_HUMAN'
    planner.reference_goal_world = (3.,0.)
    planner.reference_progress_target = (1.5,0.)
    planner.horizon = 3.
    person = HumanTrack(1,'CONFIRMED',0.,2.,0.,0.,1.,1.,0,.002,.002,planner.policy_command_stamp,.42)
    stop = planner._score_candidate(Candidate('yield_stop',0.,0.,0.), (0.,0.,0.), [person], [], [], {}, 0.)
    forward = planner._score_candidate(Candidate('forward',.65,0.,0.), (0.,0.,0.), [person], [], [], {}, 0.)
    assert not forward.rejected
    assert forward.score < stop.score


def test_braked_step_does_not_pay_for_collision_beyond_its_stopping_point(planner):
    planner.horizon = 3.
    planner.hard_collision_horizon = 3.
    planner.motion_envelope = MotionEnvelope(max_x=.65, accel_x=1.2)
    planner.measured_velocity = (0., 0., 0.)
    planner.dcpa_comfort_threshold = 1.2
    planner.ttc_warning_threshold = 4.5
    person = HumanTrack(1, 'CONFIRMED', 2., 0., 0., 0., 1., 1., 0, .002, .002, 0., .42)
    candidate = Candidate('short_step', .65, 0., 0., '', 0., .6)
    # An unbraked instantaneous command would hit this person; the actual
    # command stops more than the comfort distance away over the full horizon.
    assert planner._relative_metrics_for_candidate((0.,0.,0.), candidate, person)['tcpa'] < planner.ttc_warning_threshold
    result = planner._score_candidate(candidate, (0.,0.,0.), [person], [], [], {}, 0.)
    assert not result.rejected
    assert result.min_human_distance > planner.dcpa_comfort_threshold
    assert result.score_terms['ttc'] == 0.


@pytest.mark.parametrize('yaw', [0.,.7,math.pi])
def test_forward_overtaking_risk_activates_before_short_horizon_collision(planner,yaw):
    planner.horizon = planner.hard_collision_horizon = 3.
    planner.body_prediction_margin = .25
    c,s = math.cos(yaw),math.sin(yaw)
    planner._simulate = lambda pose,cmd:[((c*cmd.vx-s*cmd.vy)*t,(s*cmd.vx+c*cmd.vy)*t,yaw,t)
                                        for t in np.linspace(0.,3.,31)]
    person = HumanTrack(1,'CONFIRMED',-3.5*c,-3.5*s,c,s,1.,1.,0,.002,.002,0.,.42)
    forward = Candidate('forward',.45,0.,0.)
    baseline = planner._score_candidate(forward,(0.,0.,yaw),[person],[],[],{},0.)
    assert not baseline.rejected
    planner.overtaking_terminal_horizon = 6.
    guarded = planner._score_candidate(forward,(0.,0.,yaw),[person],[],[],{},0.)
    side = planner._score_candidate(Candidate('side',0.,.45,0.),(0.,0.,yaw),[person],[],[],{},0.)
    assert guarded.reason=='terminal_conflict'
    assert guarded.score_terms['overtaking_conflict']>0.
    assert not side.rejected


def test_overtaking_cue_does_not_confuse_following_or_separating_motion():
    from unitree_g1_mujoco_bridge.bridge_safety import being_overtaken
    assert being_overtaken((0.,0.),(.45,0.),(-3.5,0.),(1.,0.))
    assert not being_overtaken((0.,0.),(.45,0.),(3.5,0.),(1.,0.))
    assert not being_overtaken((0.,0.),(.45,0.),(-3.5,0.),(.2,0.))
    assert not being_overtaken((0.,0.),(.2,.45),(-3.5,0.),(1.,0.))


def test_overtaking_reserve_leaves_response_aware_side_escape_available(planner):
    planner.horizon = planner.hard_collision_horizon = 3.
    planner.body_prediction_margin = .25
    planner.motion_envelope = MotionEnvelope(max_x=.45,max_y=.45,max_planar_speed=.45,
        accel_x=2.4,accel_y=2.4,forward_gain=.96,lateral_gain=.8,linear_time_constant=.4)
    planner.measured_velocity = (.432,0.,0.)
    planner.policy_command_stamp = None
    person = HumanTrack(1,'CONFIRMED',-4.2,0.,1.,0.,1.,1.,0,.002,.002,0.,.42)
    forward = Candidate('forward',.45,0.,0.)
    assert not planner._score_candidate(forward,(0.,0.,0.),[person],[],[],{},0.).rejected
    planner.overtaking_terminal_horizon = 6.
    assert planner._score_candidate(forward,(0.,0.,0.),[person],[],[],{},0.).reason=='terminal_conflict'
    side = planner._score_candidate(Candidate('side',0.,.45,0.),(0.,0.,0.),[person],[],[],{},0.)
    assert not side.rejected


def test_overtaking_reserve_starts_lateral_avoidance_earlier_in_planar_replay(planner):
    from collections import deque
    from pathlib import Path as FilePath
    import yaml
    profile = yaml.safe_load((FilePath(__file__).parents[2]/'unitree_g1_sim_bringup/config/social_navigation_onboard_v1.yaml').read_text())['human_aware_motion_planner']['ros__parameters']
    for key in planner.weights:
        if key in profile:
            planner.weights[key] = profile[key]
    planner.horizon = planner.hard_collision_horizon = 3.
    planner.body_prediction_margin = .25
    planner.nav2_path_reference = planner.prefer_forward_travel = True
    planner.reference_goal_world = (8.,0.)
    planner.reference_progress_target = None
    planner.state = 'HUMAN_CAUTION'
    planner.latest_cmd.linear.x = .45
    planner.motion_envelope = MotionEnvelope(max_x=.45,max_y=.45,max_planar_speed=.45,
        accel_x=2.4,accel_y=2.4,forward_gain=.96,lateral_gain=.8,linear_time_constant=.4)
    commands = [(0.,0.),(.45,0.),(-.45,0.),(0.,.45),(0.,-.45),(.318,.318),(.318,-.318)]
    def replay(reserve):
        planner.overtaking_terminal_horizon = reserve
        position,velocity,filtered = np.zeros(2),np.array((.432,0.)),np.array((.45,0.))
        delayed = deque([filtered.copy(),filtered.copy()])
        first_lateral = None
        minimum = math.inf
        planner.last_cmd = Twist()
        for step in range(100):
            t = step*.1
            person = HumanTrack(1,'CONFIRMED',-4.2+t,0.,1.,0.,1.,1.,0,.002,.002,0.,.42)
            planner.prediction_cache.clear()
            planner.measured_velocity = (*velocity,0.)
            planner.policy_command = (*filtered,0.)
            planner.policy_command_stamp = planner.get_clock().now().nanoseconds*1e-9
            scored = [planner._score_candidate(Candidate(str(index),x,y,0.),(*position,0.),[person],[],[],{},0.)
                      for index,(x,y) in enumerate(commands)]
            feasible = [result for result in scored if not result.rejected]
            if not feasible:
                feasible = [result for result in scored if result.reason in ('human_margin','terminal_conflict')]
            selected = min(feasible,key=lambda result:result.score).candidate if feasible else Candidate('stop',0.,0.,0.)
            requested = np.array((selected.vx,selected.vy))
            if first_lateral is None and abs(requested[1])>.1:
                first_lateral = t
            planner.last_cmd.linear.x,planner.last_cmd.linear.y = map(float,requested)
            delayed.append(requested)
            target = delayed.popleft()
            proposed = filtered+np.clip(target-filtered,-.24,.24)
            filtered = np.asarray(planner.motion_envelope.limit_planar((*proposed,0.),(*filtered,0.))[:2])
            before = velocity.copy()
            velocity += (filtered*np.array((.96,.8))-velocity)*(1.-math.exp(-.1/.4))
            position += .05*(before+velocity)
            minimum = min(minimum,math.dist(position,(-4.2+t+.1,0.)))
        return first_lateral,minimum
    baseline,guarded = replay(0.),replay(6.)
    assert guarded[0] is not None and (baseline[0] is None or guarded[0]<baseline[0]), (baseline,guarded)
    assert guarded[1]>.8, (baseline,guarded)


@pytest.mark.parametrize('distance,margin', [(2.5,.25), (3.5,.75)])
def test_short_escape_does_not_choose_a_retreat_that_a_faster_person_catches(planner, distance, margin):
    planner.horizon = planner.hard_collision_horizon = 3.
    planner.body_prediction_margin = margin
    planner._simulate = lambda pose, c: [(c.vx*t, c.vy*t, 0., t) for t in np.linspace(0.,3.,31)]
    person = HumanTrack(1,'CONFIRMED',distance,0.,-1.,0.,1.,1.,0,.002,.002,0.,.42)
    candidates = [Candidate('yield_stop',0.,0.,0.), Candidate('retreat',-.45,0.,0.),
                  Candidate('sidestep',0.,.45,0.)]
    results = [planner._score_candidate(c,(0.,0.,0.),[person],[],[],{},0.) for c in candidates]
    assert all(result.rejected for result in results)
    # Retreat has the largest clearance gain at one second, but the person
    # catches it. Sidestepping preserves more clearance over the full forecast.
    if distance < 3.:
        assert results[2].min_human_distance > results[1].min_human_distance
    selected = planner._short_horizon_escape(results, [person])
    assert selected is not None
    assert selected.candidate.name == 'sidestep'


def test_physical_body_collision_is_rejected_beyond_the_uncertainty_window(planner):
    planner.horizon = 3.
    planner.hard_collision_horizon = 1.
    person = HumanTrack(1,'CONFIRMED',0.,-3.,0.,1.,1.,1.,0,.002,.002,0.,.42)
    planner.measured_velocity = (0.,0.,0.)
    result = planner._score_candidate(Candidate('hold',0.,0.,0.),(0.,0.,0.),[person],[],[],{},0.)
    assert result.reason == 'hard_human_collision'
    assert result.min_tcpa > planner.hard_collision_horizon


def test_near_term_prediction_uncertainty_is_checked_as_clearance(planner):
    planner.horizon = planner.hard_collision_horizon = 3.
    planner.body_prediction_margin = .4
    planner.hard_safety_margin = .4
    person = HumanTrack(1,'CONFIRMED',1.,1.3,0.,0.,1.,1.,0,.002,.002,0.,.42)
    predictions = {1:[PredictionSample(t,1.,1.3,0.,0.,cov,cov,1.)
                      for t,cov in ((0.,.002),(1.,.16),(3.,1.))]}
    candidate = Candidate('passing',.35,0.,0.)
    nominal = planner._score_candidate(candidate,(0.,0.,0.),[person],[],[],predictions,0.)
    assert not nominal.rejected
    planner.body_uncertainty_scale = .5
    robust = planner._score_candidate(candidate,(0.,0.,0.),[person],[],[],predictions,0.)
    assert robust.reason == 'hard_human_collision'
    sigma = next(iter(planner.prediction_cache.values()))[0][:,5]
    assert max(sigma) == pytest.approx(.4)


def test_social_cost_prefers_leaving_a_two_person_squeeze(planner):
    planner.weights['w_enclosure'] = 12.
    planner.motion_envelope = MotionEnvelope(max_y=.45,accel_y=2.4)
    planner.measured_velocity = (0.,0.,0.)
    people = [HumanTrack(i,'CONFIRMED',x,0.,0.,0.,1.,1.,0,.002,.002,0.,.42)
              for i,x in enumerate((-2.,2.))]
    stop = planner._score_candidate(Candidate('stop',0.,0.,0.),(0.,0.,0.),people,[],[],{},0.)
    leave = planner._score_candidate(Candidate('leave',0.,.45,0.),(0.,0.,0.),people,[],[],{},0.)
    assert not stop.rejected and not leave.rejected
    assert leave.score_terms['enclosure'] < stop.score_terms['enclosure']
    single = planner._score_candidate(Candidate('stop',0.,0.,0.),(0.,0.,0.),people[:1],[],[],{},0.)
    assert single.score_terms['enclosure'] == 0.


def test_centroid_noise_does_not_create_walking_intent_but_geometry_still_applies(planner):
    from dataclasses import replace
    planner.human_intent_min_speed = .25
    person = HumanTrack(1,'CONFIRMED',1.,-1.,0.,.2,1.,1.,0,.002,.002,0.,.42)
    command = Candidate('crossing',.65,0.,0.)
    assert not planner._relative_metrics_for_candidate((0.,0.,0.),command,person)['front_cut']
    assert planner._relative_metrics_for_candidate((0.,0.,0.),command,replace(person,vy=.6))['front_cut']
    stationary = replace(person, x=.5, y=0., vy=0.)
    assert planner._score_candidate(command,(0.,0.,0.),[stationary],[],[],{},0.).reason == 'hard_human_collision'


def test_standing_people_keep_onset_room_without_blocking_separating_motion(planner):
    from dataclasses import replace
    planner.stationary_person_clearance = 2.2
    planner.horizon = planner.hard_collision_horizon = 3.
    person = HumanTrack(1,'CONFIRMED',2.6,0.,0.,0.,1.,1.,0,.002,.002,0.,.42)
    approach = planner._score_candidate(Candidate('approach',.35,0.,0.),(0.,0.,0.),[person],[],[],{},0.)
    assert not approach.rejected
    assert approach.score_terms['onset'] > 0.
    planner.prediction_cache = {}
    departing = replace(person, vx=.7)
    follow = planner._score_candidate(Candidate('follow',.35,0.,0.),(0.,0.,0.),[departing],[],[],{},0.)
    assert not follow.rejected
    assert follow.score_terms['onset'] == 0.


def test_goal_capture_can_reach_goal_region_while_preserving_standing_person_space(planner):
    planner.stationary_person_clearance = 2.2
    planner.horizon = planner.hard_collision_horizon = 3.
    planner.path_reference_speed = .6
    planner.max_forward_near_human = .65
    planner.max_reverse_velocity = planner.max_lateral_near_human = .45
    planner.motion_envelope = MotionEnvelope(max_x=.65,max_y=.45,accel_x=2.4,accel_y=2.4,
                                             linear_time_constant=.4)
    planner.measured_velocity = (0.,0.,0.)
    planner.reference_goal_world = (0.,0.)
    person = HumanTrack(1,'CONFIRMED',1.8,0.,0.,0.,1.,1.,0,.002,.002,0.,.42)
    pose = (-.9,0.,0.)
    results = [planner._score_candidate(candidate,pose,[person],[],[],{},0.)
               for candidate in planner._goal_capture_candidates(pose,[person])]
    assert any(not result.rejected and result.min_human_distance >= 2.2 and
               min(math.hypot(x,y) for x,y,_,_ in result.trajectory) < .5 for result in results)


def test_escape_prefers_later_wall_over_earlier_person_contact(planner):
    planner.horizon = planner.hard_collision_horizon = 4.5
    planner.body_prediction_margin = .4
    planner._simulate = lambda pose,c: [(c.vx*t,c.vy*t,0.,t) for t in np.linspace(0.,4.5,46)]
    field = StaticDistanceField([(0.,2.)],(0.,0.),4.,1.25)
    planner.static_field_cache = field
    planner._static_obstacle_cost = lambda path: (0.,field.trajectory_is_blocked(np.array(path)[:,:2],.48))
    person = HumanTrack(1,'CONFIRMED',2.5,0.,-1.,0.,1.,1.,0,.002,.002,0.,.42)
    results = [planner._score_candidate(c,(0.,0.,0.),[person],[],[],{},0.) for c in
               (Candidate('yield_stop',0.,0.,0.),Candidate('retreat',-.45,0.,0.),Candidate('sidestep',0.,.45,0.))]
    selected = planner._short_horizon_escape(results,[person])
    assert selected is not None and selected.candidate.name == 'sidestep'
    assert selected.score_terms['violation_time'] > 3.


def test_short_recovery_does_not_claim_motion_from_an_unexecuted_future_phase(planner):
    planner.body_prediction_margin = .4
    planner.horizon = planner.hard_collision_horizon = 3.
    planner.measured_velocity = (0.,0.,0.)
    person = HumanTrack(1,'CONFIRMED',1.1,0.,0.,0.,1.,1.,0,.002,.002,0.,.42)
    results = [planner._score_candidate(c,(0.,0.,0.),[person],[],[],{},0.) for c in
               (Candidate('yield_stop',0.,0.,0.),Candidate('wait_then_route',0.,0.,0.,'',-.35,.3))]
    assert planner._short_horizon_escape(results,[person]) is None


def test_short_escape_can_leave_a_stationary_person_margin_overlap(planner):
    planner.body_prediction_margin = .4
    planner.horizon = planner.hard_collision_horizon = 3.
    planner.measured_velocity = (0.,0.,0.)
    person = HumanTrack(1,'CONFIRMED',1.1,0.,0.,0.,1.,1.,0,.002,.002,0.,.42)
    results = [planner._score_candidate(c,(0.,0.,0.),[person],[],[],{},0.)
               for c in (Candidate('yield_stop',0.,0.,0.),Candidate('away',-.35,0.,0.))]
    assert all(r.reason == 'hard_human_collision' for r in results)
    selected = planner._short_horizon_escape(results,[person])
    assert selected is not None and selected.candidate.name == 'away'


def test_full_horizon_clearance_recovery_still_rejects_body_contact(planner):
    planner.horizon = planner.hard_collision_horizon = 3.
    planner.body_prediction_margin = .25
    planner.hard_safety_margin = .40
    candidate = Candidate('passing', .35, 0., 0.)
    near = HumanTrack(1,'CONFIRMED',.5,1.08,0.,0.,1.,1.,0,.002,.002,0.,.42)
    result = planner._score_candidate(candidate,(0.,0.,0.),[near],[],[],{},0.)
    assert result.reason == 'human_margin'
    contact = HumanTrack(2,'CONFIRMED',.5,.7,0.,0.,1.,1.,0,.002,.002,0.,.42)
    result = planner._score_candidate(candidate,(0.,0.,0.),[contact],[],[],{},0.)
    assert result.reason == 'hard_human_collision'


def test_safe_goal_crossing_is_credited_even_if_rollout_continues_past_goal(planner):
    planner.nav2_path_reference = True
    planner.reference_goal_world = (.55, 0.)
    planner.reference_progress_target = (.55, 0.)
    planner.motion_envelope = MotionEnvelope(max_x=.65, accel_x=1.2)
    planner.measured_velocity = (.65,0.,0.)
    planner.command_duration = 1.7
    result = planner._score_candidate(Candidate('cross_goal',.65,0.,0.), (0.,0.,0.), [], [], [], {}, 0.)
    assert not result.rejected
    # Endpoint distance would show almost no progress despite passing directly
    # through the requested goal before the end of the command interval.
    assert result.score_terms['goal'] / planner.weights['w_goal'] < -.08


def test_entering_goal_region_earns_same_progress_as_crossing_its_centre(planner):
    planner.goal_cost_radius = .4
    planner.nav2_path_reference = True
    planner.reference_goal_world = planner.reference_progress_target = (0.,0.)
    planner._simulate = lambda pose,c: [(-1.,0.,0.,0.),(c.vx,0.,0.,1.7)]
    edge = planner._score_candidate(Candidate('edge',-.35,0.,0.),(-1.,0.,0.),[],[],[],{},0.)
    centre = planner._score_candidate(Candidate('centre',0.,0.,0.),(-1.,0.,0.),[],[],[],{},0.)
    assert edge.score_terms['goal'] == pytest.approx(centre.score_terms['goal'])


def test_checked_continuation_tolerates_small_reserve_changes_but_not_body_conflicts(planner):
    from dataclasses import replace
    planner.horizon = planner.hard_collision_horizon = 3.
    planner.body_prediction_margin = .4
    planner.continuation_prediction_margin = .3
    planner.hard_safety_margin = .55
    person = HumanTrack(1,'CONFIRMED',.5,1.15,0.,0.,1.,1.,0,.002,.002,0.,.42)
    candidate = Candidate('escape_pending',.35,0.,0.)
    normal = planner._score_candidate(candidate,(0.,0.,0.),[person],[],[],{},0.)
    assert normal.reason == 'hard_human_collision'
    continued = planner._checked_continuation(candidate,(0.,0.,0.),[person],[],[],{},0.)
    assert continued.reason == 'human_margin'
    assert continued.score_terms['continuation_margin'] == .3
    planner.prediction_cache = {}
    conflict = planner._checked_continuation(candidate,(0.,0.,0.),[replace(person,y=.7)],[],[],{},0.)
    assert conflict.reason == 'hard_human_collision'


def test_goal_urgency_requires_stagnation_and_never_bypasses_collision_checks(planner):
    planner.goal_urgency_max = 5.
    planner.goal_progress_timeout = 6.
    planner.mission_active = True
    planner.reference_goal_world = (3.,0.)
    planner._update_goal_urgency((0.,0.,0.),10.)
    assert planner.goal_urgency == 1.
    planner._update_goal_urgency((0.,0.,0.),22.)
    assert planner.goal_urgency == 5.
    person = HumanTrack(1,'CONFIRMED',.5,0.,0.,0.,1.,1.,0,.002,.002,0.,.42)
    assert planner._score_candidate(Candidate('forward',.35,0.,0.),(0.,0.,0.),[person],[],[],{},0.).rejected
    planner._update_goal_urgency((.3,0.,0.),23.)
    assert planner.goal_urgency == 1.
    planner._goal_callback(PoseStamped())
    assert planner.goal_progress_stamp is None


def test_selection_hysteresis_retains_safe_motion_but_not_stops_or_risky_commands(planner):
    planner.selection_hysteresis = .75
    previous = SimpleNamespace(candidate=Candidate('previous_command',.4,0.,0.),score=1.4,
                               min_human_distance=1.6,rejected=False)
    stop = SimpleNamespace(candidate=Candidate('yield_stop',0.,0.,0.),score=1.,
                           min_human_distance=1.65,rejected=False)
    assert planner._stable_selection([stop,previous]) is previous
    previous.rejected = True
    assert planner._stable_selection([stop,previous]) is stop
    previous.rejected = False
    previous.min_human_distance = 1.1
    assert planner._stable_selection([stop,previous]) is stop
    previous.min_human_distance = 1.6
    previous.candidate.vx = 0.
    assert planner._stable_selection([stop,previous]) is stop


def test_wait_then_go_can_clear_a_crossing_that_immediate_motion_hits(planner):
    planner.horizon = planner.hard_collision_horizon = 3.
    planner.body_prediction_margin = .4
    planner.hard_safety_margin = .55
    planner.motion_envelope = MotionEnvelope(max_x=.55,accel_x=2.4,linear_time_constant=.4)
    planner.measured_velocity = (0.,0.,0.)
    planner.policy_command = (0.,0.,0.)
    planner.policy_command_stamp = planner.get_clock().now().nanoseconds*1e-9
    person = HumanTrack(1,'CONFIRMED',1.5,-1.4,0.,1.,1.,1.,0,.002,.002,0.,.42)
    immediate = planner._score_candidate(Candidate('go',.55,0.,0.),(0.,0.,0.),[person],[],[],{},0.)
    delayed = planner._score_candidate(Candidate('wait_then_route',0.,0.,0.,'',.55,1.2),(0.,0.,0.),[person],[],[],{},0.)
    assert immediate.reason == 'hard_human_collision'
    assert not delayed.rejected
    assert delayed.trajectory[-1][0] > .5


def test_escape_continuation_clock_advances_and_new_goals_cancel_it(planner):
    from copy import deepcopy
    planner.mission_active = True
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x,goal.pose.position.y = 2.,1.
    planner._goal_callback(goal)
    planner.reference_goal_world = (2.,1.)
    candidate = Candidate('escape_then_route', -.45, .45, 0., '', .5, .6, .1)
    planner.escape_plan = (candidate,10.,planner._goal_key(planner.reference_goal))
    assert planner._pending_escape(10.2).switch_time == pytest.approx(.4)
    # SLAM correction changes map->odom coordinates, not the requested goal.
    planner.reference_goal_world = (2.01,1.01)
    refreshed = deepcopy(goal)
    refreshed.header.stamp.sec = 11
    planner._goal_callback(refreshed)
    assert planner._pending_escape(10.4).switch_time == pytest.approx(.2)
    following = planner._pending_escape(10.61)
    assert (following.vx,following.vy,following.wz) == (.5,.1,0.)
    assert planner.escape_plan is None
    planner.escape_plan = (candidate,10.,planner._goal_key(planner.reference_goal))
    goal.pose.position.x = -2.
    planner._goal_callback(goal)
    assert planner._pending_escape(10.2) is None
    planner.escape_plan = (candidate,10.,planner._goal_key(planner.reference_goal))
    planner._mission_callback(Bool(data=False))
    assert planner.escape_plan is None


def test_two_phase_escape_preserves_and_checks_route_resumption(planner):
    planner.horizon = 3.
    planner.hard_collision_horizon = 3.
    planner.motion_envelope = MotionEnvelope(max_x=.65, max_y=.45, accel_x=1.2, accel_y=1.2)
    planner.measured_velocity = (0.,0.,0.)
    left = Candidate('escape_then_route', .45, 0., 0., '', 0., .6, .45)
    right = Candidate('escape_then_route', .45, 0., 0., '', 0., .6, -.45)
    assert len(planner._executable_candidates([left, right])) == 2
    trajectory = planner._simulate((0.,0.,0.), left)
    assert trajectory[-1][0] < .5
    assert trajectory[-1][1] > .8
    person = HumanTrack(1,'CONFIRMED',.25,1.5,0.,0.,1.,1.,0,.002,.002,0.,.42)
    result = planner._score_candidate(left,(0.,0.,0.),[person],[],[],{},0.)
    assert result.reason == 'hard_human_collision'
    assert result.min_tcpa > left.switch_time


def test_short_route_command_still_checks_people_during_its_braking_tail(planner):
    planner.prefer_forward_travel = True
    planner.nav2_path_reference = True
    planner.horizon = 3.
    planner.measured_velocity = (0.,0.,0.)
    planner.latest_cmd.linear.x = .35
    candidate = planner._candidate_commands(planner.latest_cmd, 'NORMAL_NAVIGATION', (0.,0.,0.), [])[0]
    assert candidate.follow_vx == 0. and candidate.switch_time == .8
    stamp = planner.get_clock().now().nanoseconds*1e-9
    person = HumanTrack(1,'CONFIRMED',.15,-2.,0.,1.,1.,1.,0,.002,.002,stamp,.42)
    result = planner._score_candidate(candidate,(0.,0.,0.),[person],[],[],{},0.)
    assert result.rejected and result.reason == 'hard_human_collision'
    assert result.min_tcpa > candidate.switch_time


def test_passing_side_commitment_does_not_forbid_alignment_turns(planner):
    planner.side_commitment = -1.
    assert planner._side_switch_cost(Candidate('turn',0.,0.,.6)) == 0.
    assert planner._side_switch_cost(Candidate('opposite_side',0.,.45,0.)) > 0.


def test_passing_side_follows_an_admitted_route_detour(planner):
    planner.prefer_forward_travel = True
    planner.reference_progress_target = (1.,1.)
    person = HumanTrack(1,'CONFIRMED',1.,.1,0.,0.,1.,1.,0,.002,.002,0.,.42)
    assert planner._preferred_side((0.,0.,0.),[person]) == 1.


def test_supervisor_searches_checked_escape_when_stopping_is_unsafe():
    from unitree_g1_mujoco_bridge.human_collision_avoidance import time_aligned_prediction_risk
    node = HumanCollisionAvoidance()
    try:
        node.predictive_escape_search = True
        node.motion_envelope = MotionEnvelope(max_x=.65, max_y=.45, accel_x=1.2, accel_y=1.2)
        now = node.get_clock().now()
        seconds = now.nanoseconds*1e-9
        node.prediction_epoch = node.odom_stamp = node.scan_stamp = seconds
        node.scan_frame = node.robot_frame
        node.measured_velocity = (0.,0.,0.)
        node.latest_predictions = [[(float(t),1.8-t,0.,0.,0.,1.) for t in np.linspace(0.,3.,31)]]
        cmd, reason = node._validate_final_command(Twist(),(0.,0.,0.),now)
        assert reason == 'validated_escape'
        assert math.hypot(cmd.linear.x,cmd.linear.y) > 0.
        velocity = (cmd.linear.x,cmd.linear.y,cmd.angular.z)
        rollout = node.motion_envelope.rollout((0.,0.,0.),velocity,(0.,0.,0.),1.,.1,node.response_delay)
        risk = time_aligned_prediction_risk(node.latest_predictions,0.,(0.,0.,0.),velocity,1.,.1,
                                           math.inf,node.prediction_uncertainty_scale,node.prediction_min_confidence,rollout)
        assert risk[0] >= .8
        node.scan_obstacles = [(0.2,0.)]
        blocked, reason = node._validate_final_command(Twist(),(0.,0.,0.),now)
        assert reason == 'temporal_override'
        assert blocked.linear.x == blocked.linear.y == 0.
        node.scan_obstacles = []
        node.scan_stamp = seconds-2.
        stale, reason = node._validate_final_command(Twist(),(0.,0.,0.),now)
        assert reason == 'temporal_override'
        assert stale.linear.x == stale.linear.y == 0.
    finally:
        node.destroy_node()


def test_supervisor_constructs_critical_feedback_subscriptions():
    node = HumanCollisionAvoidance()
    try:
        topics = {subscription.topic_name for subscription in node.subscriptions}
        assert {'/odom','/g1/policy_cmd_vel'} <= topics
    finally:
        node.destroy_node()


def test_supervisor_rechecks_and_executes_fresh_matching_timed_stop():
    import json
    from std_msgs.msg import String
    node = HumanCollisionAvoidance()
    try:
        node.use_planned_sequence = True
        node.motion_envelope = MotionEnvelope(max_x=.55,accel_x=2.4,linear_time_constant=.4)
        now = node.get_clock().now()
        seconds = now.nanoseconds*1e-9
        node.prediction_epoch = node.odom_stamp = seconds
        node.measured_velocity = (0.,0.,0.)
        node.latest_predictions = [[(float(t),1.2,0.,0.,0.,1.) for t in np.linspace(0.,3.,31)]]
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.pose.position.x = 2.
        node._goal_callback(goal)
        node._mission_callback(Bool(data=True))
        node.mission_state_time = now
        payload = dict(stamp=seconds,frame=node.robot_frame,active=True,first=[.5,0.,0.],
                       then=[0.,0.,0.],duration=.2,goal=node.goal_signature)
        node._sequence_callback(String(data=json.dumps(payload)))
        request = Twist()
        request.linear.x = .5
        safe, reason = node._validate_final_command(request,(0.,0.,0.),now)
        assert reason == 'planned_sequence' and safe.linear.x == .5
        stopped, reason = node._validate_final_command(request,(0.,0.,0.),now+Duration(seconds=.21))
        assert reason == 'planned_sequence' and stopped.linear.x == 0.
        assert node._sequence_parameters((.5,0.,0.),seconds+.3) is None
        assert node._sequence_parameters((.4,0.,0.),seconds+.1) is None
        node._mission_callback(Bool(data=False))
        assert node.command_sequence is None
        node._mission_callback(Bool(data=True))
        payload['frame'] = 'wrong_frame'
        node._sequence_callback(String(data=json.dumps(payload)))
        assert node.command_sequence is None
    finally:
        node.destroy_node()


def test_supervisor_allows_nonworsening_escape_from_existing_person_margin():
    node = HumanCollisionAvoidance()
    try:
        node.motion_envelope = MotionEnvelope(max_x=.65,accel_x=2.4)
        node.predictive_escape_search = True
        alternatives = []
        node._validated_predictive_escape = lambda *args: alternatives.append(args)
        now = node.get_clock().now()
        seconds = now.nanoseconds*1e-9
        node.prediction_epoch = node.odom_stamp = seconds
        node.measured_velocity = (0.,0.,0.)
        node.latest_predictions = [[(float(t),.95,0.,0.,0.,1.) for t in np.linspace(0.,3.,31)]]
        command = Twist()
        command.linear.x = -.3
        safe, reason = node._validate_final_command(command,(0.,0.,0.),now)
        assert reason == 'clearance_recovery'
        assert safe.linear.x < 0.
        assert alternatives == []
        command.linear.x = .3
        unsafe, reason = node._validate_final_command(command,(0.,0.,0.),now)
        assert reason == 'temporal_override' and unsafe.linear.x == 0.
        assert len(alternatives) == 1
    finally:
        node.destroy_node()


def test_supervisor_aligns_robot_and_people_to_the_same_current_epoch():
    node = HumanCollisionAvoidance()
    try:
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
        message = Odometry()
        message.header.frame_id = node.target_frame
        message.child_frame_id = node.robot_frame
        message.header.stamp.sec = 9
        message.header.stamp.nanosec = 800_000_000
        message.pose.pose.orientation.z = math.sin(math.pi/4)
        message.pose.pose.orientation.w = math.cos(math.pi/4)
        message.twist.twist.linear.x = .5
        node._odom_callback(message)
        node.tf_buffer.lookup_transform = lambda *args: (_ for _ in ()).throw(TransformException('no TF'))
        assert node._robot_pose()[:3] == pytest.approx((0., .1, math.pi/2))
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=11_000_000_000))
        assert node._robot_pose() is None
    finally:
        node.destroy_node()


def test_dynamic_path_handoff_rejects_late_results_to_the_same_goal(planner):
    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.pose.position.x = 2.
    planner._goal_callback(goal)
    current = Path()
    current.header.frame_id = "world"
    current.header.stamp.sec = 20
    current.poses = [goal]
    planner._path_callback(current)
    late = Path()
    late.header.frame_id = "world"
    late.header.stamp.sec = 19
    late.poses = [goal]
    planner._path_callback(late)
    assert planner.reference_path is current


def test_mission_heartbeat_owns_reference_and_cancellation_stops_it(planner):
    planner.nav2_path_reference = True
    planner.use_static_scene_map = True
    path = Path()
    path.header.frame_id = "world"
    point = PoseStamped()
    point.pose.position.x = 2.0
    path.poses = [point]
    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.pose.position.x = 2.0
    planner._goal_callback(goal)
    planner._path_callback(path)
    now = planner.get_clock().now()
    planner.tracks_epoch = planner.odom_stamp = now.nanoseconds * 1e-9
    planner.latest_cmd_time = now - Duration(seconds=2.0)
    planner._mission_callback(Bool(data=True))
    planner._timer_callback()
    assert planner.published[-1][0].linear.x > 0.0
    planner._mission_callback(Bool(data=False))
    planner._timer_callback()
    assert planner.published[-1][0].linear.x == 0.0
    planner._mission_callback(Bool(data=True))
    planner.mission_state_time -= Duration(seconds=2.0)
    planner._timer_callback()
    assert planner.published[-1][1] == "MISSION_STALE_STOP"


def test_missing_scan_transform_is_attempted_only_once_per_plan(planner):
    scan = LaserScan()
    scan.header.frame_id = "test_sensor"
    scan.header.stamp = planner.get_clock().now().to_msg()
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.angle_increment = 0.1
    scan.ranges = [2.0]
    planner._scan_callback(scan)
    calls = []
    def unavailable(*args):
        calls.append(args)
        raise TransformException("not ready")
    planner.tf_buffer.lookup_transform = unavailable
    trajectory = [(0.0, 0.0, 0.0, 0.0), (0.2, 0.0, 0.0, 1.0)]
    for _ in range(3):
        assert HumanAwareMotionPlanner._static_obstacle_cost(planner, trajectory)[1]
    assert len(calls) == 1


@pytest.mark.parametrize("person_start,person_velocity", [((5.0, 0.0), (-0.6, 0.0)), ((1.5, -3.0), (0.0, 0.8))])
def test_closed_loop_avoids_nonreactive_person(planner, person_start, person_velocity):
    pose = (0.0, 0.0, 0.0)
    velocity = (0.0, 0.0, 0.0)
    pending = [(0.0, 0.0, 0.0)] * 2
    minimum = math.inf
    decisions = []
    planner.motion_envelope = MotionEnvelope(max_y=0.0, max_yaw=0.35, accel_x=0.35, arc_forward=0.06, arc_yaw=0.12)
    for step in range(100):
        t = step * 0.1
        person_x = person_start[0] + person_velocity[0] * t
        person_y = person_start[1] + person_velocity[1] * t
        minimum = min(minimum, math.hypot(pose[0] - person_x, pose[1] - person_y))
        now = planner.get_clock().now()
        stamp = now.nanoseconds * 1e-9
        planner._robot_pose = lambda: pose
        heading = math.atan2(-pose[1], 6.0 - pose[0])
        error = math.atan2(math.sin(heading - pose[2]), math.cos(heading - pose[2]))
        planner.latest_cmd.linear.x = 0.35
        planner.latest_cmd.angular.z = max(-0.35, min(0.35, error))
        planner.latest_cmd_time = now
        planner.tracks_epoch = planner.odom_stamp = stamp
        planner.measured_velocity = velocity
        planner.tracks = {1: HumanTrack(1, "CONFIRMED", person_x, person_y, *person_velocity, 1.0, 1.0, 0, 0.01, 0.01, stamp, 0.42)}
        planner._timer_callback()
        cmd = planner.published[-1][0]
        if step % 10 == 0:
            decisions.append((t, planner.published[-1][1], (cmd.linear.x, cmd.linear.y, cmd.angular.z)))
        pending.append((cmd.linear.x, cmd.linear.y, cmd.angular.z))
        target = planner.motion_envelope.normalize(pending.pop(0))
        limits = (0.35, 0.16, 0.35)
        velocity = tuple(v + max(-a * 0.1, min(a * 0.1, u - v)) for v, u, a in zip(velocity, target, limits))
        x, y, yaw = pose
        pose = (x + math.cos(yaw) * velocity[0] * 0.1, y + math.sin(yaw) * velocity[0] * 0.1, yaw + velocity[2] * 0.1)
    assert minimum > 0.80, decisions
    assert pose[0] > 0.5


def test_supervisor_rechecks_changed_commands_and_stale_predictions():
    node = HumanCollisionAvoidance()
    try:
        now = node.get_clock().now()
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        node._odom_callback(odom)
        node._robot_pose = lambda: (0.0, 0.0, 0.0, 0.0)
        node.prediction_epoch = now.nanoseconds * 1e-9
        node.latest_predictions = []
        cmd = Twist()
        cmd.linear.x = 0.3
        safe, reason = node._validate_final_command(cmd, (0.0, 0.0, 0.0), now)
        assert safe.linear.x == pytest.approx(0.3)
        assert reason is None
        node._cmd_callback(cmd)
        node._timer_callback()
        node.latest_predictions = [[(0.0, 0.9, 0.0, 0.0, 0.0, 1.0), (2.0, 0.9, 0.0, 0.0, 0.0, 1.0)]]
        safe, reason = node._validate_final_command(cmd, (0.0, 0.0, 0.0), now)
        assert safe.linear.x == 0.0
        assert reason == "temporal_override"
        node.prediction_epoch -= 2.0
        assert node._validate_final_command(cmd, (0.0, 0.0, 0.0), now)[1] == "stale_predictions_stop"
    finally:
        node.destroy_node()
