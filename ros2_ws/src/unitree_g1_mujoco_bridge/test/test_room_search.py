import math

import numpy as np

from .test_exploration_search import grid
from unitree_g1_mujoco_bridge.exploration_search import ObservedSearchMap,SearchView
from unitree_g1_mujoco_bridge.room_search import RoomTopology


def rooms():
    data = np.zeros((100,160),dtype=int)
    data[:,79:81] = 100
    data[35:65,79:81] = 0
    observed = ObservedSearchMap(grid(data,resolution=.1),.55)
    return observed,RoomTopology(observed)


def test_door_neck_separates_rooms_while_floor_route_remains_connected():
    observed,topology = rooms()
    left,right = observed.world(40,50),observed.world(120,50)
    assert topology.region(left)>0 and topology.region(right)>0
    assert topology.region(left)!=topology.region(right)
    assert observed.route(left,right)


def test_visibility_memory_does_not_clear_area_behind_a_wall():
    observed,topology = rooms()
    position = observed.world(65,20)
    seen = topology.visible(position,0.,observed)
    assert seen
    wall_x = observed.world(79,20)[0]
    assert all(key[0]/2<wall_x for key in seen)


def test_room_commitment_then_bounded_switch_avoids_repeated_empty_room_views():
    observed,topology = rooms()
    left,right = observed.world(40,50),observed.world(120,50)
    goals = [SearchView(*left,0.,100.,2.,0,'visual_coverage'),
             SearchView(*right,0.,1.,10.,0,'visual_coverage')]
    chosen,summary = topology.decide(goals,left,set(),[],left)
    assert topology.region((chosen[0].x,chosen[0].y))==topology.region(left)
    inspections = [(*left,yaw) for yaw in (0.,math.pi/2,math.pi,-math.pi/2)]
    chosen,summary = topology.decide(goals,left,set(),inspections,left)
    assert topology.region((chosen[0].x,chosen[0].y))==topology.region(right)
    assert next(room for room in summary if room['region']==topology.region(left))['deferred']
    # A newly visible frontier can reopen an incompletely observed room.
    goals[0] = SearchView(*left,0.,100.,2.,12,'frontier')
    chosen,_ = topology.decide(goals,left,set(),inspections,left)
    assert topology.region((chosen[0].x,chosen[0].y))==topology.region(left)


def test_region_metadata_is_bounded_and_unknown_floor_has_no_room():
    observed = ObservedSearchMap(grid(np.full((40,40),-1)),.55)
    topology = RoomTopology(observed)
    assert topology.region((0.,0.))==0 and not topology.points
    assert topology.decide([], (0.,0.),set(),[])==([],[])


def test_open_building_cannot_collapse_into_one_unbounded_region():
    observed = ObservedSearchMap(grid(np.zeros((200,300),dtype=int),resolution=.1),.55)
    topology = RoomTopology(observed)
    assert 4<len(topology.points)<=128
    assert all(np.max(np.ptp(points,axis=0))<=8.1 for points in topology.points.values())


def test_camera_view_heading_avoids_half_turns_and_keeps_heading_without_gain():
    observed,topology = rooms()
    position = observed.world(40,50)
    goal = SearchView(*position,.2,1.,2.,0,'visual_coverage')
    selected,_ = topology.decide([goal],position,set(),[])
    error = selected[0].yaw-goal.yaw
    assert abs(math.atan2(math.sin(error),math.cos(error)))<=math.pi/3+1e-9
    seen = {key for keys in topology.keys.values() for key in keys}
    selected,_ = topology.decide([goal],position,seen,[])
    assert selected[0].yaw==goal.yaw


def test_region_identity_follows_overlap_rather_than_construction_order():
    observed,previous = rooms()
    current = RoomTopology(observed)
    labels = list(previous.points)
    mapping = {label:label+50 for label in labels}
    original = previous.labels.copy()
    for label in labels:
        previous.labels[original==label] = mapping[label]
    previous.points = {mapping[label]:points for label,points in previous.points.items()}
    previous.keys = {mapping[label]:keys for label,keys in previous.keys.items()}
    current.reconcile(previous)
    assert np.array_equal(current.labels,previous.labels)


def test_coverage_uses_calibrated_camera_horizontal_fov():
    observed,topology = rooms()
    position = observed.world(40,50)
    narrow = topology.visible(position,0.,observed,hfov=math.pi/6)
    wide = topology.visible(position,0.,observed,hfov=math.pi/2)
    assert narrow < wide


def test_inspection_is_credited_to_seen_region_not_camera_standing_region():
    observed,topology = rooms()
    left,right = observed.world(40,50),observed.world(120,50)
    region = topology.region(left)
    goals = [SearchView(*left,0.,100.,2.,0,'visual_coverage'),
             SearchView(*right,0.,1.,10.,0,'visual_coverage')]
    coverage = [frozenset(topology.keys[region]) for _ in range(3)]
    chosen,summary = topology.decide(goals,left,set(),[(*right,0.)],left,inspection_coverage=coverage)
    record = next(room for room in summary if room['region']==region)
    assert record['inspections']==3 and record['deferred']
    assert topology.region((chosen[0].x,chosen[0].y))==topology.region(right)
