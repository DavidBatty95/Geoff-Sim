from concurrent.futures import Future
from types import SimpleNamespace as S

from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from unitree_g1_mujoco_bridge.exploration_mission import ExplorationMission
from unitree_g1_mujoco_bridge.exploration_search import SearchView
from unitree_g1_mujoco_bridge.search_visualization import search_markers


def test_search_overlay_shows_candidates_selected_view_and_semantic_memory():
    goal = SearchView(2.,3.,.5,1.2,4.,8,'frontier')
    messages = search_markers(Time(sec=10),[goal],goal,[(0.,0.)],[(1.,1.)],(0.,0.),
        'box_red','navigate_to_view',8.,[('box_red',3.,4.,1.,.2)])
    assert messages.markers[0].action==Marker.DELETEALL
    assert all(marker.header.frame_id=='map' for marker in messages.markers[1:])
    arrow = next(marker for marker in messages.markers if marker.action==Marker.ADD and marker.type==Marker.ARROW)
    assert (arrow.pose.position.x,arrow.pose.position.y)==(2.,3.)
    assert any('box_red' in marker.text and 'navigate_to_view' in marker.text for marker in messages.markers)
    assert any('0.2s old' in marker.text for marker in messages.markers)


def test_destination_approach_arrow_points_inward_and_matches_goal_color():
    import math
    messages = search_markers(Time(sec=10),[],None,[],[],(0.,0.),'target_blue','inspect_support',0.,
                             [('target_blue',3.,4.,.95,.2,math.pi/2)])
    arrow = next(marker for marker in messages.markers if marker.type==Marker.ARROW and marker.action==Marker.ADD)
    assert abs(arrow.pose.position.x-3.)<1e-9 and abs(arrow.pose.position.y-3.2)<1e-9
    assert arrow.color.b>arrow.color.r and arrow.pose.orientation.z>0.
    assert any('observed approach side' in marker.text for marker in messages.markers)


def test_stale_worker_result_cannot_replace_new_search_epoch():
    future = Future()
    future.set_result(([SearchView(1.,2.,0.,1.,2.,3,'frontier')],True,1.,None))
    search = S(pending=(('old',1,False),future,0.),prepared=None,target='new',generation=2,
               mission=S(object_may_be_held=False),state='select_viewpoint',candidates=[],
               _key=lambda:('new',2,False),_fresh_map=lambda:False)
    ExplorationMission._prepare(search)
    assert search.pending is None and search.prepared is None and search.candidates==[]


def test_running_search_does_not_enqueue_more_work():
    future = Future()
    search = S(pending=(('box_red',1,False),future,0.),target='box_red',mission=S())
    ExplorationMission._prepare(search)
    assert search.pending[1] is future


def test_semantic_camera_heading_survives_cleared_navigation_visualization():
    import math
    turns,found = [],[]
    def rotate(yaw,**kwargs):
        turns.append(yaw)
        found.append({'id':'target_blue','observed_stamp':1.})
    node = S(pose=(0.,0.,0.),object_may_be_held=False,observe=lambda *args:found[-1] if found else None,
             event=lambda *args,**kwargs:None,rotate=rotate,seconds=lambda:0.,last_image_stamp=1.,
             wait=lambda predicate,*args:predicate())
    search = S(mission=node,visits={},_prepare=lambda:None,max_views=1,inspection_heading=math.pi,
               inspection_kind='semantic_reobserve',selected=None)
    result = ExplorationMission._discover(search,'target_blue')
    assert result['id']=='target_blue' and turns==[math.pi]


def test_displaced_optional_inspection_keeps_fresh_discovery():
    import time
    found,events = [],[]
    def rotate(*args,**kwargs):
        raise RuntimeError('search turn displaced beyond viewpoint reserve')
    node = S(pose=(.4,0.,.2),object_may_be_held=False,
             observe=lambda *args:found[-1] if found else None,
             event=lambda name,**kwargs:events.append(name),rotate=rotate,
             seconds=lambda:10.,last_image_stamp=11.,started=time.monotonic(),args=S(duration=600.),
             spin=lambda _:found.append({'id':'target_blue','observed_stamp':11.}),
             wait=lambda predicate,*args:predicate())
    search = S(mission=node,visits={},_prepare=lambda:None,max_views=1,
               inspection_heading=1.,inspection_kind='heading_inspection',selected=None)
    assert ExplorationMission._discover(search,'target_blue')['observed_stamp']==11.
    assert 'search_partial_turn' in events and events[-1]=='search_found'


def test_slam_frame_correction_rebases_live_visit_list_and_revokes_prepared_plan():
    visits = [(1.,2.)]
    search = S(memory_transform=(0.,0.,0.),coverage_transform=(0.,0.,0.),
        visits={'box':visits},inspections={'box':[(1.,2.,.3)]},coverage={'box':{(2,4)}},
        failures=[((3.,4.),10.)],room_anchor=(1.,2.),generation=1,prepared=object(),
        mission=S(event=lambda *args,**kwargs:None),_point=ExplorationMission._point)
    ExplorationMission._sync_memory(search,(3.,0.,0.))
    assert visits==[(4.,2.)]  # The executive keeps this list reference during search.
    assert search.inspections['box']==[(4.,2.,.3)]
    assert not search.coverage['box'] and search.prepared is None and search.generation==2


def test_dispatch_reranks_worker_novelty_after_latest_inspection():
    import numpy as np
    from .test_exploration_search import grid
    goals = [SearchView(1.2,0.,0.,100.,1.,0,'visual_coverage'),
             SearchView(3.,0.,0.,1.,1.,0,'visual_coverage')]
    search = S(mission=S(pose=(0.,0.,0.),pose_stamp=10.,seconds=lambda:10.,object_may_be_held=False),
               map=grid(np.zeros((60,60),dtype=int)),_fresh_map=lambda:True,
               _transform=lambda *_:(0.,0.,0.),_point=ExplorationMission._point,
               _people=lambda _:[],prepared=(None,None,goals,True))
    result = ExplorationMission._revalidated(search,[(0.,0.)])
    assert result[0].x==3.
    assert result[0].score<100. and result[0].path_length>result[1].path_length


def test_mission_pump_preserves_callback_iterator_fairness(monkeypatch):
    from unitree_g1_mujoco_bridge.transport_demo import TransportMission
    import unitree_g1_mujoco_bridge.transport_demo as module
    clock = [10.]
    timeouts = []
    ready_index = [0]
    mission = S(started=10.,args=S(duration=100.),active=True,pose=(0.,0.,0.),pose_stamp=6.,seconds=lambda:clock[0])
    def spin_once(timeout_sec):
        # Model rclpy restarting ready iteration when its arguments change.
        ready_index[0] = ready_index[0]+1 if timeouts and timeout_sec==timeouts[-1] else 0
        timeouts.append(timeout_sec)
        clock[0] += .003
        if ready_index[0]>=2:
            mission.pose_stamp = clock[0]
    mission.executor = S(spin_once=spin_once)
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(module.rclpy,'ok',lambda:True)
    TransportMission.spin(mission,.05)
    assert mission.pose_stamp>10. and set(timeouts)=={.01}


def test_blocked_route_snapshot_preserves_map_and_query_identity(tmp_path):
    import hashlib
    import json
    import numpy as np
    data = np.array([[0,-1],[100,0]],dtype=np.int16)
    evidence = dict(start_map=[1.,2.],goal_map=[3.,4.],resolution=.1,origin=[0.,0.],yaw=.2)
    search = S(route_blockage_snapshot=(data,evidence))
    ExplorationMission.save_snapshot(search,tmp_path)
    stored = json.loads((tmp_path/'route_blockage.json').read_text())
    assert stored['start_map']==[1.,2.] and stored['yaw']==.2
    assert stored['map_sha256']==hashlib.sha256((tmp_path/'route_blockage_map.npz').read_bytes()).hexdigest()
    with np.load(tmp_path/'route_blockage_map.npz') as snapshot:
        np.testing.assert_array_equal(snapshot['occupancy'],data)
