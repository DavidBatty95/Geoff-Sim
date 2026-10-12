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
