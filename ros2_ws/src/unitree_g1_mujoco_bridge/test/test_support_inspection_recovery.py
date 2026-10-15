"""A braked displaced view may be reobserved; faults are never blind retries."""
from types import SimpleNamespace as S

import pytest

import unitree_g1_mujoco_bridge.exploration_mission as module
from unitree_g1_mujoco_bridge.exploration_mission import ExplorationMission
from unitree_g1_mujoco_bridge.exploration_search import SearchView
from unitree_g1_mujoco_bridge.localization_recovery import BodyInstabilityError,ViewpointDisplaced
from unitree_g1_mujoco_bridge.transport_actions import ActionTerminationError
from unitree_g1_mujoco_bridge.transport_demo import MotionSettlingError


def fixture(monkeypatch,confirm_after=1,error=None,old_observation=False):
    clock = [10.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    queries,dispatches,events,turns,images = [],[],[],[],[]
    observations = {}
    node = S(started=0.,args=S(duration=60.),pose=(0.,0.,0.),pose_stamp=10.,active=False,
             object_may_be_held=False,last_image_stamp=9.,seconds=lambda:clock[0],
             observe=lambda *args:observations.get('target'),
             event=lambda name,**details:events.append((name,details)))
    class Grid:
        def __init__(self,*args):
            pass
        def semantic_viewpoints(self,robot,target,people):
            queries.append(robot)
            return [SearchView(float(len(queries)),0.,0.,1.,1.,0,'semantic_reobserve')]
    monkeypatch.setattr(module,'ObservedSearchMap',Grid)
    def wait(predicate,timeout,message):
        if predicate():
            return
        assert timeout>0.
        clock[0] += .25
        node.pose_stamp = clock[0]
        node.last_image_stamp = clock[0]
        images.append(clock[0])
        if len(turns)>=confirm_after:
            observations['target'] = dict(position=[3.,0.,1.],approach_yaw=0.,
                observed_stamp=9. if old_observation else clock[0],confirmed=True)
        assert predicate(),message
    def navigate(goal,**kwargs):
        assert not node.active
        dispatches.append((goal,kwargs['timeout']))
        node.pose = tuple(goal)
    def rotate(*args,**kwargs):
        turns.append(kwargs['timeout'])
        clock[0] += 1.
        node.pose = (node.pose[0]+.4,0.,.2)
        node.pose_stamp = clock[0]
        node.active = False  # The real rotate revokes intent before propagation.
        raise error or ViewpointDisplaced('search turn displaced beyond viewpoint reserve')
    node.wait,node.rotate = wait,rotate
    search = S(mission=node,map=object(),generation=1,prepared=object(),state='idle',
        _fresh_map=lambda:True,_transform=lambda *args:(0.,0.,0.),_people=lambda tf:[],
        _point=ExplorationMission._point,navigate_observed=navigate)
    return search,clock,queries,dispatches,events,turns,images


def test_displaced_support_view_requires_new_acquisitions_before_return(monkeypatch):
    search,clock,queries,dispatches,events,turns,images = fixture(monkeypatch)
    result = ExplorationMission.inspect_support(search,'target_red',dict(position=[3.,0.,1.]))
    assert result['observed_stamp']>11. and len(images)==2
    assert len(dispatches)==len(turns)==1 and not search.mission.active
    assert search.prepared is None and search.generation==2
    assert any(name=='support_inspection_interrupted' for name,_ in events)


def test_missing_support_replans_from_displaced_pose_with_remaining_budget(monkeypatch):
    search,clock,queries,dispatches,events,turns,images = fixture(monkeypatch,confirm_after=2)
    ExplorationMission.inspect_support(search,'target_red',dict(position=[3.,0.,1.]))
    assert queries==[(0.,0.),(1.4,0.)]
    assert len(dispatches)==2 and dispatches[1][1]<dispatches[0][1]
    assert len(images)==4 and search.generation==3


@pytest.mark.parametrize('error',[BodyInstabilityError('fall'),ActionTerminationError('unknown'),
    MotionSettlingError('timeout'),RuntimeError('search turn displaced beyond viewpoint reserve')])
def test_only_explicit_settled_displacement_is_recoverable(monkeypatch,error):
    search,clock,queries,dispatches,events,turns,images = fixture(monkeypatch,error=error)
    with pytest.raises(type(error)):
        ExplorationMission.inspect_support(search,'target_red',dict(position=[3.,0.,1.]))
    assert len(dispatches)==1 and not images
    assert not any(name=='support_inspection_interrupted' for name,_ in events)


def test_old_support_memory_does_not_satisfy_reinspection(monkeypatch):
    search,clock,queries,dispatches,events,turns,images = fixture(monkeypatch,old_observation=True)
    with pytest.raises(RuntimeError,match='support edge not confirmed'):
        ExplorationMission.inspect_support(search,'target_red',dict(position=[3.,0.,1.]))
    assert len(turns)==len(dispatches)==3 and len(images)==6


def test_displacement_cannot_extend_original_mission_deadline(monkeypatch):
    search,clock,queries,dispatches,events,turns,images = fixture(monkeypatch)
    def expire(*args,**kwargs):
        clock[0] = 61.
        raise ViewpointDisplaced('search turn displaced beyond viewpoint reserve')
    search.mission.rotate = expire
    with pytest.raises(RuntimeError,match='support inspection deadline'):
        ExplorationMission.inspect_support(search,'target_red',dict(position=[3.,0.,1.]))
    assert len(dispatches)==1 and not images
