"""Withdraw docking intent during sensed workspace intrusion, preserving deadlines."""
from types import SimpleNamespace as S

import pytest

from unitree_g1_mujoco_bridge.transport_demo import TransportMission
import unitree_g1_mujoco_bridge.transport_demo as module


def fixture(monkeypatch):
    clock = [0.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    goals,events = [],[]
    node = S(pose=(0.,0.,0.),pose_stamp=0.,seconds=lambda:clock[0],phase='dock',
        clear=True,active=False,spin=lambda duration:None,
        event=lambda phase,**details:events.append(details))
    node.workspace_clear = lambda target=None:node.clear
    node.stop_intent = lambda:setattr(node,'active',False)
    def goal(pose,mode):
        assert node.clear
        goals.append(pose)
        node.active = True
        node.pose = pose
        node.pose_stamp = clock[0]
    node.set_goal = goal
    node.wait = lambda predicate,*args:predicate()
    return node,clock,goals,events


def test_occupied_workspace_cannot_dispatch_dock_until_dwell_succeeds(monkeypatch):
    node,clock,goals,events = fixture(monkeypatch)
    node.clear = False
    def clearance(timeout,target):
        assert target==(1.,0.,0.)
        assert not node.active and not goals and timeout==30.
        clock[0] += 2.
        node.clear = True
    node.wait_workspace = clearance
    TransportMission.dock.__wrapped__(node,(1.,0.,0.))
    assert goals==[(1.,0.,0.)] and not node.active
    assert events[0]['dock_workspace_interruption'] and events[1]['dock_workspace_resumed']


def test_intrusion_during_approach_revokes_intent_before_waiting(monkeypatch):
    node,clock,goals,events = fixture(monkeypatch)
    def wait(predicate,*args):
        if len(goals)==1:
            node.clear = False
            node.pose = (.5,0.,0.)
        assert predicate()
    node.wait = wait
    def clearance(timeout,target):
        assert not node.active and node.pose==(.5,0.,0.)
        node.clear = True
    node.wait_workspace = clearance
    TransportMission.dock.__wrapped__(node,(1.,0.,0.))
    assert goals==[(1.,0.,0.)]*2 and not node.active
    assert events[-1]['dock_settled']


def test_intrusion_during_settling_does_not_become_coast_compensation(monkeypatch):
    node,clock,goals,events = fixture(monkeypatch)
    def settle(duration):
        if len(goals)==1:
            node.clear = False
            node.pose = (1.3,.2,0.)
    node.spin = settle
    def clearance(timeout,target):
        assert not node.active
        node.clear = True
    node.wait_workspace = clearance
    TransportMission.dock.__wrapped__(node,(1.,0.,0.))
    assert goals==[(1.,0.,0.)]*2  # No biased target derived from the escape.


def test_workspace_wait_uses_remaining_original_deadline(monkeypatch):
    node,clock,goals,events = fixture(monkeypatch)
    node.clear = False
    def clearance(timeout,target):
        assert timeout==2. and not node.active
        clock[0] = 2.
        node.clear = True
    node.wait_workspace = clearance
    with pytest.raises(RuntimeError,match='settle before deadline'):
        TransportMission.dock.__wrapped__(node,(1.,0.,0.),timeout=2.)
    assert not goals and not node.active


def test_clearance_failure_is_not_converted_into_docking_success(monkeypatch):
    node,clock,goals,events = fixture(monkeypatch)
    node.clear = False
    def clearance(timeout,target):
        raise RuntimeError('body tilt exceeded manipulation limit while waiting')
    node.wait_workspace = clearance
    with pytest.raises(RuntimeError,match='body tilt'):
        TransportMission.dock.__wrapped__(node,(1.,0.,0.))
    assert not goals and not node.active


@pytest.mark.parametrize('pose,stamp,expected',[
    ((0.,0.,0.),10.,True),((0.,0.,0.),9.4,False),((0.,0.,0.),10.1,False),
    ((float('nan'),0.,0.),10.,False),(None,10.,False)])
def test_fresh_empty_people_packet_cannot_hide_stale_or_invalid_robot_pose(pose,stamp,expected):
    node = S(pose=pose,pose_stamp=stamp,seconds=lambda:10.,
             people=S(header=S(frame_id='odom',stamp=S(sec=10,nanosec=0)),persons=[]))
    assert TransportMission.workspace_clear(node) is expected


def test_docking_admission_checks_corridor_in_addition_to_robot_workspace():
    person = S(pose=S(pose=S(position=S(x=2.,y=2.))),velocity=S(twist=S(linear=S(x=0.,y=-2.))))
    node = S(pose=(0.,0.,0.),pose_stamp=10.,seconds=lambda:10.,
             people=S(header=S(frame_id='odom',stamp=S(sec=10,nanosec=0)),persons=[person]))
    assert TransportMission.workspace_clear(node)
    assert not TransportMission.workspace_clear(node,(4.,0.,0.))


def test_coast_compensated_goal_is_checked_before_dispatch(monkeypatch):
    node,clock,goals,events = fixture(monkeypatch)
    checked,yields = [],[]
    def clear(target=None):
        checked.append(target)
        return target is None or target[0]>=.99 or bool(yields)
    node.workspace_clear = clear
    def settle(duration):
        if len(goals)==1:
            node.pose = (1.08,0.,0.)
    node.spin = settle
    def clearance(timeout,target):
        assert target[0]<.99 and not node.active
        yields.append(target)
    node.wait_workspace = clearance
    TransportMission.dock.__wrapped__(node,(1.,0.,0.))
    assert len(yields)==1 and yields[0] in checked
    assert goals==[(1.,0.,0.)]*2  # The blocked compensated goal was never issued.
