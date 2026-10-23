"""World-relative payload alignment must survive free-base motion before release."""
from types import SimpleNamespace as S

import numpy as np
import pytest

from unitree_g1_mujoco_bridge.arm_kinematics import rpy_matrix
from unitree_g1_mujoco_bridge.transport_demo import TransportMission
from unitree_g1_mujoco_bridge.transport_actions import ActionTerminationError
import unitree_g1_mujoco_bridge.transport_demo as module


def fixture():
    pad = np.array((.5,-.15,.95))
    events,calls = [],[]
    node = S(pose=(0.,0.,0.),pose_stamp=10.,seconds=lambda:10.,rpy=(0.,0.,0.),
        joints={'floating_base_joint':.8},offset=np.array((.05,.01,.02)),
        grasp_R=rpy_matrix((0.,0.,.4)),hand=np.zeros(3),
        observe=lambda *args:dict(position=pad.copy(),observed_stamp=9.9,confirmed=True),
        event=lambda phase,**details:events.append(details))
    node.to_pelvis = lambda point:TransportMission.to_pelvis(node,point)
    node.measured_posture_state = lambda:node.joints.copy()
    node.kin = S(forward=lambda joints:(node.hand.copy(),node.grasp_R,None,None))
    def wait(predicate,timeout,message):
        if timeout<=0. or not predicate():
            raise RuntimeError(message)
    node.wait = wait
    def move(target,orientation):
        calls.append(target.copy())
        node.hand = target.copy()
    node.move_arm = move
    return node,pad,events,calls


@pytest.mark.parametrize('pose',[(.12,-.04,0.),(0.,0.,.55),(.1,.03,.4)])
def test_lowering_reprojects_after_base_motion_instead_of_accepting_pelvis_accuracy(pose):
    node,pad,events,calls = fixture()
    move = node.move_arm
    def drift(target,orientation):
        move(target,orientation)
        node.pose = pose
    node.move_arm = drift
    observed = TransportMission.lower_to_pad(node,'target_blue')
    assert len(calls)==2
    assert events[0]['payload_alignment_error_m']>.035
    assert events[-1]['payload_alignment_error_m']<1e-9
    assert not np.allclose(calls[0],calls[1])
    np.testing.assert_allclose(observed,pad)


def test_correct_payload_offset_and_orientation_need_no_extra_reach():
    node,pad,events,calls = fixture()
    node.pose = (.1,.1,.5)
    node.rpy = (.05,-.04,0.)
    TransportMission.lower_to_pad(node,'target_blue')
    assert len(calls)==1 and events[0]['payload_alignment_error_m']<1e-9


def test_dex3_lowering_uses_cylinder_half_height_and_small_release_gap():
    node,pad,events,calls = fixture()
    node.dex3 = True
    TransportMission.lower_to_pad(node,'target_blue')
    estimated = node.hand+node.grasp_R@node.offset
    np.testing.assert_allclose(estimated,node.to_pelvis(pad+np.array([0.,0.,.05])))


@pytest.mark.parametrize('recovery',['none','fresh','cached'])
def test_dex3_revoked_pad_requires_new_camera_confirmation_after_withdrawal(recovery):
    node,pad,events,calls = fixture()
    node.dex3 = True
    move = node.move_arm
    withdrawals = []
    def occlude(target,orientation):
        move(target,orientation)
        if len(calls)==1:
            node.observe = lambda *args:None
    def withdraw(orientation):
        withdrawals.append(True)
        if recovery!='none':
            node.seconds = lambda:11.
            node.pose_stamp = 11.
            node.observe = lambda *args:dict(position=pad.copy(),observed_stamp=10.5 if recovery=='fresh' else 9.9,confirmed=True)
    node.move_arm,node.move_to_carry = occlude,withdraw
    if recovery=='fresh':
        np.testing.assert_allclose(TransportMission.lower_to_pad(node,'target_red'),pad)
        assert len(calls)==2
    else:
        with pytest.raises(RuntimeError,match='not reobserved'):
            TransportMission.lower_to_pad(node,'target_red')
        assert len(calls)==1
    assert withdrawals==[True]


def test_persistent_payload_misalignment_has_bounded_retries():
    node,pad,events,calls = fixture()
    move = node.move_arm
    def miss(target,orientation):
        move(target,orientation)
        node.hand += (.06,0.,0.)
    node.move_arm = miss
    with pytest.raises(RuntimeError,match='payload not aligned'):
        TransportMission.lower_to_pad(node,'target_blue')
    assert len(calls)==3 and all(row['payload_alignment_error_m']>.035 for row in events)


@pytest.mark.parametrize('failure',['pose_stale','revoked_pad'])
def test_lost_alignment_evidence_cannot_authorize_release(failure):
    node,pad,events,calls = fixture()
    move = node.move_arm
    def unavailable(target,orientation):
        move(target,orientation)
        if failure=='pose_stale':
            node.pose_stamp = 9.
        else:
            node.observe = lambda *args:None
    node.move_arm = unavailable
    with pytest.raises(RuntimeError,match='after lowering'):
        TransportMission.lower_to_pad(node,'target_blue')
    assert len(calls)==1 and not events


def test_alignment_deadline_does_not_accept_a_late_arm_result(monkeypatch):
    node,pad,events,calls = fixture()
    now = [0.]
    monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    move = node.move_arm
    def slow(target,orientation):
        move(target,orientation)
        now[0] = 31.
    node.move_arm = slow
    with pytest.raises(RuntimeError,match='alignment deadline'):
        TransportMission.lower_to_pad(node,'target_blue')
    assert len(calls)==1


def test_unknown_action_termination_is_not_retried():
    node,pad,events,calls = fixture()
    def fail(*args):
        calls.append('dispatch')
        raise ActionTerminationError('unknown termination')
    node.move_arm = fail
    with pytest.raises(ActionTerminationError):
        TransportMission.lower_to_pad(node,'target_blue')
    assert calls==['dispatch']
