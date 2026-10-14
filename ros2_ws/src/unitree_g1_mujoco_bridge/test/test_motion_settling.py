"""Measured settling must gate normal phase completion, including cleanup."""
from contextlib import contextmanager
from types import SimpleNamespace as S

import pytest
from geometry_msgs.msg import PoseStamped

from unitree_g1_mujoco_bridge import transport_demo as module
from unitree_g1_mujoco_bridge.transport_demo import TransportMission,MotionSettlingError


def harness(monkeypatch,update):
    clock = [10.]
    events = []
    mission = S(phase='navigate',pose_stamp=10.,base_velocity=(0.,0.,0.),
                seconds=lambda:clock[0],_heartbeat=lambda:None,
                event=lambda phase,**details:events.append(details))
    def spin(duration):
        clock[0] += duration
        update(mission,clock[0])
    mission.spin = spin
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    return mission,events


def test_braking_requires_multiple_progressing_measurements(monkeypatch):
    mission,events = harness(monkeypatch,lambda m,t:setattr(m,'pose_stamp',t))
    assert TransportMission.brake_motion(mission)=='settled'
    assert events[-1]['brake_outcome']=='settled'
    assert events[-1]['brake_elapsed_s']>=.2


@pytest.mark.parametrize('kind,outcome',[
    ('moving','timeout'),('frozen','stale_state'),('future','stale_state'),('nan','stale_state')])
def test_unsettled_measurements_raise_explicit_failure(monkeypatch,kind,outcome):
    def update(m,t):
        if kind!='frozen':
            m.pose_stamp = t+1. if kind=='future' else t
        m.base_velocity = (.2,0.,0.) if kind=='moving' else (float('nan'),0.,0.) if kind=='nan' else (0.,0.,0.)
    mission,events = harness(monkeypatch,update)
    with pytest.raises(MotionSettlingError) as error:
        TransportMission.brake_motion(mission,.6)
    assert error.value.outcome==outcome and events[-1]['brake_outcome']==outcome


def test_clock_regression_resets_settling_evidence(monkeypatch):
    stamps = iter([10.,10.1,10.05,10.2])
    def update(m,t):
        m.pose_stamp = next(stamps,10.2)
    mission,events = harness(monkeypatch,update)
    # Regression must not combine the earlier sample with a later frozen one.
    with pytest.raises(MotionSettlingError):
        TransportMission.brake_motion(mission,.25)
    assert events[-1]['brake_outcome']!='settled'


def test_zero_remaining_budget_cannot_confirm_settling(monkeypatch):
    mission,events = harness(monkeypatch,lambda m,t:pytest.fail('must not extend deadline'))
    with pytest.raises(MotionSettlingError,match='timeout'):
        TransportMission.brake_motion(mission,0.)


def test_measured_stop_after_old_allowance_can_settle_within_original_deadline(monkeypatch):
    def update(m,t):
        m.pose_stamp = t
        m.base_velocity = (.15,0.,0.) if t<12. else (0.,0.,0.)
    mission,events = harness(monkeypatch,update)
    assert TransportMission.brake_motion(mission,5.)=='settled'
    assert 2.2<=events[-1]['brake_elapsed_s']<5.


@pytest.mark.parametrize('operation',['navigate','rotate'])
@pytest.mark.parametrize('timeout,expected',[(.7,.7),(20.,5.)])
def test_braking_allowance_never_extends_motion_deadline(monkeypatch,operation,timeout,expected):
    from contextlib import nullcontext
    monkeypatch.setattr(module.time,'monotonic',lambda:10.)
    budgets = []
    mission = S(pose=(0.,0.,0.),goal=PoseStamped(),active=True,
        nav=S(server_is_ready=lambda:True),set_goal=lambda *args:None,
        _heartbeat=lambda:None,wait=lambda predicate,*args:predicate(),
        action=lambda *args:nullcontext(S(result=S(done=lambda:False))),
        brake_motion=budgets.append,observe=lambda *args:{'id':'box_red'},spin=lambda _:None)
    getattr(TransportMission,operation).__wrapped__(mission,(0.,0.,0.) if operation=='navigate' else 1.,
        timeout=timeout,stop_when_observed='box_red')
    assert budgets==[pytest.approx(expected)]


def test_navigation_braking_failure_terminates_action_and_revokes_intent():
    events = []
    @contextmanager
    def action(*args):
        try:
            yield S(result=S(done=lambda:False))
        finally:
            events.append('terminated')
    def brake(timeout):
        raise MotionSettlingError('timeout')
    mission = S(goal=PoseStamped(),active=True,nav=S(server_is_ready=lambda:True),
        set_goal=lambda *args:None,_heartbeat=lambda:events.append('heartbeat'),
        wait=lambda predicate,*args:predicate(),action=action,brake_motion=brake,
        observe=lambda *args:{'id':'box_red','confirmed':True},
        spin=lambda _:pytest.fail('must not advance to post-navigation settling'))
    with pytest.raises(MotionSettlingError):
        TransportMission.navigate.__wrapped__(mission,(3.,0.,0.),stop_when_observed='box_red')
    assert events==['terminated','heartbeat'] and not mission.active


def test_rotation_braking_failure_revokes_intent():
    def brake(timeout):
        raise MotionSettlingError('stale_state')
    mission = S(pose=(0.,0.,0.),active=True,set_goal=lambda *args:None,
        wait=lambda predicate,*args:predicate(),observe=lambda *args:{'id':'box_red'},
        brake_motion=brake,_heartbeat=lambda:None,
        spin=lambda _:pytest.fail('must not advance after failed brake'))
    with pytest.raises(MotionSettlingError):
        TransportMission.rotate.__wrapped__(mission,1.,stop_when_observed='box_red')
    assert not mission.active
