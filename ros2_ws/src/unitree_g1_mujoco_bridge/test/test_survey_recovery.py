from types import SimpleNamespace

import pytest

from unitree_g1_mujoco_bridge.transport_demo import TransportMission, ViewpointDisplaced,MotionSettlingError,SurveyYieldRequired
import unitree_g1_mujoco_bridge.transport_demo as module


def test_displaced_survey_yields_and_reprojects_without_extending_deadline(monkeypatch):
    now = [0.]
    monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    calls = []
    mission = SimpleNamespace(pose=(1.,0.,0.),search_hints={'pad':dict(position=(1.,2.,1.))},
                              event=lambda *args,**kw:None,stop_intent=lambda:None)
    def rotate(yaw,timeout,**kwargs):
        assert kwargs['settle_before_turn'] is True
        calls.append((yaw,timeout))
        if len(calls)==1:
            now[0]=5.
            raise ViewpointDisplaced('crowd avoidance')
    def yielding(deadline):
        assert deadline==45.
        now[0]=10.
    mission.rotate,mission.yield_survey = rotate,yielding
    TransportMission.survey_rotate(mission,0.,'pad')
    assert calls==[(0.,45.),(pytest.approx(1.570796),35.)]


def test_survey_recovery_is_bounded_and_does_not_retry_other_failures():
    calls = []
    mission = SimpleNamespace(search_hints={},event=lambda *args,**kw:None,
                              yield_survey=lambda deadline:calls.append('yield'),stop_intent=lambda:None)
    def displaced(*args,**kwargs):
        calls.append('turn')
        raise ViewpointDisplaced('drift')
    mission.rotate = displaced
    with pytest.raises(ViewpointDisplaced):
        TransportMission.survey_rotate(mission,0.,'box')
    assert calls==['turn','yield','turn','yield','turn']
    def failed(*args,**kwargs):
        raise RuntimeError('braking failed')
    mission.rotate = failed
    with pytest.raises(RuntimeError,match='braking failed'):
        TransportMission.survey_rotate(mission,0.,'box')
    assert len(calls)==5


@pytest.mark.parametrize('preserve',[False,True])
def test_yield_keeps_avoidance_active_until_clear_and_measured_settled(preserve):
    mission = SimpleNamespace(pose=(0.,0.,0.),pose_stamp=10.,seconds=lambda:10.,
        base_velocity=(.3,0.,0.),event=lambda *args,**kw:None,clear=False,
        policy_feedback=(10.,(.3,0.,0.)),rpy=(0.,0.,0.),rpy_stamp=10.)
    mission.workspace_clear = lambda:mission.clear
    mission.set_goal = lambda *args:setattr(mission,'active',True)
    if preserve:
        mission.set_goal=lambda *args:pytest.fail('handoff must not replace the existing goal')
    mission._heartbeat=lambda:None
    mission.stop_intent = lambda:setattr(mission,'active',False)
    def wait(predicate,*args):
        assert mission.active and not predicate()
        mission.base_velocity=(0.,0.,0.)
        assert not predicate()  # A stationary pose in an incoming corridor is not safe.
        mission.clear=True
        assert not predicate()  # Stillness against an obstacle is not a stopped command.
        mission.policy_feedback=(10.,(0.,0.,0.))
        assert not predicate()
        assert not predicate()  # Repeated old samples cannot prove dwell.
        mission.pose_stamp=10.4
        mission.seconds=lambda:10.4
        mission.policy_feedback=(10.4,(0.,0.,0.))
        mission.rpy_stamp=10.4
        assert predicate()
    mission.wait=wait
    stamp=TransportMission.yield_survey(mission,module.time.monotonic()+15.,preserve_intent=preserve)
    assert stamp==10.4
    assert not mission.active


@pytest.mark.parametrize('final_pose',[(.4,0.,1.),(0.,0.,.7)])
def test_scan_rechecks_position_and_heading_after_checked_braking(final_pose):
    mission = SimpleNamespace(pose=(0.,0.,0.),pose_stamp=10.,seconds=lambda:10.,
        active=False,events=[],observe=lambda *args:None,spin=lambda *args:None)
    mission.set_goal=lambda *args:setattr(mission,'active',True)
    mission._heartbeat=lambda:None
    mission.event=lambda name,**kw:mission.events.append((name,kw))
    def wait(predicate,*args):
        mission.pose=(0.,0.,1.)
        assert predicate()
    mission.wait=wait
    mission.brake_motion=lambda *args:setattr(mission,'pose',final_pose)
    with pytest.raises(ViewpointDisplaced):
        TransportMission.rotate.__wrapped__(mission,1.)
    assert not mission.active
    assert mission.events[-1][1]['after_braking']


@pytest.mark.parametrize('checked',[False,True])
@pytest.mark.parametrize('outcome',['timeout','stale_state'])
def test_only_checked_avoidance_timeout_can_handoff_without_revoking_intent(checked,outcome):
    mission=SimpleNamespace(pose=(0.,0.,0.),pose_stamp=10.,seconds=lambda:10.,active=True,
        observe=lambda *args:None,event=lambda *args,**kw:None,_heartbeat=lambda:None,
        set_goal=lambda *args:None,checked_avoidance_active=lambda:checked)
    def wait(predicate,*args):
        mission.pose=(0.,0.,1.)
        assert predicate()
    mission.wait=wait
    def brake(*args):
        raise MotionSettlingError(outcome)
    mission.brake_motion=brake
    handoff=checked and outcome=='timeout'
    with pytest.raises(SurveyYieldRequired if handoff else MotionSettlingError):
        TransportMission.rotate.__wrapped__(mission,1.,allow_avoidance_yield=True)
    assert mission.active==handoff


@pytest.mark.parametrize('change',[
    {'avoidance_state':(99.,'CLEARANCE_RECOVERY')},
    {'avoidance_state':(100.,'NORMAL_NAVIGATION')},
    {'policy_feedback':(9.,(.2,0.,0.))},
    {'policy_feedback':(10.,(0.,0.,0.))},
    {'pose_stamp':9.}, {'rpy':(.4,0.,0.)},
])
def test_survey_handoff_requires_fresh_checked_translation_and_stable_body(monkeypatch,change):
    monkeypatch.setattr(module.time,'monotonic',lambda:100.)
    mission=SimpleNamespace(pose=(0.,0.,0.),pose_stamp=10.,seconds=lambda:10.,rpy=(0.,0.,0.),rpy_stamp=10.,
        avoidance_state=(100.,'CLEARANCE_RECOVERY'),policy_feedback=(10.,(.2,0.,0.)))
    assert TransportMission.checked_avoidance_active(mission)
    for key,value in change.items():
        setattr(mission,key,value)
    assert not TransportMission.checked_avoidance_active(mission)


def test_survey_owner_cleans_up_handoff_and_requires_post_yield_observation():
    calls=[]
    mission=SimpleNamespace(search_hints={},event=lambda *args,**kw:None,active=True)
    def rotate(yaw,timeout,**kwargs):
        calls.append('turn')
        if len(calls)==1:
            raise SurveyYieldRequired('avoidance translating')
        assert kwargs['stop_when_observed']=='pad'
        assert kwargs['observed_after']==10.5
    def yielding(deadline,preserve_intent=False):
        assert preserve_intent and mission.active
        return 10.5
    mission.rotate,mission.yield_survey=rotate,yielding
    mission.stop_intent=lambda:setattr(mission,'active',False)
    TransportMission.survey_rotate(mission,0.,'pad')
    assert not mission.active and len(calls)==2


@pytest.mark.parametrize('fresh',[False,True])
def test_discovery_cannot_reuse_pre_yield_confirmation(fresh):
    calls=[]
    observation={'id':'pad','observed_stamp':9.,'position':[1.,0.,1.]}
    mission=SimpleNamespace(pose=(0.,0.,0.),search_hints={},survey_observation_after={'pad':10.},
        observe=lambda *args:observation,event=lambda *args,**kw:None,spin=lambda *args:None)
    def turn(*args):
        calls.append(1)
        if fresh:
            observation['observed_stamp']=10.5
    mission.survey_rotate=turn
    if fresh:
        assert TransportMission.discover(mission,'pad')['observed_stamp']==10.5
        assert len(calls)==1
    else:
        with pytest.raises(RuntimeError,match='did not discover'):
            TransportMission.discover(mission,'pad')
        assert len(calls)==9
