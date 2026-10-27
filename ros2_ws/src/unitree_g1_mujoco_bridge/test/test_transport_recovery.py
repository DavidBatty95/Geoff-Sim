from types import SimpleNamespace

import numpy as np
import pytest

from unitree_g1_mujoco_bridge.transport_actions import ActionTerminationError
from unitree_g1_mujoco_bridge.transport_demo import TransportMission,WorkspaceInterrupted,ReachUnreachable
from unitree_g1_mujoco_bridge.transport_demo import turn_command_drives_heading
from unitree_g1_mujoco_bridge.localization_recovery import ViewpointDisplaced


def mission_fixture():
    events = []
    mission = SimpleNamespace(pose=(0.,0.,0.),phase='reach',events=events,
                              wait_workspace=lambda **kwargs:None,workspace_clear=lambda *args:True)
    def event(phase,**details):
        mission.phase = phase
        events.append((phase,details))
    mission.event = event
    mission.arm_with_recovery = lambda *args,**kwargs:TransportMission.arm_with_recovery(mission,*args,**kwargs)
    return mission


def test_turn_stall_monitor_requires_fresh_unopposed_turn_intent():
    assert turn_command_drives_heading((10.,(0.,0.,.5)),10.1,1.)
    assert not turn_command_drives_heading((10.,(.3,0.,.5)),10.1,1.)
    assert not turn_command_drives_heading((10.,(0.,0.,0.)),10.1,1.)
    assert not turn_command_drives_heading((10.,(0.,0.,-.5)),10.1,1.)
    assert not turn_command_drives_heading((10.,(0.,0.,.5)),11.,1.)
    assert not turn_command_drives_heading(None,10.,1.)


def test_arm_resumes_from_measured_state_after_terminated_workspace_interruption():
    mission = mission_fixture()
    calls = []
    target = np.array((.36,-.2,.37))
    def move(point,orientation,**kwargs):
        calls.append(point.copy())
        if len(calls) == 1:
            raise WorkspaceInterrupted('person crossing')
        return point
    mission._move_arm = move
    TransportMission.move_arm(mission,target)
    assert len(calls) == 2
    assert any(details.get('resumed') for _,details in mission.events)


def test_recovery_does_not_retry_when_action_termination_is_unknown():
    mission = mission_fixture()
    def move(*args,**kwargs):
        raise ActionTerminationError('server disappeared')
    mission._move_arm = move
    with pytest.raises(ActionTerminationError):
        TransportMission.move_arm(mission,np.zeros(3))
    assert not mission.events


def test_repeated_workspace_intrusions_have_a_bounded_retry_count():
    mission = mission_fixture()
    calls = []
    def move(*args,**kwargs):
        calls.append(1)
        raise WorkspaceInterrupted('person crossing')
    mission._move_arm = move
    with pytest.raises(RuntimeError,match='recovery limit'):
        TransportMission.move_arm(mission,np.zeros(3))
    assert len(calls) == 3


@pytest.mark.parametrize('pose',[(.05,0.,0.),(0.,0.,.11)])
def test_base_displacement_requires_redocking_instead_of_replaying_old_target(pose):
    mission = mission_fixture()
    def move(*args,**kwargs):
        raise WorkspaceInterrupted('person crossing')
    mission._move_arm = move
    mission.wait_workspace = lambda:setattr(mission,'pose',pose)
    with pytest.raises(RuntimeError,match='re-docking required'):
        TransportMission.move_arm(mission,np.zeros(3))


def test_fixed_carry_posture_can_resume_after_base_drift_during_people_wait():
    mission = mission_fixture()
    calls = []
    joints = {'right_elbow_joint':-.2}
    def move(target,orientation,**kwargs):
        calls.append(kwargs['joint_solution'])
        if len(calls)==1:
            raise WorkspaceInterrupted('person entering workspace')
        return target
    mission._move_arm = move
    mission.wait_workspace = lambda:setattr(mission,'pose',(.08,0.,.12))
    TransportMission.arm_with_recovery(mission,np.zeros(3),joint_solution=joints)
    assert calls==[joints,joints]
    assert any(details.get('resumed') and details.get('base_moved') and
               details.get('target_kind')=='joint_posture' for _,details in mission.events)


def test_nav2_success_with_stale_pose_requires_reobservation():
    from contextlib import nullcontext
    from geometry_msgs.msg import PoseStamped
    from unitree_g1_mujoco_bridge.localization_recovery import LocalizationInterrupted
    mission = SimpleNamespace(pose=(0.,0.,0.),pose_stamp=1.,seconds=lambda:10.,
        goal=PoseStamped(),active=True,nav=SimpleNamespace(server_is_ready=lambda:True),
        set_goal=lambda *args:None,_heartbeat=lambda:None,wait=lambda predicate,*args:predicate())
    result = SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(status=4))
    mission.action = lambda *args:nullcontext(SimpleNamespace(result=result))
    with pytest.raises(LocalizationInterrupted,match='fresh sensed staging'):
        TransportMission.navigate.__wrapped__(mission,(0.,0.,0.))
    assert not mission.active


def test_retreat_compensates_forward_coast_without_relaxing_settled_clearance():
    mission = mission_fixture()
    mission.pose_stamp = 12.
    mission.seconds = lambda:12.
    goals = []
    def goal(pose,mode):
        goals.append(pose)
        mission.pose,mission.active = pose,True
    mission.set_goal = goal
    mission.stop_intent = lambda:setattr(mission,'active',False)
    mission.wait = lambda predicate,*args:predicate()
    mission.spin = lambda duration:setattr(mission,'pose',(mission.pose[0]+.09,0.,0.))
    TransportMission.retreat(mission,.5,0.)
    assert len(goals)==2 and goals[1][0] == pytest.approx(-.6)
    assert -mission.pose[0] >= .43


def test_search_detection_terminates_navigation_before_returning():
    from contextlib import contextmanager
    from geometry_msgs.msg import PoseStamped
    events = []
    @contextmanager
    def action(*args):
        events.append('dispatched')
        try:
            yield SimpleNamespace(result=SimpleNamespace(done=lambda:False))
        finally:
            events.append('terminated')
    mission = SimpleNamespace(goal=PoseStamped(),active=True,nav=SimpleNamespace(server_is_ready=lambda:True),
        set_goal=lambda *args:None,_heartbeat=lambda:None,wait=lambda predicate,*args:predicate(),action=action,
        brake_motion=lambda timeout:events.append('braking'),
        observe=lambda *args:{'id':'box_red','confirmed':True},spin=lambda duration:events.append(('settle',duration)))
    observed = TransportMission.navigate.__wrapped__(mission,(3.,0.,0.),stop_when_observed='box_red',settle_time=.1)
    assert observed['id']=='box_red' and observed['confirmed']
    assert events==['dispatched','braking','terminated',('settle',.1)]
    assert not mission.active


def test_inspection_turn_waits_for_fresh_measured_settling():
    mission = mission_fixture()
    mission.pose_stamp = 10.
    mission.seconds = lambda:mission.pose_stamp
    mission.base_velocity = (.4,0.,0.)
    mission.stop_intent = lambda:setattr(mission,'active',False)
    mission._heartbeat = lambda:None
    mission.spin = lambda _:None
    mission.brake_motion = lambda timeout:None
    goals = []
    mission.set_goal = lambda *args:goals.append(args)
    def wait(predicate,*args):
        if not goals:
            assert not predicate()
            mission.base_velocity = (0.,0.,0.)
            assert not predicate()  # One still sample is insufficient.
            mission.pose_stamp = 10.25
            assert predicate()
        else:
            mission.pose = (0.,0.,1.)
            assert predicate()
    mission.wait = wait
    TransportMission.rotate(mission,1.,settle_before_turn=True)
    assert len(goals)==1 and goals[0][1]=='scan'


def test_unsettled_inspection_does_not_issue_a_rotation_goal():
    mission = mission_fixture()
    mission.pose_stamp = 10.
    mission.seconds = lambda:10.
    mission.base_velocity = (.3,0.,0.)
    mission.stop_intent = lambda:setattr(mission,'active',False)
    mission.set_goal = lambda *args:pytest.fail('must not turn while the base is moving')
    def wait(predicate,timeout,message):
        assert not predicate() and timeout<=2.
        raise RuntimeError(message)
    mission.wait = wait
    with pytest.raises(RuntimeError,match='base did not settle'):
        TransportMission.rotate(mission,1.,settle_before_turn=True)
    assert not mission.active


def test_turn_discovery_survives_occlusion_during_braking():
    seen = [{'id':'target_blue','observed_stamp':10.,'confirmed':True}]
    mission = mission_fixture()
    mission.active = True
    mission.pose_stamp = 10.
    mission.seconds = lambda:10.
    mission.set_goal = lambda *args:None
    mission.observe = lambda *args:seen[0] if seen else None
    mission.wait = lambda predicate,*args:predicate()
    mission.brake_motion = lambda *args:seen.clear()
    mission._heartbeat = lambda:None
    mission.spin = lambda duration:None
    observed = TransportMission.rotate.__wrapped__(mission,1.,stop_when_observed='target_blue')
    assert observed['id']=='target_blue' and not seen and not mission.active


def test_displaced_turn_brakes_before_revoking_intent():
    mission = mission_fixture()
    events = []
    mission.pose = (4.35,1.67,-1.17)
    mission.pose_stamp = 10.
    mission.seconds = lambda:10.
    mission.active = True
    mission.set_goal = lambda *args:None
    mission.event = lambda name,**kw:events.append(name)
    mission.brake_motion = lambda *args:events.append(('brake',mission.active))
    mission._heartbeat = lambda:events.append(('active',mission.active))
    def wait(predicate,*args):
        mission.pose = (4.35,1.87,-1.2)
        assert not predicate()
        mission.pose = (4.35,2.04,-1.3)
        assert predicate()  # Braking occurs before any further scan request.
    mission.wait = wait
    with pytest.raises(ViewpointDisplaced,match='turn displaced'):
        TransportMission.rotate.__wrapped__(mission,-1.66)
    assert events == ['turn_displaced',('brake',True),'turn_displaced',('active',False)]


def test_stale_displaced_pose_does_not_trigger_turn_drift_decision():
    mission = mission_fixture()
    mission.pose = (0.,0.,0.)
    mission.pose_stamp = 9.
    mission.seconds = lambda:10.
    mission.set_goal = lambda *args:None
    mission.brake_motion = lambda *args:None
    mission._heartbeat = lambda:None
    mission.spin = lambda *args:None
    def wait(predicate,*args):
        mission.pose = (1.,0.,0.)
        assert not predicate()
        mission.pose_stamp = 10.
        mission.pose = (0.,0.,1.)
        assert predicate()
    mission.wait = wait
    TransportMission.rotate.__wrapped__(mission,1.)


@pytest.mark.parametrize('dex3',[False,True])
def test_placement_reobserves_from_retreat_viewpoint_after_release_ack(dex3):
    mission = mission_fixture()
    mission.dex3,mission.joints = dex3,{}
    mission.kin = SimpleNamespace(forward=lambda joints:(np.array([.4,-.15,.2]),))
    mission.to_pelvis = lambda position:position
    calls = []
    def withdraw(target,orientation):
        assert calls==['release_ack']
        np.testing.assert_allclose(target,[.3,-.15,.2])
        calls.append('withdraw')
    mission.move_arm = withdraw
    mission.release,mission.grasp_R = object(),np.eye(3)
    mission.seconds = lambda:10.
    mission.service = lambda *args,**kw:calls.append('release_ack')
    mission.move_to_carry = lambda *args:calls.append('retract')
    mission.retreat = lambda *args:calls.append('retreat')
    mission.spin = lambda *args:None
    def observed(identifier,pad,stamp):
        assert calls == ['release_ack']+(['withdraw'] if dex3 else [])+['retract','retreat']
        assert stamp == 10.
        calls.append('post_release_observation')
        return True
    mission.placement_observed = observed
    mission.wait = lambda predicate,*args:predicate()
    TransportMission.release_and_verify(mission,'box','pad',np.zeros(3),(1.,0.),0.)
    assert calls[-1] == 'post_release_observation'


def test_uncertain_release_does_not_initiate_retreat_or_verification():
    mission = mission_fixture()
    mission.release = object()
    def service(*args,**kwargs):
        raise RuntimeError('release acknowledgement lost')
    mission.service = service
    with pytest.raises(RuntimeError,match='acknowledgement lost'):
        TransportMission.release_and_verify(mission,'box','pad',np.zeros(3),(1.,0.),0.)
    assert [phase for phase,_ in mission.events] == ['release']


@pytest.mark.parametrize('dex3',[False,True])
def test_unreachable_pickup_redocks_and_reprojects_before_grasp(dex3):
    mission = mission_fixture()
    mission.dex3 = dex3
    box = np.array((1.5,0.,1.))
    mission.joints = {}
    mission.observe = lambda *args:dict(position=box)
    mission.wait = lambda predicate,*args:predicate()
    mission.to_pelvis = lambda point:point-np.array((mission.pose[0],mission.pose[1],.8))
    hand = [np.array((.3,0.,.2))]
    calls = []
    def move(target,orientation):
        calls.append(target.copy())
        if target[0] > .5:
            raise ReachUnreachable('base drifted from table')
        hand[0] = target
        return target
    mission.move_arm = move
    mission.arm_with_recovery = move
    carried = []
    mission.move_to_carry = lambda:carried.append(True)
    restored = []
    mission.restore_arm = lambda:restored.append(True)
    def dock(pose):
        if dex3:
            assert carried and restored and hand[0][0] < .2
        mission.pose = pose
    mission.dock = dock
    mission.dock_for_pickup = lambda *args:dock((box[0]-.43,.15,0.))
    mission.kin = SimpleNamespace(forward=lambda joints:(hand[0],None,None,None))
    result = TransportMission.reach_box(mission,'box',np.array((1.,0.)),np.array((0.,1.)),0.)
    np.testing.assert_array_equal(result,box)
    assert len(calls) == (4 if dex3 else 2) and calls[1][0] < .5
    assert any(phase == 'redock_pickup' for phase,_ in mission.events)


def test_docking_recloses_after_coasting_outside_tolerance():
    mission = mission_fixture()
    mission.pose_stamp = 12.
    mission.seconds = lambda:12.
    goals = []
    def goal(pose,mode):
        goals.append(pose)
        mission.pose,mission.active = pose,True
    mission.set_goal = goal
    mission.stop_intent = lambda:setattr(mission,'active',False)
    mission.wait = lambda predicate,*args:predicate()
    def settle(duration):
        if len(goals) == 1:
            mission.pose = (.2,0.,0.)
    mission.spin = settle
    TransportMission.dock(mission,(0.,0.,0.))
    assert len(goals) == 2
    assert not mission.active
    assert mission.events[-1][1]['dock_settled']


@pytest.mark.parametrize('blocked',['moving','far','stale','heading','missing_object'])
def test_pickup_does_not_raise_arm_before_settled_observed_arrival(blocked):
    mission = pickup_approach_fixture()
    def wait(predicate,timeout,message):
        if 'before pickup docking' in message:
            assert predicate()
            return
        if blocked=='moving':
            mission.base_velocity = (.2,0.,0.)
        elif blocked=='far':
            mission.pose = (-.5,.15,0.)
        elif blocked=='stale':
            mission.pose_stamp = 9.
        elif blocked=='heading':
            mission.pose = (0.,.15,.3)
        else:
            mission.observe = lambda *args:None
        assert not predicate()
        raise RuntimeError(message)
    mission.wait = wait
    with pytest.raises(RuntimeError,match='not settled'):
        TransportMission.dock_for_pickup(mission,'box',np.array((1.,0.)),np.array((0.,1.)),0.)
    assert mission.calls==['dock','stop']


def pickup_approach_fixture():
    mission = mission_fixture()
    mission.args = SimpleNamespace(pickup_standoff=.43)
    mission.dex3 = True
    mission.pose,mission.pose_stamp = (0.,.15,0.),10.5
    mission.seconds = lambda:10.5
    mission.base_velocity = (0.,0.,0.)
    mission.observe = lambda *args:dict(position=(.43,0.,1.))
    mission.rest_arm_positions = None
    mission.joints = {'elbow':.1}
    mission.kin = SimpleNamespace(names=['elbow'])
    mission.authority,mission.open_hand = 'authority','open'
    mission.calls = []
    mission.dock = lambda *args:mission.calls.append('dock')
    mission.stop_intent = lambda:mission.calls.append('stop')
    mission.service = lambda service,*args:mission.calls.append(service)
    mission.move_to_carry = lambda *args:mission.calls.append('raise')
    return mission


def test_pickup_waits_for_advancing_settled_samples_before_arm_authority():
    mission = pickup_approach_fixture()
    def wait(predicate,timeout,message):
        if 'before pickup docking' in message:
            assert predicate()
            return
        mission.pose_stamp = 10.3
        assert not predicate()
        assert not predicate()  # Repeated receipt cannot satisfy the dwell.
        mission.pose_stamp = 10.7
        mission.seconds = lambda:10.7
        assert predicate()
        assert mission.calls==['dock','stop']
    mission.wait = wait
    TransportMission.dock_for_pickup(mission,'box',np.array((1.,0.)),np.array((0.,1.)),0.)
    assert mission.calls==['dock','stop','authority','open','raise']
    assert mission.rest_arm_positions=={'elbow':.1}


def test_docking_learns_bounded_coast_offset_without_changing_settled_goal():
    mission = mission_fixture()
    mission.pose_stamp = 12.
    mission.seconds = lambda:12.
    goals = []
    def goal(pose,mode):
        goals.append(pose)
        mission.pose = pose
    mission.set_goal = goal
    mission.stop_intent = lambda:None
    mission.wait = lambda predicate,*args:predicate()
    mission.spin = lambda _:setattr(mission,'pose',(mission.pose[0]+.08,mission.pose[1],mission.pose[2]))
    TransportMission.dock(mission,(0.,0.,0.))
    assert len(goals)==2 and -.040001<=goals[-1][0]<0.
    assert abs(mission.pose[0])<.07


def test_docking_retries_share_original_deadline(monkeypatch):
    import unitree_g1_mujoco_bridge.transport_demo as module
    now = [0.]
    monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    mission = mission_fixture()
    mission.pose_stamp = 12.
    mission.seconds = lambda:12.
    mission.set_goal = lambda pose,mode:setattr(mission,'pose',pose)
    mission.stop_intent = lambda:setattr(mission,'active',False)
    mission.wait = lambda predicate,*args:predicate()
    def drift(duration):
        now[0] += duration
        mission.pose = (.2,0.,0.)
    mission.spin = drift
    with pytest.raises(RuntimeError,match='settle before deadline'):
        TransportMission.dock(mission,(0.,0.,0.),timeout=2.)
    assert now[0] == 2.
    assert not mission.active


def test_dock_approach_leaves_settling_margin_without_relaxing_final_tolerance():
    mission = mission_fixture()
    mission.pose_stamp = 12.
    mission.seconds = lambda:12.
    mission.set_goal = lambda *args:setattr(mission,'pose',(.06,0.,0.))
    mission.stop_intent = lambda:setattr(mission,'active',False)
    def wait(predicate,*args):
        assert not predicate()  # Moving 6 cm arrival must not terminate early.
        mission.pose = (.03,0.,0.)
        assert predicate()
    mission.wait = wait
    mission.spin = lambda _:setattr(mission,'pose',(.06,0.,0.))
    TransportMission.dock(mission,(0.,0.,0.))
    assert mission.events[-1][1]['dock_settled']


def test_stale_pose_cannot_confirm_docking_and_failure_clears_intent():
    mission = mission_fixture()
    mission.pose_stamp = 10.
    mission.seconds = lambda:12.
    mission.set_goal = lambda *args:setattr(mission,'active',True)
    mission.stop_intent = lambda:setattr(mission,'active',False)
    def wait(predicate,*args):
        assert not predicate()
        raise RuntimeError('stale pose')
    mission.wait = wait
    with pytest.raises(RuntimeError,match='stale pose'):
        TransportMission.dock(mission,(0.,0.,0.))
    assert not mission.active


def test_retreat_accepts_safe_coast_past_clearance_without_forward_reversal():
    mission = mission_fixture()
    mission.pose_stamp = 12.
    mission.seconds = lambda:12.
    goals = []
    def goal(pose,mode):
        goals.append(pose)
        mission.pose,mission.active = pose,True
    mission.set_goal = goal
    mission.stop_intent = lambda:setattr(mission,'active',False)
    mission.wait = lambda predicate,*args:predicate()
    mission.spin = lambda duration:setattr(mission,'pose',(-.7,.05,0.))
    TransportMission.retreat(mission,.5,0.)
    assert len(goals) == 1
    assert mission.events[-1][1]['distance_m'] == pytest.approx(.7)
    assert not mission.active
