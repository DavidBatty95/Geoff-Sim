"""Real-URDF regressions for fixed postures while the policy balances the waist."""
from pathlib import Path
from types import SimpleNamespace as S

import numpy as np
import pytest

from unitree_g1_mujoco_bridge.arm_kinematics import ArmKinematics
from unitree_g1_mujoco_bridge.payload_postures import PAYLOAD_V1_POSTURES
from unitree_g1_mujoco_bridge.transport_demo import TransportMission,WorkspaceInterrupted


class Mission:
    measured_posture_state = TransportMission.measured_posture_state
    move_arm_posture = TransportMission.move_arm_posture
    move_to_carry = TransportMission.move_to_carry
    restore_arm = TransportMission.restore_arm
    arm_with_recovery = TransportMission.arm_with_recovery

    def __init__(self):
        urdf = Path(__file__).resolve().parents[2]/'unitree_g1_description/urdf/g1_29dof_mode_15_brainco_hand.urdf'
        self.kin = ArmKinematics(urdf.read_text(),tool_offset=(.12,.01,0.))
        self.joints = {name:0. for name,kind,*_ in self.kin.chain if kind in ('revolute','continuous')}
        self.rest_arm_positions = {name:0. for name in self.kin.names}
        self.pose,self.phase = (0.,0.,0.),'carry'
        self.args = S(carry_posture='payload_v1')
        self.events,self.releases = [],[]
        self.authority = object()

    def event(self,phase,**details):
        self.phase = phase
        self.events.append((phase,details))

    def service(self,client,request):
        assert client is self.authority and not request.data
        self.releases.append('release')


@pytest.mark.parametrize('posture',['carry','rest'])
@pytest.mark.parametrize('waist',['waist_yaw_joint','waist_roll_joint','waist_pitch_joint'])
def test_fixed_posture_accepts_current_waist_without_relaxing_error(posture,waist):
    mission = Mission()
    old_reference = []
    def move(target,orientation,*,joint_solution):
        assert set(joint_solution)==set(mission.kin.names)
        old_reference.append(target.copy())
        mission.joints.update(joint_solution)
        mission.joints[waist] = .5
        return mission.kin.forward(mission.joints)[0]
    mission._move_arm = move
    if posture=='carry':
        mission.move_to_carry()
        assert not mission.releases
    else:
        mission.restore_arm()
        assert mission.releases==['release']
    measured = mission.kin.forward(mission.joints)[0]
    assert np.linalg.norm(measured-old_reference[0])>.035
    details = mission.events[-1][1]
    assert details[posture+'_posture_error_m']==pytest.approx(0.)
    assert details['reference']=='current_waist_fixed_arm_posture'


def test_restoration_error_keeps_authority_and_cannot_complete():
    mission = Mission()
    def move(target,orientation,*,joint_solution):
        mission.joints.update(joint_solution)
        mission.joints['waist_yaw_joint'] = .5
        mission.joints['right_shoulder_pitch_joint'] += .4
        return target  # A claimed result cannot replace measured geometry.
    mission._move_arm = move
    with pytest.raises(RuntimeError,match='rest posture outside tolerance'):
        mission.restore_arm()
    assert not mission.releases
    assert mission.events[-1][1]['rest_posture_error_m']>.035
    assert all(phase!='complete' for phase,_ in mission.events)


@pytest.mark.parametrize('when',['before','after'])
@pytest.mark.parametrize('invalid',['missing','nan'])
def test_incomplete_measurements_cannot_satisfy_restoration(when,invalid):
    mission = Mission()
    def corrupt():
        if invalid=='missing':
            del mission.joints['waist_yaw_joint']
        else:
            mission.joints['right_elbow_joint'] = float('nan')
    def move(*args,**kwargs):
        assert when=='after', 'invalid initial state must not dispatch'
        corrupt()
    mission._move_arm = move
    if when=='before':
        corrupt()
    with pytest.raises(RuntimeError,match='finite measured arm and waist'):
        mission.restore_arm()
    assert not mission.releases


def test_interrupted_restoration_uses_new_waist_and_releases_only_after_validation():
    mission = Mission()
    calls = []
    def move(target,orientation,*,joint_solution):
        calls.append(dict(joint_solution))
        assert not mission.releases
        if len(calls)==1:
            raise WorkspaceInterrupted('person crossing')
        mission.joints.update(joint_solution)
        return mission.kin.forward(mission.joints)[0]
    def wait():
        mission.pose = (.1,0.,.2)
        mission.joints['waist_yaw_joint'] = .5
    mission._move_arm,mission.wait_workspace = move,wait
    mission.restore_arm()
    assert calls==[mission.rest_arm_positions]*2
    assert mission.releases==['release']
    assert any(details.get('resumed') and details.get('base_moved') for _,details in mission.events)


@pytest.mark.parametrize('invalid',['missing','waist_target','nan'])
def test_bad_posture_cannot_dispatch(invalid):
    mission = Mission()
    posture = dict(zip(mission.kin.names,PAYLOAD_V1_POSTURES[0]))
    if invalid=='missing':
        posture.pop(mission.kin.names[0])
    elif invalid=='waist_target':
        posture['waist_yaw_joint'] = .1
    else:
        posture[mission.kin.names[0]] = float('nan')
    mission._move_arm = lambda *a,**kw:pytest.fail('invalid target must not dispatch')
    with pytest.raises(RuntimeError,match='complete finite arm-only'):
        mission.move_arm_posture(posture,'rest')


def test_cartesian_carry_retains_its_world_reach_contract():
    mission = Mission()
    mission.args.carry_posture = 'ik'
    targets = []
    mission.move_arm = lambda target,orientation:targets.append((target,orientation))
    orientation = np.eye(3)
    mission.move_to_carry(orientation)
    np.testing.assert_allclose(targets[0][0],(.36,-.20,.37))
    assert targets[0][1] is orientation
