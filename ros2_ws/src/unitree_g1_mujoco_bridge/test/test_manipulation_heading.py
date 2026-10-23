import math
from types import SimpleNamespace as S

import pytest

from unitree_g1_mujoco_bridge.manipulation_heading import heading_hold_reference
from unitree_g1_mujoco_bridge.human_aware_motion_planner import HumanAwareMotionPlanner
from unitree_g1_mujoco_bridge.transport_demo import TransportMission


def test_heading_reference_wraps_damps_and_respects_limits():
    assert heading_hold_reference(.05,0.)==pytest.approx(.2)
    assert heading_hold_reference(2*math.pi+.05,0.)==pytest.approx(.2)
    assert heading_hold_reference(0.,.2)<0.
    assert heading_hold_reference(1.,0.,.25)==.25
    assert heading_hold_reference(-1.,0.)==-.6
    assert heading_hold_reference(float('nan'),0.)==0.


def test_manipulation_reference_does_not_chase_translation_or_use_scan_deadband():
    goal = S(header=S(frame_id='odom'),pose=S(position=S(x=4.,y=3.),
        orientation=S(w=1.,x=0.,y=0.,z=0.)))
    planner = S(reference_goal=goal,target_frame='odom',navigation_mode='manipulate',
        measured_velocity=(0.,0.,0.),max_angular_near_human=.6,_wrap_pi=lambda v:math.atan2(math.sin(v),math.cos(v)))
    cmd = HumanAwareMotionPlanner._dock_reference(planner,(0.,0.,.05))
    assert cmd.linear.x==cmd.linear.y==0.
    assert cmd.angular.z==pytest.approx(-.2)


@pytest.mark.parametrize('fail',[False,True])
def test_arm_heading_intent_is_revoked_on_completion_and_failure(fail):
    node = S(args=S(manipulation_heading_hold=True),pose=(0.,0.,.2),goal=None,
             active=False,wait_workspace=lambda:None)
    modes = []
    def set_goal(pose,mode):
        assert pose[2]==.2
        modes.append(mode)
        node.active = True
    def move(*args,**kwargs):
        assert node.active
        if fail:
            raise RuntimeError('arm failure')
        return [1.,2.,3.]
    node.set_goal,node._move_arm = set_goal,move
    node.stop_intent = lambda:setattr(node,'active',False)
    if fail:
        with pytest.raises(RuntimeError,match='arm failure'):
            TransportMission.arm_with_recovery(node,[1.,2.,3.])
    else:
        assert TransportMission.arm_with_recovery(node,[1.,2.,3.])==[1.,2.,3.]
    assert modes==['manipulate'] and not node.active
