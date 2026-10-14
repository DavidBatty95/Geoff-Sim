import math

import pytest

from unitree_g1_mujoco_bridge.policy_response import calibrated_turn_reference,pivot_yaw_rate,NATIVE_CONTACT_PROFILE
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope


def test_inverse_response_and_feedback_taper_both_turn_directions():
    for loaded in (False,True):
        for sign in (-1.,1.):
            command = calibrated_turn_reference(sign*.5,sign*.45,loaded)
            assert pivot_yaw_rate(command,loaded)==pytest.approx(sign*.45)
            near = calibrated_turn_reference(sign*.15,0.,loaded)
            assert abs(near)<abs(command)
            assert calibrated_turn_reference(sign*.09,0.,loaded)==0.
            assert abs(calibrated_turn_reference(sign*.5,sign*.8,loaded))<abs(command)
    assert calibrated_turn_reference(math.nan,0.)==0.


def test_policy_specific_rollout_uses_physical_yaw_instead_of_old_constant_gain():
    model = MotionEnvelope(max_yaw=.9,accel_yaw=10.,yaw_gain=.32,response_profile=NATIVE_CONTACT_PROFILE)
    path = model.rollout((0.,0.,0.),(0.,0.,.6),(0.,0.,0.),5.,.02,delay=0.,filtered_command=(0.,0.,.6))
    assert 2.0<path[-1][2]<2.4  # Measured ~.465 rad/s, not .6*.32.
    assert path[-1][0]==path[-1][1]==0.
