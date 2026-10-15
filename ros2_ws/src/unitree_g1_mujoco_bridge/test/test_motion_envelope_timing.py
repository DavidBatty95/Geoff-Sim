import math

import pytest

from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope


@pytest.mark.parametrize('filtered',[None,(.4,0.,0.)])
@pytest.mark.parametrize('dt',[.05,.1,.2])
def test_handoff_cannot_brake_before_its_own_response_delay(filtered,dt):
    model = MotionEnvelope(max_x=1.,accel_x=1.)
    trajectory = model.rollout((0.,0.,0.),(.4,0.,0.),(.4,0.,0.),.35,dt,.15,
                               command_duration=.2,next_command=(0.,0.,0.),filtered_command=filtered)
    assert trajectory[-1][0]==pytest.approx(.4*.35)
    assert len(trajectory)==math.ceil(.35/dt)+1


@pytest.mark.parametrize('filtered',[None,(0.,0.,0.)])
def test_short_pulse_is_delayed_not_erased(filtered):
    model = MotionEnvelope(max_x=1.,accel_x=10.,linear_time_constant=.05)
    trajectory = model.rollout((0.,0.,0.),(.4,0.,0.),(0.,0.,0.),.4,.05,.15,
                               command_duration=.05,next_command=(0.,0.,0.),filtered_command=filtered)
    assert all(abs(point[0])<1e-12 for point in trajectory if point[3]<=.15)
    assert trajectory[-1][0]>.005


def test_partial_delay_step_preserves_coast_before_deceleration():
    model = MotionEnvelope(max_x=1.,accel_x=1.)
    result = model.rollout((0.,0.,0.),(0.,0.,0.),(.4,0.,0.),.2,.1,.15)
    assert result[-1][0]==pytest.approx(.4*.15+(.4+.35)*.5*.05)


@pytest.mark.parametrize('filtered',[None,(.3,0.,0.)])
def test_reversal_preserves_first_phase_through_delayed_handoff(filtered):
    model = MotionEnvelope(max_x=1.,accel_x=1.)
    result = model.rollout((.2,-.1,.7),(.3,0.,0.),(.3,0.,0.),.4,.1,.17,
                           command_duration=.23,next_command=(-.3,0.,0.),filtered_command=filtered)
    assert result[-1][:3]==pytest.approx((.2+.12*math.cos(.7),-.1+.12*math.sin(.7),.7))


def test_stop_response_is_consistent_across_a_delay_straddling_grid():
    model = MotionEnvelope(max_x=1.,accel_x=1.,linear_time_constant=.4)
    coarse = model.rollout((0.,0.,0.),(0.,0.,0.),(.4,0.,0.),.2,.1,.15,filtered_command=(.4,0.,0.))
    fine = model.rollout((0.,0.,0.),(0.,0.,0.),(.4,0.,0.),.2,.02,.15,filtered_command=(.4,0.,0.))
    assert coarse[-1][0]==pytest.approx(fine[-1][0],abs=5e-5)
