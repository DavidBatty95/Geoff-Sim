import math

import numpy as np
import pytest

from unitree_g1_mujoco_bridge.kinodynamic_search import remaining_schedule, rollout_schedule, search_sequence,response_step
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope


@pytest.mark.parametrize('seed',range(10))
@pytest.mark.parametrize('profile',('legacy','native_contact_cycle05'))
@pytest.mark.parametrize('delay,duration',((.1,.6),(.08,.03),(.8,.45)))
def test_vectorized_response_matches_the_shared_scalar_rollout(seed,delay,duration,profile):
    rng = np.random.default_rng(seed)
    envelope = MotionEnvelope(max_x=.55,max_y=.45,max_yaw=.9,accel_x=2.4,accel_y=2.4,accel_yaw=.9,
                              forward_gain=.9,lateral_gain=.8,yaw_gain=.32,walking_yaw_gain=.55,
                              lateral_yaw_gain=.1,linear_time_constant=.4,
                              negative_lateral_gain=.9,positive_lateral_time_constant=.3,max_planar_speed=.65,
                              response_profile=profile)
    pose = (*rng.uniform(-1.,1.,2),rng.uniform(-math.pi,math.pi))
    velocity = tuple(rng.uniform(-.3,.3,3))
    filtered = tuple(rng.uniform(-.3,.3,3))
    first = tuple(rng.uniform(-.6,.6,3)) if seed%2 else (0.,0.,0.)
    second = tuple(rng.uniform(-.6,.6,3)) if seed%3 else (0.,0.,0.)
    expected = envelope.rollout(pose,first,velocity,3.,.15,delay,duration,second,filtered)
    actual = rollout_schedule(envelope,pose,velocity,filtered,((duration,*first),(3.-duration,*second)),3.,.15,delay)
    np.testing.assert_allclose(actual,expected,atol=1e-12)


def test_schedule_progresses_across_multiple_phases():
    schedule = ((.45,.5,0.,0.),(.45,0.,.4,0.),(.45,0.,0.,0.))
    np.testing.assert_allclose(remaining_schedule(schedule,.55),((.35,0.,.4,0.),(.45,0.,0.,0.)))
    assert remaining_schedule(schedule,2.) == ()


def test_search_finds_progress_with_swept_clearance_and_stopping_tail():
    envelope = MotionEnvelope(max_x=.55,max_y=.45,accel_x=2.4,accel_y=2.4,linear_time_constant=.4)
    times = np.linspace(0.,3.,21)
    people = np.array([[[1.,-1.+.65*t]] for t in times])
    radii = np.full((21,1),.7)
    actions = [(0.,0.,0.),(.55,0.,0.),(0.,.45,0.),(0.,-.45,0.),(.4,.3,0.),(.4,-.3,0.)]
    arguments = (envelope,(0.,0.,0.),(0.,0.,0.),(0.,0.,0.),actions,people,radii,(1.5,0.),3.,.15,.1)
    schedule = search_sequence(*arguments,budget_ms=0.)
    assert schedule is not None
    assert schedule[-1][1:] == (0.,0.,0.)
    trajectory = np.array(rollout_schedule(envelope,(0.,0.,0.),(0.,0.,0.),(0.,0.,0.),schedule,3.,.15,.1))
    relative = trajectory[:,:2]-people[:,0,:]
    delta = np.diff(relative,axis=0)
    fraction = np.clip(-np.sum(relative[:-1]*delta,axis=1)/np.maximum(1e-12,np.sum(delta*delta,axis=1)),0.,1.)
    assert np.min(np.linalg.norm(relative[:-1]+fraction[:,None]*delta,axis=1)) >= .7-1e-9
    assert np.min(np.linalg.norm(trajectory[:,:2]-(1.5,0.),axis=1)) < 1.3
    assert search_sequence(*arguments,static_query=lambda points: np.zeros(len(points)),budget_ms=0.) is None
    assert search_sequence(*arguments,budget_ms=1e-12) is None


def test_terminal_hold_rejects_person_arriving_after_the_motion_horizon():
    envelope = MotionEnvelope()
    times = np.linspace(0.,3.,21)
    # Standing is clear through t=3, but the walker reaches the stopped robot
    # during the additional hold. A soft terminal penalty used to admit it.
    people = np.array([[[4.-t,0.]] for t in times])
    args = (envelope,(0.,0.,0.),(0.,0.,0.),(0.,0.,0.),[(0.,0.,0.)],people,
            np.full((21,1),.7),(0.,0.),3.,.15,.1)
    assert search_sequence(*args,budget_ms=0.,terminal_hold_s=0.) is not None
    assert search_sequence(*args,budget_ms=0.) is None


@pytest.mark.parametrize('profile',('legacy','native_contact_cycle05'))
@pytest.mark.parametrize('loaded',(False,True))
def test_small_batch_fast_path_matches_vector_path_for_different_admission_delays(profile,loaded):
    rng=np.random.default_rng(42)
    model=MotionEnvelope(max_x=.65,max_y=.45,max_yaw=.6,max_planar_speed=.45,
        accel_x=2.4,accel_y=2.4,accel_yaw=.9,forward_gain=.96,lateral_gain=.8,
        yaw_gain=.32,walking_yaw_gain=.55,lateral_yaw_gain=.1,negative_lateral_gain=.9,
        positive_lateral_time_constant=.3,linear_time_constant=.4,
        response_profile=profile,payload_active=loaded)
    states=rng.uniform(-.6,.6,(24,9))
    commands=rng.uniform(-.6,.6,(24,3))
    commands[:4]=0.
    active=np.tile((0.,.03,.15),8)
    expected=response_step(states,commands,model,.15,active)
    actual=np.vstack([response_step(states[i:i+6],commands[i:i+6],model,.15,active[i:i+6])
                      for i in range(0,24,6)])
    np.testing.assert_allclose(actual,expected,atol=1e-12)
