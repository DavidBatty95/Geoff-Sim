import math

import numpy as np
import pytest

from unitree_g1_mujoco_bridge.kinodynamic_search import remaining_schedule, rollout_schedule, search_sequence
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope


@pytest.mark.parametrize('seed',range(10))
@pytest.mark.parametrize('delay,duration',((.1,.6),(.08,.03),(.8,.45)))
def test_vectorized_response_matches_the_shared_scalar_rollout(seed,delay,duration):
    rng = np.random.default_rng(seed)
    envelope = MotionEnvelope(max_x=.55,max_y=.45,max_yaw=.9,accel_x=2.4,accel_y=2.4,accel_yaw=.9,
                              forward_gain=.9,lateral_gain=.8,yaw_gain=.32,walking_yaw_gain=.55,
                              lateral_yaw_gain=.1,linear_time_constant=.4,
                              negative_lateral_gain=.9,positive_lateral_time_constant=.3,max_planar_speed=.65)
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
