import math

import pytest

from unitree_g1_mujoco_bridge.transport_people import closest_person_distance


def test_crossing_between_discrete_prediction_samples_is_not_missed():
    # All three former sample instants looked clear, but the pedestrian passes
    # within the manipulation workspace halfway between them.
    position,velocity = (-1.,1.5),(2.,0.)
    assert all(math.hypot(position[0]+velocity[0]*t,position[1])>1.6 for t in (0.,1.,2.))
    assert closest_person_distance(position,velocity,(0.,0.),0.)==pytest.approx(1.5)


def test_prediction_window_starts_at_the_current_observation_age():
    assert closest_person_distance((3.5,0.),(-1.,0.),(0.,0.),.6)==pytest.approx(.9)
    assert closest_person_distance((3.5,0.),(-1.,0.),(0.,0.),0.)==pytest.approx(1.5)
    assert closest_person_distance((0.,0.),(2.,0.),(0.,0.),.8)==pytest.approx(1.6)


def test_stationary_safe_tracks_and_invalid_tracks():
    assert closest_person_distance((4.,1.),(0.,0.),(1.,1.),.2)==3.
    assert closest_person_distance((float('nan'),1.),(0.,0.),(0.,0.),0.)==0.
