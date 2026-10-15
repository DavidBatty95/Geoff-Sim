import math

import pytest

from unitree_g1_mujoco_bridge.transport_people import closest_person_distance,closest_person_corridor_distance


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


def test_predicted_crossing_blocks_dock_corridor_even_when_both_endpoints_clear():
    person,velocity,start,end = (2.,2.),(0.,-2.),(0.,0.),(4.,0.)
    assert closest_person_distance(person,velocity,start,0.)>1.6
    assert closest_person_distance(person,velocity,end,0.)>1.6
    assert closest_person_corridor_distance(person,velocity,start,end,0.)==0.


@pytest.mark.parametrize('person,velocity,start,end,expected',[
    ((0.,2.),(1.,0.),(0.,0.),(3.,0.),2.),
    ((2.,0.),(1.,0.),(0.,0.),(3.,0.),0.),
    ((10.,0.),(1.,0.),(0.,0.),(3.,0.),7.),
    ((4.,0.),(0.,0.),(0.,0.),(4.,0.),0.),
    ((3.5,0.),(-1.,0.),(0.,0.),(0.,0.),1.5),
])
def test_corridor_distance_handles_parallel_stationary_and_degenerate_segments(person,velocity,start,end,expected):
    assert closest_person_corridor_distance(person,velocity,start,end,0.)==pytest.approx(expected)
    assert closest_person_corridor_distance(person,velocity,end,start,0.)==pytest.approx(expected)


def test_corridor_prediction_starts_at_measurement_age_and_fails_closed():
    assert closest_person_corridor_distance((4.,3.),(0.,-1.),(0.,0.),(4.,0.),.5)==pytest.approx(.5)
    assert closest_person_corridor_distance((4.,3.),(0.,-1.),(0.,0.),(4.,0.),0.)==1.
    assert closest_person_corridor_distance((float('nan'),0.),(0.,0.),(0.,0.),(4.,0.),0.)==0.
    assert closest_person_corridor_distance((4.,3.),(0.,-1.),(0.,0.),(4.,0.),-.1)==0.
