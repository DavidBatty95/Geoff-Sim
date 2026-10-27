from types import SimpleNamespace as S

import numpy as np
import pytest

from unitree_g1_mujoco_bridge.strategic_avoidance import crowd_forecast,predict_pincer,predict_strategic_risk,refuge_candidates


def forecast(tracks):
    times=np.linspace(0.,7.,36)
    people,radii=crowd_forecast(tracks,times,.48,.35)
    return people,radii,times


def person(x,y,vx,vy):
    return S(x=x,y=y,vx=vx,vy=vy,body_radius=.4)


def test_opposed_approach_is_detected_before_the_short_horizon_conflict():
    people,radii,times=forecast([person(-4.,0.,.5,0.),person(4.,0.,-.5,0.)])
    risk=predict_pincer(np.zeros((len(times),2)),people,radii,times)
    assert risk['active'] and risk['pairs']==[[0,1]]
    assert 3.<risk['time_to_enclosure_s']<5.


@pytest.mark.parametrize('tracks',[
    [person(-4.,0.,-.5,0.),person(4.,0.,.5,0.)],
    [person(-4.,0.,.5,0.),person(4.,0.,.5,0.)],
    [person(-10.,0.,.5,0.),person(10.,0.,-.5,0.)],
    [person(-3.,0.,.5,0.)],
])
def test_diverging_parallel_distant_or_single_person_is_not_a_pincer(tracks):
    people,radii,times=forecast(tracks)
    assert not predict_pincer(np.zeros((len(times),2)),people,radii,times)['active']


def test_refuge_considers_second_and_third_people_and_terminal_hold():
    # Left/right walkers close the horizontal lane; a third closes the north.
    tracks=[person(-3.,0.,.5,0.),person(3.,0.,-.5,0.),person(0.,3.,0.,-.25)]
    people,radii,times=forecast(tracks)
    ranked=refuge_candidates((0.,0.,0.),people,radii,times)
    assert ranked[0]['goal'][1]<-1.
    assert ranked[0]['minimum_margin_m']>0.
    assert ranked[0]['terminal_margin_m']>0.
    hold=next(row for row in ranked if row['travel_m']==0.)
    assert hold['minimum_margin_m']<0.


def test_static_barrier_excludes_the_preferred_refuge_and_keeps_other_side():
    people,radii,times=forecast([person(-3.,0.,.5,0.),person(3.,0.,-.5,0.)])
    ranked=refuge_candidates((0.,0.,0.),people,radii,times,
        static_query=lambda points:np.where(points[:,1]<-.1,0.,10.))
    assert ranked[0]['goal'][1]>1.
    assert ranked[0]['minimum_margin_m']>0.
    assert not refuge_candidates((0.,0.,0.),people,radii,times,
        static_query=lambda points:np.zeros(len(points)))


def test_three_fast_converging_people_can_leave_no_clear_coarse_refuge():
    people,radii,times=forecast([person(-3.,0.,.5,0.),person(3.,0.,-.5,0.),person(0.,3.,0.,-.5)])
    ranked=refuge_candidates((0.,0.,0.),people,radii,times)
    assert ranked[0]['minimum_margin_m']<0.  # A direction hint is not safety admission.


def test_refuge_commitment_keeps_a_clear_choice_but_discards_a_blocked_one():
    people,radii,times=forecast([person(-3.,0.,.5,0.),person(3.,0.,-.5,0.)])
    previous=(0.,1.4)
    ranked=refuge_candidates((0.,0.,0.),people,radii,times,previous=previous)
    assert ranked[0]['retained'] and ranked[0]['goal']==list(previous)
    blocked=refuge_candidates((0.,0.,0.),people,radii,times,previous=previous,
        static_query=lambda points:np.where(points[:,1]>.1,0.,10.))
    assert not blocked[0]['retained'] and blocked[0]['goal'][1]<-1.


def test_invalid_forecast_is_not_a_valid_prediction():
    people,radii,times=forecast([person(-3.,0.,.5,0.),person(3.,0.,-.5,0.)])
    people[1,0,0]=np.nan
    with pytest.raises(ValueError):
        predict_pincer(np.zeros((len(times),2)),people,radii,times)


def test_closing_corridor_starts_planning_before_a_two_sided_pincer():
    people,radii,times=forecast([person(-4.,0.,.5,0.),person(4.,3.,0.,0.)])
    route=np.zeros((len(times),2))
    assert not predict_pincer(route,people,radii,times)['active']
    risk=predict_strategic_risk(route,people,radii,times)
    assert risk['active'] and not risk['pincer_active']
    assert risk['reason']=='closing_corridor' and risk['watch_indices']==[0]
    assert risk['time_to_reserve_s']>3.


@pytest.mark.parametrize('speed',[-.5,.05])
def test_early_watch_rejects_diverging_motion_and_stationary_jitter(speed):
    people,radii,times=forecast([person(-2.,0.,speed,0.),person(4.,3.,0.,0.)])
    assert not predict_strategic_risk(np.zeros((len(times),2)),people,radii,times)['active']


def test_robot_approach_to_stationary_crowd_also_activates_watch():
    people,radii,times=forecast([person(3.,0.,0.,0.),person(3.,3.,0.,0.)])
    route=np.column_stack((.4*times,np.zeros(len(times))))
    assert predict_strategic_risk(route,people,radii,times)['active']
