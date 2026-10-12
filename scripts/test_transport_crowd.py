import pytest

from run_transport_demo import pedestrian_settings,transport_navigation_profile
from transport_crowd import crowd_summary


def test_natural_walkers_have_explicit_replayable_settings_and_no_goal_gate():
    settings = pedestrian_settings(2,48,10.)
    assert settings['mujoco_pedestrian_scenario']=='social_natural_plaza'
    assert settings['mujoco_pedestrians_count']=='2'
    assert settings['mujoco_pedestrian_seed']=='48'
    assert settings['mujoco_pedestrian_start_gate']=='false'
    for delay in (-1.,float('nan'),float('inf'),121.):
        with pytest.raises(ValueError):
            pedestrian_settings(2,48,delay)


def test_motion_audit_excludes_replays_and_unobserved_gaps():
    def sample(stamp,x):
        return dict(topic='evaluation_aligned',data=dict(sim_time=stamp,
            people=[dict(id=1,x=x,y=0.,vx=.6,vy=0.)]))
    report = crowd_summary([sample(1.,0.),sample(1.,10.),sample(2.,.6),sample(5.,10.),
        dict(topic='planner_debug',data=dict(state='YIELD',track_count=1,planning_ms=8.,
             candidate_results=[dict(score_terms=dict(overtaking_conflict=3.)),dict(score_terms=dict(overtaking_conflict=4.))]))])
    assert report['actors']['1']['path_length_m']==pytest.approx(.6)
    assert report['actors']['1']['samples']==3
    assert report['moving_actor_count']==1 and report['unmeasured_motion_gaps']==1
    assert report['max_observed_tracks']==1 and report['planner_planning_ms']['p95']==8.
    assert report['overtaking_penalty_samples']==1


def test_overtaking_lookahead_is_explicit_and_does_not_change_motion_caps():
    off = transport_navigation_profile({},.1)
    on = transport_navigation_profile({},.1,early_overtaking=True)
    key = 'human_aware_motion_planner'
    assert off[key]['ros__parameters']['overtaking_terminal_horizon']==0.
    assert on[key]['ros__parameters']['overtaking_terminal_horizon']==6.
    assert on[key]['ros__parameters']['max_planar_speed']==off[key]['ros__parameters']['max_planar_speed']==.45
