import pytest

from run_transport_demo import transport_navigation_profile


def test_transport_clearance_and_diagonal_speed_are_shared_without_disabling_strafing():
    base = {'human_aware_motion_planner':{'ros__parameters':{
        'body_prediction_margin':.25,'path_reference_speed':.6,'max_planar_speed':.65,
        'max_lateral_near_human':.45}},
        'human_collision_avoidance':{'ros__parameters':{'prediction_collision_radius':1.,'max_planar_speed':.65}}}
    profile = transport_navigation_profile(base,.1)
    planner = profile['human_aware_motion_planner']['ros__parameters']
    supervisor = profile['human_collision_avoidance']['ros__parameters']
    assert planner['body_prediction_margin']==pytest.approx(.35)
    assert supervisor['prediction_collision_radius']==pytest.approx(1.1)
    assert planner['path_reference_speed']==.45
    policy = profile['g1_policy_cmd_vel_controller']['ros__parameters']
    assert planner['max_planar_speed']==supervisor['max_planar_speed']==policy['max_planar_speed']==.45
    assert planner['max_lateral_near_human']==.45
    assert base['human_aware_motion_planner']['ros__parameters']['body_prediction_margin']==.25


@pytest.mark.parametrize('extra',[-.1,.6,float('nan'),float('inf')])
def test_invalid_clearance_is_rejected(extra):
    with pytest.raises(ValueError):
        transport_navigation_profile({},extra)
