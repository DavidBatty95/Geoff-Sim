import copy

import pytest

from run_checkpoint_stop_branches import compare_branch,STATE_FIELDS
from score_native_stopping import score_probe
from test_native_stopping import stopped_probe
from native_probe_timing import command_steps


def branch_pair():
    before = {key:0. for key in STATE_FIELDS}
    before.update(time=6.98,command=[0.,-.45,0.])
    moving = dict(before,time=7.)
    parent = dict(samples=[before,moving])
    branch = dict(samples=[copy.deepcopy(before),dict(moving,command=[0.,0.,0.])],
                  branch_prefix_samples=1,branch_stop_time_s=7.)
    return parent,branch


@pytest.mark.parametrize('fault',[None,'prefix','state','command','time'])
def test_stop_branch_preserves_prefix_and_physical_state(fault):
    parent,branch = branch_pair()
    if fault=='prefix':
        branch['samples'][0]['yaw'] = 1.
    elif fault=='state':
        branch['samples'][1]['yaw'] = 1.
    elif fault=='command':
        branch['samples'][1]['command'] = [0.,-.45,0.]
    elif fault=='time':
        branch['branch_stop_time_s'] = 7.02
    if fault is None:
        assert compare_branch(parent,branch)['time']==7.
    else:
        with pytest.raises(ValueError):
            compare_branch(parent,branch)


def test_branch_is_scored_from_actual_override_not_original_scheduled_stop():
    data = stopped_probe()
    data.update(command_duration_s=4.,branch_stop=True,branch_stop_time_s=7.,checkpoint_time_s=6.99,
                branch_prefix_samples=1,resumed_checkpoint_sha256='a'*64)
    for row in data['samples']:
        row['xy'] = [min(.4,(row['time']-7.)*.4),0.]
    data['samples'].insert(0,dict(time=6.98,xy=[0.,0.],body_velocity=[.4,0.],yaw_rate=0.,command=[.4,0.,0.]))
    result = score_probe(data)
    assert result['scope']=='checkpoint_stop_branch' and result['command_end_s']==7.
    assert not result['passed'] and 'post_stop_travel_exceeded' in result['reasons']
    data['branch_stop_time_s'] = 9.
    assert score_probe(data)['reasons']==['invalid_checkpoint_stop_branch']


def test_integer_clock_avoids_delaying_exact_boundaries_and_rounds_up_partial_steps():
    assert command_steps(2.,15.)==(2500,3500)
    assert command_steps(.0001,15.)==(2500,2501)
    assert command_steps(.002,15.)==(2500,2501)


def test_step_clock_scores_the_recorded_stop_even_when_float_time_is_below_seven():
    data = stopped_probe()
    for i,row in enumerate(data['samples']):
        row['time'] -= 1e-12
        row['physics_step'] = 3500+i*10
    data['samples'].insert(0,dict(time=5.-1e-12,physics_step=2500,xy=[0.,0.],body_velocity=[.4,0.],
                                yaw_rate=0.,command=[.4,0.,0.]))
    data.update(command_clock='physics_step',command_start_step=2500,command_end_step=3500,
                actual_command_start_s=5.-1e-12,actual_command_end_s=7.-1e-12)
    result = score_probe(data)
    assert result['passed'] and result['command_end_s']==7.-1e-12
    data['actual_command_end_s'] = 7.
    assert score_probe(data)['reasons']==['invalid_physics_step_command_transition']
