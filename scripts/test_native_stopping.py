import copy
import json
import sys

import pytest

from score_native_stopping import score_probe
import score_native_stopping as scorer


def stopped_probe():
    return dict(command_duration_s=2.,requested_duration_s=13.,fallen=False,sampled_nonfloor_robot_contacts={},
        samples=[dict(time=7.+i*.02,xy=[0.,0.],body_velocity=[0.,0.],yaw_rate=0.,command=[0.,0.,0.])
                 for i in range(301)])


def test_upright_drift_is_not_a_qualified_stop():
    data = stopped_probe()
    for i,row in enumerate(data['samples']):
        row.update(xy=[i*.01,0.],body_velocity=[.5,0.])
    result = score_probe(data)
    assert not result['passed'] and 'post_stop_travel_exceeded' in result['reasons']


def test_stationary_dwell_and_full_observation_pass():
    result = score_probe(stopped_probe())
    assert result['passed'] and result['settled_at_s']==pytest.approx(1.)


@pytest.mark.parametrize('kind',['gap','reorder','nan','short','resume','contact','fall'])
def test_incomplete_or_unstable_evidence_cannot_pass(kind):
    data = copy.deepcopy(stopped_probe())
    if kind=='gap':
        del data['samples'][10:30]
    elif kind=='reorder':
        data['samples'][3]['time'] = 7.
    elif kind=='nan':
        data['samples'][3]['body_velocity'][0] = float('nan')
    elif kind=='short':
        data['samples'] = data['samples'][:100]
    elif kind=='resume':
        data['samples'][-1]['body_velocity'][0] = .2
    elif kind=='contact':
        data['sampled_nonfloor_robot_contacts'] = {'table':1}
    else:
        data['fallen'] = True
    assert not score_probe(data)['passed']


def test_missing_stop_protocol_is_explicit():
    assert score_probe({})['reasons']==['fixed_command_stop_required']


def test_pre_boundary_float_sample_is_not_mislabeled_as_post_stop_motion():
    data = stopped_probe()
    data['samples'].insert(0,dict(time=7.-1e-12,xy=[0.,0.],body_velocity=[.1,0.],
                                  yaw_rate=0.,command=[.3,0.,0.]))
    assert score_probe(data)['passed']


def test_requested_window_cannot_be_silently_truncated_after_a_good_stop():
    data = stopped_probe()
    data['requested_duration_s'] = 20.
    assert 'incomplete_requested_probe' in score_probe(data)['reasons']


def test_require_pass_returns_failure_but_retains_report(tmp_path,monkeypatch):
    data = stopped_probe()
    data['samples'][-1]['xy'] = [1.,0.]
    source,output = tmp_path/'probe.json',tmp_path/'report.json'
    source.write_text(json.dumps(data))
    monkeypatch.setattr(sys,'argv',['score',str(source),'--output',str(output),'--require-pass'])
    assert scorer.main()==1
    report = json.loads(output.read_text())
    assert report['passed']==0 and len(report['attempts'])==1
    assert report['attempts'][0]['metadata']['fallen'] is False
