import json
import sys

import pytest

import run_cumulative_stopping as runner


def prepare_source(tmp_path,monkeypatch):
    policy = tmp_path/'policy.onnx'
    policy.write_bytes(b'test policy')
    case = runner.cases(['old'],['payload'],[2],[2.])[0]
    source = dict(complete=True,require_motion_progress=True,command_clock='physics_step',
        joint_feedback='sensors',pd_feedback='sensors',duration_s=15.,payload_offset_x_m=None,
        policies={'old':{'sha256':runner.digest(policy)}},
        attempts=[dict(case,passed=True,motion_progress={'passed':True})])
    path = tmp_path/'source.json'
    path.write_text(json.dumps(source))
    monkeypatch.setattr(sys,'argv',['cumulative','--policy',f'candidate={policy}',
        '--sources',str(path),'--command-durations','2','--output',str(tmp_path/'output')])
    return path,source


@pytest.mark.parametrize('valid_protocol',[True,False])
def test_only_matching_evidence_is_reused_and_missing_failures_are_retained(tmp_path,monkeypatch,valid_protocol):
    path,source = prepare_source(tmp_path,monkeypatch)
    if not valid_protocol:
        source['joint_feedback'] = 'state'
        path.write_text(json.dumps(source))
    calls = []
    def run(case,*args):
        calls.append(case)
        return dict(case,passed=False,error='probe_timeout')
    monkeypatch.setattr(runner,'run_case',run)
    assert runner.main()==1
    result = json.loads((tmp_path/'output/summary.json').read_text())
    assert result['complete'] and len(result['attempts'])==8
    assert len(calls)==(7 if valid_protocol else 8)
    assert result['passed']==int(valid_protocol)
    assert sum('reused_evidence' in row for row in result['attempts'])==int(valid_protocol)


def test_contradictory_cached_verdicts_are_not_cherry_picked(tmp_path,monkeypatch):
    path,source = prepare_source(tmp_path,monkeypatch)
    source['attempts'].append(dict(source['attempts'][0],passed=False))
    path.write_text(json.dumps(source))
    with pytest.raises(ValueError,match='contradictory'):
        runner.main()
