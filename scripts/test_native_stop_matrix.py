from types import SimpleNamespace
import json
from pathlib import Path
import subprocess
import sys

import pytest

import run_native_stop_matrix as matrix
from test_native_stopping import stopped_probe


def test_plan_pairs_every_policy_with_identical_conditions():
    plan = matrix.cases(['baseline','candidate'],['payload','held-empty'],[2,3],[2.])
    assert len(plan)==16 and len({row['case_id'] for row in plan})==16
    conditions = lambda name: {(r['load'],r['delay_steps'],r['linear_y'],r['command_duration_s'])
                               for r in plan if r['policy']==name}
    assert conditions('baseline')==conditions('candidate')


@pytest.mark.parametrize('failure',['exit','missing','timeout'])
def test_process_failure_retains_attempt_instead_of_becoming_success(tmp_path,monkeypatch,failure):
    def run(*args,**kwargs):
        if failure=='timeout':
            raise subprocess.TimeoutExpired(args[0],1.)
        return SimpleNamespace(returncode=2 if failure=='exit' else 0)
    monkeypatch.setattr(matrix.subprocess,'run',run)
    case = matrix.cases(['candidate'],['payload'],[2],[2.])[0]
    result = matrix.run_case(case,tmp_path/'policy.onnx','hash',tmp_path,15.,1.)
    assert not result['passed']
    assert result['error']==dict(exit='probe_process_failed',missing='missing_probe_result',timeout='probe_timeout')[failure]
    assert result['case_id']==case['case_id']


@pytest.mark.parametrize('changed_policy',[False,True])
def test_scored_success_cannot_hide_mismatched_policy(tmp_path,monkeypatch,changed_policy):
    data = stopped_probe()
    data.update(policy_sha256='changed' if changed_policy else 'hash',delay_steps=2,
                requested_command=[0.,.45,0.],action_feedback='published',payload=False,
                held_empty=True,observation_steps=[10],sensor_delay_steps=0,
                mujoco_version='3.3.6',record_actions=True,ramp=False)
    for i,row in enumerate(data['samples']):
        row.update(published_sequence=i,applied_sequence=i-2 if i>=2 else -1,
                   published_action=[0.]*29,applied_action=[0.]*29)
    def run(command,**kwargs):
        Path(command[command.index('--output')+1]).write_text(json.dumps(data))
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(matrix.subprocess,'run',run)
    case = matrix.cases(['candidate'],['held-empty'],[2],[2.])[0]
    result = matrix.run_case(case,tmp_path/'policy.onnx','hash',tmp_path,13.,1.)
    assert result['passed'] is (not changed_policy)
    if changed_policy:
        assert result['mismatches']==['policy_sha256']


def test_export_retains_incomplete_plan_and_failed_attempt(tmp_path,monkeypatch):
    source,destination = tmp_path/'summary.json',tmp_path/'portable.json'
    plan = matrix.cases(['candidate'],['payload'],[2],[2.])
    source.write_text(json.dumps(dict(complete=False,policies={'candidate':{}},plan=plan,
        attempts=[dict(**plan[0],passed=False,error='probe_timeout')])))
    monkeypatch.setattr(sys,'argv',['matrix','--export-summary',str(source),'--report',str(destination)])
    assert matrix.main()==0
    report = json.loads(destination.read_text())
    assert not report['complete'] and report['by_policy']['candidate']['planned']==2
    assert report['by_policy']['candidate']['attempted']==1 and report['passed']==0
    assert report['source_summary']['sha256']==matrix.digest(source)


def test_matrix_continues_after_failed_probe_and_returns_failure(tmp_path,monkeypatch):
    policy = tmp_path/'policy.onnx'
    policy.write_bytes(b'fake policy for orchestration test')
    output = tmp_path/'matrix'
    def run(case,*args):
        return dict(**case,passed=case['linear_y']<0.,error='synthetic_failure') if case['linear_y']>0. else dict(**case,passed=True)
    monkeypatch.setattr(matrix,'run_case',run)
    monkeypatch.setattr(sys,'argv',['matrix','--policy',f'candidate={policy}','--output',str(output),
                                    '--loads','payload','--delays','2'])
    assert matrix.main()==1
    report = json.loads((output/'summary.json').read_text())
    assert report['complete'] and len(report['attempts'])==2 and report['passed']==1
