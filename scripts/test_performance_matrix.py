import json
import subprocess
import sys
import fcntl

import pytest
import yaml

import run_performance_matrix as matrix
from performance_outcome import strict_success


def good_result():
    return dict(success=True,mission_complete=True,data_valid=True,fallen=False,contact_samples=0)


@pytest.mark.parametrize('change',[
    {'mission_complete':False}, {'data_valid':False}, {'fallen':True},
    {'contact_samples':1}, {'contact_samples':False}, {'success':1},
])
def test_strict_outcome_rejects_incomplete_or_invalid_success(change):
    assert not strict_success(dict(good_result(),**change))


@pytest.mark.parametrize('content',['{', '[]', None])
def test_corrupt_and_missing_results_are_retained(tmp_path,content):
    if content is not None:
        (tmp_path/'result.json').write_text(content)
    record = dict(returncode=0,success=False)
    matrix.collect_result(record,tmp_path)
    assert not record['success'] and record['artifact_error']


def test_nonzero_exit_cannot_be_upgraded_by_result(tmp_path):
    (tmp_path/'result.json').write_text(json.dumps(good_result()))
    record = dict(returncode=1)
    matrix.collect_result(record,tmp_path)
    assert not record['success']


def configure(tmp_path,monkeypatch):
    policy = tmp_path/'policy.onnx'
    policy.write_bytes(b'test-policy')
    output = tmp_path/'matrix'
    monkeypatch.setattr(matrix,'RUNS',tmp_path/'results'/'transport_demo')
    monkeypatch.setattr(sys,'argv',['matrix','--policy',str(policy),'--output',str(output),
                                  '--tasks','red:red','blue:blue','--layouts','0'])
    return output


def test_planned_cohort_survives_timeout_and_does_not_continue(tmp_path,monkeypatch):
    output = configure(tmp_path,monkeypatch)
    def timeout(command,budget,log):
        persisted = json.loads((output/'summary.json').read_text())
        assert [r['status'] for r in persisted['attempts']]==['running','planned']
        raise subprocess.TimeoutExpired(command,budget)
    monkeypatch.setattr(matrix,'execute',timeout)
    assert matrix.main()==1
    summary = json.loads((output/'summary.json').read_text())
    assert not summary['complete']
    assert [r['status'] for r in summary['attempts']]==['timeout','planned']
    with pytest.raises(FileExistsError):
        matrix.main()


def test_malformed_attempt_does_not_abort_remaining_cohort(tmp_path,monkeypatch):
    output = configure(tmp_path,monkeypatch)
    calls = []
    def execute(command,budget,log):
        run_id = command[command.index('--run-id')+1]
        run = matrix.RUNS/run_id
        run.mkdir()
        calls.append(run_id)
        (run/'result.json').write_text('{' if len(calls)==1 else json.dumps(good_result()))
        return 0
    monkeypatch.setattr(matrix,'execute',execute)
    assert matrix.main()==1
    summary = json.loads((output/'summary.json').read_text())
    assert summary['complete'] and len(set(calls))==2
    assert [r['success'] for r in summary['attempts']]==[False,True]


def test_concurrent_matrix_is_rejected_before_launch(tmp_path,monkeypatch):
    output = configure(tmp_path,monkeypatch)
    matrix.RUNS.parent.mkdir(parents=True)
    with (matrix.RUNS.parent/'.performance_matrix.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(SystemExit):
            matrix.main()
    assert not (output/'summary.json').exists()


def test_execution_timeout_reaps_child(tmp_path):
    with (tmp_path/'child.log').open('w') as log:
        with pytest.raises(subprocess.TimeoutExpired):
            matrix.execute([sys.executable,'-c','import time; time.sleep(60)'],.1,log)


def test_bootstrap_revisions_match_patch_bases_and_gitlinks():
    root = matrix.ROOT
    repos = yaml.safe_load((root/'ros2_ws/unitree.repos').read_text())['repositories']
    sources = json.loads((root/'patches/manifest.json').read_text())['sources']
    for source in sources:
        entry = subprocess.check_output(['git','ls-tree','HEAD',source['path']],cwd=root,text=True)
        assert entry.split()[2]==source['base_commit']
        if source['path'].startswith('ros2_ws/src/'):
            assert repos[source['path'].split('/')[-1]]['version']==source['base_commit']
