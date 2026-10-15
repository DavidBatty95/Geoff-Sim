import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from run_transport_demo import numeric_thread_environment

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('threads',[None,1,4])
def test_explicit_limits_reach_simulation_container_without_changing_default(tmp_path,threads):
    docker = tmp_path/'docker'
    docker.write_text(f'#!{sys.executable}\nimport json,os,sys\nopen(os.environ["CAPTURE"],"w").write(json.dumps(sys.argv[1:]))\n')
    docker.chmod(0o755)
    output = tmp_path/'command.json'
    env = dict(os.environ,PATH=str(tmp_path)+':'+os.environ['PATH'],CAPTURE=str(output))
    env.pop('GEOFF_NUMERIC_THREADS',None)
    if threads is not None:
        env['GEOFF_NUMERIC_THREADS'] = str(threads)
    subprocess.run(['bash',str(ROOT/'scripts/bringup_full_sim.sh'),'run_rviz:=false'],env=env,check=True)
    arguments = json.loads(output.read_text())
    expected = numeric_thread_environment(threads)
    for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):
        settings = [arg for arg in arguments if arg.startswith(key+'=')]
        assert settings==([] if threads is None else [key+'='+expected[key]])
    assert arguments[-1]=='run_rviz:=false'


@pytest.mark.parametrize('value',[0,33,-1,True,1.5])
def test_invalid_limits_cannot_become_container_configuration(value):
    with pytest.raises(ValueError):
        numeric_thread_environment(value)


def test_shell_rejects_invalid_limit_before_starting_docker():
    result = subprocess.run(['bash',str(ROOT/'scripts/bringup_full_sim.sh')],
        env=dict(os.environ,GEOFF_NUMERIC_THREADS='1; invalid'),capture_output=True,text=True)
    assert result.returncode==2 and 'must be an integer' in result.stderr
