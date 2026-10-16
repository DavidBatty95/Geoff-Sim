"""Pinned MuJoCo tests; run in the probe environment with plugin autoload off."""
import json

import mujoco
import numpy as np
import pytest

from native_probe_checkpoint import save_checkpoint,restore_checkpoint,array_state,restore_array
from native_probe_support import support_state


def fixture_model():
    return mujoco.MjModel.from_xml_string('''<mujoco><option timestep=".002"/>
      <worldbody><geom type="plane" size="2 2 .1"/>
      <body pos="0 0 .12"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
      </worldbody></mujoco>''')


def test_action_dtype_survives_json_roundtrip():
    action = np.linspace(-1.,1.,29,dtype=np.float32)
    restored = restore_array(json.loads(json.dumps(array_state(action))))
    assert restored.dtype==action.dtype
    assert np.linalg.norm(restored)==np.linalg.norm(action)
    np.testing.assert_array_equal(restored,action)


def test_support_diagnostic_distinguishes_loaded_and_airborne_foot():
    feet = ''.join(f'''<body name="{side}_ankle_roll_link" pos="{x} 0 {z}">
      <joint name="{side}_hip_pitch_joint" type="slide" axis="1 0 0"/>
      <joint name="{side}_knee_joint" type="slide" axis="0 0 1"/>
      <geom type="sphere" size=".1" mass="1"/></body>'''
      for side,x,z in (('left',0,.09),('right',1,1.)))
    model = mujoco.MjModel.from_xml_string(f'<mujoco><worldbody><geom type="plane" size="2 2 .1"/>{feet}</worldbody></mujoco>')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    state = support_state(model,data)
    assert state['foot_normal_force_n']['left']>0.
    assert state['foot_normal_force_n']['right']==0.
    assert state['ankle_body_height_m']==dict(left=.09,right=1.)


def test_pending_step_restores_contact_solver_and_applied_force(tmp_path):
    model = fixture_model()
    data = mujoco.MjData(model)
    for _ in range(100):
        mujoco.mj_step(model,data)
    data.qfrc_applied[0] = .2
    path = tmp_path/'checkpoint.json'
    identity = {'policy':'example','configuration':{'delay':2}}
    controller = {'queue':[[2,[.3]]],'history':[[1.,2.]],'pending_step':100}
    save_checkpoint(path,model,data,identity,controller)
    restored_model = fixture_model()
    restored = mujoco.MjData(restored_model)
    assert restore_checkpoint(path,restored_model,restored,identity)==controller
    for _ in range(30):
        mujoco.mj_step(model,data)
        mujoco.mj_step(restored_model,restored)
        np.testing.assert_array_equal(data.qpos,restored.qpos)
        np.testing.assert_array_equal(data.qvel,restored.qvel)
        np.testing.assert_array_equal(data.qacc_warmstart,restored.qacc_warmstart)


@pytest.mark.parametrize('fault',['identity','engine','state','model','overwrite'])
def test_incompatible_or_corrupt_checkpoint_is_rejected(tmp_path,fault):
    model = fixture_model()
    data = mujoco.MjData(model)
    path = tmp_path/'checkpoint.json'
    save_checkpoint(path,model,data,{'policy':'A'},{})
    if fault=='overwrite':
        with pytest.raises(FileExistsError):
            save_checkpoint(path,model,data,{'policy':'A'},{})
        return
    record = json.loads(path.read_text())
    if fault=='identity':
        record['identity']['policy'] = 'B'
    elif fault=='engine':
        record['mujoco_version'] = 'different'
    elif fault=='state':
        record['state'] = [0.]
    else:
        model.opt.timestep = .01
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError,match='checkpoint'):
        restore_checkpoint(path,model,data,{'policy':'A'})
