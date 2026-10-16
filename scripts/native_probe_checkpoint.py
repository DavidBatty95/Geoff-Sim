"""JSON checkpoints at the native probe's pre-physics-step boundary.

Restore must execute the pending mj_step before reading derived sensors/contacts.
This is a synchronous diagnostic checkpoint, not a ROS mission snapshot.
"""
import hashlib
import json

import mujoco
import numpy as np


def array_state(value):
    return dict(dtype=str(value.dtype),values=value.tolist())


def restore_array(record):
    if record['dtype'] not in ('float32','float64'):
        raise ValueError('unsupported checkpoint controller dtype')
    value = np.asarray(record['values'],dtype=record['dtype'])
    if value.shape!=(29,) or not np.all(np.isfinite(value)):
        raise ValueError('invalid checkpoint controller vector')
    return value


def model_digest(model):
    buffer = np.empty(mujoco.mj_sizeModel(model),dtype=np.uint8)
    mujoco.mj_saveModel(model,None,buffer)
    return hashlib.sha256(buffer.tobytes()).hexdigest()


def encode(value):
    if isinstance(value,np.ndarray):
        return value.tolist()
    if isinstance(value,np.generic):
        return value.item()
    raise TypeError(f'unsupported checkpoint value {type(value).__name__}')


def save_checkpoint(path,model,data,identity,controller):
    state_type = mujoco.mjtState.mjSTATE_INTEGRATION
    state = np.empty(mujoco.mj_stateSize(model,state_type))
    mujoco.mj_getState(model,data,state,state_type)
    record = dict(schema_version=1,boundary='before_mj_step',mujoco_version=mujoco.__version__,
                  identity=identity,model_sha256=model_digest(model),state_type=int(state_type),
                  state=state,controller=controller)
    encoded = json.dumps(record,default=encode,allow_nan=False)
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x') as stream:
        stream.write(encoded+'\n')


def restore_checkpoint(path,model,data,identity):
    record = json.loads(path.read_text())
    json.dumps(record,allow_nan=False)
    if (record.get('schema_version')!=1 or record.get('boundary')!='before_mj_step' or
            record.get('mujoco_version')!=mujoco.__version__ or record.get('identity')!=identity or
            record.get('model_sha256')!=model_digest(model) or
            record.get('state_type')!=int(mujoco.mjtState.mjSTATE_INTEGRATION)):
        raise ValueError('checkpoint engine/model/policy/configuration/source mismatch')
    state = np.asarray(record['state'],dtype=float)
    state_type = mujoco.mjtState.mjSTATE_INTEGRATION
    if state.shape!=(mujoco.mj_stateSize(model,state_type),) or not np.all(np.isfinite(state)):
        raise ValueError('invalid checkpoint integration state')
    if not isinstance(record.get('controller'),dict):
        raise ValueError('missing checkpoint controller state')
    mujoco.mj_setState(model,data,state,state_type)
    return record['controller']
