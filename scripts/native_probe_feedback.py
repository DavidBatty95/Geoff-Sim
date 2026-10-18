"""Select native joint-sensor feedback without silently refreshing its epoch."""
import mujoco
import numpy as np


def joint_sensor_addresses(model,joint_names):
    result = []
    for sensor_type in (mujoco.mjtSensor.mjSENS_JOINTPOS,mujoco.mjtSensor.mjSENS_JOINTVEL):
        addresses = []
        for name in joint_names:
            joint = model.joint(name).id
            matches = np.flatnonzero((model.sensor_type==sensor_type)&
                                    (model.sensor_objtype==mujoco.mjtObj.mjOBJ_JOINT)&(model.sensor_objid==joint))
            if len(matches)!=1 or model.sensor_dim[matches[0]]!=1:
                raise ValueError(f'unique scalar joint sensor required for {name}')
            addresses.append(model.sensor_adr[matches[0]])
        result.append(np.asarray(addresses,dtype=int))
    return tuple(result)


def joint_feedback(data,qids,dids,addresses,source):
    if source=='state':
        return data.qpos[qids].copy(),data.qvel[dids].copy()
    if source=='sensors':
        return data.sensordata[addresses[0]].copy(),data.sensordata[addresses[1]].copy()
    raise ValueError('unknown joint feedback source')
