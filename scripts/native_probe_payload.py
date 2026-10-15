"""Offline loaded-start fixture using the native magnetic weld and object mass."""
import numpy as np


def attach_payload(model,data):
    import mujoco

    wrist = model.body('right_wrist_yaw_link').id
    body = model.body('manip_object_box_red').id
    weld = model.equality('grasp_right_box_red').id
    joint = model.body_jntadr[body]
    if model.jnt_type[joint]!=mujoco.mjtJoint.mjJNT_FREE:
        raise ValueError('probe payload must have a free joint')
    offset = model.numeric('transport_hand_capture').data[:3].copy()
    qpos = model.jnt_qposadr[joint]
    dof = model.jnt_dofadr[joint]
    data.qpos[qpos:qpos+3] = data.xpos[wrist]+data.xmat[wrist].reshape(3,3)@offset
    data.qpos[qpos+3:qpos+7] = data.xquat[wrist]
    data.qvel[dof:dof+6] = 0.
    model.eq_data[weld,:3] = 0.
    model.eq_data[weld,3:6] = offset
    model.eq_data[weld,6:10] = (1.,0.,0.,0.)
    model.eq_data[weld,10] = 1.
    data.eq_active[weld] = True
    mujoco.mj_forward(model,data)
    return dict(body=body,wrist=wrist,weld=weld,offset=offset,
                mass_kg=float(model.body_mass[body]))


def payload_sample(model,data,payload):
    expected = data.xpos[payload['wrist']]+data.xmat[payload['wrist']].reshape(3,3)@payload['offset']
    return dict(payload_attached=bool(data.eq_active[payload['weld']]),
                payload_position_error_m=float(np.linalg.norm(data.xpos[payload['body']]-expected)))
