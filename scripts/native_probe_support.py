"""Offline foot-support diagnostics; never policy inputs or stop permissions."""
import mujoco
import numpy as np


def support_state(model,data):
    feet = {model.body(f'{side}_ankle_roll_link').id:side for side in ('left','right')}
    loads = dict(left=0.,right=0.)
    force = np.zeros(6)
    for index,contact in enumerate(data.contact):
        a,b = int(contact.geom1),int(contact.geom2)
        for foot,other in ((a,b),(b,a)):
            side = feet.get(int(model.geom_bodyid[foot]))
            if side is not None and model.geom_type[other]==mujoco.mjtGeom.mjGEOM_PLANE:
                mujoco.mj_contactForce(model,data,index,force)
                loads[side] += max(0.,float(force[0]))
    return dict(foot_normal_force_n=loads,
        ankle_body_height_m={side:float(data.xpos[body,2]) for body,side in feet.items()},
        leg_joint_pos_rad={f'{side}_{joint}':float(data.qpos[model.joint(f'{side}_{joint}_joint').qposadr[0]])
                           for side in ('left','right') for joint in ('hip_pitch','knee')})
