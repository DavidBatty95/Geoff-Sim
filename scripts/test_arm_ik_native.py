"""Offline robot-geometry regression; run through run_mjlab.sh with pytest."""
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/unitree_g1_mujoco_bridge'))
from unitree_g1_mujoco_bridge.arm_kinematics import ArmKinematics


def test_failed_pickup_ik_branch_is_replaced_without_torso_penetration():
    kin = ArmKinematics((ROOT/'ros2_ws/src/unitree_g1_description/urdf/g1_29dof_mode_15_brainco_hand.urdf').read_text(),
                        tool_offset=(.12,.01,0.))
    model = mujoco.MjModel.from_xml_path(str(ROOT/'ros2_ws/src/unitree_mujoco/unitree_robots/g1/g1_29dof_transport_medium_camera.xml'))
    # Right-arm target recorded by run 20261012T165825Z. Only robot geometry
    # is used here: no world objects, evaluator state, or simulated dynamics.
    blocked = dict(zip(kin.names,(-.00567139,.1525948,.2773698,-.7602669,
                                  -.2090796,.7514498,-.1019206)))

    def torso_penetration(joints):
        data = mujoco.MjData(model)
        for name,value in joints.items():
            data.qpos[model.joint(name).qposadr[0]] = value
        mujoco.mj_forward(model,data)
        distances = []
        for contact in data.contact:
            bodies = {model.body(model.geom_bodyid[g]).name for g in (contact.geom1,contact.geom2)}
            if 'torso_link' in bodies and any(b.startswith('right_') for b in bodies):
                distances.append(float(contact.dist))
        return min(distances,default=0.)

    assert torso_penetration(blocked) < -.02
    target,orientation,_,_ = kin.forward(blocked)
    solution,error = kin.solve(target,blocked,orientation)
    assert error < .003
    assert np.linalg.norm(kin.forward(solution)[1]-orientation) < .05
    assert torso_penetration(solution) >= 0.
