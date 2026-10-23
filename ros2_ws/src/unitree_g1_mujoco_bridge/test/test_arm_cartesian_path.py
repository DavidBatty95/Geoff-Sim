"""Endpoint-only IK must not bow the open hand through the pickup workspace."""
from pathlib import Path

import numpy as np
import pytest

from unitree_g1_mujoco_bridge.arm_kinematics import ArmKinematics


def fixture():
    urdf = Path(__file__).resolve().parents[2]/'unitree_g1_description/urdf/g1_29dof_dex3.urdf'
    kin = ArmKinematics(urdf.read_text(),tool_offset=(.135,.04,0.))
    joints = dict(zip(kin.names,[-.92,-1.04,.01,-.2,1.45,.89,-.65]))
    return kin,joints


def test_staging_path_keeps_interpolated_tool_inside_cartesian_corridor():
    kin,joints = fixture()
    target = np.array([.25,-.23,.20])
    start = kin.forward(joints)[0]
    path,error = kin.cartesian_path(target,joints,np.eye(3))
    assert len(path)>1 and error<.008
    previous = joints.copy()
    for index,waypoint in enumerate(path):
        for alpha in np.linspace(0.,1.,11):
            state = dict(previous,**{name:(1-alpha)*previous[name]+alpha*waypoint[name] for name in kin.names})
            expected = start+((index+alpha)/len(path))*(target-start)
            assert np.linalg.norm(kin.forward(state)[0]-expected)<.008
        previous.update(waypoint)
    np.testing.assert_allclose(kin.forward(previous)[0],target,atol=.008)


def test_unreachable_path_is_rejected_before_dispatch():
    kin,joints = fixture()
    with pytest.raises(ValueError,match='unreachable|bounded'):
        kin.cartesian_path([1.5,0.,.2],joints,np.eye(3))
