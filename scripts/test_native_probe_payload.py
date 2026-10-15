"""Run with the probe's pinned MuJoCo environment."""
from pathlib import Path

import mujoco
import numpy as np

from native_probe_payload import attach_payload,payload_sample


def test_native_payload_has_mass_and_pose_consistent_weld():
    root = Path(__file__).resolve().parents[1]
    model = mujoco.MjModel.from_xml_path(str(root/'ros2_ws/src/unitree_mujoco/unitree_robots/g1/scene_29dof_transport_transport_dual.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('transport_unitree_rl_lab_stand').id)
    mujoco.mj_forward(model,data)
    payload = attach_payload(model,data)
    assert payload['mass_kg']==.15
    sample = payload_sample(model,data,payload)
    assert sample['payload_attached'] and sample['payload_position_error_m']<1e-12
    # The initialized weld has no translational/rotational constraint error.
    rows = (data.efc_type==mujoco.mjtConstraint.mjCNSTR_EQUALITY)&(data.efc_id==payload['weld'])
    assert np.count_nonzero(rows)==6
    np.testing.assert_allclose(data.efc_pos[rows],0.,atol=1e-12)
    data.eq_active[payload['weld']] = False
    assert not payload_sample(model,data,payload)['payload_attached']
