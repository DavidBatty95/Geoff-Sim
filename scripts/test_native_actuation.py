import mujoco
import numpy as np
import pytest

from compare_native_actuation import step_variant


@pytest.mark.parametrize('target',[.2,10.,-10.])
def test_equal_instantaneous_pd_force_does_not_imply_equal_integrated_motion(target):
    model = mujoco.MjModel.from_xml_string('''<mujoco><option timestep=".002" gravity="0 0 0"/>
      <worldbody><body><joint name="hinge" armature=".01"/>
      <geom type="sphere" size=".1" mass="1"/></body></worldbody>
      <actuator><motor joint="hinge" ctrllimited="true" ctrlrange="-100 100"/></actuator></mujoco>''')
    data = mujoco.MjData(model)
    data.qpos[0],data.qvel[0] = .1,.3
    state = np.empty(mujoco.mj_stateSize(model,mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_getState(model,data,state,mujoco.mjtState.mjSTATE_INTEGRATION)
    args = (model,state,np.array([target]),np.array([0]),np.array([0]),np.array([0]),np.array([160.]),np.array([4.]))
    native = step_variant(*args)
    euler = step_variant(*args,position=True)
    implicit = step_variant(*args,position=True,implicit=True)
    np.testing.assert_allclose(euler['force'],native['force'],atol=1e-12)
    np.testing.assert_allclose(implicit['force'],native['force'],atol=1e-12)
    np.testing.assert_allclose(euler['qvel'],native['qvel'],atol=1e-12)
    if target==.2:
        assert np.max(np.abs(implicit['qvel']-native['qvel']))>.1
    else:
        assert abs(native['force'][0])==100.
