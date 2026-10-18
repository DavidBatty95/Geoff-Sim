import mujoco
import numpy as np
import pytest

from native_probe_feedback import joint_feedback,joint_sensor_addresses


def model_with_sensors(extra=''):
    return mujoco.MjModel.from_xml_string(f'''<mujoco><option timestep=".002" gravity="0 0 0"/>
      <worldbody><body><joint name="joint" armature=".01"/><geom type="sphere" size=".1" mass="1"/></body></worldbody>
      <actuator><motor joint="joint"/></actuator>
      <sensor><jointpos name="q" joint="joint"/><jointvel name="dq" joint="joint"/>{extra}</sensor></mujoco>''')


def test_sensor_epoch_is_previous_physics_state_after_step():
    model = model_with_sensors()
    data = mujoco.MjData(model)
    data.qpos[0],data.qvel[0],data.ctrl[0] = .1,.3,1.
    mujoco.mj_forward(model,data)
    addresses = joint_sensor_addresses(model,['joint'])
    before = joint_feedback(data,[0],[0],addresses,'state')
    mujoco.mj_step(model,data)
    sensed = joint_feedback(data,[0],[0],addresses,'sensors')
    current = joint_feedback(data,[0],[0],addresses,'state')
    np.testing.assert_array_equal(sensed,before)
    assert current[0][0]!=sensed[0][0] and current[1][0]!=sensed[1][0]
    assert data.time==.002


@pytest.mark.parametrize('kind',['missing','duplicate'])
def test_missing_or_ambiguous_joint_sensor_is_rejected(kind):
    extra = '<jointpos name="other_q" joint="joint"/>' if kind=='duplicate' else ''
    model = model_with_sensors(extra)
    if kind=='missing':
        model.sensor_type[1] = mujoco.mjtSensor.mjSENS_USER
    with pytest.raises(ValueError,match='sensor'):
        joint_sensor_addresses(model,['joint'])
