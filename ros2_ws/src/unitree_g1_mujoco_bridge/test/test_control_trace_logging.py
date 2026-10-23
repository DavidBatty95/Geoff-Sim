import json
from types import SimpleNamespace

from unitree_hg.msg import LowCmd,LowState
from unitree_g1_mujoco_bridge.human_aware_run_logger import HumanAwareRunLogger


def test_native_control_messages_are_json_serializable_including_float32_imu():
    records = []
    node = SimpleNamespace(_write=lambda topic,data:records.append(json.dumps(dict(topic=topic,data=data))))
    state = LowState()
    state.tick = 42
    state.imu_state.quaternion[0] = 1.
    HumanAwareRunLogger._lowstate_callback(node,state)
    HumanAwareRunLogger._lowcmd_callback(node,LowCmd())
    decoded = [json.loads(row) for row in records]
    assert decoded[0]['data']['device_tick_ms']==42
    assert decoded[0]['data']['quaternion']==[1.,0.,0.,0.]
    assert len(decoded[1]['data']['motors'])==29
