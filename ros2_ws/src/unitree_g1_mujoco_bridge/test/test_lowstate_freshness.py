"""Frozen DDS traffic must not keep a device-clock policy alive indefinitely."""
from types import SimpleNamespace as S

import pytest
from unitree_hg.msg import LowState

from unitree_g1_mujoco_bridge.policy_timing import DeviceTickFreshness,DeviceTickCadence
from unitree_g1_mujoco_bridge.g1_policy_cmd_vel_controller import G1PolicyCmdVelController
import unitree_g1_mujoco_bridge.g1_policy_cmd_vel_controller as controller


def test_duplicate_and_reordered_packets_do_not_renew_progress():
    progress = DeviceTickFreshness()
    assert progress.observe(1000,10.)
    assert not progress.observe(1000,10.05)
    assert not progress.observe(999,10.09)
    assert progress.age(10.12)==pytest.approx(.12)
    assert progress.duplicates==progress.reordered==1
    assert progress.observe(1003,10.13)
    assert progress.age(10.14)==pytest.approx(.01)


def test_wrap_and_restart_match_cadence_contract():
    progress = DeviceTickFreshness()
    for tick in (0xfffffff0,4,2024,0):
        assert progress.observe(tick,10.)
    assert progress.resets==1 and progress.last==0
    assert not progress.observe(0,10.1)
    assert progress.age(9.)==float('inf')


def fixture(monkeypatch):
    clock = [10.]
    monkeypatch.setattr(controller.time,'monotonic',lambda:clock[0])
    progress = DeviceTickFreshness()
    progress.observe(1000,10.)
    state = LowState()
    state.tick = 1000
    events = []
    node = S(use_device_tick=True,device_tick_freshness=progress,
        device_tick_cadence=DeviceTickCadence(.02),body_joint_names=list(range(29)),
        last_lowstate=state,last_lowstate_arrival=10.,lowstate_count=0,lowstate_stale=False,
        inference_context={'old':True},first_lowstate_time=object(),last_lowstate_dds_delivery={},
        get_clock=lambda:S(now=lambda:S(nanoseconds=int(clock[0]*1e9))),
        get_parameter=lambda key:S(value={'lowstate_timeout':.1,'startup_upright_tilt_threshold':.5}[key]),
        get_logger=lambda:S(error=lambda *a,**k:None,warn=lambda *a,**k:None),
        policy_worker=S(invalidate=lambda:events.append('invalidate')),
        _publish_policy_command=lambda value:events.append(('command',value)),
        _policy_safe_hold=lambda state:events.append(state),
        _lowstate_tilt=lambda msg:0.,_detect_robot_reset=lambda *args:False,
        _update_upright_state=lambda tilt:events.append('upright'),
        _control_tick=lambda:events.append('control'))
    return node,clock,events


@pytest.mark.parametrize('tick',[1000,999])
def test_callback_does_not_replace_state_or_clear_fault_on_old_packet(monkeypatch,tick):
    node,clock,events = fixture(monkeypatch)
    original = node.last_lowstate
    node.lowstate_stale = True
    message = LowState()
    message.tick = tick
    clock[0] = 10.05
    G1PolicyCmdVelController._lowstate_callback(node,message)
    assert node.last_lowstate is original and node.lowstate_stale
    assert node.last_lowstate_arrival==10.05 and not events
    assert node.device_tick_freshness.progress_at==10.


@pytest.mark.parametrize('consume_only',[False,True])
def test_frozen_stream_invalidates_once_before_result_consumption(monkeypatch,consume_only):
    node,clock,events = fixture(monkeypatch)
    clock[0] = 10.2
    node.last_lowstate_arrival = 10.2  # DDS is live, measurement is frozen.
    for _ in range(2):
        G1PolicyCmdVelController._control_tick(node,consume_only=consume_only)
    assert events.count('invalidate')==1 and node.inference_context is None
    assert events.count('frozen_lowstate_hold')==2
    assert events.count(('command',(0.,0.,0.)))==2
    assert node.lowstate_stale
    # A genuinely newer observation releases the fault and triggers fresh work.
    message = LowState()
    message.tick = 1020
    G1PolicyCmdVelController._lowstate_callback(node,message)
    assert node.last_lowstate is message and not node.lowstate_stale
    assert events[-2:]==['upright','control']


def test_missing_messages_keep_existing_stale_hold(monkeypatch):
    node,clock,events = fixture(monkeypatch)
    clock[0] = 10.2
    G1PolicyCmdVelController._control_tick(node)
    assert events[-1]=='stale_lowstate_hold'


def test_legacy_mode_does_not_require_a_progressing_device_tick(monkeypatch):
    node,clock,events = fixture(monkeypatch)
    node.use_device_tick = False
    message = LowState()
    message.tick = 1000
    clock[0] = 10.2
    G1PolicyCmdVelController._lowstate_callback(node,message)
    assert node.last_lowstate is message and events==['upright']
