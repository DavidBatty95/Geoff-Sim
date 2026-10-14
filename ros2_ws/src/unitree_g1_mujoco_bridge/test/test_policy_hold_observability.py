from collections import deque
from types import SimpleNamespace as S

from unitree_g1_mujoco_bridge.g1_policy_cmd_vel_controller import G1PolicyCmdVelController


def hold_fixture():
    resets,published = [],[]
    node = S(control_state='policy',policy_hold_count=0,policy_hold_events=deque(maxlen=16),
             inference_hold_started=None,policy_inference_deadline=.05,
             last_safe_targets=[.2]*29,last_lowstate=S(tick=1234),
             filtered_command=[0.,0.,.6],last_action_norm=2.,last_tilt=.08,
             unitree_rl_lab_adapter=S(reset=lambda:resets.append(True)),
             get_clock=lambda:S(now=lambda:S(nanoseconds=10_000_000_000)),
             _safe_hold_positions=lambda:[.1]*29,_publish_targets=published.append,
             lower_joint_indexes=list(range(12)),body_joint_names=list(range(29)))
    return node,resets,published


def test_state_fault_hold_preserves_pre_reset_evidence():
    node,resets,published = hold_fixture()
    G1PolicyCmdVelController._policy_safe_hold(node,'policy_deadline_hold')
    event = node.policy_hold_events[-1]
    assert event['command_before_hold']==[0.,0.,.6]
    assert event['device_tick_ms']==1234 and event['stamp']==10.
    assert event['action_norm_before_hold']==2. and event['target_jump_rad']==.1
    assert event['history_reset'] and resets==[True]
    assert node.filtered_command==[0.,0.,0.] and node.last_actions[29]==[0.]*29
    assert published==[[.1]*29]
    # Repeated ticks in one hold do not flood evidence; a later new hold does.
    G1PolicyCmdVelController._policy_safe_hold(node,'policy_deadline_hold')
    assert node.policy_hold_count==1
    for _ in range(20):
        node.control_state='policy'
        G1PolicyCmdVelController._policy_safe_hold(node,'policy_deadline_hold')
    assert node.policy_hold_count==21 and len(node.policy_hold_events)==16
    assert node.policy_hold_events[0]['sequence']==6


def test_inference_dropout_holds_validated_target_and_matching_policy_state():
    node,resets,published = hold_fixture()
    node.last_safe_targets = [1.15]*29
    previous_actions = {12:[.4]*12,29:[.7]*29}
    node.last_actions = previous_actions
    node.gait_phase,node.first_policy_time = .3,123.
    for state in ('policy_deadline_hold','policy_result_stale_hold'):
        node.control_state = 'policy'
        G1PolicyCmdVelController._policy_safe_hold(node,state,reset_history=False)
        assert published[-1]==[1.15]*29 and not resets
        assert node.last_actions is previous_actions and node.last_action_norm==2.
        assert node.gait_phase==.3 and node.first_policy_time==123.
        assert node.policy_hold_events[-1]['target_jump_rad']==0.
        assert not node.policy_hold_events[-1]['history_reset']
    # A subsequent sensor-clock reset still clears history and uses state hold.
    G1PolicyCmdVelController._policy_safe_hold(node,'device_clock_reset_hold')
    assert resets==[True] and published[-1]==[.1]*29
    assert node.last_actions[29]==[0.]*29


def test_inference_hold_without_any_validated_target_uses_state_fallback():
    node,resets,published = hold_fixture()
    node.last_safe_targets = None
    G1PolicyCmdVelController._policy_safe_hold(node,'policy_deadline_hold',reset_history=False)
    assert published==[[.1]*29] and not resets
    assert node.policy_hold_events[-1]['target_jump_rad'] is None


def test_sustained_inference_failure_escalates_without_restart_of_budget(monkeypatch):
    import unitree_g1_mujoco_bridge.g1_policy_cmd_vel_controller as module
    clock = [10.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    node,resets,published = hold_fixture()
    G1PolicyCmdVelController._policy_safe_hold(node,'policy_deadline_hold',reset_history=False)
    clock[0] += .06
    G1PolicyCmdVelController._policy_safe_hold(node,'policy_result_stale_hold',reset_history=False)
    assert not resets and published[-1]==[.2]*29
    clock[0] += .05
    G1PolicyCmdVelController._policy_safe_hold(node,'policy_result_stale_hold',reset_history=False)
    assert resets==[True] and published[-1]==[.1]*29
    assert node.policy_hold_events[-1]['recovery_kind']=='state_hold'
    assert node.inference_hold_started==10.
