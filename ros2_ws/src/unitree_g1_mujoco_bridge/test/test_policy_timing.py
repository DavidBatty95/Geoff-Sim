from types import SimpleNamespace

import pytest

from unitree_g1_mujoco_bridge.policy_timing import inference_timing,result_is_stale,dds_delivery_timing,leg_target_fingerprint


def fixture():
    return (SimpleNamespace(generation=2,sequence=7,submitted_at=10.,completed_at=10.004),
            dict(key=(2,7),command=[.3,0.,0.],command_received_ros_s=1000.,
                 lowstate_receipt_age_at_submit_s=.002))


def test_timing_uses_monotonic_durations_and_explicit_ros_receipt_age():
    result,context = fixture()
    event = inference_timing(result,context,10.020,1000.035)
    assert event['submit_to_complete_ms'] == pytest.approx(4.)
    assert event['complete_to_publish_ms'] == pytest.approx(16.)
    assert event['submit_to_publish_ms'] == pytest.approx(20.)
    assert event['command_receipt_age_at_target_publish_s'] == pytest.approx(.035)
    assert event['lowstate_receipt_age_at_target_publish_s'] == pytest.approx(.022)
    assert event['role'] == 'policy_joint_target_publication_not_actuator_ack'


def test_actual_device_interval_remains_distinct_from_bounded_filter_dt():
    result,context = fixture()
    context.update(observation_dt_s=.24,observation_filter_dt_s=.1,
                   observation_period_s=.02,observation_cadence='minimum_spaced')
    event = inference_timing(result,context,10.02,1000.035)
    assert event['observation_dt_s']==.24 and event['observation_filter_dt_s']==.1
    assert event['observation_cadence']=='minimum_spaced'
    context['key'] = (0,0)
    assert 'observation_dt_s' not in inference_timing(result,context,10.02,1000.035)


def test_old_generation_never_inherits_a_new_commands_context():
    result,context = fixture()
    context['key'] = (3,7)
    event = inference_timing(result,context,10.020,1000.035)
    assert not event['context_matched']
    assert 'command_at_submit' not in event
    assert 'command_receipt_age_at_target_publish_s' not in event


def test_clock_regression_is_not_clamped_into_a_fast_measurement():
    result,context = fixture()
    event = inference_timing(result,context,9.,999.)
    assert not event['timing_valid']
    assert 'submit_to_publish_ms' not in event
    assert event['command_receipt_age_at_target_publish_s'] is None


def test_completed_inference_cannot_be_applied_after_a_delayed_control_tick():
    result,context = fixture()
    assert not result_is_stale(result,10.02,.05)
    assert result_is_stale(result,10.1,.05)
    assert result_is_stale(result,9.,.05)
    assert result_is_stale(result,float('nan'),.05)


def test_dds_delivery_exposes_queue_age_hidden_by_callback_receipt_time():
    result,context = fixture()
    context['lowstate_dds_delivery'] = dds_delivery_timing(999_970_000_000,999_990_000_000,1000.)
    event = inference_timing(result,context,10.02,1000.035)
    assert event['lowstate_dds_source_to_callback_ms']==pytest.approx(30.)
    assert event['lowstate_dds_receive_to_callback_ms']==pytest.approx(10.)
    assert event['lowstate_dds_source_to_target_publish_ms']==pytest.approx(65.)
    assert event['lowstate_receipt_age_at_target_publish_s']==pytest.approx(.022)
    context['key'] = (3,7)
    assert 'lowstate_dds_source_to_callback_ms' not in inference_timing(result,context,10.02,1000.035)


def test_missing_and_future_dds_timestamps_are_not_fabricated_latencies():
    assert dds_delivery_timing(0,0,1000.)=={}
    assert dds_delivery_timing(1001_000_000_000,-1,1000.)=={}
    assert dds_delivery_timing(float('nan'),float('inf'),1000.)=={}


def test_target_lineage_matches_wire_float_rounding_and_excludes_arm_composition():
    import struct
    targets = [.123456789]*29
    wire = list(struct.unpack('<29f',struct.pack('<29f',*targets)))
    wire[15:] = [.8]*14
    assert leg_target_fingerprint(targets)==leg_target_fingerprint(wire)
    wire[0] += .01
    assert leg_target_fingerprint(targets)!=leg_target_fingerprint(wire)
    with pytest.raises(ValueError):
        leg_target_fingerprint([float('nan')]*15)
