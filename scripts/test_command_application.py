import pytest
from analyze_command_application import correlate


def application(crc=7):
    return dict(sequence=12,crc=crc,received_monotonic_ns=110_000_000,applied_monotonic_ns=112_000_000)


def publication(before,after):
    return dict(crc=7,publish_before_monotonic_ns=before,publish_after_monotonic_ns=after,leg_target_sha256='target')


def test_repeated_content_yields_bounds_not_a_fabricated_unique_ack():
    report = correlate([application()],[publication(100_000_000,101_000_000),publication(105_000_000,106_000_000)],
                       [dict(timing_valid=True,leg_target_sha256='target',published_monotonic_s=.098,
                             context_matched=True,lowstate_dds_source_to_target_publish_ms=20.)])
    sample = report['samples'][0]
    assert report['ambiguous_lowcmd']==1
    assert sample['receipt_to_apply_ms']==2.
    assert sample['lowcmd_to_apply_lower_ms']==6. and sample['lowcmd_to_apply_upper_ms']==12.
    assert sample['policy_to_apply_lower_ms']==pytest.approx(14.)
    assert sample['dds_source_to_apply_lower_ms']==pytest.approx(34.)


def test_missing_future_and_expired_content_cannot_gain_lineage():
    report = correlate([application(8)],[],[])
    assert report['unmatched_lowcmd']==1
    assert report['metrics']['policy_to_apply_lower_ms']['count']==0
    report = correlate([application()],[publication(111_000_000,112_000_000)],[])
    assert report['unmatched_lowcmd']==1
    report = correlate([application()],[publication(1,2)],[],window_s=.05)
    assert report['unmatched_lowcmd']==1


def test_publication_return_can_follow_dds_receipt_without_negative_latency():
    report = correlate([application()],[publication(100_000_000,114_000_000)],[])
    assert report['samples'][0]['lowcmd_to_apply_lower_ms']==0.
    with pytest.raises(ValueError,match='clock ordering'):
        correlate([dict(application(),applied_monotonic_ns=1)],[],[])


def test_device_age_handles_wrap_and_rejects_reset_without_equating_wall_time():
    policy = dict(timing_valid=True,leg_target_sha256='target',published_monotonic_s=.098,
                  context_matched=True,observation_clock='device_tick',device_tick_ms=0xfffffff0)
    report = correlate([dict(application(),device_tick_ms=20)], [publication(100_000_000,101_000_000)],[policy])
    sample = report['samples'][0]
    assert sample['device_observation_to_apply_lower_ms']==36
    assert sample['policy_to_apply_lower_ms']==pytest.approx(14.)
    policy['device_tick_ms']=1000
    report = correlate([dict(application(),device_tick_ms=20)], [publication(100_000_000,101_000_000)],[policy])
    assert report['metrics']['device_observation_to_apply_lower_ms']['count']==0


def test_traffic_summary_keeps_repeat_and_missing_publication_evidence():
    rows = [dict(publication(100_000_000,101_000_000),sequence=10),
            dict(publication(120_000_000,121_000_000),sequence=12)]
    traffic = correlate([],rows,[])['publication_traffic']
    assert traffic['recorded_rate_hz']==50.
    assert traffic['consecutive_repeat_fraction']==1.
    assert traffic['sequence_gaps']==1
    empty = correlate([],[],[])['publication_traffic']
    assert empty['recorded_rate_hz'] is None and empty['sequence_gaps'] is None
