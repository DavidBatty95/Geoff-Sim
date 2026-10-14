import json

from publish_performance_evidence import attempt,training_attempt


def test_verified_placement_does_not_upgrade_failed_mission(tmp_path):
    result = dict(success=False,mission_complete=False,data_valid=True,fallen=False,contact_samples=0,
                  deliveries=[{'verified':True}],events=[{'phase':'delivered','elapsed_s':40.}],
                  error='deadline during restoration')
    (tmp_path/'result.json').write_text(json.dumps(result))
    record = attempt(tmp_path)
    assert not record['success'] and record['completion_s'] is None
    assert record['deliveries'][0]['verified'] and record['error']==result['error']


def test_missing_result_is_retained_without_fabricated_evaluator_data(tmp_path):
    record = attempt(tmp_path)
    assert not record['artifact_complete'] and not record['success']
    assert record['data_valid'] is None and record['contacts'] is None


def test_application_summary_retains_ambiguity_and_source_identity(tmp_path):
    data = dict(content_transitions_only=True,ambiguous_lowcmd=3,
                metrics={'receipt_to_apply_ms':{'count':10}},samples=[{'crc':1}])
    (tmp_path/'command_application_diagnostics.json').write_text(json.dumps(data))
    record = attempt(tmp_path)
    assert record['command_application']['ambiguous_lowcmd']==3
    assert record['command_application']['content_transitions_only']
    assert 'samples' not in record['command_application']
    assert record['command_application_sha256'] is not None


def test_training_export_does_not_upgrade_incomplete_artifacts_or_smoke_to_acceptance(tmp_path):
    (tmp_path/'manifest.json').write_text(json.dumps(dict(mode='smoke',iterations=2)))
    record = training_attempt(tmp_path)
    assert not record['artifact_complete'] and record['policy_sha256'] is None
    assert record['manifest']['mode']=='smoke' and 'success' not in record
    assert record['source_sha256']['payload_sensor_delay.py'] is None
