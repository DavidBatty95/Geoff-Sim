import json
import sys
import pytest

from publish_performance_evidence import attempt,training_attempt
import publish_performance_evidence as publisher


def test_verified_placement_does_not_upgrade_failed_mission(tmp_path):
    result = dict(success=False,mission_complete=False,data_valid=True,fallen=False,contact_samples=0,
                  attachment_state='held',object_may_be_held=True,
                  deliveries=[{'verified':True}],events=[{'phase':'delivered','elapsed_s':40.}],
                  error='deadline during restoration')
    (tmp_path/'result.json').write_text(json.dumps(result))
    record = attempt(tmp_path)
    assert not record['success'] and record['completion_s'] is None
    assert record['deliveries'][0]['verified'] and record['error']==result['error']
    assert record['attachment_state']=='held' and record['object_may_be_held']


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
    assert record['transition_audit']=={} and record['transition_audit_sha256'] is None
    audit = dict(move_to_stop_requests=23,scope='sampled requests, not completed transitions')
    (tmp_path/'transition_audit.json').write_text(json.dumps(audit))
    record = training_attempt(tmp_path)
    assert record['transition_audit']==audit and record['transition_audit_sha256'] is not None
    assert not record['artifact_complete'] and 'success' not in record


def test_prefix_export_keeps_failures_and_records_selection(tmp_path,monkeypatch):
    results = tmp_path/'results'
    for name,success in [('20261015T170001Z-pass',True),('20261015T170002Z-failure',False),
                         ('20261015T150000Z-older',True)]:
        run = results/'transport_demo'/name
        run.mkdir(parents=True)
        (run/'result.json').write_text(json.dumps(dict(success=success,mission_complete=success,
            data_valid=True,fallen=False,contact_samples=0,minimum_person_clearance_m=.8,
            events=[dict(phase='dock',dock_workspace_interruption=True,elapsed_s=1.)])))
    output = tmp_path/'report.json'
    monkeypatch.setattr(publisher,'RESULTS',results)
    monkeypatch.setattr(sys,'argv',['publisher','--date','20261015','--run-prefix','20261015T17','--output',str(output)])
    publisher.main()
    report = json.loads(output.read_text())
    assert len(report['attempts'])==2
    assert [row['success'] for row in report['attempts']]==[True,False]
    assert all(row['workspace_recovery_events'] for row in report['attempts'])
    assert report['attempts'][1]['minimum_person_clearance_m']==.8
    assert report['selection']==dict(run_prefix='20261015T17',includes_probes_and_training=False)
    assert report['training_runs']==report['probes']==[]


@pytest.mark.parametrize('prefix',['../','20261014T17','20261015T17*'])
def test_prefix_export_rejects_paths_wildcards_and_other_dates(tmp_path,monkeypatch,prefix):
    monkeypatch.setattr(sys,'argv',['publisher','--date','20261015','--run-prefix',prefix,'--output',str(tmp_path/'out.json')])
    with pytest.raises(SystemExit):
        publisher.main()
