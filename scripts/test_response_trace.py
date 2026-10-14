import json

import pytest

from analyze_response_trace import analyze,write_report


def row(topic,data,stamp=100.):
    return dict(topic=topic,data=data,stamp=stamp)


def test_planner_tail_and_actual_period_overruns_are_not_budget_exhaustion():
    report = analyze([row('planner_debug',dict(planning_ms=value,planner_period_s=.05,
        planning_budget_ms=45.,search_budget_exhausted=False)) for value in (10.,20.,80.)])
    assert report['metrics']['planner_wall_ms']['median'] == 20.
    assert report['metrics']['planner_wall_ms']['p95'] == pytest.approx(74.)
    assert report['planner_period']['overruns'] == 1
    assert report['planner_budget']['overruns'] == 1


def test_legacy_trace_does_not_fabricate_publication_ages_or_policy_latency():
    report = analyze([row('planner_debug',dict(planning_ms=80.,scan_age_s=.2))])
    assert report['metrics']['scan_age_at_planning_snapshot_s']['median'] == .2
    assert report['metrics']['scan_age_at_command_publish_s']['count'] == 0
    assert report['metrics']['policy_submit_to_publish_ms']['p99'] is None
    assert report['planner_period']['fraction'] is None


def test_duplicate_and_unmatched_policy_context_are_not_counted_twice():
    data = dict(generation=0,sequence=1,submitted_monotonic_s=10.,timing_valid=True,
                context_matched=False,submit_to_publish_ms=20.,command_receipt_age_at_target_publish_s=.1)
    report = analyze([row('policy_timing',data),row('policy_timing',data)])
    assert report['duplicate_policy_events'] == 1
    assert report['metrics']['policy_submit_to_publish_ms']['count'] == 1
    assert report['metrics']['command_receipt_age_at_target_publish_s']['count'] == 0


def test_recovery_ring_deduplicates_heartbeats_but_keeps_restarted_sequences():
    first = dict(sequence=1,stamp=100.,state='policy_deadline_hold',target_jump_rad=.2)
    restarted = dict(first,stamp=110.)
    report = analyze([row('policy_diagnostics',dict(policy_hold_events=[first])),
                      row('policy_diagnostics',dict(policy_hold_events=[first,restarted,{}])),
                      row('policy_diagnostics',dict(policy_hold_events=None))])
    assert report['policy_hold_events']==[first,restarted]
    assert report['invalid_samples']['policy_hold_event']==2
    assert analyze([row('policy_diagnostics',{})])['policy_hold_events']==[]


def test_bad_clock_values_are_visible_not_reported_as_zero_latency():
    report = analyze([row('odom',dict(source_stamp=101.)),
                      row('planner_debug',dict(planning_ms=float('nan')),99.)])
    assert report['ros_receipt_clock_regressions'] == 1
    assert report['invalid_samples']['odom_source_age_at_logger_receipt_s'] == 1
    assert report['invalid_samples']['planner_wall_ms'] == 1
    json.dumps(report,allow_nan=False)


def test_real_time_factor_uses_monotonic_clock_despite_ros_reset():
    report = analyze([dict(row('evaluation_aligned',dict(sim_time=1.),100.),receipt_monotonic_s=10.),
                      dict(row('evaluation_aligned',dict(sim_time=2.),90.),receipt_monotonic_s=11.),
                      dict(row('evaluation_aligned',dict(sim_time=4.),91.),receipt_monotonic_s=12.)])
    assert report['metrics']['rtf_monotonic_receipt_windows']['median'] == 2.
    assert report['ros_receipt_clock_regressions'] == 1


def test_truncated_trace_is_retained_as_diagnostic_error(tmp_path):
    trace = tmp_path/'trace.jsonl'
    trace.write_text(json.dumps(row('planner_debug',dict(planning_ms=5.)))+'\n{"partial":')
    result = write_report(trace,tmp_path/'report.json')
    assert result['malformed_lines'] == 1
    assert result['metrics']['planner_wall_ms']['count'] == 1
    assert result['trace_sha256']


def test_parameter_sidecar_does_not_look_like_a_second_benchmark_result(tmp_path):
    from run_avoidance_matrix import benchmark_artifacts
    result = tmp_path/'person_avoidance_smoke_seed_42_20261005T000000Z.json'
    result.write_text('{}')
    result.with_name(result.stem+'_parameters.json').write_text('{}')
    assert benchmark_artifacts(tmp_path) == [result]


def test_inactive_overruns_do_not_become_active_mission_overruns():
    report = analyze([row('planner_debug',dict(planning_ms=80.,planner_period_s=.05,mission_active=False)),
                      row('planner_debug',dict(planning_ms=10.,planner_period_s=.05,mission_active=True))])
    assert report['planner_period']['overruns'] == 1
    assert report['planner_active_period'] == dict(samples=1,overruns=0,fraction=0.)


def test_command_lineage_joins_by_epoch_and_sequence_not_callback_order():
    supervisor = row('safety_debug',dict(command_envelope_valid=True,command_publisher_id='planner',
        command_sequence_id=3,input_cmd=dict(x=.3,y=0.,z=0.)))
    planner = row('planner_debug',dict(command_sequence=dict(publisher_id='planner',sequence_id=3,first=[.3,0.,0.])))
    report = analyze([supervisor,planner])
    assert report['planner_supervisor_lineage']['matched'] == 1
    planner['data']['command_sequence']['publisher_id'] = 'older_planner'
    assert analyze([supervisor,planner])['planner_supervisor_lineage']['missing_planner_record'] == 1
    planner['data']['command_sequence'].update(publisher_id='planner',first=[.6,0.,0.])
    assert analyze([supervisor,planner])['planner_supervisor_lineage']['command_mismatch'] == 1


def test_async_handoff_time_is_not_reported_as_worker_execution_time():
    report = analyze([row('safety_debug',dict(async_escape_search=True,async_escape_state='revalidated',
        predictive_search_ms=None,escape_control_path_ms=2.,escape_worker=dict(completed=3,last_search_ms=18.)))])
    assert report['metrics']['supervisor_escape_control_path_ms']['median'] == 2.
    assert report['metrics']['supervisor_predictive_search_ms']['count'] == 0
    assert report['escape_worker_latest']['last_search_ms'] == 18.
    assert report['async_escape_states'] == {'revalidated':1}


def test_worker_timing_is_counted_per_completion_not_repeated_status_message():
    def status(completed,duration):
        return row('safety_debug',dict(async_escape_search=True,escape_worker=dict(completed=completed,last_search_ms=duration)))
    report = analyze([status(1,10.),status(1,10.),status(3,30.)])
    assert report['metrics']['escape_worker_submit_to_complete_ms']['count'] == 2
    assert report['metrics']['escape_worker_submit_to_complete_ms']['median'] == 20.
    assert report['unobserved_worker_completion_durations'] == 1
