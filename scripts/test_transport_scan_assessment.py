import json

import pytest

from assess_transport_scan import assess


@pytest.fixture
def retained_run(tmp_path):
    files = {
        'result.json': dict(events=[], contact_samples=0, success=False),
        'manifest.json': dict(people=1, seed=42, pedestrian_start_delay_s=10),
        'response_diagnostics.json': dict(metrics={}, planner_active_period={},
            planner_supervisor_lineage={}, policy_hold_events=[]),
        'effective_parameters.json': {},
    }
    for name, data in files.items():
        (tmp_path/name).write_text(json.dumps(data))
    (tmp_path/'source_snapshot.zip').write_bytes(b'retained source fixture')
    rows = []
    def row(topic, stamp, **data):
        rows.append(dict(topic=topic, stamp=stamp, data=data))
    for t in (1., 2.):
        row('evaluation_aligned', t, sim_time=t, robot=dict(x=t, y=0),
            people=[dict(id=1, x=2.5, y=0, radius_m=.42)])
        row('policy_cmd', t, x=0, y=0, z=0)
        row('planner_debug', t, navigation_mode='brake', mission_active=False)
        row('evaluation_odom', t, source_stamp=t, x=t, y=0)
        # Receipts arrive much later; pairing must use source timestamps.
        row('odom', t+10, source_stamp=t, x=t+.01, y=0)
    (tmp_path/'trace.jsonl').write_text('\n'.join(map(json.dumps, rows)))
    return tmp_path


def test_assessment_preserves_report_and_distinguishes_full_trace_overlap(retained_run):
    report = assess(retained_run)
    assert report['reported_result']['contact_samples'] == 0
    assert report['full_trace_clearance']['overlapping_person_samples'] == 1
    assert report['full_trace_clearance']['minimum_m'] == pytest.approx(-.27)
    assert report['first_translation'] is None
    window = report['mode_windows'][0]
    assert window['measured_world_displacement_m'] == 1.
    assert window['max_policy_planar_command_mps'] == 0.


def test_odometry_comparison_uses_acquisition_time_not_receipt(retained_run):
    comparison = assess(retained_run)['odom_truth_matched_xy']
    assert comparison['samples'] == 2
    assert comparison['max_error_m'] == pytest.approx(.01)
    assert comparison['mean_error_m'] == pytest.approx(.01)
