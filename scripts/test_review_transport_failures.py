import json

from review_transport_failures import review


def test_review_separates_overlap_fall_and_post_failure_timing(tmp_path):
    rows = [
        dict(receipt_monotonic_s=100.,topic='planner_debug',data=dict(state='EMERGENCY_STOP',planning_ms=60.,track_count=1)),
        dict(receipt_monotonic_s=101.,topic='evaluation_aligned',data=dict(robot=dict(x=0.,y=0.),people=[dict(x=.5,y=0.,radius_m=.42)])),
        dict(receipt_monotonic_s=102.,topic='evaluation_odom',data=dict(qx=0.,qy=0.,qz=0.,qw=1.,z=.3)),
        dict(receipt_monotonic_s=103.,topic='evaluation_aligned',data=dict(robot=dict(x=0.,y=0.),people=[dict(x=.5,y=0.,radius_m=.42)])),
        dict(receipt_monotonic_s=104.,topic='planner_debug',data=dict(planning_ms=2.)),
        dict(receipt_monotonic_s=105.,topic='transport_state',data=dict(elapsed=5.)),
    ]
    (tmp_path/'trace.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    (tmp_path/'result.json').write_text(json.dumps(dict(success=False,fallen=True,error='localization timeout',
        events=[dict(phase='search_route',elapsed_s=0.)])))
    report = review(tmp_path)
    assert report['overlap_to_fall_s']==1.
    assert report['overlap_samples_before_first_fall']==report['overlap_samples_after_first_fall']==1
    assert report['planner_before_first_fall']['p95_ms']==60.
    assert report['planner_after_first_fall']['p95_ms']==2.
    assert report['phase_time_s']['search_route']==5.


def test_failure_label_does_not_fabricate_fall_or_contact(tmp_path):
    (tmp_path/'trace.jsonl').write_text(json.dumps(dict(receipt_monotonic_s=1.,topic='transport_state',data=dict(elapsed=2.)))+'\n')
    (tmp_path/'result.json').write_text(json.dumps(dict(success=False,fallen=False,error='localization timeout')))
    report = review(tmp_path)
    assert report['first_geometric_person_overlap_s'] is None
    assert report['first_fall_threshold_s'] is None
    assert report['overlap_to_fall_s'] is None
