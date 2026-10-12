from analyze_transport_perception import search_summary,pickup_summary


def test_search_timing_distinguishes_selection_dispatch_and_discovery():
    report = search_summary([
        dict(phase='search_request',object_id='box_red',elapsed_s=5.,initially_visible=False),
        dict(phase='search_viewpoint',object_id='box_red',elapsed_s=7.,viewpoint_map=[1.,0.],
             planning_ms=120.,selection_wait_ms=40.),
        dict(phase='search_viewpoint',elapsed_s=7.1,goal_odom=[1.,0.,0.]),
        dict(phase='search_found',object_id='box_red',elapsed_s=24.),
        dict(phase='search_request',object_id='target_red',elapsed_s=100.,initially_visible=False),
    ])
    assert report['selected_viewpoints']==1
    assert report['requests'][0]['first_goal_after_s']==2.
    assert report['requests'][0]['found_after_s']==19.
    assert report['requests'][1]['found_after_s'] is None
    assert report['planning_ms']['median']==120.
    assert report['selection_wait_ms']['median']==40.


def test_absent_search_events_do_not_imply_discovery():
    report = search_summary([])
    assert report['requests']==[] and report['selected_viewpoints']==0
    assert report['planning_ms']==dict(count=0) and report['selection_wait_ms']==dict(count=0)
    assert report['room_decisions']==0 and report['route_planning_ms']==dict(count=0)


def test_grasp_or_lift_request_alone_is_not_verified_pickup():
    events = [dict(phase='search_found',object_id='box_red',elapsed_s=5.),
              dict(phase='grasp',perception_source='registered_rgbd',elapsed_s=12.),
              dict(phase='grasp',attachment_state='held',elapsed_s=12.2),
              dict(phase='lift',elapsed_s=12.3)]
    row = pickup_summary(events)[0]
    assert row['grasp_request_after_s']==7.
    assert row['held_ack_after_s']==12.2-5.
    assert row['verified_lift_after_s'] is None
    assert row['discovery_to_placement_s'] is None and row['grasp_to_placement_s'] is None
    events.append(dict(phase='retreat_pickup',elapsed_s=16.))
    assert pickup_summary(events)[0]['verified_lift_after_s']==11.
    events.append(dict(phase='delivered',object_id='box_red',elapsed_s=40.))
    assert pickup_summary(events)[0]['discovery_to_placement_s']==35.
    assert pickup_summary(events)[0]['grasp_to_placement_s']==40.-12.2
