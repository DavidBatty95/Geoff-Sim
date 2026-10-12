import numpy as np

from unitree_g1_mujoco_bridge.wall_evidence import WallEvidence


def test_rotated_wall_continuation_rejects_wall_but_not_person_in_front():
    for angle in (0.,.4,1.2):
        direction = np.array((np.cos(angle),np.sin(angle)))
        normal = np.array((-direction[1],direction[0]))
        centre = np.array((3.,-2.))
        xy = centre+np.tile(np.linspace(-.4,.4,12),8)[:,None]*direction
        points = np.column_stack((xy,np.repeat(np.linspace(2.4,3.2,8),12)))
        evidence = WallEvidence(points,2.3)
        assert evidence.supports(*centre)
        assert not evidence.supports(*(centre+.3*normal))


def test_stationary_or_sparse_person_is_not_wall_evidence():
    rng = np.random.default_rng(48)
    person = np.column_stack((rng.uniform(-.25,.25,(200,2)),rng.uniform(.1,1.85,200)))
    assert not WallEvidence(person,2.15).supports(0.,0.)
    assert not WallEvidence([(0.,0.,2.2)]*20,2.15).supports(0.,0.)
    assert not WallEvidence([],2.15).supports(0.,0.)


def test_unstructured_high_returns_do_not_prove_a_wall():
    points = [(x,y,2.18) for x in (-.8,0.,.8) for y in (-.8,0.,.8)]
    assert not WallEvidence(points,2.15).supports(0.,0.)


def test_overhead_beam_does_not_erase_a_person_beneath_it():
    points = [(x,0.,3.2) for x in np.linspace(-1.3,1.3,30)]
    assert not WallEvidence(points,2.3).supports(0.,0.)


def test_wall_returns_are_retained_separately_and_replays_cannot_replace_them():
    from types import SimpleNamespace as S
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header
    from unitree_g1_mujoco_bridge.dynamic_obstacle_tracker import DynamicObstacleTracker
    published = []
    tracker = S(target_frame='odom',last_processed_stamp=0.,reject_wall_supported_clusters=True,
        min_range=.1,max_range=10.,ground_rejection_height=-.03,min_height=.05,max_height=2.2,
        max_person_height=2.15,max_accumulated_points=100,debug_events=[],
        _lookup_robot_xy=lambda _: (0.,0.),_outside_local_tracking_radius=lambda *args:False,
        _iter_cloud_points=lambda _:iter([(1.,0.,1.7,False),(1.,0.,3.,False),(1.,0.,3.,True)]),
        _publish_points=lambda *args:published.append(args))
    msg = PointCloud2(header=Header(frame_id='odom',stamp=Time(sec=1)))
    DynamicObstacleTracker._cloud_callback(tracker,msg)
    assert published[0][0]==[(1.,0.,1.7)]
    assert tracker.wall_observation==(1.,'odom',[(1.,0.,3.)])
    tracker.last_processed_stamp = 1.
    tracker._iter_cloud_points = lambda _:iter([(9.,0.,3.,False)])
    DynamicObstacleTracker._cloud_callback(tracker,msg)
    assert tracker.wall_observation==(1.,'odom',[(1.,0.,3.)]) and len(published)==1
    msg.header.stamp = Time(sec=2)
    DynamicObstacleTracker._cloud_callback(tracker,msg)
    assert tracker.wall_observation==(2.,'odom',[(9.,0.,3.)])
    assert len(tracker.wall_history)==1
