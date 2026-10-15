import numpy as np
import pytest

from unitree_g1_mujoco_bridge.lidar_odometry import ScanOdometry, registration_points, rotation
from unitree_g1_mujoco_bridge.lidar_odometry import registration_displacement_limit


def room():
    axis = np.linspace(-4, 4, 500)
    return np.concatenate((np.column_stack((axis, np.full_like(axis, 4))),
                           np.column_stack((np.full_like(axis, 4), axis)),
                           np.column_stack((axis, np.full_like(axis, -4))),
                           np.column_stack((np.full_like(axis, -4), axis))))


def test_estimates_translation_with_heading_and_moving_outliers():
    rng = np.random.default_rng(6)
    world = room()
    odom = ScanOdometry()
    assert odom.update(world, 0.7)
    for index in range(1, 20):
        xy = np.array((index * 0.04, index * -0.025))
        yaw = index * 0.008
        local = (world - xy) @ rotation(yaw)
        local += rng.normal(0, 0.007, local.shape)
        local = np.concatenate((local, rng.uniform(-3, 3, (80, 2))))
        assert odom.update(local, 0.7 + yaw)
        assert np.linalg.norm(odom.xy - xy) < 0.12


def test_failed_overlap_cannot_update_pose_or_reference():
    odom = ScanOdometry()
    assert odom.update(room(), 0.)
    previous = odom.reference.copy()
    assert not odom.update(room() + 50, 0.)
    np.testing.assert_array_equal(odom.xy, (0., 0.))
    np.testing.assert_array_equal(odom.reference, previous)
    assert not odom.update(np.empty((0, 2)), 0.)


def test_full_band_recovers_when_upper_structure_changes_visibility():
    odom = ScanOdometry()
    world = room()
    assert odom.update(world,0.,fallback_points=world*.7)
    translation = np.array((.1,-.04))
    assert odom.update(world+20.,0.,fallback_points=world*.7-translation)
    assert odom.quality['registration_fallback']
    np.testing.assert_allclose(odom.xy,translation,atol=.02)
    previous = odom.xy.copy()
    assert not odom.update(world+100.,0.,fallback_points=world+100.)
    np.testing.assert_array_equal(odom.xy,previous)


def test_point_to_plane_rejects_tangential_resampling_bias():
    axis = np.arange(-3.8,3.8,.19)
    def walls(shift):
        tangent = axis+shift
        return np.concatenate((np.column_stack((tangent,np.full_like(tangent,4.))),
            np.column_stack((tangent,np.full_like(tangent,-4.))),
            np.column_stack((np.full_like(tangent,4.),tangent)),
            np.column_stack((np.full_like(tangent,-4.),tangent))))
    odom = ScanOdometry()
    assert odom.update(walls(0.),0.)
    for index in range(1,20):
        xy = np.array((.02*index,-.01*index))
        assert odom.update(walls(.07)-xy,0.)
        assert np.linalg.norm(odom.xy-xy) < .01
        assert odom.quality['method'] == 'point_to_plane'


def test_upper_structure_registration_excludes_low_returns_with_fallback():
    walls = np.column_stack((room(),np.full(len(room()),2.5)))
    low = np.column_stack((room()*.5,np.ones(len(room()))))
    selected,band = registration_points(np.concatenate((walls,low)))
    assert band == 'upper_structure'
    np.testing.assert_array_equal(selected,walls[:,:2])
    selected,band = registration_points(low)
    assert band == 'full_band'
    np.testing.assert_array_equal(selected,low[:,:2])


def test_short_interval_pose_jump_is_rejected_without_poisoning_reference():
    world = room()
    odom = ScanOdometry()
    assert odom.update(world,0.)
    reference = odom.reference.copy()
    assert not odom.update(world-(.20,0.),0.,elapsed_s=.04)
    assert odom.quality['reason']=='translation_exceeds_motion_bound'
    assert odom.quality['translation_limit_m']==pytest.approx(.09)
    assert odom.quality['translation_m']>.18
    np.testing.assert_array_equal(odom.xy,(0.,0.))
    np.testing.assert_array_equal(odom.reference,reference)
    # A later scan uses time since the last ACCEPTED acquisition, not receipt
    # or the rejected frame; real accumulated motion can still reacquire.
    assert odom.update(world-(.20,0.),0.,elapsed_s=.20)
    np.testing.assert_allclose(odom.xy,(.20,0.),atol=.02)


def test_full_band_fallback_cannot_bypass_motion_bound():
    world = room()
    odom = ScanOdometry()
    assert odom.update(world,0.,fallback_points=world*.7)
    reference = odom.fallback.reference.copy()
    assert not odom.update(world+20.,0.,fallback_points=world*.7-(.20,0.),elapsed_s=.04)
    assert odom.quality['fallback_rejection']['reason']=='translation_exceeds_motion_bound'
    np.testing.assert_array_equal(odom.xy,(0.,0.))
    np.testing.assert_array_equal(odom.fallback.reference,reference)
    assert odom.update(world+20.,0.,fallback_points=world*.7-(.20,0.),elapsed_s=.20)
    assert odom.quality['registration_fallback']


def test_registration_gate_keeps_normal_walk_and_original_long_gap_cap():
    world = room()
    odom = ScanOdometry()
    assert odom.update(world,0.)
    for i in range(1,12):
        xy = np.array((.04*i,-.02*i))
        assert odom.update(world-xy,0.,elapsed_s=.07)
        np.testing.assert_allclose(odom.xy,xy,atol=.03)
    assert registration_displacement_limit(20.)==.30
    assert registration_displacement_limit(None)==.30


@pytest.mark.parametrize('elapsed',[0.,-.1,float('nan'),float('inf')])
def test_invalid_acquisition_intervals_are_rejected(elapsed):
    with pytest.raises(ValueError,match='acquisition interval'):
        registration_displacement_limit(elapsed)
