"""Offline heading attribution must respect acquisition time and angle wrapping."""
import math

import pytest

from analyze_invalidation_heading import diagnose, heading_at
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope
from dataclasses import asdict


def row(stamp, angle, rate=0.):
    return dict(source_stamp=stamp, x=0., y=0., qx=0., qy=0.,
                qz=math.sin(angle/2), qw=math.cos(angle/2), wz=rate)


def test_interpolation_wraps_and_refuses_missing_brackets():
    rows = [row(1., math.pi-.02), row(1.1, -math.pi+.02)]
    assert abs(heading_at(rows, 1.05)) == pytest.approx(math.pi)
    assert heading_at(rows, .9) is None
    assert heading_at(rows, 1.2) is None
    assert heading_at([row(1., 0.), row(2., 1.)], 1.5) is None


def state(stamp, epoch, angle, rate):
    return dict(odom_stamp=stamp, forecast_epoch=epoch, pose=[0., 0., angle],
        velocity=[0., 0., rate], filtered=[0., 0., 0.],
        envelope=asdict(MotionEnvelope()), program=[[1., 0., 0., 0.]],
        horizon=1., dt=.1, delay=0., goal=[0., 0.],
        people=[[[100., 100.]] for _ in range(11)], radii=[[.3] for _ in range(11)])


def test_rate_change_is_separated_from_physical_rotation():
    pair = dict(previous=state(1., 1.1, -.1, -1.), current=state(1.05, 1.15, .1, 1.))
    odom = [row(1., 0., -1.), row(1.05, 0., 1.)]
    truth = [row(1.+i*.05, 0.) for i in range(5)]
    report = diagnose(pair, odom, truth)
    assert report['valid']
    assert report['source_yaw_delta_rad'] == 0.
    assert report['physical_yaw_delta_rad'] == 0.
    assert report['extrapolation_increment_delta_rad'] == pytest.approx(.2)
    assert report['captured_minus_physical_delta_rad'] == pytest.approx(.2)
    assert max(abs(s['reconstruction_yaw_residual_rad']) for s in report['samples']) < 1e-12


def test_missing_exact_odometry_is_not_attributed():
    pair = dict(previous=state(1., 1.1, 0., 0.), current=state(1.05, 1.15, 0., 0.))
    assert not diagnose(pair, [row(.99, 0.)], [])['valid']


def test_missing_truth_keeps_source_decomposition_but_no_physical_claim():
    pair = dict(previous=state(1., 1.1, 0., 0.), current=state(1.05, 1.15, 0., 0.))
    report = diagnose(pair, [row(1., 0.), row(1.05, 0.)], [])
    assert report['valid']
    assert report['physical_yaw_delta_rad'] is None
    assert report['response_minus_physical_delta_rad'] is None
