import math

import pytest

from unitree_g1_mujoco_bridge.heading_history import HeadingHistory


def add(history, stamp, yaw, rate=0., tick=None):
    return history.observe(stamp, stamp+.001, int(stamp*1000) if tick is None else tick,
                           (0.,0.,yaw), (0.,0.,rate))


def test_history_tracks_rotation_instead_of_extrapolating_old_rate():
    history=HeadingHistory()
    for i in range(21):
        stamp=1.+i*.01
        assert add(history,stamp,.5*(stamp-1.),.5)
    result,details=history.propagate((0.,0.,1.),1.05,1.22)
    assert result==pytest.approx((1.085,.5))
    assert details['clock']=='dds_source_publication_not_acquisition'
    assert details['valid']


def test_wrap_and_body_gyro_to_euler_rate():
    history=HeadingHistory()
    assert history.observe(1.,1.001,1,(.2,.3,math.pi-.01),(0.,.4,.5))
    assert history.observe(1.02,1.021,2,(.2,.3,-math.pi+.01),(0.,.4,.5))
    result,_=history.propagate((0.,0.,2.),1.01,1.02)
    assert result[0]==pytest.approx(2.01)
    assert result[1]==pytest.approx((math.sin(.2)*.4+math.cos(.2)*.5)/math.cos(.3))


def test_duplicate_tick_and_late_or_missing_metadata_cannot_refresh():
    history=HeadingHistory()
    assert add(history,1.,0.,tick=1)
    assert not add(history,1.01,.1,tick=1)
    assert not history.observe(0.,1.02,2,(0.,0.,0.),(0.,0.,0.))
    assert not history.observe(1.01,1.2,2,(0.,0.,0.),(0.,0.,0.))
    assert history.propagate((0.,0.,0.),1.,1.2)[0] is None


def test_gaps_and_unbracketed_source_fail_closed():
    history=HeadingHistory()
    add(history,1.,0.)
    add(history,1.1,.1)
    assert history.propagate((0.,0.,0.),1.05,1.11)[1]['reason']=='attitude_gap'
    assert history.propagate((0.,0.,0.),.9,1.11)[0] is None


def test_future_regressing_nonfinite_and_tilted_samples_rejected():
    history=HeadingHistory()
    assert add(history,1.,0.)
    assert not add(history,.99,0.)
    for stamp,receipt,rpy in [(1.02,1.01,(0.,0.,0.)),(1.02,1.03,(0.,0.,float('nan'))),
                              (1.02,1.03,(0.,.7,0.))]:
        assert not history.observe(stamp,receipt,2000,rpy,(0.,0.,0.))
