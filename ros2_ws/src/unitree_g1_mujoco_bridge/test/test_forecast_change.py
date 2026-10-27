from types import SimpleNamespace as S

import numpy as np
import pytest

from unitree_g1_mujoco_bridge.forecast_change import ForecastChangeReserve


def track(identity,vx):
    return S(track_id=identity,vx=vx,vy=0.)


def test_reserve_uses_only_recent_sensed_changes_and_includes_hold_time():
    guard=ForecastChangeReserve()
    guard.observe(10.,[track(1,.4)])
    guard.observe(10.1,[track(1,.5)])
    np.testing.assert_allclose(guard.reserve([track(1,.5)],[0.,1.,4.5],10.2)[:,0],[.14,.21,.35])
    assert not np.any(guard.reserve([track(1,.5)],[0.,1.,4.5],10.7))


def test_identity_order_and_regressing_updates_do_not_mix_innovations():
    guard=ForecastChangeReserve()
    guard.observe(10.,[track(1,.4),track(2,.2)])
    guard.observe(10.1,[track(2,.4),track(1,.5)])
    guard.observe(9.,[track(1,2.),track(2,2.)])
    values=guard.reserve([track(2,.4),track(1,.5)],[0.],10.2)[0]
    np.testing.assert_allclose(values,[.34,.14])


def test_minor_jitter_and_reborn_identity_do_not_inherit_a_large_reserve():
    guard=ForecastChangeReserve()
    guard.observe(10.,[track(1,.1)])
    guard.observe(10.1,[track(1,.11)])
    assert guard.reserve([track(1,.11)],[4.5],10.2)[0,0]==0.
    guard.observe(10.2,[])
    guard.observe(10.3,[track(1,1.)])
    assert guard.reserve([track(1,1.)],[4.5],10.4)[0,0]==0.
