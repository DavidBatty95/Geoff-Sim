import numpy as np

from unitree_g1_mujoco_bridge.static_distance_field import StaticFieldCache


def test_same_geometry_reuses_field_but_new_obstacle_revokes_clearance():
    cache = StaticFieldCache()
    field = cache.get([(2.,0.)],(0.,0.),3.,1.)
    assert not field.trajectory_is_blocked(np.array(((0.,0.),(.5,0.))),.3)
    assert cache.get([(2.,0.)],(.01,0.),3.,1.) is field
    replacement = cache.get([(2.,0.),(.4,0.)],(.01,0.),3.,1.)
    assert replacement is not field
    assert replacement.trajectory_is_blocked(np.array(((0.,0.),(.5,0.))),.3)
    assert cache.get([(2.,0.)],(.01,0.),3.,1.) is not replacement
