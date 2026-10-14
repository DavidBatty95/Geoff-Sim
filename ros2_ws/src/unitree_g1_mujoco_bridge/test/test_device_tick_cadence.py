import pytest

from unitree_g1_mujoco_bridge.policy_timing import DeviceTickCadence


def test_device_clock_ignores_wall_polling_and_keeps_phase_with_sparse_samples():
    cadence = DeviceTickCadence(.02)
    samples = [(tick,cadence.take(tick)) for tick in range(0,202,3)]
    taken = [(tick,step) for tick,step in samples if step is not None]
    assert len(taken)==11 and taken[-1][0]==201
    assert sum(step[0] for _,step in taken[1:])==pytest.approx(.201)
    assert cadence.take(201) is None and cadence.take(199) is None
    assert not cadence.due(201) and cadence.due(222)


def test_device_wrap_is_not_restart_and_real_restart_is_fenced():
    cadence = DeviceTickCadence(.02)
    cadence.take(0xfffffff0)
    assert cadence.take(4)==(.02,False)
    cadence.take(2024)
    assert cadence.take(0)==(.02,True)
    assert cadence.resets==1
