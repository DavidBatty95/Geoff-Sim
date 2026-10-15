import pytest

from unitree_g1_mujoco_bridge.policy_timing import DeviceTickCadence


def test_device_clock_ignores_wall_polling_and_spaces_sparse_samples():
    cadence = DeviceTickCadence(.02)
    samples = [(tick,cadence.take(tick)) for tick in range(0,202,3)]
    taken = [(tick,step) for tick,step in samples if step is not None]
    assert len(taken)==10 and taken[-1][0]==189
    assert all(step[0]>=.020 for _,step in taken[1:])
    assert sum(step[0] for _,step in taken[1:])==pytest.approx(.189)
    assert cadence.take(201) is None and cadence.take(199) is None
    assert not cadence.due(201) and cadence.due(222)


def test_device_wrap_is_not_restart_and_real_restart_is_fenced():
    cadence = DeviceTickCadence(.02)
    cadence.take(0xfffffff0)
    assert cadence.take(4)==(.02,False)
    cadence.take(2024)
    assert cadence.take(0)==(.02,True)
    assert cadence.resets==1


def test_late_sample_does_not_trigger_two_millisecond_catchup():
    cadence = DeviceTickCadence(.02)
    cadence.take(0)
    assert cadence.take(38)==(.038,False)
    assert not cadence.due(40)
    assert cadence.take(40) is None
    assert cadence.due(58)
    assert cadence.take(58)==(.020,False)
    assert cadence.take(60) is None
    assert cadence.take(78)==(.020,False)


def test_irregular_delivery_never_compresses_history_or_hides_long_gaps():
    cadence = DeviceTickCadence(.02)
    samples = []
    for tick in (0,2,38,40,41,57,59,60,62,83,84,99,100,101,103,310,312,330):
        step = cadence.take(tick)
        if step is not None:
            samples.append((tick,step[0]))
    assert all(b[0]-a[0]>=20 for a,b in zip(samples,samples[1:]))
    assert max(dt for _,dt in samples)>.2
    assert cadence.missed>0


def test_adapter_history_contains_no_synthetic_catchup_frames():
    from unitree_g1_mujoco_bridge.unitree_rl_lab_policy import UnitreeRLLabG1PolicyAdapter
    adapter = UnitreeRLLabG1PolicyAdapter()
    cadence = DeviceTickCadence(.02)
    accepted = []
    for tick in (0,38,40,58,60,78):
        if cadence.take(tick) is not None:
            accepted.append(tick)
            observation = adapter.build_observation([tick/1000.,0.,0.],
                adapter.sdk_default_positions,[0.]*29,[0.]*3,[1.,0.,0.,0.],[0.]*29)
    assert accepted==[0,38,58,78]
    # Term-major layout: gyro and gravity each have 5 × 3 values before commands.
    assert observation[0,30:45:3].tolist()==pytest.approx([0.,0.,.038,.058,.078])
