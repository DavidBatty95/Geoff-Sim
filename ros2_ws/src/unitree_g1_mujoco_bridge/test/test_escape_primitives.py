import numpy as np
import pytest

from unitree_g1_mujoco_bridge.escape_primitives import templates,check_programs,snapshot,search_snapshot,process_worker
from unitree_g1_mujoco_bridge.escape_search import EscapeSearchWorker
from unitree_g1_mujoco_bridge.kinodynamic_search import rollout_schedule,search_sequence
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope


def envelope(profile='native_contact_cycle05'):
    return MotionEnvelope(max_x=.65,max_y=.45,max_yaw=.6,max_planar_speed=.45,
        accel_x=2.4,accel_y=2.4,accel_yaw=.9,forward_gain=.96,lateral_gain=.8,
        yaw_gain=.32,walking_yaw_gain=.55,lateral_yaw_gain=.1,
        linear_time_constant=.4,response_profile=profile)


@pytest.mark.parametrize('profile',['legacy','native_contact_cycle05'])
def test_primitive_rollout_matches_shared_response_for_non_grid_handoffs(profile):
    model=envelope(profile)
    program=((.37,.4,.15,.2),(1.43,-.3,.2,-.1),(2.7,0.,0.,0.))
    pose=(.1,-.2,.4)
    velocity=(.1,-.05,.08)
    filtered=(.1,.2,-.05)
    diagnostics={}
    selected=check_programs(model,pose,velocity,filtered,[program],np.zeros((31,0,2)),
        np.zeros((31,0)),(0.,0.),4.5,.15,.11,budget_ms=0.,diagnostics=diagnostics)
    assert selected==program and diagnostics['reason']=='complete'
    scalar=rollout_schedule(model,pose,velocity,filtered,program,4.5,.15,.11)
    np.testing.assert_allclose(diagnostics['endpoint'],scalar[-1][:2],atol=1e-10)


def test_template_controls_respect_asymmetric_command_limits():
    programs=templates(envelope(),4.5,.3,.2,.15)
    for program in programs:
        assert sum(part[0] for part in program)==pytest.approx(4.5)
        assert program[-1][1:]==(0.,0.,0.)
        for _,vx,vy,wz in program:
            assert -.2<=vx<=.3 and abs(vy)<=.15 and wz==0.


def test_hold_checks_reject_conflict_beyond_the_motion_horizon():
    times=np.linspace(0.,4.5,31)
    people=np.array([[[6.-t,0.]] for t in times])
    details={}
    selected=check_programs(envelope(),(0.,0.,0.),(0.,0.,0.),(0.,0.,0.),
        [((4.5,0.,0.,0.),)],people,np.full((31,1),.9),(0.,0.),4.5,.15,.1,
        budget_ms=0.,diagnostics=details)
    assert selected is None and details['reason']=='unsafe_terminal_state_or_hold'


def test_reusing_a_program_does_not_reuse_its_previous_clearance():
    model=envelope()
    programs=templates(model,4.5)
    args=(model,(0.,0.,0.),(0.,0.,0.),(0.,0.,0.),programs,np.zeros((31,0,2)),np.zeros((31,0)),(1.,0.),4.5,.15,.1)
    selected=check_programs(*args,budget_ms=0.)
    assert selected is not None
    details={}
    assert check_programs(*args,static_query=lambda points:np.zeros(len(points)),
                          budget_ms=0.,diagnostics=details) is None
    assert details['reason']=='no_feasible_prefix' and details['static_rejections']==len(programs)


def test_timed_out_program_batch_never_returns_partial_plan():
    details={}
    model=envelope()
    assert check_programs(model,(0.,0.,0.),(0.,0.,0.),(0.,0.,0.),templates(model,4.5),
        np.zeros((31,0,2)),np.zeros((31,0)),(1.,0.),4.5,.15,.1,
        budget_ms=1e-12,diagnostics=details) is None
    assert details['reason']=='deadline'


def test_beam_diagnostic_distinguishes_timeout_from_blocked_prefix():
    model=envelope()
    args=(model,(0.,0.,0.),(0.,0.,0.),(0.,0.,0.),[(.3,0.,0.)],
          np.zeros((31,0,2)),np.zeros((31,0)),(1.,0.),4.5,.15,.1)
    details={}
    assert search_sequence(*args,budget_ms=1e-12,diagnostics=details) is None
    assert details['reason']=='deadline'
    assert search_sequence(*args,budget_ms=0.,static_query=lambda points:np.zeros(len(points)),diagnostics=details) is None
    assert details['reason']=='no_feasible_prefix'


def test_async_snapshot_is_immutable_and_returned_program_needs_new_clearance():
    model=envelope()
    people=np.full((31,1,2),10.)
    inputs=dict(envelope=model,pose=(0.,0.,0.),velocity=(0.,0.,0.),filtered=(0.,0.,0.),
        programs=templates(model,4.5),people=people,radii=np.full((31,1),.9),goal=(1.,0.),
        horizon=4.5,dt=.15,delay=.1,budget_ms=0.)
    request=snapshot(inputs,('goal_a',))
    people[:]=0.  # New perception invalidates the route after submission.
    assert np.all(request.inputs['people']==10.)
    assert not request.inputs['people'].flags.writeable
    worker=EscapeSearchWorker(search=search_snapshot)
    try:
        assert worker.submit(request)
        worker.future.result(timeout=2.)
        result=worker.poll(2.)
        assert result.command is not None and result.snapshot.context==('goal_a',)
        assert check_programs(**{**inputs,'programs':[result.command]}) is None
    finally:
        worker.shutdown()


def test_process_worker_round_trips_snapshot_and_static_geometry():
    from unitree_g1_mujoco_bridge.static_distance_field import StaticDistanceField
    model=envelope()
    static=StaticDistanceField(np.array([[4.,4.]]),np.zeros(2),5.,3.)
    inputs=dict(envelope=model,pose=(0.,0.,0.),velocity=(0.,0.,0.),filtered=(0.,0.,0.),
        programs=templates(model,4.5),people=np.full((31,1,2),10.),radii=np.full((31,1),.9),
        goal=(1.,0.),horizon=4.5,dt=.15,delay=.1,budget_ms=0.,static_query=static.query)
    worker=process_worker()
    try:
        assert worker.warmup.result(timeout=10.)
        assert worker.submit(snapshot(inputs,('goal_a','scan')))
        worker.future.result(timeout=10.)
        result=worker.poll(10.)
        assert result.command is not None
        assert result.snapshot.context==('goal_a','scan')
        assert result.snapshot.diagnostics['reason']=='complete'
        assert not result.snapshot.inputs['people'].flags.writeable
    finally:
        worker.shutdown()


def test_all_delay_variants_must_clear_not_just_the_nominal_program():
    model=MotionEnvelope(max_x=.4,accel_x=100.,linear_time_constant=.01)
    program=((2.,.4,0.,0.),(2.5,0.,0.,0.))
    times=np.linspace(0.,4.5,31)
    people=np.array([[[.26,1.-t]] for t in times])
    base=(model,(0.,0.,0.),(0.,0.,0.),(0.,0.,0.))
    assert check_programs(*base,[program],people,np.full((31,1),.07),(1.,0.),
                          4.5,.15,0.,budget_ms=0.) is not None
    details={}
    assert check_programs(*base,[program,program],people,np.full((31,1),.07),(1.,0.),
        4.5,.15,np.array([0.,.3]),admission_group_size=2,budget_ms=0.,diagnostics=details) is None
    assert details['reason']=='no_complete_admission_group'


def test_extra_worker_reserve_does_not_change_nominal_admission_threshold():
    people=np.tile([[[1.25,0.]]],(31,1,1))
    args=(envelope(),(0.,0.,0.),(0.,0.,0.),(0.,0.,0.),[((4.5,0.,0.,0.),)],
          people,np.full((31,1),1.23),(0.,0.),4.5,.15,.1)
    assert check_programs(*args,budget_ms=0.) is not None
    assert check_programs(*args,budget_ms=0.,minimum_margin=.1) is None


def test_admission_delay_retains_the_complete_braking_and_hold_window():
    program=((3.6,.4,0.,0.),(.9,0.,0.,0.))
    request=snapshot(dict(envelope=envelope(),pose=(0.,0.,0.),velocity=(0.,0.,0.),
        filtered=(0.,0.,0.),programs=[program],people=np.zeros((31,0,2)),radii=np.zeros((31,0)),
        goal=(1.,0.),horizon=4.5,dt=.15,delay=.1,budget_ms=0.,
        admission_delays=(0.,.1,.2),minimum_margin=.1),('goal_a',))
    assert search_snapshot(request)==program
    assert request.diagnostics['stage']=='admission'
    assert request.diagnostics['admission_horizon_s']==pytest.approx(4.7)


def test_priority_failure_falls_back_to_full_library_without_relaxing_delay_checks(monkeypatch):
    import unitree_g1_mujoco_bridge.escape_primitives as module
    model=MotionEnvelope(max_x=.4,accel_x=100.,linear_time_constant=.01)
    forward=((2.,.4,0.,0.),(2.5,0.,0.,0.))
    backward=((2.,-.4,0.,0.),(2.5,0.,0.,0.))
    monkeypatch.setattr(module,'prioritized_programs',lambda inputs:(forward,))
    times=np.linspace(0.,4.5,31)
    people=np.array([[[.29,1.-t]] for t in times])
    request=snapshot(dict(envelope=model,pose=(0.,0.,0.),velocity=(0.,0.,0.),filtered=(0.,0.,0.),
        programs=(forward,backward),people=people,radii=np.full((31,1),.06),goal=(1.,0.),
        horizon=4.5,dt=.15,delay=0.,budget_ms=0.,admission_delays=(0.,.1,.2)),())
    assert search_snapshot(request)==backward
    assert request.diagnostics['priority_attempt']['stage']=='admission'
    assert request.diagnostics['priority_attempt']['reason']=='no_complete_admission_group'
    assert request.diagnostics['reason']=='complete'


@pytest.mark.parametrize('scenario',('clear','crossing','wall'))
def test_small_clearance_path_matches_vector_batch_admission(scenario):
    program=((2.,.4,0.,0.),(2.5,0.,0.,0.))
    times=np.linspace(0.,4.5,31)
    people=np.array([[[.26,1.-t]] if scenario=='crossing' else [[5.,3.]] for t in times])
    query=(lambda points:np.where(points[:,0]>.3,0.,10.)) if scenario=='wall' else None
    outcomes=[]
    for count in (1,13):
        details={}
        plan=check_programs(envelope(),(0.,0.,0.),(0.,0.,0.),(0.,0.,0.),[program]*count,
            people,np.full((31,1),.1),(1.,0.),4.5,.15,.1,
            budget_ms=0.,static_query=query,diagnostics=details)
        outcomes.append((plan,details))
    assert outcomes[0][0]==outcomes[1][0]
    assert outcomes[0][1]['reason']==outcomes[1][1]['reason']
    if outcomes[0][0] is not None:
        assert outcomes[0][1]['minimum_margin_m']==pytest.approx(outcomes[1][1]['minimum_margin_m'],abs=1e-12)
