from types import SimpleNamespace

import pytest

from unitree_g1_mujoco_bridge.dex3_grasp import Dex3GraspController,decode_state,grasp_contact,close_positions,contact_lift_verified,SIM_CONTACT_MARKER


def packet():
    return SimpleNamespace(motor_state=[SimpleNamespace(q=v) for v in (0.,-.4,-.85,1.05,1.2,1.05,1.2)],
        press_sensor_state=[SimpleNamespace(pressure=[.5]+[0.]*11,lost=0) for _ in range(9)],
        reserve=[100,SIM_CONTACT_MARKER],error=[0,0])


def test_opening_speed_is_bounded_for_both_signs_and_timer_delays():
    from unitree_g1_mujoco_bridge.dex3_grasp import opening_step
    q = (.01,-.01,1.,-1.,0.,.001,-.001)
    assert opening_step(q,.02)==pytest.approx((.004,-.004,.994,-.994,0.,0.,0.))
    assert opening_step(q,2.)==opening_step(q,.04)
    assert opening_step(q,-.02)==q


def test_grasp_dwell_waits_for_native_closure_interval(monkeypatch):
    import unitree_g1_mujoco_bridge.dex3_grasp as module
    from unitree_g1_mujoco_bridge.dex3_grasp import closure_ready_tick
    controller = object.__new__(Dex3GraspController)
    now = [0.]
    monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    q,forces,_ = decode_state(packet())
    ready = closure_ready_tick((0.,)*7,close_positions('right'),0)
    assert ready>=1000
    results = []
    controller.targets = dict(right=close_positions('right'))
    controller.publishers = dict(right=SimpleNamespace(publish=lambda msg:None))
    controller.status = SimpleNamespace(publish=lambda msg:None)
    controller.holding = dict(right=False)
    controller.reset_generation = dict(right=0)
    controller.pending = dict(right=dict(close=True,deadline=6.,since=None,last_step=0.,generation=0,
        ready_tick=ready,future=SimpleNamespace(set_result=results.append)))
    for wall,tick in ((.4,400),(1.2,800),(1.4,ready),(1.6,ready+200)):
        now[0] = wall
        controller.samples = dict(right=(wall,q,forces))
        controller.sequence = dict(right=tick)
        controller.tick()
        assert not results
    now[0] = 1.8
    controller.samples['right'] = (1.8,q,forces)
    controller.sequence['right'] = ready+400
    controller.tick()
    assert results[0][0] and controller.holding['right']


def test_opposing_contacts_and_loaded_fingers_required():
    q,forces,tick = decode_state(packet())
    assert tick==100 and grasp_contact(q,forces,'right')
    assert not grasp_contact(q,(0.,2.,2.),'right')
    assert not grasp_contact(q,(2.,0.,0.),'right')
    assert not grasp_contact(close_positions('right'),forces,'right')
    assert close_positions('left')[0] > 0.
    assert not grasp_contact(close_positions('left'),forces,'left')


@pytest.mark.parametrize('fault',['marker','stale','lost','nonfinite','count'])
def test_unverified_tactile_packets_fail_closed(fault):
    message = packet()
    if fault=='marker':
        message.reserve[1]=0
    elif fault=='stale':
        message.error[0]=1
    elif fault=='lost':
        message.press_sensor_state[0].lost=1
    elif fault=='nonfinite':
        message.motor_state[0].q=float('nan')
    else:
        message.motor_state.pop()
    with pytest.raises(ValueError):
        decode_state(message)


def test_occluded_lift_requires_actual_rise_and_fresh_opposed_contact_feedback():
    state = dict(grasp_model='dex3_contact_only',fresh=True,contact_verified=True,holding_requested=True)
    assert contact_lift_verified(1.,1.14,state,(.01,.01,.02,.03))
    assert not contact_lift_verified(1.,1.02,state,(.01,.01,.02,.03))
    assert not contact_lift_verified(1.,1.14,state,(.01,.31,.02,.03))
    assert not contact_lift_verified(1.,1.14,dict(state,contact_verified=False),(.01,.01,.02,.03))
    assert not contact_lift_verified(1.,float('nan'),state,(.01,.01,.02,.03))


def test_feedback_reset_invalidates_pending_operation_generation():
    controller = object.__new__(Dex3GraspController)
    controller.samples,controller.sequence,controller.reset_generation = {},{},dict(right=0)
    message=packet()
    controller.receive(message,'right')
    assert 'right' in controller.samples
    received=controller.samples['right'][0]
    controller.receive(message,'right')
    assert controller.samples['right'][0]==received
    message.reserve[0]=20
    controller.receive(message,'right')
    assert 'right' not in controller.samples and controller.reset_generation['right']==1
    message.reserve[0]=40
    controller.receive(message,'right')
    assert 'right' in controller.samples


def test_rgbd_recognizes_small_cylinder_without_layout_coordinates():
    import numpy as np
    from unitree_g1_mujoco_bridge.transport_perception import colour_detections
    h,w=80,120
    v,u=np.indices((h,w))
    horizontal=(u-60)/120.
    a=1+horizontal**2
    disc=4-4*a*(1-.02**2)
    depth=np.where(disc>=0,(2-np.sqrt(np.maximum(0,disc)))/(2*a),0.)
    height=1.-(v-40)*depth/120.
    mask=(disc>=0)&(height>=.95)&(height<=1.04)
    depth=np.where(mask,depth,0.)
    rgb=np.zeros((h,w,3),dtype=np.uint8)
    rgb[:,:,0]=np.where(mask,220,0)
    K=np.array([[120.,0,60],[0,120.,40],[0,0,1.]])
    R=np.array([[0,0,1],[-1,0,0],[0,-1,0.]])
    found=colour_detections(rgb,depth,K,np.array([0.,0.,1.]),R,object_profile='dex3_cylinder')
    item=next(item for item in found if item['id']=='box_red')
    np.testing.assert_allclose(item['position'][:2],[1.,0.],atol=.005)
    clipped = rgb.copy()
    clipped[height<1.02] = 0  # Only a short cap remains visible through the hand.
    assert not any(item['id']=='box_red' for item in colour_detections(
        clipped,depth,K,np.array([0.,0.,1.]),R,object_profile='dex3_cylinder'))


@pytest.mark.parametrize('tracking_error',[.020,.15])
def test_mobile_reach_corrects_loaded_arm_error_without_relaxing_alignment(tracking_error):
    import numpy as np
    from unitree_g1_mujoco_bridge.transport_demo import TransportMission
    node = object.__new__(TransportMission)
    node.dex3 = True
    node.joints = {}
    node.wait = lambda predicate,*args: predicate()
    node.observe = lambda *args: dict(position=[.4,-.15,.2])
    base = np.zeros(3)
    measured = np.zeros(3)
    targets = []
    node.to_pelvis = lambda box: box-base
    node.kin = SimpleNamespace(forward=lambda joints:(measured.copy(),))
    node.event = lambda *args,**kwargs: None
    staged = []
    node.move_arm = lambda target,orientation:staged.append(target.copy())
    def move(target,orientation):
        targets.append(target.copy())
        measured[:] = target-np.array([0.,0.,tracking_error])
        base[0] += .003  # An unpinned base shifts during each trajectory.
        return measured.copy()
    node.arm_with_recovery = move
    if tracking_error > .08:
        with pytest.raises(RuntimeError,match='correction exceeded bound'):
            node.reach_box('box_red',np.array([1.,0.]),np.array([0.,1.]),0.)
    else:
        box = node.reach_box('box_red',np.array([1.,0.]),np.array([0.,1.]),0.)
        assert len(targets)==2
        assert staged[0][0] == pytest.approx(.25)
        assert targets[1][0] < targets[0][0]
        assert np.linalg.norm(measured-node.to_pelvis(box)) <= .012


def test_occluded_pickup_withdraws_and_reobserves_instead_of_grasping_old_target():
    import numpy as np
    from unitree_g1_mujoco_bridge.transport_demo import TransportMission
    node = object.__new__(TransportMission)
    node.dex3,node.joints = True,{}
    state = dict(visible=True,box=np.array([.4,-.15,.2]),hand=np.zeros(3),reaches=0)
    node.wait = lambda predicate,*args:predicate()
    node.observe = lambda identifier,age:dict(position=state['box']) if state['visible'] else None
    node.to_pelvis = lambda position:position
    node.kin = SimpleNamespace(forward=lambda joints:(state['hand'],))
    events,stages = [],[]
    node.event = lambda phase,**details:events.append(phase)
    def move(target,orientation):
        stages.append(target.copy())
        state['hand'] = target.copy()
        if not state['visible']:
            state['visible'] = True
            state['box'] = np.array([.42,-.15,.2])
    def reach(target,orientation):
        state['hand'] = target.copy()
        state['reaches'] += 1
        state['visible'] = state['reaches'] > 1
        return target.copy()
    node.move_arm,node.arm_with_recovery = move,reach
    box = node.reach_box('box_red',np.array([1.,0.]),np.array([0.,1.]),0.)
    assert state['reaches']==2 and 'reobserve_pickup' in events
    np.testing.assert_allclose(stages[1],[.25,-.15,.2])
    np.testing.assert_allclose(box,[.42,-.15,.2])


def test_base_retreat_is_reprojected_without_double_counting_in_tracking_bias():
    import numpy as np
    from unitree_g1_mujoco_bridge.transport_demo import TransportMission
    node = object.__new__(TransportMission)
    node.dex3,node.joints = True,{}
    base,hand = np.zeros(3),np.zeros(3)
    box = np.array([.4,-.15,.2])
    node.wait = lambda predicate,*args:predicate()
    node.observe = lambda *args:dict(position=box)
    node.to_pelvis = lambda point:point-base
    node.move_arm = lambda *args:None
    node.event = lambda *args,**kwargs:None
    node.kin = SimpleNamespace(forward=lambda joints:(hand.copy(),))
    targets = []
    def move(target,orientation):
        assert target[0] < .51, 'base retreat was counted twice'
        targets.append(target.copy())
        hand[:] = target-[0.,0.,.005]
        base[0] = -.07  # One balance step, then stable support.
        return hand.copy()
    node.arm_with_recovery = move
    node.reach_box('box_red',np.array([1.,0.]),np.array([0.,1.]),0.)
    assert len(targets)==2
    assert targets[1][0]==pytest.approx(.47)


@pytest.mark.parametrize('height',[0.,.02])
def test_staging_sag_is_compensated_before_advancing_over_table(height):
    import numpy as np
    from unitree_g1_mujoco_bridge.transport_demo import TransportMission
    node = object.__new__(TransportMission)
    node.dex3,node.joints = True,{}
    node.args = SimpleNamespace(grasp_height_offset=height)
    box,hand = np.array([.4,-.15,.2]),np.zeros(3)
    node.wait = lambda predicate,*args:predicate()
    node.observe = lambda *args:dict(position=box)
    node.to_pelvis = lambda point:point
    node.event = lambda *args,**kwargs:None
    node.kin = SimpleNamespace(forward=lambda joints:(hand.copy(),))
    node.move_arm = lambda target,orientation:target-np.array([0.,0.,.02])
    targets = []
    def move(target,orientation):
        targets.append(target.copy())
        hand[:] = target-[0.,0.,.02]
        return hand.copy()
    node.arm_with_recovery = move
    returned = node.reach_box('box_red',np.array([1.,0.]),np.array([0.,1.]),0.)
    np.testing.assert_allclose(returned,box)
    assert len(targets)==1 and targets[0][2]==pytest.approx(.216+height)
