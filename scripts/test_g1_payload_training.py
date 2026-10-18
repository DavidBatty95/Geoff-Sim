"""CPU checks for the GPU training contract (run in the MJLab uv environment)."""
from types import SimpleNamespace
import importlib

import numpy as np
import pytest

@pytest.fixture
def training():
    # Runtime skips: ROS launch-testing imports files during directory
    # discovery, where a module-level skip can suppress unrelated tests.
    pytest.importorskip('torch')
    pytest.importorskip('mujoco')
    pytest.importorskip('mjlab')
    return importlib.import_module('g1_payload_training')


def test_import_matches_source_and_arm_initialization_remains_trainable(training):
    torch = training.torch
    class Actor(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.mlp = torch.nn.Sequential(
                torch.nn.Linear(480, 512), torch.nn.ELU(),
                torch.nn.Linear(512, 256), torch.nn.ELU(),
                torch.nn.Linear(256, 128), torch.nn.ELU(), torch.nn.Linear(128, 29),
            )
    torch.set_num_threads(2)
    actor = Actor()
    assert training.load_onnx_actor(actor, training.SOURCE) < 2e-5
    observations = torch.randn(4, 480)
    masked = observations.clone()
    columns = [start + history*29 + joint for start in (45, 190, 335)
               for history in range(5) for joint in training.ARM_POLICY_IDS]
    masked[:, columns] = 0.
    with torch.no_grad():
        expected = actor.mlp(masked)
    training.initialize_arm_inputs(actor)
    actual = actor.mlp(observations)
    torch.testing.assert_close(actual, expected)
    actual.square().sum().backward()
    assert actor.mlp[0].weight.grad[:, columns].abs().sum() > 0


def test_payload_has_real_mass_and_cube_inertia_at_the_wrist(training):
    model = training.payload_spec().compile()
    payload = model.body('carried_payload')
    wrist = model.body('right_wrist_yaw_link')
    assert model.body_parentid[payload.id] == wrist.id
    assert model.body_jntnum[payload.id] == 0
    np.testing.assert_allclose(payload.pos, (.19, .01, 0.))
    np.testing.assert_allclose(payload.mass, .15)
    np.testing.assert_allclose(payload.inertia, np.full(3, .15*.11**2/6))


def test_virtual_cup_axis_matches_mujoco_nominal_carry_pose(training):
    model = training.payload_spec().compile()
    data = training.mujoco.MjData(model)
    joints = dict(zip(training.POLICY_NAMES,training.UNITREE_RL_LAB_G1_DEFAULT_POS))
    joints.update(dict(zip(training.G1_29DOF_JOINTS[22:],training.PAYLOAD_V1_POSTURES[0])))
    data.qpos[3:7] = (1.,0.,0.,0.)
    for name,value in joints.items():
        data.qpos[model.joint(name).qposadr[0]] = value
    training.mujoco.mj_forward(model,data)
    R = data.xmat[model.body('carried_payload').id].reshape(3,3)
    np.testing.assert_allclose(R@training.cup_up_axis(),(0.,0.,1.),atol=1e-4)


def test_fluid_objective_preserves_actor_contract_and_covers_native_yaw_range(training):
    cfg = training.build_env(2,42,objective='fluid')
    assert cfg.observations['actor'].history_length == 5
    assert 'base_lin_vel' not in cfg.observations['actor'].terms
    assert cfg.commands['twist'].yaw_speed_max == .9
    assert cfg.rewards['fluid_carry'].weight < 0.
    assert cfg.rewards['track_angular_velocity'].weight > 0.


def test_episode_resamples_mass_before_writing_physics_and_scopes_arm_gains(training):
    torch = training.torch
    action = SimpleNamespace(mass=torch.zeros(2), held=torch.zeros(2, dtype=torch.bool))
    def sample(ids):
        action.mass[ids] = torch.tensor((0., .35))
        action.held[ids] = torch.tensor((False, True))
    action.sample_episode = sample
    robot = SimpleNamespace(
        find_bodies=lambda _: ([0], ['carried_payload']),
        indexing=SimpleNamespace(body_ids=torch.tensor([0]), ctrl_ids=torch.arange(29)),
        actuator_names=tuple(training.G1_29DOF_JOINTS),
    )
    model = SimpleNamespace(body_mass=torch.zeros(2, 1), body_inertia=torch.zeros(2, 1, 3),
                            actuator_gainprm=torch.full((2, 29, 10), 123.),
                            actuator_biasprm=torch.full((2, 29, 10), 123.))
    env = SimpleNamespace(action_manager=SimpleNamespace(get_term=lambda _: action),
                          scene={'robot': robot}, sim=SimpleNamespace(model=model), num_envs=2, device='cpu')
    training.reset_payload(env, None)
    torch.testing.assert_close(model.body_mass[:, 0], torch.tensor((1e-6, .35)))
    torch.testing.assert_close(model.body_inertia[1, 0], torch.full((3,), .35*.11**2/6))
    assert torch.all(model.actuator_gainprm[0, 15:, 0] == 40.)
    assert torch.all(model.actuator_gainprm[1, 15:, 0] == 160.)
    assert torch.all(model.actuator_biasprm[0, 15:, 2] == -10.)
    assert torch.all(model.actuator_biasprm[1, 15:, 2] == -4.)
    assert torch.all(model.actuator_gainprm[:, :15] == 123.)
def test_native_inertial_configuration_matches_torso_geometry_and_motor_limits(training):
    import numpy as np
    native = training.native_robot_model()
    aligned = training.native_inertial_payload_spec().compile()
    for name in ('torso_link','waist_roll_link','waist_yaw_link'):
        a,b = native.body(name).id,aligned.body(name).id
        assert aligned.body_mass[b] == pytest.approx(native.body_mass[a])
        np.testing.assert_allclose(aligned.body_pos[b],native.body_pos[a])
        np.testing.assert_allclose(aligned.body_inertia[b],native.body_inertia[a])
    cfg = training.build_env(2,42,dynamics='native_inertial',max_delay_steps=2)
    actuators = cfg.scene.entities['robot'].articulation.actuators
    assert actuators[1].effort_limit == 88.
    assert actuators[12].effort_limit == 88.
    assert cfg.actions['joint_pos'].max_delay_steps == 2


def test_native_contact_profile_matches_deployment_feet_without_changing_actor_contract(training):
    native = training.native_robot_model()
    aligned = training.native_contact_payload_spec().compile()
    for side in ('left','right'):
        body = native.body(f'{side}_ankle_roll_link').id
        contacts = [i for i in range(native.ngeom) if native.geom_bodyid[i]==body and
                    (native.geom_contype[i] or native.geom_conaffinity[i])]
        actual = [i for i in range(aligned.ngeom) if aligned.geom(i).name.startswith(side+'_foot')]
        assert len(actual)==len(contacts)==4
        for a,b in zip(actual,contacts):
            assert aligned.geom_type[a]==native.geom_type[b]
            for field in ('geom_pos','geom_size','geom_friction','geom_solref','geom_solimp'):
                np.testing.assert_allclose(getattr(aligned,field)[a],getattr(native,field)[b])
    cfg = training.build_env(2,74,dynamics='native_contact',objective='reactive',action_hold_probability=.05)
    assert cfg.observations['actor'].history_length==5
    assert cfg.actions['joint_pos'].action_hold_probability==.05
    assert cfg.rewards['turn_drift'].weight<0.


def test_turn_drift_penalty_is_scoped_to_pivots(training):
    torch = training.torch
    env = SimpleNamespace(command_manager=SimpleNamespace(get_command=lambda _:torch.tensor(
        [[0.,0.,.6],[.4,0.,.6],[0.,0.,0.]])),scene={'robot':SimpleNamespace(data=SimpleNamespace(
            root_link_lin_vel_b=torch.tensor([[.2,.1,0.],[.4,.1,0.],[.2,.1,0.]])))})
    torch.testing.assert_close(training.turn_drift_penalty(env),torch.tensor([.05,0.,0.]))


def test_sensor_delay_keeps_packet_coherence_and_does_not_leak_reset_history(training):
    from payload_sensor_delay import SensorFrameDelay
    torch = training.torch
    a,b = SensorFrameDelay(3,1,2,'cpu'),SensorFrameDelay(3,1,2,'cpu')
    lag = torch.tensor([0,1,2])
    for step in range(4):
        current = (torch.arange(3)+10.*step)[:,None]
        delayed = a.push(current,lag,step)
        torch.testing.assert_close(b.push(current+100.,lag,step)-delayed,torch.full((3,1),100.))
    torch.testing.assert_close(delayed[:,0],torch.tensor([30.,21.,12.]))
    torch.testing.assert_close(a.push(current+999.,lag,3),delayed)
    a.reset(torch.tensor([1]))
    fresh = current.clone()
    fresh[1] = 400.
    torch.testing.assert_close(a.push(fresh,lag,3)[:,0],torch.tensor([30.,400.,12.]))


def test_sensor_delay_changes_only_actor_measurements_and_preserves_history_order(training):
    cfg = training.build_env(2,75,dynamics='native_contact',max_delay_steps=2,
                             max_sensor_delay_steps=3,action_feedback='published')
    actor,critic = cfg.observations['actor'],cfg.observations['critic']
    assert actor.history_length==5
    assert list(actor.terms)==['base_ang_vel','projected_gravity','velocity_commands','joint_pos_rel','joint_vel_rel','last_action']
    for name in ('base_ang_vel','projected_gravity','joint_pos_rel','joint_vel_rel'):
        assert actor.terms[name].func is training.DelayedSensorTerm
        assert critic.terms[name].func is not training.DelayedSensorTerm
        assert actor.terms[name].params['max_lag']==3
    assert actor.terms['last_action'].func is training.applied_policy_action
    assert actor.terms['velocity_commands'].func is training.mdp.generated_commands
    assert 'base_lin_vel' not in actor.terms


def test_action_feedback_distinguishes_published_from_motor_applied(training):
    torch = training.torch
    action = SimpleNamespace(cfg=SimpleNamespace(action_feedback='published'),
                             published_action=torch.ones(2,29),raw_action=torch.zeros(2,29))
    env = SimpleNamespace(action_manager=SimpleNamespace(get_term=lambda _:action))
    assert training.applied_policy_action(env) is action.published_action
    action.cfg.action_feedback='applied'
    assert training.applied_policy_action(env) is action.raw_action


def test_stop_objective_penalizes_residual_motion_only_on_explicit_stop(training):
    torch = training.torch
    env = SimpleNamespace(command_manager=SimpleNamespace(get_command=lambda _:torch.tensor(
        [[0.,0.,0.],[0.,-.45,0.],[0.,0.,.6],[.001,0.,0.]])),
        scene={'robot':SimpleNamespace(data=SimpleNamespace(
            root_link_lin_vel_b=torch.tensor([[.3,.4,0.]]*4),
            root_link_ang_vel_b=torch.tensor([[0.,0.,.2]]*4)))})
    torch.testing.assert_close(training.stopping_velocity_penalty(env),torch.tensor([.26,0.,0.,0.]))
    cfg = training.build_env(4,76,objective='stopping',dynamics='native_contact',
                             max_delay_steps=2,action_feedback='published')
    assert cfg.rewards['stop_velocity'].weight<0.
    assert 'base_lin_vel' not in cfg.observations['actor'].terms
    assert cfg.observations['actor'].history_length==5
    assert cfg.actions['joint_pos'].action_feedback=='published'
    assert cfg.commands['twist'].ranges.lin_vel_y==(-.45,.45)
    phases = training.SIGNED_STOP_SCHEDULE
    assert {values[1] for _,_,values in phases}=={-.45,0.,.45}
    for index,(_,duration,values) in enumerate(phases):
        if values[1]:
            assert duration in (2.,3.)
            assert phases[index+1][1:]==(6.,(0.,0.,0.))


def test_transition_timer_resamples_on_exact_requested_control_tick(training):
    from payload_transition_commands import phase_deadlines,sample_phase_ticks
    torch = training.torch
    ticks = torch.arange(75,301)
    timers = phase_deadlines(ticks,.02)
    fired = torch.zeros_like(ticks)
    for step in range(1,302):
        timers -= .02
        fired[(timers<=0.)&(fired==0)] = step
    torch.testing.assert_close(fired,ticks)
    torch.manual_seed(77)
    standing = torch.arange(4096)%2==0
    samples = sample_phase_ticks(standing,(75,175),(250,300))
    assert set(samples[~standing].tolist())==set(range(75,176))
    assert set(samples[standing].tolist())==set(range(250,301))


def test_transition_profile_preserves_actor_and_alternates_movement_with_long_stops(training):
    from payload_transition_commands import TransitionVelocityCommandCfg
    cfg = training.build_env(4,77,objective='reactive',dynamics='native_contact',max_delay_steps=3,
                             action_feedback='published',command_profile='transitions')
    command = cfg.commands['twist']
    assert isinstance(command,TransitionVelocityCommandCfg)
    assert command.initial_standing_probability==command.moving_stop_probability==1.
    assert command.standing_hold_probability==0.
    assert command.moving_ticks==(75,175) and command.standing_ticks==(250,300)
    assert command.lateral_probability==.5 and command.lateral_endpoint_probability==.5
    assert 'stop_velocity' not in cfg.rewards
    assert cfg.observations['actor'].history_length==5
    assert 'base_lin_vel' not in cfg.observations['actor'].terms
    assert cfg.actions['joint_pos'].action_feedback=='published'


def test_native_euler_option_changes_only_integration_configuration(training):
    implicit = training.build_env(2,77,objective='reactive',dynamics='native_contact',action_feedback='published')
    explicit = training.build_env(2,77,objective='reactive',dynamics='native_contact',action_feedback='published',integrator='euler')
    assert implicit.sim.mujoco.integrator=='implicitfast'
    assert explicit.sim.mujoco.integrator=='euler'
    assert explicit.sim.mujoco.timestep==implicit.sim.mujoco.timestep==.002
    assert explicit.actions==implicit.actions and explicit.commands==implicit.commands
    assert explicit.scene.entities['robot'].articulation.actuators==implicit.scene.entities['robot'].articulation.actuators
    assert list(explicit.observations['actor'].terms)==list(implicit.observations['actor'].terms)
    assert explicit.observations['actor'].history_length==5
    with pytest.raises(ValueError,match='integration'):
        training.build_env(2,77,integrator='unknown')


def test_policy_order_soft_limits_match_deployment_sdk_clipping(training):
    from unitree_g1_mujoco_bridge.policy_safety import clamp_joint_targets
    from unitree_g1_mujoco_bridge.unitree_rl_lab_policy import UnitreeRLLabG1PolicyAdapter
    torch = training.torch
    torch.manual_seed(3)
    targets = torch.randn(10,29)*3.
    lower,upper = training.native_policy_target_bounds(.9,'cpu')
    actual = torch.clamp(targets,lower,upper)
    adapter = UnitreeRLLabG1PolicyAdapter()
    expected = torch.tensor([adapter.sdk_to_policy(clamp_joint_targets(adapter.policy_to_sdk(row))) for row in targets.tolist()])
    torch.testing.assert_close(actual,expected,atol=3e-7,rtol=1e-6)
    cfg = training.build_env(2,77,target_soft_limit_factor=.9)
    assert cfg.actions['joint_pos'].target_soft_limit_factor==.9
    assert training.build_env(2,77).actions['joint_pos'].target_soft_limit_factor is None
    with pytest.raises(ValueError,match='soft-limit'):
        training.native_policy_target_bounds(1.1,'cpu')


def test_native_joint_friction_survives_final_scene_compilation(training):
    from mjlab.scene import Scene
    native = training.native_robot_model()
    legacy = Scene(training.build_env(1,77,dynamics='native_contact').scene,device='cpu').compile()
    aligned = Scene(training.build_env(1,77,dynamics='native_contact',native_joint_friction=True).scene,device='cpu').compile()
    wrist = legacy.joint('robot/right_wrist_roll_joint').dofadr[0]
    assert legacy.dof_frictionloss[wrist]==.1
    for name in training.G1_29DOF_JOINTS:
        expected = native.dof_frictionloss[native.joint(name).dofadr[0]]
        actual = aligned.dof_frictionloss[aligned.joint('robot/'+name).dofadr[0]]
        assert actual==pytest.approx(expected)
    with pytest.raises(ValueError,match='native_contact'):
        training.build_env(1,77,native_joint_friction=True)
