import math
import time
from types import SimpleNamespace

import numpy as np

from unitree_g1_mujoco_bridge.human_aware_motion_planner import HumanAwareMotionPlanner
from unitree_g1_mujoco_bridge.transport_demo import TransportEvaluator
from unitree_g1_mujoco_bridge.transport_health import EvaluationHealth
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope


def evaluator_fixture(offset=0.):
    now = time.monotonic()
    start = np.array((0.,0.,1.005))
    placed = np.array((2.+offset,0.,1.009))
    target = np.array((2.,0.,.954))
    history = [(now-8.,start), (now-6.,np.array((0.,0.,1.2))),
               (now-4.,np.array((1.5,0.,1.2)))]
    history += [(now-1.1+.1*i,placed.copy()) for i in range(12)]
    health = EvaluationHealth(['robot','box_red','target_red'])
    for key in health.required:
        health.observe(key)
    health.start()
    return SimpleNamespace(pairs=[dict(object_id='box_red',destination_id='target_red')],
        objects={'box_red':placed,'target_red':target},
        initial={'box_red':start},history={'box_red':history,'target_red':[(now,target)]},
        shapes={'box_red':(np.eye(3),np.full(3,.11)),
                'target_red':(np.eye(3),np.array((.36,.36,.004)))},
        people_requested=0,aligned_samples=0,robot_updated=now,contacts=0,
        fallen=False,drops=set(),minimum_clearance=math.inf,health=health)


def test_complete_lift_carry_and_stable_placement_is_verified():
    result = TransportEvaluator.report(evaluator_fixture())
    assert result['success'] and result['data_valid']
    assert result['deliveries'][0]['lift_m'] > .19
    assert result['deliveries'][0]['travel_m'] == 2.


def test_centre_close_to_pad_is_not_enough_when_a_corner_overhangs():
    result = TransportEvaluator.report(evaluator_fixture(offset=.125))
    assert result['data_valid']
    assert not result['success']
    assert not result['deliveries'][0]['verified']


def test_missing_or_stale_destination_invalidates_evaluation():
    evaluator = evaluator_fixture()
    evaluator.history['target_red'][0] = (time.monotonic()-2.,evaluator.objects['target_red'])
    assert not TransportEvaluator.report(evaluator)['data_valid']
    del evaluator.objects['target_red']
    del evaluator.history['target_red']
    assert not TransportEvaluator.report(evaluator)['data_valid']


def test_search_heading_does_not_chase_position_drift():
    goal = SimpleNamespace(header=SimpleNamespace(frame_id='odom'),
        pose=SimpleNamespace(position=SimpleNamespace(x=0.,y=0.),
                             orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
    planner = SimpleNamespace(reference_goal=goal,target_frame='odom',navigation_mode='scan',
                              motion_envelope=MotionEnvelope(),
                              _wrap_pi=lambda value:math.atan2(math.sin(value),math.cos(value)))
    command = HumanAwareMotionPlanner._dock_reference(planner,(.4,.3,0.))
    assert command.linear.x == command.linear.y == command.angular.z == 0.
    command = HumanAwareMotionPlanner._dock_reference(planner,(.4,.3,1.))
    assert command.linear.x == command.linear.y == 0.
    assert command.angular.z < 0.


def test_dock_reference_can_reach_inner_settling_band():
    goal = SimpleNamespace(header=SimpleNamespace(frame_id='odom'),
        pose=SimpleNamespace(position=SimpleNamespace(x=0.,y=0.),
                             orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
    planner = SimpleNamespace(reference_goal=goal,target_frame='odom',navigation_mode='dock',
                              max_reverse_velocity=.45,max_forward_near_human=.4,
                              _wrap_pi=lambda value:math.atan2(math.sin(value),math.cos(value)))
    assert HumanAwareMotionPlanner._dock_reference(planner,(.04,0.,0.)).linear.x<0.
    assert HumanAwareMotionPlanner._dock_reference(planner,(.02,0.,0.)).linear.x==0.
    assert HumanAwareMotionPlanner._dock_reference(planner,(-.5,0.,0.)).linear.x==.4
    planner.payload_active = True
    planner.max_reverse_velocity = .4
    assert HumanAwareMotionPlanner._dock_reference(planner,(1.,0.,0.)).linear.x==-.4
    assert HumanAwareMotionPlanner._dock_reference(planner,(.1,0.,0.)).linear.x==-.4
    planner.payload_active = False
    assert HumanAwareMotionPlanner._dock_reference(planner,(1.,0.,0.)).linear.x==-.4


def test_burst_of_samples_does_not_prove_one_second_of_support():
    evaluator = evaluator_fixture()
    now = time.monotonic()
    evaluator.history['box_red'][-12:] = [(now-i*.001,evaluator.objects['box_red']) for i in range(12,0,-1)]
    assert not TransportEvaluator.report(evaluator)['success']


def test_past_evaluation_outage_cannot_be_erased_by_fresh_final_samples():
    evaluator = evaluator_fixture()
    evaluator.health.fail('robot: receipt gap exceeded')
    assert not TransportEvaluator.report(evaluator)['data_valid']


def test_cross_color_task_scores_requested_goal_not_matching_color():
    evaluator = evaluator_fixture()
    now = time.monotonic()
    evaluator.pairs = [dict(object_id='box_red',destination_id='target_blue',color='red',destination_color='blue')]
    evaluator.objects['target_blue'] = np.array((4.,0.,.954))
    evaluator.history['target_blue'] = [(now,evaluator.objects['target_blue'])]
    evaluator.shapes['target_blue'] = evaluator.shapes['target_red']
    evaluator.health = EvaluationHealth(['robot','box_red','target_blue'])
    for key in evaluator.health.required:
        evaluator.health.observe(key)
    evaluator.health.start()
    wrong = TransportEvaluator.report(evaluator)
    assert wrong['data_valid'] and not wrong['success']  # Red box is on red pad.
    placed = np.array((4.,0.,1.009))
    evaluator.objects['box_red'] = placed
    evaluator.history['box_red'][-12:] = [(now-1.1+.1*i,placed.copy()) for i in range(12)]
    correct = TransportEvaluator.report(evaluator)
    assert correct['success'] and correct['deliveries'][0]['destination_id']=='target_blue'
