import time

import pytest
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint
from unitree_hg.msg import LowState

from unitree_g1_mujoco_bridge.arm_command_compositor import ArmCommandCompositorNode,BODY_DOF,TargetPublicationGate


@pytest.mark.parametrize('ending',['success','cancel','stale','shutdown'])
def test_single_thread_arm_action_keeps_command_and_watchdog_callbacks_live(ending):
    rclpy.init()
    server = ArmCommandCompositorNode()
    server.publication_gate = TargetPublicationGate(.02)
    client = Node('arm_async_test_client')
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client)
    action = ActionClient(client,FollowJointTrajectory,'/arm_trajectory_controller/follow_joint_trajectory')
    authority = client.create_client(SetBool,'/g1_arm_command_compositor/set_arm_authority')
    locomotion = client.create_publisher(Float64MultiArray,'/g1/locomotion_position_commands',1)
    measured = client.create_publisher(LowState,'/lowstate',1)
    published = []
    client.create_subscription(Float64MultiArray,'/g1/body_position_commands',lambda msg:published.append(msg.data),1)
    feed = [True]
    def update():
        if not feed[0]:
            return
        values = [0.]*BODY_DOF
        values[0] = .01*(len(published)%10)
        locomotion.publish(Float64MultiArray(data=values))
        state = LowState()
        for index,value in enumerate(server.core.arm_hold or [0.]*14,15):
            state.motor_state[index].q = float(value)
        measured.publish(state)
    client.create_timer(.01,update)
    def wait(predicate):
        deadline = time.monotonic()+5.
        while not predicate() and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.01)
        assert predicate()
    try:
        wait(lambda:action.server_is_ready() and authority.service_is_ready() and server.core.measured is not None)
        acquired = authority.call_async(SetBool.Request(data=True))
        wait(acquired.done)
        assert acquired.result().success
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['right_shoulder_pitch_joint']
        point = JointTrajectoryPoint(positions=[.2])
        point.time_from_start.sec = 1
        goal.trajectory.points = [point]
        accepted = action.send_goal_async(goal)
        wait(accepted.done)
        handle = accepted.result()
        assert handle.accepted
        result = handle.get_result_async()
        wait(lambda:server._execution_waiter is not None)
        before = len(published)
        wait(lambda:len(published)>=before+5)
        if ending=='cancel':
            canceled = handle.cancel_goal_async()
            wait(canceled.done)
        elif ending=='stale':
            feed[0] = False
        elif ending=='shutdown':
            server.stop_execution()
        wait(result.done)
        wrapped = result.result()
        assert wrapped.status == {'success':4,'cancel':5,'stale':6,'shutdown':6}[ending]
        if ending=='stale':
            assert 'stale' in wrapped.result.error_string
        if ending=='shutdown':
            assert wrapped.result.error_string=='ROS shutdown'
        assert server._execution_waiter is None
        assert any(row[0]>0. for row in published[before:])
    finally:
        action.destroy()
        executor.shutdown(timeout_sec=1.)
        client.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


def test_repeat_gate_never_delays_new_targets_and_retains_heartbeat():
    gate = TargetPublicationGate(.02)
    target = [0.]*29
    assert gate.accept(target,10.)
    assert not gate.accept(target,10.005)
    target[0] = .1
    assert gate.accept(target,10.006)  # Mutation of the caller's list is a new target.
    assert not gate.accept(target,10.015)
    assert gate.accept(target,10.027)
    assert gate.accept(target,1.)  # A clock regression cannot suppress output indefinitely.


def test_stale_core_is_evaluated_before_duplicate_suppression():
    import threading
    from types import SimpleNamespace as S
    from rclpy.task import Future
    from unitree_g1_mujoco_bridge.arm_command_compositor import (
        ArmCommandCompositorCore,ARM_JOINTS,parse_trajectory,TrajectoryPointData)
    core = ArmCommandCompositorCore(input_timeout=.01)
    core.update_locomotion([0.]*29,10.)
    core.update_measured([0.]*29,10.)
    assert core.acquire(10.)[0]
    core.start_trajectory(parse_trajectory([ARM_JOINTS[0]],[TrajectoryPointData((0.,),.1)],[0.]*14),10.)
    clock,published = [10.],[]
    waiter = Future()
    node = S(core=core,publication_gate=TargetPublicationGate(.02),_lock=threading.RLock(),
             _execution_waiter=waiter,_now=lambda:clock[0],_publisher=S(publish=published.append))
    ArmCommandCompositorNode._publish(node)
    clock[0] = 10.015
    ArmCommandCompositorNode._publish(node)
    assert len(published)==1 and waiter.done()
    assert waiter.result()[0]=='stale'


def test_concurrent_publication_cannot_overtake_selected_target():
    import threading
    from concurrent.futures import ThreadPoolExecutor
    from types import SimpleNamespace as S

    publishing,release,contending = threading.Event(),threading.Event(),threading.Event()
    computed,published = [],[]
    class ObservedLock:
        def __init__(self):
            self.lock = threading.RLock()
        def __enter__(self):
            if publishing.is_set():
                contending.set()
            self.lock.acquire()
        def __exit__(self,*args):
            self.lock.release()
    def output(now):
        computed.append(len(computed)+1)
        return [float(computed[-1])]*BODY_DOF
    def publish(message):
        if message.data[0]==1.:
            publishing.set()
            assert release.wait(3.)
        published.append(message.data[0])
    node = S(core=S(output=output),publication_gate=TargetPublicationGate(0.),
             _lock=ObservedLock(),_execution_waiter=None,_now=lambda:10.,_publisher=S(publish=publish))
    with ThreadPoolExecutor(max_workers=2) as workers:
        first = workers.submit(ArmCommandCompositorNode._publish,node)
        try:
            assert publishing.wait(3.)
            second = workers.submit(ArmCommandCompositorNode._publish,node)
            assert contending.wait(3.)
            assert computed==[1] and published==[]
        finally:
            release.set()
        first.result(timeout=3.)
        second.result(timeout=3.)
    assert computed==[1,2] and published==[1.,2.]
