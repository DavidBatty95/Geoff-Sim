"""Sensor-driven demonstration: discover, dock, attach, carry and place.

The grasp actuator is the documented capture-radius weld. No base pin, object
pose setter, simulator layout or evaluation subscription is used by the mission.
"""
import argparse
import hashlib
import json
import math
import time
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import PoseStamped,PointStamped,Vector3Stamped,Polygon,Point32,TwistStamped
from nav_msgs.msg import Odometry,Path as RosPath
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool,String
from std_srvs.srv import SetBool,Trigger
from g1_msgs.srv import GraspObject
from unitree_social_navigation_msgs.msg import TrackedPersons
from vision_msgs.msg import Detection3DArray
from tf2_ros import Buffer,TransformListener,TransformException

from .arm_kinematics import ArmKinematics,rpy_matrix
from .transport_perception import TransportPerception
from .transport_actions import ManagedAction
from .transport_health import EvaluationHealth
from .payload_postures import PAYLOAD_V1_POSTURES
from .runtime_parameters import capture_parameters
from .localization_recovery import BodyInstabilityError,LocalizationInterrupted,recover_localization
from .transport_people import closest_person_distance


def angle(value):
    return math.atan2(math.sin(value),math.cos(value))


def navigation_footprint(radius):
    """Circumscribed polygon for the planner's circular navigation reserve."""
    if not math.isfinite(radius) or radius<=0.:
        raise ValueError('positive finite navigation radius required')
    outer = radius/math.cos(math.pi/16)+1e-6
    return Polygon(points=[Point32(x=outer*math.cos(2*math.pi*i/16),
                                   y=outer*math.sin(2*math.pi*i/16),z=0.) for i in range(16)])


def turn_command_drives_heading(feedback,now,error):
    if feedback is None:
        return False
    stamp,command = feedback
    return (0.<=now-stamp<=.3 and math.hypot(*command[:2])<.1 and
            abs(command[2])>=.2 and command[2]*error>0.)


class WorkspaceInterrupted(RuntimeError):
    """A terminated arm action may be retried after measured workspace recovery."""


class ReachUnreachable(RuntimeError):
    """No trajectory was dispatched because sensed geometry is out of reach."""


class MotionSettlingError(RuntimeError):
    """Normal motion completion lacks measured settling; do not advance phase."""

    def __init__(self,outcome):
        self.outcome = outcome
        super().__init__(f'base did not settle during braking: {outcome}')


class TransportMission(Node):
    def __init__(self,args,executor):
        super().__init__('g1_transport_demo')
        self.args,self.executor = args,executor
        self.ik_worker = ThreadPoolExecutor(max_workers=1)
        self.rest_arm_positions = None
        self.pose = None
        self.joints = {}
        self.rpy = (0.,0.,0.)
        self.observations = {}
        self.search_hints = {}
        self.support_memory = []
        self.last_image_pose = None
        self.last_image_hfov = math.pi/3
        self.last_image_stamp = None
        self.exploration = None
        self.people = None
        self.policy_feedback = None
        self.kin = None
        self.active = False
        self.mode = 'navigate'
        self.attachment_state = 'empty'
        self.offset = np.zeros(3)
        self.grasp_R = None
        self.goal = None
        self.events = []
        self.phase = 'initializing'
        self.started = time.monotonic()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.goal_pub = self.create_publisher(PoseStamped,'/benchmark/current_goal',1)
        self.active_pub = self.create_publisher(Bool,'/benchmark/goal_active',1)
        self.mode_pub = self.create_publisher(String,'/navigation/goal_mode',1)
        self.carry_pub = self.create_publisher(Bool,'/navigation/carrying',1)
        self.payload_pub = self.create_publisher(PointStamped,'/navigation/payload_pose',1)
        self.path_pub = self.create_publisher(RosPath,'/plan',1)
        self.status_pub = self.create_publisher(String,'/transport/mission_state',10)
        self.footprint_pubs = [self.create_publisher(Polygon,topic,1) for topic in
                               ('/local_costmap/footprint','/global_costmap/footprint')]
        self.navigation_empty_radius = .48
        self.navigation_radius = None
        self.last_footprint_time = -math.inf
        self.nav = ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.arm = ActionClient(self,FollowJointTrajectory,'/arm_trajectory_controller/follow_joint_trajectory')
        self.authority = self.create_client(SetBool,'/g1_arm_command_compositor/set_arm_authority')
        self.grasp = self.create_client(GraspObject,'/g1_sim_manipulation_io/grasp_object')
        self.release = self.create_client(Trigger,'/g1_sim_manipulation_io/release_right')
        qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        durable = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String,'/robot_description',self._description,durable)
        self.create_subscription(String,'/transport/observations',self._observations,10)
        self.create_subscription(Odometry,'/odom',self._odom,qos)
        self.create_subscription(JointState,'/joint_states',lambda m:setattr(self,'joints',dict(zip(m.name,m.position))),qos)
        self.create_subscription(Vector3Stamped,'/onboard/measured_rpy',self._body_orientation,qos)
        self.create_subscription(TrackedPersons,'/tracked_persons',lambda m:setattr(self,'people',m),qos)
        self.create_subscription(TwistStamped,'/g1/policy_cmd_vel',self._policy_feedback,qos)
        self.create_timer(.1,self._heartbeat)
        if getattr(args,'explore_unknown',False):
            from .exploration_mission import ExplorationMission
            self.exploration = ExplorationMission(self,args.search_max_viewpoints)

    def seconds(self):
        return self.get_clock().now().nanoseconds*1e-9

    @property
    def held(self):
        return self.attachment_state == 'held'

    @property
    def object_may_be_held(self):
        return self.attachment_state != 'empty'

    def _description(self,msg):
        if self.kin is None:
            # Measured point on the native rubber hand, in wrist coordinates.
            self.kin = ArmKinematics(msg.data,tool_offset=(.12,.01,0.))

    def _observations(self,msg):
        data = json.loads(msg.data)
        if data.get('frame') == 'odom':
            image_stamp = data.get('image_stamp')
            if isinstance(image_stamp,(int,float)) and math.isfinite(image_stamp) and 0.<=self.seconds()-image_stamp<=5.:
                if image_stamp>= (self.last_image_stamp or -math.inf):
                    self.last_image_stamp = image_stamp
                    pose = data.get('image_pose')
                    self.last_image_pose = tuple(pose) if pose is not None and len(pose)==3 and all(map(math.isfinite,pose)) else None
                    hfov = data.get('image_hfov',math.pi/3)
                    if isinstance(hfov,(int,float)) and math.isfinite(hfov) and 0.<hfov<math.pi:
                        self.last_image_hfov = hfov
                memory = getattr(self,'support_memory',[])
                for surface in data.get('surfaces',[]):
                    point = surface.get('position',[])
                    stamp = surface.get('observed_stamp',-math.inf)
                    if len(point)!=3 or not all(map(math.isfinite,point)) or not 0.<=self.seconds()-stamp<=5.:
                        continue
                    memory = [old for old in memory if math.dist(old['position'][:2],point[:2])>.6]
                    memory.append(surface)
                self.support_memory = [item for item in memory if 0.<=self.seconds()-item['observed_stamp']<120.][-32:]
            self.observations = {item['id']:item for item in data['objects']}
            for identifier,item in self.observations.items():
                if item.get('confirmed',False):
                    self.search_hints[identifier] = dict(item)

    def _odom(self,msg):
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        self.pose = (p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        self.pose_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        velocity = msg.twist.twist
        self.base_velocity = velocity.linear.x,velocity.linear.y,velocity.angular.z

    def _policy_feedback(self,msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        values = msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z
        if all(map(math.isfinite,(*values,stamp))):
            self.policy_feedback = stamp,values

    def _body_orientation(self,msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        rpy = msg.vector.x,msg.vector.y,msg.vector.z
        previous = getattr(self,'rpy_stamp',-math.inf)
        if not all(map(math.isfinite,(*rpy,stamp))) or stamp<=previous:
            return
        if max(abs(rpy[0]),abs(rpy[1]))>.7:
            if getattr(self,'unstable_since',None) is None or stamp-previous>.25:
                self.unstable_since = stamp
        else:
            self.unstable_since = None
        self.rpy,self.rpy_stamp = rpy,stamp

    def _heartbeat(self):
        self.active_pub.publish(Bool(data=self.active))
        self.mode_pub.publish(String(data=self.mode))
        self.carry_pub.publish(Bool(data=self.object_may_be_held))
        radius = self.navigation_empty_radius+(.16 if self.object_may_be_held else 0.)
        if radius!=self.navigation_radius or time.monotonic()-self.last_footprint_time>=1.:
            footprint = navigation_footprint(radius)
            for publisher in self.footprint_pubs:
                publisher.publish(footprint)
            self.navigation_radius,self.last_footprint_time = radius,time.monotonic()
        self.status_pub.publish(String(data=json.dumps(dict(phase=self.phase,held=self.held,
            attachment_state=self.attachment_state,object_may_be_held=self.object_may_be_held,
            navigation_radius_m=self.navigation_radius,
            elapsed=time.monotonic()-self.started))))
        if self.active and self.mode in ('dock','scan','reserve_recovery') and self.pose is not None and self.goal is not None:
            path = RosPath()
            path.header.frame_id = 'odom'
            path.header.stamp = self.get_clock().now().to_msg()
            current = self.pose_message(self.pose,'odom')
            path.poses = [current,self.goal]
            self.path_pub.publish(path)
        if self.object_may_be_held and self.kin is not None:
            wrist,R,_,_ = self.kin.forward(self.joints)
            point = rpy_matrix((self.rpy[0],self.rpy[1],0.))@(wrist+R@self.offset)
            point[2] += self.joints.get('floating_base_joint',.793)
            msg = PointStamped()
            msg.header.frame_id = 'base_footprint'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x,msg.point.y,msg.point.z = map(float,point)
            self.payload_pub.publish(msg)

    def event(self,phase,**details):
        self.phase = phase
        self.events.append(dict(phase=phase,elapsed_s=time.monotonic()-self.started,**details))
        self.get_logger().info(f'{phase}: {details}')

    def spin(self,duration=.05):
        end = time.monotonic()+duration
        while rclpy.ok() and time.monotonic()<end:
            # rclpy caches its ready-callback iterator by spin arguments. A
            # different remaining-time float on every call restarted that
            # iterator and could starve odometry behind busy TF/action inputs.
            # A fixed quantum preserves fairness; deadline overshoot is <=10 ms
            # plus the callback runtime, without relaxing freshness checks.
            self.executor.spin_once(timeout_sec=.01)
        unstable = getattr(self,'unstable_since',None)
        if (unstable is not None and 0.<=self.seconds()-self.rpy_stamp<=.3 and
                self.rpy_stamp-unstable>=.2):
            self.stop_intent()
            self.event('body_instability',rpy=list(self.rpy),duration_s=self.rpy_stamp-unstable,
                       source='onboard_measured_rpy')
            raise BodyInstabilityError('sustained measured body instability')
        if time.monotonic()-self.started > self.args.duration:
            raise RuntimeError('mission deadline exceeded')
        if self.active and self.pose is not None and self.seconds()-self.pose_stamp > 2.:
            self.event(self.phase,localization_interruption=True,pose_age_s=self.seconds()-self.pose_stamp)
            raise LocalizationInterrupted('measured odometry stale during motion')

    def wait_localization(self,deadline,recovery):
        phase = self.phase
        self.event('wait_localization',resume_phase=phase,recovery=recovery)
        stamps = []
        def fresh():
            if self.pose is None or not 0.<=self.seconds()-self.pose_stamp<=.5:
                stamps.clear()
                return False
            if not stamps or self.pose_stamp>stamps[-1]:
                stamps.append(self.pose_stamp)
            return len(stamps)>=3
        self.wait(fresh,min(5.,max(0.,deadline-time.monotonic())),
                  'localization did not recover before deadline')
        self.event(phase,localization_recovered=True,recovery=recovery)

    def wait(self,predicate,timeout,message):
        end = time.monotonic()+timeout
        while rclpy.ok() and time.monotonic()<end:
            if predicate():
                return
            self.spin()
        raise RuntimeError(message)

    def future(self,future,timeout=15.):
        self.wait(future.done,timeout,'operation timed out')
        return future.result()

    def action(self,name,client,goal):
        return ManagedAction(name,client.send_goal_async(goal),self.future,
            lambda timeout:self.executor.spin_once(timeout_sec=timeout),rclpy.ok,
            lambda **details:self.event(self.phase,**details),before_cleanup=self.stop_intent)

    def stop_intent(self):
        self.active = False
        self._heartbeat()

    def brake_motion(self,timeout=5.):
        """Normal completion requests checked deceleration before relinquishing.

        Returns 'settled' or raises MotionSettlingError with a recorded outcome.
        Fault cleanup continues to use immediate stop_intent, never this wait.
        The planner and supervisor check every braking request against people.
        """
        if not math.isfinite(timeout) or timeout<0.:
            raise ValueError('braking timeout must be finite and nonnegative')
        self.mode = 'brake'
        self._heartbeat()
        started = time.monotonic()
        deadline = started+timeout
        stable = None
        previous_stamp = None
        outcome = 'timeout'
        while time.monotonic()<deadline:
            self.spin(min(.05,max(0.,deadline-time.monotonic())))
            if time.monotonic()>=deadline:
                break
            velocity = getattr(self,'base_velocity',(math.inf,math.inf,math.inf))
            stamp = getattr(self,'pose_stamp',-math.inf)
            fresh = (0.<=self.seconds()-stamp<=.3 and all(map(math.isfinite,velocity)))
            outcome = 'timeout' if fresh else 'stale_state'
            if previous_stamp is not None and stamp<previous_stamp:
                stable = None
            previous_stamp = stamp
            if (fresh and math.hypot(*velocity[:2])<.08 and abs(velocity[2])<.12):
                stable = stamp if stable is None else stable
                if stamp-stable>=.2:
                    self.event(self.phase,brake_outcome='settled',brake_elapsed_s=time.monotonic()-started)
                    return 'settled'
            else:
                stable = None
        self.event(self.phase,brake_outcome=outcome,brake_elapsed_s=time.monotonic()-started)
        raise MotionSettlingError(outcome)

    def service(self,client,request,*,attachment_after=None):
        self.wait(client.service_is_ready,15.,'manipulation service unavailable')
        if attachment_after is not None:
            # Once dispatched, timeout/rejection cannot prove actuator state.
            # Keep authority and conservative payload geometry on every failure.
            self.attachment_state = 'uncertain'
            self.event(self.phase,attachment_state=self.attachment_state)
            self._heartbeat()
        response = self.future(client.call_async(request))
        if not response.success:
            raise RuntimeError(response.message)
        if attachment_after is not None:
            self.attachment_state = attachment_after
            self.event(self.phase,attachment_state=self.attachment_state)
            self._heartbeat()
        return response

    @staticmethod
    def pose_message(pose,frame):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.pose.position.x,msg.pose.position.y = float(pose[0]),float(pose[1])
        msg.pose.orientation.z,msg.pose.orientation.w = math.sin(pose[2]/2),math.cos(pose[2]/2)
        return msg

    def set_goal(self,pose,mode):
        self.mode = mode
        self.goal = self.pose_message(pose,'odom')
        self.goal.header.stamp = self.get_clock().now().to_msg()
        if mode == 'navigate':
            transform = self.tf.lookup_transform('map','odom',rclpy.time.Time()).transform
            q = transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.goal = self.pose_message((transform.translation.x+math.cos(yaw)*pose[0]-math.sin(yaw)*pose[1],
                transform.translation.y+math.sin(yaw)*pose[0]+math.cos(yaw)*pose[1],pose[2]+yaw),'map')
            self.goal.header.stamp = self.get_clock().now().to_msg()
        self.active = True
        self.goal_pub.publish(self.goal)
        self._heartbeat()
        self.event(self.phase,goal_odom=list(pose),start_pose=self.pose,mode=mode)

    @recover_localization
    def navigate(self,pose,timeout=90.,stop_when_observed=None,settle_time=1.):
        deadline = time.monotonic()+timeout
        discovery = None
        self.wait(self.nav.server_is_ready,min(30.,timeout),'Nav2 unavailable')
        self.set_goal(pose,'navigate')
        request = NavigateToPose.Goal()
        request.pose = self.goal
        try:
            with self.action('navigation',self.nav,request) as action:
                def arrived():
                    nonlocal discovery
                    observed = self.observe(stop_when_observed,5.) if stop_when_observed is not None else None
                    if observed is not None:
                        discovery = dict(observed)
                        # ManagedAction cancels and awaits termination before
                        # handing the observed object to the next mission phase.
                        return True
                    close = (self.pose is not None and 0.<=self.seconds()-self.pose_stamp<=.5 and
                             math.hypot(self.pose[0]-pose[0],self.pose[1]-pose[1]) < .50)
                    if action.result.done():
                        wrapped = action.result.result()
                        if wrapped.status != 4:
                            raise RuntimeError(f'navigation terminated with status {wrapped.status}')
                        if not close:
                            # Nav2's map-frame goal may finish while the sensed
                            # odom pose lags or map->odom changes during yielding.
                            # Use the existing bounded localization retry to
                            # reobserve/reissue, never accept a stale arrival.
                            raise LocalizationInterrupted('navigation succeeded outside fresh sensed staging tolerance')
                    return close
                self.wait(arrived,max(0.,deadline-time.monotonic()),'navigation did not reach sensed staging pose')
                self.brake_motion(min(5.,max(0.,deadline-time.monotonic())))
        finally:
            self.active = False
            self._heartbeat()
        self.spin(max(0.,settle_time))
        # Discovery evidence can become occluded during braking/cancellation.
        # Preserve the acquisition that caused the stop; manipulation still
        # independently requires a fresh observation at its staging pose.
        return discovery

    @recover_localization
    def dock(self,pose,timeout=75.):
        deadline = time.monotonic()+timeout
        corrections = 0
        settling_bias = np.zeros(2)
        def arrived(position_tolerance=.07,target=pose):
            return (self.pose is not None and 0. <= self.seconds()-self.pose_stamp <= .5 and
                    math.hypot(self.pose[0]-target[0],self.pose[1]-target[1]) < position_tolerance and
                    abs(angle(self.pose[2]-target[2])) < .15)
        try:
            while time.monotonic() < deadline:
                target = (pose[0]-settling_bias[0],pose[1]-settling_bias[1],pose[2])
                self.set_goal(target,'dock')
                # Approach inside the final acceptance band before stopping:
                # measured gait settling can move the base back by 3--4 cm.
                self.wait(lambda:arrived(.035,target),max(0.,deadline-time.monotonic()),'precision docking failed')
                stopped_at = np.asarray(self.pose[:2])
                self.stop_intent()
                self.spin(min(1.5,max(0.,deadline-time.monotonic())))
                if time.monotonic() < deadline and arrived():
                    self.event(self.phase,dock_settled=True,corrections=corrections,settled_pose=self.pose)
                    return
                corrections += 1
                if 0.<=self.seconds()-self.pose_stamp<=.5:
                    settling_bias = .5*settling_bias+.5*(np.asarray(self.pose[:2])-stopped_at)
                    length = float(np.linalg.norm(settling_bias))
                    if length>.04:
                        settling_bias *= .04/length
                self.event(self.phase,dock_settled=False,corrections=corrections,settled_pose=self.pose,
                           settling_compensation_m=settling_bias.tolist())
            raise RuntimeError('precision docking did not settle before deadline')
        finally:
            self.stop_intent()

    def observe(self,identifier,max_age=5.):
        item = self.observations.get(identifier)
        return item if item is not None and item.get('confirmed',False) and 0. <= self.seconds()-item['observed_stamp'] <= max_age else None

    def retreat(self,distance,yaw,timeout=45.):
        """Clear a work surface without chasing an unnecessarily exact endpoint."""
        start = np.array(self.pose[:2])
        forward = np.array((math.cos(yaw),math.sin(yaw)))
        target = start-distance*forward
        deadline = time.monotonic()+timeout
        def clear(reserve=.07):
            return (0. <= self.seconds()-self.pose_stamp <= .5 and
                    float((start-np.array(self.pose[:2]))@forward) >= distance-reserve)
        try:
            while time.monotonic()<deadline:
                if not clear():
                    self.set_goal((*target,yaw),'dock')
                    # Cross an inner band before stopping; otherwise millimetre
                    # gait oscillations repeatedly satisfy the moving check and
                    # fail the unchanged settled clearance requirement.
                    self.wait(lambda:clear(.03),max(0.,deadline-time.monotonic()),'retreat clearance not reached')
                self.stop_intent()
                self.spin(min(1.5,max(0.,deadline-time.monotonic())))
                if time.monotonic()<deadline and clear():
                    self.event(self.phase,retreat_settled=True,
                               distance_m=float((start-np.array(self.pose[:2]))@forward))
                    return
            raise RuntimeError('retreat did not settle before deadline')
        finally:
            self.stop_intent()

    def placement_observed(self,identifier,pad,release_stamp):
        item = self.observe(identifier)
        return bool(item is not None and item['observed_stamp'] > release_stamp and
            np.linalg.norm(np.array(item['position'])[:2]-pad[:2]) < .13 and
            abs(item['position'][2]-pad[2]-.055) < .06)

    def release_and_verify(self,identifier,destination,pad,forward,yaw):
        self.event('release')
        self.service(self.release,Trigger.Request(),attachment_after='empty')
        release_stamp = self.seconds()
        # A torso-mounted camera cannot see the tabletop directly below it.
        # Clear the hand and retreat to an observation viewpoint before asking
        # for fresh evidence. Never substitute the pre-release observation.
        self.event('retract',release_ack_stamp=release_stamp)
        self.move_to_carry(self.grasp_R)
        self.retreat(.5,yaw)
        self.event('verify_place',object_id=identifier,destination_id=destination,release_ack_stamp=release_stamp)
        self.spin(2.)
        self.wait(lambda: self.placement_observed(identifier,pad,release_stamp),
                  8.,'camera did not verify post-release placement from retreat viewpoint')

    @recover_localization
    def rotate(self,yaw,timeout=45.,settle_before_turn=False,stop_when_observed=None):
        # Search requires a view direction, not centimetre-level base-position
        # recovery after every step of a loaded turn.
        deadline = time.monotonic()+timeout
        if settle_before_turn:
            self.stop_intent()
            stable_since = None
            def settled():
                nonlocal stable_since
                velocity = getattr(self,'base_velocity',None)
                valid = (velocity is not None and all(map(math.isfinite,velocity)) and
                         0.<=self.seconds()-self.pose_stamp<=.3 and
                         math.hypot(*velocity[:2])<.08 and abs(velocity[2])<.15)
                if not valid:
                    stable_since = None
                    return False
                if stable_since is None:
                    stable_since = self.pose_stamp
                return self.pose_stamp-stable_since>=.2
            self.wait(settled,min(2.,max(0.,deadline-time.monotonic())),
                      'base did not settle before inspection turn')
        self.set_goal((self.pose[0],self.pose[1],yaw),'scan')
        turn_origin = tuple(self.pose[:2])
        drifted = False
        discovery = None
        best_error = abs(angle(self.pose[2]-yaw))
        progress_at = time.monotonic()
        def aligned():
            nonlocal best_error,progress_at,discovery,drifted
            observed = self.observe(stop_when_observed,2.) if stop_when_observed is not None else None
            if observed is not None:
                discovery = dict(observed)
                return True
            if not 0.<=self.seconds()-self.pose_stamp<=.5:
                return False
            drift = math.dist(self.pose[:2],turn_origin)
            if drift>.35:
                # A view turn is not permission for metres of unplanned travel.
                # Stop the scan through checked nominal braking; exploration
                # can then inspect/replan from the actual acquisition pose.
                drifted = True
                self.event('turn_displaced',displacement_m=drift,limit_m=.35,
                           start_pose_xy=list(turn_origin),measured_pose=list(self.pose))
                return True
            error = abs(angle(self.pose[2]-yaw))
            if not turn_command_drives_heading(getattr(self,'policy_feedback',None),self.seconds(),angle(yaw-self.pose[2])):
                # Safety/people intervention is not evidence of a weak turn.
                # Keep the original overall deadline, but do not fail early
                # while the checked command intentionally stops or sidesteps.
                progress_at = time.monotonic()
            if error<best_error-.03:
                best_error,progress_at = error,time.monotonic()
            if error<.14:
                return True
            if time.monotonic()-progress_at>6.:
                self.event('turn_stalled',heading_error_rad=error,measured_velocity=getattr(self,'base_velocity',None))
                raise RuntimeError('search heading stalled')
            return False
        try:
            self.wait(aligned,max(0.,deadline-time.monotonic()),'search heading not reached')
            self.brake_motion(min(5.,max(0.,deadline-time.monotonic())))
            if drifted:
                raise RuntimeError('search turn displaced beyond viewpoint reserve')
        finally:
            self.active = False
            self._heartbeat()
        self.spin(.2)
        return discovery

    def discover(self,identifier):
        if getattr(self,'exploration',None) is not None:
            return self.exploration.discover(identifier)
        self.event('discover',object_id=identifier)
        hint = self.search_hints.get(identifier)
        if self.observe(identifier,20.) is None and hint is not None:
            # Historical sensor geometry is only a viewing-direction cue. It
            # cannot satisfy discovery, docking, grasp or placement freshness.
            point = hint['position']
            self.event('discover',search_hint=identifier,hint_age_s=self.seconds()-hint['observed_stamp'])
            self.rotate(math.atan2(point[1]-self.pose[1],point[0]-self.pose[0]))
            self.spin(1.5)
            if self.observe(identifier,20.) is None:
                delta = np.array(point[:2])-np.array(self.pose[:2])
                distance = float(np.linalg.norm(delta))
                if distance>1.6:
                    # A distant outline may be below useful RGB-D resolution.
                    # Request a Nav2-checked observation viewpoint, stopping
                    # short of the remembered position; reacquisition remains
                    # mandatory before any tabletop approach or manipulation.
                    view = np.array(point[:2])-1.4*delta/distance
                    yaw = math.atan2(delta[1],delta[0])
                    self.event('discover',search_viewpoint=identifier,viewpoint_odom=view.tolist())
                    self.navigate((*view,yaw),timeout=30.)
                    self.rotate(math.atan2(point[1]-self.pose[1],point[0]-self.pose[0]))
                    self.spin(1.5)
        for _ in range(9):
            if self.observe(identifier,20.) is not None:
                return self.observe(identifier,20.)
            self.rotate(angle(self.pose[2]+math.pi/4))
            self.spin(1.5)
        raise RuntimeError(f'RGB-D did not discover {identifier}')

    def workspace_clear(self):
        if self.people is None:
            return False
        stamp = self.people.header.stamp.sec+self.people.header.stamp.nanosec*1e-9
        age = self.seconds()-stamp
        if not 0. <= age <= .8 or self.people.header.frame_id != 'odom':
            return False
        for person in self.people.persons:
            p,v = person.pose.pose.position,person.velocity.twist.linear
            if closest_person_distance((p.x,p.y),(v.x,v.y),self.pose[:2],age)<1.6:
                return False
        return True

    def to_pelvis(self,position):
        relative = np.asarray(position)-np.array((self.pose[0],self.pose[1],self.joints.get('floating_base_joint',.793)))
        return rpy_matrix((self.rpy[0],self.rpy[1],self.pose[2])).T@relative

    def arm_with_recovery(self,target,orientation=None,joint_solution=None):
        for recovery in range(3):
            anchor = self.pose
            try:
                return self._move_arm(target,orientation,joint_solution=joint_solution)
            except WorkspaceInterrupted:
                if recovery == 2:
                    raise RuntimeError('arm workspace recovery limit exceeded; authority retained')
                phase = self.phase
                self.event('wait_for_clearance',resume_phase=phase,recovery=recovery+1)
                self.wait_workspace()
                base_moved = (math.hypot(self.pose[0]-anchor[0],self.pose[1]-anchor[1]) > .04 or
                              abs(angle(self.pose[2]-anchor[2])) > .10)
                if base_moved and joint_solution is None:
                    # The world-referenced reach must be re-observed/reprojected
                    # by reach_box; never replay the old pelvis-frame target.
                    raise ReachUnreachable('base moved during arm interruption; re-docking required')
                # Fixed carry/rest joint postures do not depend on base x/y/yaw.
                # _move_arm starts the new trajectory at the latest measured
                # joints and rechecks body tilt and workspace throughout.
                self.event(phase,recovery=recovery+1,resumed=True,base_moved=base_moved,
                           target_kind='joint_posture' if joint_solution is not None else 'cartesian')

    def move_arm(self,target,orientation=None):
        desired = np.asarray(target,dtype=float)
        correction = np.zeros(3)
        for attempt in range(3):
            measured = self.arm_with_recovery(desired+correction,orientation)
            error = desired-measured
            distance = float(np.linalg.norm(error))
            self.event(self.phase,hand_error_m=distance,servo_iteration=attempt)
            if distance <= .025:
                return
            correction += .8*error
            if np.linalg.norm(correction) > .08:
                break
        raise RuntimeError(f'measured hand missed target ({distance:.3f} m)')

    def wait_workspace(self,timeout=30.,dwell=.3):
        clear_since = None
        def ready():
            nonlocal clear_since
            if max(abs(self.rpy[0]),abs(self.rpy[1])) > .35:
                raise RuntimeError('body tilt exceeded manipulation limit while waiting')
            if not self.workspace_clear():
                clear_since = None
                return False
            now = time.monotonic()
            if clear_since is None:
                clear_since = now
            return now-clear_since >= dwell
        self.wait(ready,timeout,'manipulation workspace did not remain clear')

    def move_to_carry(self,orientation=None):
        if self.args.carry_posture == 'ik':
            return self.move_arm(np.array((.36,-.20,.37)),orientation)
        # Keep the locomotion actor inside its explicitly trained arm-posture
        # distribution instead of selecting a new redundant IK branch each lift.
        self.move_arm_posture(dict(zip(self.kin.names,PAYLOAD_V1_POSTURES[0])),'carry')

    def measured_posture_state(self):
        """Do not let FK's zero defaults stand in for measured arm/waist joints."""
        joints = dict(self.joints)
        required = [name for name,kind,*_ in self.kin.chain if kind in ('revolute','continuous')]
        if any(name not in joints or not math.isfinite(joints[name]) for name in required):
            raise RuntimeError('fixed arm posture requires finite measured arm and waist joints')
        return joints

    def move_arm_posture(self,posture,label):
        if (posture is None or set(posture)!=set(self.kin.names) or
                not all(math.isfinite(value) for value in posture.values())):
            raise RuntimeError('fixed arm posture requires a complete finite arm-only target')
        # Snapshot the requested arm joints; never freeze policy-owned waist
        # coordinates into a joint-space completion reference.
        posture = dict(posture)
        joints = self.measured_posture_state()
        position,orientation,_,_ = self.kin.forward(dict(joints,**posture))
        self.arm_with_recovery(position,orientation,joint_solution=posture)
        joints = self.measured_posture_state()
        expected = self.kin.forward(dict(joints,**posture))[0]
        measured = self.kin.forward(joints)[0]
        error = float(np.linalg.norm(measured-expected))
        self.event(self.phase,**{label+'_posture_error_m':error},
                   reference='current_waist_fixed_arm_posture',
                   posture_joint_error_rad=max(abs(joints[name]-value) for name,value in posture.items()))
        if not math.isfinite(error) or error > .035:
            raise RuntimeError(f'measured {label} posture outside tolerance')

    def restore_arm(self):
        self.event('restore_arm')
        self.move_arm_posture(self.rest_arm_positions,'rest')
        self.service(self.authority,SetBool.Request(data=False))

    def _move_arm(self,target,orientation=None,joint_solution=None):
        self.wait_workspace()
        if joint_solution is None:
            # Numerical IK must not prevent servicing measured-state callbacks.
            solution,error = self.future(self.ik_worker.submit(self.kin.solve,target,dict(self.joints),orientation))
        else:
            solution,error = joint_solution,0.
        self.event(self.phase,hand_target_pelvis=np.asarray(target).tolist(),ik_error_m=error)
        if error > .012:
            raise ReachUnreachable(f'hand target unreachable ({error:.3f} m)')
        # IK can outlast a people measurement. Never dispatch using the snapshot
        # that was fresh before the solve; revalidate after computation.
        if not self.workspace_clear():
            raise WorkspaceInterrupted('workspace changed during IK')
        names = self.kin.names
        initial = [float(self.joints.get(name,0.)) for name in names]
        positions = [solution[name] for name in names]
        duration = max(2.,max(abs(a-b) for a,b in zip(initial,positions))/.35)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        first = JointTrajectoryPoint()
        first.positions = initial
        final = JointTrajectoryPoint()
        final.positions = positions
        final.time_from_start.sec = int(duration)
        final.time_from_start.nanosec = int((duration-int(duration))*1e9)
        goal.trajectory.points = [first,final]
        self.event(self.phase,commanded_joints=dict(zip(names,positions)),duration_s=duration)
        self.wait(self.arm.server_is_ready,15.,'arm compositor unavailable')
        with self.action('arm',self.arm,goal) as action:
            end = time.monotonic()+duration+8.
            while not action.result.done():
                if max(abs(self.rpy[0]),abs(self.rpy[1])) > .35:
                    self.event(self.phase,interruption='body_tilt',body_rpy=list(self.rpy))
                    raise RuntimeError('body tilt exceeded manipulation limit')
                if not self.workspace_clear() or time.monotonic()>end:
                    stamp = None if self.people is None else self.people.header.stamp.sec+self.people.header.stamp.nanosec*1e-9
                    self.event(self.phase,interruption='workspace_or_timeout',people_age_s=None if stamp is None else self.seconds()-stamp,
                               people_frame=None if self.people is None else self.people.header.frame_id,
                               people_count=None if self.people is None else len(self.people.persons),arm_timeout=time.monotonic()>end)
                    if time.monotonic()>end:
                        raise RuntimeError('arm trajectory timed out; authority retained')
                    raise WorkspaceInterrupted('arm workspace interrupted; authority retained')
                self.spin()
            wrapped = action.result.result()
        self.event(self.phase,joint_target=dict(zip(names,positions)),
                   joint_measured={name:self.joints.get(name) for name in names},body_rpy=list(self.rpy))
        if wrapped.status != 4 or wrapped.result.error_code != 0:
            raise RuntimeError(f'arm execution failed: {wrapped.result.error_string}')
        self.spin(.6)
        measured = self.kin.forward(self.joints)[0]
        reference = (self.kin.forward(dict(self.joints,**{name:solution[name] for name in names}))[0]
                     if joint_solution is not None else target)
        measured_error = float(np.linalg.norm(measured-reference))
        self.event(self.phase,measured_hand_pelvis=measured.tolist(),hand_error_m=measured_error,
                   reference='current_waist_fixed_arm_posture' if joint_solution is not None else 'cartesian_target')
        return measured

    def reach_box(self,identifier,forward,left,yaw):
        # Reproject after each attempt: moving the arm shifts an unpinned base.
        for correction in range(3):
            self.wait(lambda: self.observe(identifier,20.) is not None,8.,'tabletop box observation expired')
            box = np.array(self.observe(identifier,20.)['position'])
            try:
                self.move_arm(self.to_pelvis(box)+np.array((-.055,0.,0.)),np.eye(3))
            except ReachUnreachable:
                if correction == 2:
                    raise
                self.event('redock_pickup',recovery=correction+1)
                self.dock((*(box[:2]-.43*forward+.15*left),yaw))
                continue
            # A stationary tabletop box can be occluded by the reaching hand.
            # Retain its recent estimate, but never enlarge attachment tolerance.
            self.wait(lambda: self.observe(identifier,20.) is not None,8.,'box observation expired at attachment')
            box = np.array(self.observe(identifier,20.)['position'])
            hand_distance = float(np.linalg.norm(self.kin.forward(self.joints)[0]-self.to_pelvis(box)))
            self.event('attachment_check',hand_to_box_m=hand_distance,correction=correction)
            if hand_distance <= .085:
                return box
        raise RuntimeError('measured hand outside magnetic attachment range')

    def run(self):
        self.wait(lambda: self.pose is not None and self.kin is not None and len(self.joints)>20,40.,'robot state unavailable')
        from dynamic_obstacle_benchmarks.source_contract import audit_onboard_graph
        self.wait(lambda: not audit_onboard_graph(self),30.,'onboard source audit failed')
        snapshot = capture_parameters(self,lambda seconds:self.spin(seconds))
        encoded = (json.dumps(snapshot,indent=2,allow_nan=False)+'\n').encode()
        self.args.output.parent.mkdir(parents=True,exist_ok=True)
        (self.args.output.parent/'effective_parameters.json').write_bytes(encoded)
        self.event('runtime_parameters',required_complete=snapshot['required_complete'],
                   sha256=hashlib.sha256(encoded).hexdigest(),
                   incomplete_required_nodes=snapshot['incomplete_required_nodes'])
        if not snapshot['required_complete']:
            raise RuntimeError('required effective parameter snapshot incomplete')
        if not snapshot['motion_parameter_agreement']['consistent']:
            raise RuntimeError('planner/supervisor/policy motion parameter mismatch; see effective_parameters.json')
        self.navigation_empty_radius = float(snapshot['nodes']['/human_aware_motion_planner']['parameters']['static_clearance']['value'])
        navigation_footprint(self.navigation_empty_radius)  # Validate before any motion.
        self._heartbeat()
        self.spin(3.)
        for pair in self.args.pairs:
            identifier,destination = pair['object_id'],pair['destination_id']
            # Observe the destination with an empty hand, before a carried cube
            # can obscure it. This is measured scene memory, not a layout lookup;
            # placement still requires a recent complete destination observation.
            destination_view = None
            if self.exploration is None:
                self.event('survey_destination',destination_id=destination)
                destination_view = dict(self.discover(destination))
            item = self.discover(identifier)
            if self.exploration is not None:
                # A loaded blind search is slower and can obscure the camera.
                # Establish both task locations while empty, then reobserve the
                # box and destination at their respective manipulation stages.
                if self.observe(destination,30.) is None and self.exploration.survey_before_pickup(item,destination):
                    self.event('survey_task_goal',object_id=identifier,destination_id=destination,
                               reason='avoid_loaded_blind_search')
                    destination_view = dict(self.discover(destination))
                item = self.exploration.inspect_support(identifier,item)
            box = np.array(item['position'])
            yaw = item.get('approach_yaw',math.atan2(box[1]-self.pose[1],box[0]-self.pose[0]))
            self.event('pickup_approach',source='observed_support_edge' if 'approach_yaw' in item else 'box_bearing',yaw=yaw)
            forward,left = np.array((math.cos(yaw),math.sin(yaw))),np.array((-math.sin(yaw),math.cos(yaw)))
            self.event('navigate_pickup',object_id=identifier)
            stage = box[:2]-1.05*forward+.15*left
            if self.exploration is not None:
                self.exploration.navigate_observed((*stage,yaw))
                if abs(angle(self.pose[2]-yaw))>.15:
                    self.rotate(yaw)
                self.wait(lambda:self.observe(identifier,2.) is not None,8.,'no fresh box at pickup staging')
                box = np.array(self.observe(identifier,2.)['position'])
            else:
                self.navigate((*stage,yaw))
            # Complete the large arm/ownership transition at the staging
            # distance. Precision docking then measures and corrects its base
            # displacement before approaching the table with the hand raised.
            self.event('prepare_pickup',object_id=identifier)
            if self.rest_arm_positions is None:
                self.rest_arm_positions = {name:self.joints[name] for name in self.kin.names}
            self.service(self.authority,SetBool.Request(data=True))
            self.move_to_carry(np.eye(3))
            self.event('dock_pickup')
            dock = box[:2]-.43*forward+.15*left
            self.dock((*dock,yaw))
            self.wait(lambda: self.observe(identifier,20.) is not None,8.,'no recent tabletop box observation')
            self.event('reach')
            box = np.array(self.observe(identifier,20.)['position'])
            box = self.reach_box(identifier,forward,left,yaw)
            observation = self.observe(identifier,20.)
            if observation is None:
                raise RuntimeError('confirmed image observation expired before grasp')
            self.event('grasp',object_id=identifier,perception_source='registered_rgbd',
                       observed_position=observation['position'],observation_age_s=self.seconds()-observation['observed_stamp'],
                       observation_count=observation.get('observation_count',0))
            grasp_stamp = self.seconds()
            # Establish a conservative payload pose before a request can attach
            # the box, including when its acknowledgement is subsequently lost.
            wrist,R,_,_ = self.kin.forward(self.joints)
            self.offset = R.T@(self.to_pelvis(box)-wrist)
            self.grasp_R = R
            self.service(self.grasp,GraspObject.Request(arm='right',object_id=identifier),attachment_after='held')
            self.event('lift')
            self.move_to_carry(R)
            self.wait(lambda: self.observe(identifier) is not None and
                      self.observe(identifier)['observed_stamp'] > grasp_stamp and
                      self.observe(identifier)['position'][2] > box[2]+.08,
                      8.,'camera did not verify the box lift')
            self.event('retreat_pickup')
            # Leave enough room for loaded-turn drift before searching; a
            # near-table scan can put Nav2's start inside its inflation region.
            self.retreat(1.1,yaw)
            if self.exploration is not None:
                self.event('search_destination',destination_id=destination)
                pad = self.discover(destination)
            else:
                pad = self.observe(destination,60.) or destination_view
            if self.exploration is not None:
                pad = self.exploration.inspect_support(destination,pad)
            elif 'approach_yaw' not in pad:
                # This is an observation stand-off, not permission to dock from
                # an arbitrary side of the destination square.
                point = np.array(pad['position'])
                bearing = math.atan2(point[1]-self.pose[1],point[0]-self.pose[0])
                self.event('inspect_support',object_id=destination)
                view = point[:2]-1.6*np.array((math.cos(bearing),math.sin(bearing)))
                self.navigate((*view,bearing))
                self.rotate(bearing)
                self.wait(lambda:self.observe(destination,5.) is not None and
                          'approach_yaw' in self.observe(destination,5.),8.,'destination approach side not confirmed')
                pad = self.observe(destination,5.)
            self.event('remembered_destination',destination_id=destination,
                        observation_age_s=self.seconds()-pad['observed_stamp'])
            point = np.array(pad['position'])
            yaw = pad['approach_yaw']
            self.event('delivery_approach',source='rgbd_support_edge',yaw=yaw,
                       approach_observation_count=pad.get('approach_observation_count',0))
            forward,left = np.array((math.cos(yaw),math.sin(yaw))),np.array((-math.sin(yaw),math.cos(yaw)))
            self.event('navigate_delivery')
            stage = point[:2]-1.4*forward+.15*left
            if self.exploration is not None:
                self.exploration.navigate_observed((*stage,yaw),phase='delivery_route')
            else:
                self.navigate((*stage,yaw))
            # At range, a horizontal pad spans very few image rows. Obtain a
            # closer face-direction observation before the final table approach.
            self.event('inspect_delivery')
            bearing = math.atan2(point[1]-self.pose[1],point[0]-self.pose[0])
            self.rotate(bearing)
            self.wait(lambda:self.observe(destination,2.) is not None and
                      'approach_yaw' in self.observe(destination,2.),8.,'destination support side not reobserved')
            latest = self.observe(destination,2.)
            if latest is not None:
                point = np.array(latest['position'])
                yaw = latest['approach_yaw']
                forward,left = np.array((math.cos(yaw),math.sin(yaw))),np.array((-math.sin(yaw),math.cos(yaw)))
                self.event('delivery_approach',source='rgbd_support_edge',yaw=yaw,
                           approach_observation_count=latest.get('approach_observation_count',0))
            self.event('dock_delivery')
            latest = self.observe(destination,15.)
            if latest is not None:
                point = np.array(latest['position'])
            self.dock((*(point[:2]-.46*forward+.15*left),yaw))
            # The hand/payload can occlude the near pad edge. Use the recent
            # complete outline, not the biased centre of a clipped fragment.
            self.wait(lambda: self.observe(destination,45.) is not None,8.,'no recent complete destination outline')
            pad = np.array(self.observe(destination,45.)['position'])
            self.event('destination_observation',pad=pad.tolist(),age_s=self.seconds()-self.observe(destination,45.)['observed_stamp'])
            desired = self.to_pelvis(pad+np.array((0.,0.,.075)))
            hand_target = desired-self.grasp_R@self.offset
            for correction in range(2):
                _,error = self.future(self.ik_worker.submit(self.kin.solve,hand_target,dict(self.joints),self.grasp_R))
                if error <= .012:
                    break
                self.event('refine_delivery_dock',ik_error_m=error)
                self.dock((*(pad[:2]-(.42-.03*correction)*forward+.15*left),yaw))
                latest = self.observe(destination,45.)
                if latest is None:
                    raise RuntimeError('destination observation expired while refining reach')
                pad = np.array(latest['position'])
                hand_target = self.to_pelvis(pad+np.array((0.,0.,.075)))-self.grasp_R@self.offset
            self.event('lower')
            self.move_arm(hand_target,self.grasp_R)
            self.release_and_verify(identifier,destination,pad,forward,yaw)
            self.event('delivered',object_id=identifier,destination_id=destination)
        # Keep the already-retracted arm under continuous ownership between
        # deliveries, avoiding an unnecessary down/up and gain handover cycle.
        self.restore_arm()
        self.event('complete')


class TransportEvaluator(Node):
    """Independent truth-side check; its observations never drive the mission."""
    def __init__(self,pairs,people=0):
        super().__init__('transport_demo_evaluator')
        self.pairs = pairs
        self.people_requested = people
        self.aligned_samples = 0
        self.fallen = False
        self.robot_updated = -math.inf
        self.shapes = {}
        self.drops = set()
        self.objects,self.initial,self.history = {},{},{}
        self.object_timestamp_sources = set()
        self.contacts = 0
        self.minimum_clearance = math.inf
        required = ['robot',*[key for pair in pairs for key in (pair['object_id'],pair['destination_id'])]]
        if people:
            required.append('people')
        self.health = EvaluationHealth(required)
        self.create_subscription(Detection3DArray,'/evaluation/objects',self._objects,10)
        self.create_subscription(String,'/evaluation/aligned_state',self._people,10)
        self.create_subscription(Odometry,'/evaluation/odom',self._robot,10)
        self.create_subscription(String,'/transport/mission_state',self._mission,10)
        self.create_timer(.1,self.health.check)

    def _mission(self,msg):
        if json.loads(msg.data).get('phase','initializing') != 'initializing':
            self.health.start()

    def _robot(self,msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not self.health.observe('robot',stamp):
            return
        self.robot_updated = time.monotonic()
        q = msg.pose.pose.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
        self.fallen |= msg.pose.pose.position.z < .5 or max(abs(roll),abs(pitch)) > .6

    def _objects(self,msg):
        now = time.monotonic()
        for detection in msg.detections:
            identity = detection.results[0].hypothesis.class_id
            p = detection.bbox.center.position
            position = np.array((p.x,p.y,p.z))
            if not np.all(np.isfinite(position)):
                self.health.fail(f'{identity}: nonfinite pose')
                continue
            native_stamp = getattr(detection,'id','').startswith('native_acquisition_v2:')
            sources = getattr(self,'object_timestamp_sources',set())
            sources.add('native_acquisition_v2' if native_stamp else 'legacy_receipt')
            self.object_timestamp_sources = sources
            stamp = detection.header.stamp.sec+detection.header.stamp.nanosec*1e-9 if native_stamp else None
            if native_stamp:
                age = self.get_clock().now().nanoseconds*1e-9-stamp
                if not 0.<=age<=self.health.max_gap:
                    self.health.fail(f'{identity}: native acquisition age invalid')
                    continue
            if not self.health.observe(identity,stamp):
                continue
            self.objects[identity] = position
            q = detection.bbox.center.orientation
            x,y,z,w = q.x,q.y,q.z,q.w
            R = np.array(((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
                          (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
                          (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y))))
            size = detection.bbox.size
            self.shapes[identity] = (R,np.array((size.x,size.y,size.z)))
            if identity.startswith('box_') and p.z < .7:
                self.drops.add(identity)
            self.initial.setdefault(identity,position.copy())
            self.history.setdefault(identity,[]).append((now,position))

    def _people(self,msg):
        data = json.loads(msg.data)
        if (data.get('population_count') != self.people_requested or
                data.get('active_count') != self.people_requested or
                len(data['people']) != self.people_requested):
            self.health.fail('people: population mismatch')
            return
        if not self.health.observe('people',data['sim_time']):
            return
        self.aligned_samples += 1
        robot = data['robot']
        for person in data['people']:
            clearance = math.hypot(robot['x']-person['x'],robot['y']-person['y'])-.35-person['radius_m']
            self.minimum_clearance = min(self.minimum_clearance,clearance)
            if clearance < 0.:
                self.contacts += 1
            for identity,position in self.objects.items():
                if identity.startswith('box_') and position[2] > 1.08:
                    if math.hypot(position[0]-person['x'],position[1]-person['y']) < .095+person['radius_m']:
                        self.contacts += 1

    def report(self):
        deliveries = []
        now = time.monotonic()
        for pair in self.pairs:
            identifier,target = pair['object_id'],pair['destination_id']
            points = self.history.get(identifier,[])
            p,q = self.objects.get(identifier),self.objects.get(target)
            passed = False
            lift = travel = 0.
            if points and p is not None and q is not None:
                start = self.initial[identifier]
                lift = max(v[2]-start[2] for _,v in points)
                travel = max(float(np.linalg.norm(v[:2]-start[:2])) for _,v in points)
                recent = [v for stamp,v in points if now-stamp <= 1.5]
                recent_stamps = [stamp for stamp,_ in points if now-stamp <= 1.5]
                stable = (len(recent)>=10 and recent_stamps[-1]-recent_stamps[0] >= 1. and
                          max(float(np.linalg.norm(v-p)) for v in recent) < .02)
                R,size = self.shapes[identifier]
                pad_R,pad_size = self.shapes[target]
                corners = np.array([(x,y,z) for x in (-1.,1.) for y in (-1.,1.) for z in (-1.,1.)])*size*.5
                local = (corners@R.T+p-q)@pad_R
                inside = np.all(np.abs(local[:,:2]) <= pad_size[:2]*.5-.005)
                passed = bool(lift>.08 and travel>.8 and inside and abs(p[2]-q[2]-.055)<.035 and stable)
            deliveries.append(dict(**pair,verified=passed,lift_m=lift,travel_m=travel))
        health = self.health.report()
        health['object_timestamp_scope'] = sorted(getattr(self,'object_timestamp_sources',{'legacy_receipt'}))
        healthy = health['valid'] and now-self.robot_updated < 1. and (self.people_requested == 0 or self.aligned_samples >= 10)
        required = {pair[key] for pair in self.pairs for key in ('object_id','destination_id')}
        healthy = healthy and all(self.history.get(key) and now-self.history[key][-1][0] < 1. for key in required)
        return dict(deliveries=deliveries,contact_samples=self.contacts,fallen=self.fallen,dropped_objects=sorted(self.drops),data_valid=healthy,
                    evaluation_health=health,
                    minimum_person_clearance_m=None if not math.isfinite(self.minimum_clearance) else self.minimum_clearance,
                    observed_object_ids=sorted(self.objects),
                    success=bool(deliveries) and all(d['verified'] for d in deliveries) and self.contacts==0 and not self.fallen and not self.drops and healthy)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pairs',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=420.)
    parser.add_argument('--people',type=int,default=0)
    parser.add_argument('--carry-posture',choices=('ik','payload_v1'),default='ik')
    parser.add_argument('--camera-pitch-deg',type=float,default=0.)
    parser.add_argument('--explore-unknown',action='store_true')
    parser.add_argument('--search-max-viewpoints',type=int,default=20)
    parser.add_argument('--response-profile',default='legacy',help='Policy-identified search-time estimate; motion parameters are audited independently')
    args = parser.parse_args()
    if args.search_max_viewpoints<=0:
        parser.error('--search-max-viewpoints must be positive')
    objective = json.loads(args.pairs.read_text())
    args.pairs = objective['pairs']
    args.objective = objective.get('description','; '.join(f'{pair["object_id"]} -> {pair["destination_id"]}' for pair in args.pairs))
    rclpy.init()
    # Dedicated executors avoid high-rate IMU/image work starving the mission's
    # odometry and workspace callbacks in a shared thread-pool queue.
    executor = SingleThreadedExecutor()
    perception = TransportPerception(args.camera_pitch_deg)
    evaluator = TransportEvaluator(args.pairs,args.people)
    mission = TransportMission(args,executor)
    executor.add_node(mission)
    background = []
    for node in (perception,evaluator):
        worker = SingleThreadedExecutor()
        worker.add_node(node)
        thread = threading.Thread(target=worker.spin,daemon=True)
        thread.start()
        background.append((worker,thread))
    success,error = False,None
    try:
        mission.run()
        mission.spin(2.)
        success = True
    except (Exception,KeyboardInterrupt) as exc:
        error = str(exc) or 'interrupted'
        mission.get_logger().error(f'Mission failed in {mission.phase}: {error}')
    finally:
        mission.active = False
        mission._heartbeat()
        mission.ik_worker.shutdown(wait=True,cancel_futures=True)
        if mission.exploration is not None:
            mission.exploration.close()
        for worker,thread in background:
            worker.shutdown()
            thread.join()
        perception.save_snapshot(args.output.parent)
        result = evaluator.report()
        result.update(mission_complete=success,error=error,events=mission.events,
                      final_sensed_pose=mission.pose,pose_age_s=mission.seconds()-getattr(mission,'pose_stamp',0.),
                       grasp_model='capture_radius_weld',base_pinning=False,perception='registered_rgbd',
                       attachment_state=mission.attachment_state,object_may_be_held=mission.object_may_be_held)
        result['carry_posture'] = args.carry_posture
        result['objective'] = objective
        result['mission_kind'] = 'explore_pick_find_place' if args.explore_unknown else 'transport'
        if mission.exploration is not None:
            result['search_summary'] = dict(inspected_viewpoints={key:len(value) for key,value in mission.exploration.visits.items()},
                failed_viewpoints=len(mission.exploration.failures),source='observed_slam_map_and_registered_rgbd')
        result['success'] = result['success'] and success
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({key:value for key,value in result.items() if key != 'events'},indent=2))
        executor.shutdown()
        for node in (mission,evaluator,perception):
            node.destroy_node()
        rclpy.shutdown()
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
