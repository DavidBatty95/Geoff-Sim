"""Dex3 radian commands and contact-confirmed grasp/release (no attachment API)."""
import json
import math
import time

CHANNELS = ('thumb_0','thumb_1','thumb_2','middle_0','middle_1','index_0','index_1')
SIM_CONTACT_MARKER = 0x44583343
RIGHT_CLOSE = (0.,-1.,-.7,1.5,1.5,1.5,1.5)
# The mirrored right-hand closure slips in extended left-hand tests. A small
# thumb opposition preload and deeper distal thumb target retain the cylinder.
LEFT_CLOSE = (.05,1.,1.,-1.5,-1.5,-1.5,-1.5)
GRIP_KP = 3.
GRIP_KD = .08
OPEN = (0.,)*7


def opening_step(positions,dt):
    """Bound opening speed and missed-timer catch-up; never cross the open pose."""
    step = .3*max(0.,min(.04,dt))
    return tuple(math.copysign(max(0.,abs(value)-step),value) for value in positions)


def closure_ready_tick(positions,target,start_tick):
    """Allow the native 1.5 rad/s reference ramp plus a command-transfer margin."""
    travel = max(abs(a-b) for a,b in zip(positions,target))
    return start_tick+math.ceil(1000.*(travel/1.5+.1))


def close_positions(side):
    if side not in ('left','right'):
        raise ValueError('invalid hand side')
    return RIGHT_CLOSE if side=='right' else LEFT_CLOSE


def joint_names(side):
    return tuple(f'{side}_hand_{name}_joint' for name in CHANNELS)


def decode_state(message):
    """Decode explicitly labelled simulated tactile forces, never object truth."""
    if (len(message.motor_state)!=7 or len(message.press_sensor_state)!=9 or
            message.reserve[1]!=SIM_CONTACT_MARKER or any(message.error)):
        raise ValueError('invalid or stale Dex3 contact packet')
    positions = tuple(float(m.q) for m in message.motor_state)
    if not all(math.isfinite(v) for v in positions):
        raise ValueError('nonfinite hand joints')
    values = []
    for sensor in message.press_sensor_state:
        if sensor.lost or not all(math.isfinite(v) and v>=0 for v in sensor.pressure):
            raise ValueError('invalid tactile sample')
        values.append(float(sensor.pressure[0]))
    return positions,tuple(sum(values[i:i+3]) for i in (0,3,6)),int(message.reserve[0])


def grasp_contact(positions,forces,side):
    target = close_positions(side)
    thumb_loaded = max(abs(target[i]-positions[i]) for i in (1,2))>.03
    fingers_loaded = max(abs(target[i]-positions[i]) for i in (3,5))>.03
    return bool(forces[0]>.15 and max(forces[1:])>.15 and thumb_loaded and fingers_loaded)


def contact_lift_verified(initial_height,current_height,state,ages):
    """Confirm sensed hand rise while opposed contacts persist through occlusion."""
    return bool(isinstance(state,dict) and all(type(v) in (int,float) and math.isfinite(v) for v in (initial_height,current_height,*ages))
        and all(0<=age<=.3 for age in ages) and current_height-initial_height>.08
        and state.get('grasp_model')=='dex3_contact_only' and state.get('fresh') is True
        and state.get('contact_verified') is True and state.get('holding_requested') is True)


class Dex3GraspController:
    """ROS service adapter: close fingers, await tactile confirmation, keep holding."""
    def __init__(self,node):
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from unitree_hg.msg import HandCmd,HandState
        from g1_msgs.srv import GraspObject
        self.node = node
        self.group = ReentrantCallbackGroup()
        self.samples = {}
        self.targets = {side:OPEN for side in ('left','right')}
        self.pending = {}
        self.holding = {side:False for side in ('left','right')}
        self.sequence = {}
        self.reset_generation = {side:0 for side in ('left','right')}
        qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
        self.publishers = {side:node.create_publisher(HandCmd,f'/dex3/{side}/cmd',qos) for side in self.targets}
        for side in self.targets:
            node.create_subscription(HandState,f'/lf/dex3/{side}/state',lambda msg,side=side:self.receive(msg,side),qos,callback_group=self.group)
        self.status = node.create_publisher(String,'/g1/dex3_grasp_state',qos)
        node.create_service(GraspObject,'~/grasp_object',self.grasp,callback_group=self.group)
        node.create_service(Trigger,'~/release_right',self.release_right,callback_group=self.group)
        node.create_service(Trigger,'~/release_left',self.release_left,callback_group=self.group)
        node.create_service(Trigger,'~/open_right',self.release_right,callback_group=self.group)
        node.create_service(Trigger,'~/open_left',self.release_left,callback_group=self.group)
        node.create_service(Trigger,'~/verify_right',lambda req,res:self.verify('right',res),callback_group=self.group)
        node.create_service(Trigger,'~/verify_left',lambda req,res:self.verify('left',res),callback_group=self.group)
        node.create_timer(.02,self.tick,callback_group=self.group)

    def receive(self,message,side):
        try:
            q,forces,sequence = decode_state(message)
        except ValueError:
            self.samples.pop(side,None)
            return
        previous = self.sequence.get(side,-1)
        if sequence<=previous:
            if sequence<previous:
                self.reset_generation[side]+=1
                self.sequence[side]=sequence
                self.samples.pop(side,None)
            return
        self.sequence[side]=sequence
        self.samples[side]=(time.monotonic(),q,forces)

    async def operate(self,side,close,response):
        from rclpy.task import Future
        if side not in self.targets or side in self.pending or (close and self.holding.get(side,False)):
            response.success,response.message=False,'invalid hand or hand operation already active'
            return response
        future = Future()
        now = time.monotonic()
        sample = self.samples.get(side)
        if close and (sample is None or not 0<=now-sample[0]<=.2):
            response.success,response.message=False,'fresh hand feedback required before closing'
            return response
        ready_tick = None
        if close:
            self.targets[side] = close_positions(side)
            ready_tick = closure_ready_tick(sample[1],self.targets[side],self.sequence[side])
        elif sample is not None and 0<=now-sample[0]<=.2:
            self.targets[side] = sample[1]
        self.pending[side] = dict(future=future,close=close,deadline=now+6.,since=None,last_step=now,
                                  generation=self.reset_generation[side],ready_tick=ready_tick)
        success,message = await future
        response.success,response.message=success,message
        return response

    async def grasp(self,request,response):
        from .sim_manipulation_io import _validate_identifier,ProtocolError
        try:
            _validate_identifier(request.object_id)
        except ProtocolError:
            response.success,response.message=False,'valid object identifier required'
            return response
        return await self.operate(request.arm,True,response)

    async def release_right(self,request,response):
        return await self.operate('right',False,response)

    async def release_left(self,request,response):
        return await self.operate('left',False,response)

    def verify(self,side,response):
        sample = self.samples.get(side)
        response.success = bool(self.holding[side] and sample is not None and
            0<=time.monotonic()-sample[0]<=.2 and grasp_contact(sample[1],sample[2],side))
        response.message = 'opposing contacts remain present' if response.success else 'physical grasp lost or feedback stale'
        return response

    def tick(self):
        from std_msgs.msg import String
        from unitree_hg.msg import HandCmd,MotorCmd
        now = time.monotonic()
        for side,target in self.targets.items():
            pending = self.pending.get(side)
            if pending and not pending['close']:
                target = opening_step(target,now-pending['last_step'])
                self.targets[side] = target
                pending['last_step'] = now
            command = HandCmd()
            for i,value in enumerate(target):
                motor = MotorCmd()
                motor.mode = 0x10|i
                motor.q,motor.dq,motor.tau,motor.kp,motor.kd = float(value),0.,0.,GRIP_KP,GRIP_KD
                command.motor_cmd.append(motor)
            self.publishers[side].publish(command)
            sample = self.samples.get(side)
            fresh = sample is not None and 0<=now-sample[0]<=.2
            contact = bool(fresh and grasp_contact(sample[1],sample[2],side))
            pending = self.pending.get(side)
            if pending:
                valid = (contact and self.sequence.get(side,-1)>=pending['ready_tick']) if pending['close'] else bool(fresh and max(abs(v) for v in sample[1])<.08 and max(sample[2])<.1)
                pending['since'] = (pending['since'] or now) if valid else None
                outcome = None
                if pending['generation']!=self.reset_generation[side]:
                    outcome = (False,'Dex3 feedback reset during hand operation')
                elif pending['since'] is not None and now-pending['since']>=.3:
                    self.holding[side]=pending['close']
                    outcome = (True,'opposing finger contacts confirmed after closure interval' if pending['close'] else 'open fingers and released contacts confirmed')
                elif now>=pending['deadline']:
                    outcome = (False,'Dex3 contact confirmation timed out; last finger target retained')
                if outcome:
                    self.pending.pop(side)
                    pending['future'].set_result(outcome)
            self.status.publish(String(data=json.dumps(dict(side=side,fresh=fresh,contact_verified=contact,
                holding_requested=self.holding[side],forces_n=list(sample[2]) if fresh else None,
                joints_rad=list(sample[1]) if fresh else None,
                closure_ready_tick=pending['ready_tick'] if pending and pending['close'] else None,
                pending=side in self.pending,grasp_model='dex3_contact_only'))))
