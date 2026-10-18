"""Native-like sensor-epoch observations and explicit sensor-feedback motor PD."""
from dataclasses import dataclass,field,replace

import mujoco
import torch

from mjlab.actuator.pd_actuator import IdealPdActuator,IdealPdActuatorCfg
from mjlab.utils.spec import create_motor_actuator


def add_packet_sensors(spec,joint_names):
    for name in joint_names:
        for kind,sensor_type in (('pos',mujoco.mjtSensor.mjSENS_JOINTPOS),('vel',mujoco.mjtSensor.mjSENS_JOINTVEL)):
            spec.add_sensor(name=f'native_{kind}_{name}',type=sensor_type,
                            objtype=mujoco.mjtObj.mjOBJ_JOINT,objname=name)
    spec.add_sensor(name='native_packet_quat',type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
                    objtype=mujoco.mjtObj.mjOBJ_SITE,objname='imu_in_pelvis')
    return spec


class PacketLatch:
    def __init__(self,num_envs,width,device):
        self.values = torch.zeros(num_envs,width,device=device)
        self.pending_reset = torch.ones(num_envs,dtype=torch.bool,device=device)

    def reset(self,env_ids):
        self.pending_reset[env_ids] = True

    def capture(self,values):
        self.values.copy_(values)
        self.pending_reset.fill_(False)

    def read(self,fresh):
        self.values.copy_(torch.where(self.pending_reset[:,None],fresh,self.values))
        self.pending_reset.fill_(False)
        return self.values


class NativeSensorPacket(PacketLatch):
    def __init__(self,env,robot):
        super().__init__(env.num_envs,65,env.device)
        self.env = env
        model = env.sim.mj_model
        names = [f'robot/native_{kind}_{name}' for kind in ('pos','vel') for name in robot.joint_names]
        addresses = [int(model.sensor_adr[model.sensor(name).id]) for name in names]
        for name,width in (('robot/imu_ang_vel',3),('robot/native_packet_quat',4)):
            sensor = model.sensor(name).id
            if model.sensor_dim[sensor]!=width:
                raise ValueError('native packet sensor width mismatch')
            addresses.extend(range(model.sensor_adr[sensor],model.sensor_adr[sensor]+width))
        if len(addresses)!=65:
            raise ValueError('native packet requires 29 joints, gyro and quaternion')
        self.addresses = torch.tensor(addresses,dtype=torch.long,device=env.device)

    def fresh(self):
        return self.env.sim.data.sensordata[:,self.addresses]

    def current(self):
        return self.read(self.fresh())


def sensor_pd_command(cmd,packet,target_ids):
    # Reset writes occur before the environment's final forward pass. Use the
    # newly reset integrated joints for those rows, never the prior episode.
    pos = torch.where(packet.pending_reset[:,None],cmd.pos,packet.values[:,:29][:,target_ids])
    vel = torch.where(packet.pending_reset[:,None],cmd.vel,packet.values[:,29:58][:,target_ids])
    return replace(cmd,pos=pos,vel=vel)


@dataclass(kw_only=True)
class NativeSensorPdCfg(IdealPdActuatorCfg):
    gains: dict[str,tuple[float,float,float]] = field(default_factory=dict)

    def build(self,entity,target_ids,target_names):
        return NativeSensorPd(self,entity,target_ids,target_names)


class NativeSensorPd(IdealPdActuator):
    packet = None

    def edit_spec(self,spec,target_names):
        for name in target_names:
            self._mjs_actuators.append(create_motor_actuator(spec,name,effort_limit=self.cfg.gains[name][2],
                armature=self.cfg.armature,frictionloss=self.cfg.frictionloss,viscous_damping=self.cfg.viscous_damping,
                transmission_type=self.cfg.transmission_type))

    def initialize(self,mj_model,model,data,device):
        super().initialize(mj_model,model,data,device)
        gains = torch.tensor([self.cfg.gains[name] for name in self.target_names],device=device)
        self.stiffness[:] = gains[:,0]
        self.damping[:] = gains[:,1]
        self.force_limit[:] = gains[:,2]
        self.default_stiffness = self.stiffness.clone()
        self.default_damping = self.damping.clone()
        self.default_force_limit = self.force_limit.clone()

    def compute(self,cmd):
        # Diagnostic references to the pre-step integrated state; actor inputs
        # still come exclusively from the latched packet.
        self.last_integrated_pos = cmd.pos
        self.last_integrated_vel = cmd.vel
        if self.packet is not None:
            cmd = sensor_pd_command(cmd,self.packet,self.target_ids)
        return super().compute(cmd)

    def set_joint_gains(self,env_ids,name,kp,kd):
        column = self.target_names.index(name)
        self.stiffness[env_ids,column] = kp
        self.damping[env_ids,column] = kd


def capture_native_packet(env):
    action = env.action_manager.get_term('joint_pos')
    packet = action.native_packet
    packet.capture(packet.fresh())
    # The returned norm is diagnostic; capture itself is required. It occurs
    # after mj_step and before the final forward
    # refresh, so observations and the next motor update retain one shared epoch.
    return torch.linalg.vector_norm(env.scene['robot'].data.joint_pos-packet.values[:,:29],dim=1)


def native_joint_positions(env):
    action = env.action_manager.get_term('joint_pos')
    return action.native_packet.current()[:,:29][:,action.joint_ids]-action._offset


def native_joint_velocities(env):
    action = env.action_manager.get_term('joint_pos')
    return action.native_packet.current()[:,29:58][:,action.joint_ids]


def native_gyro(env):
    return env.action_manager.get_term('joint_pos').native_packet.current()[:,58:61].clone()


def packet_gravity(quat):
    q = torch.nn.functional.normalize(quat,dim=1)
    w,x,y,z = q.unbind(dim=1)
    return torch.stack((2*(w*y-x*z),-2*(w*x+y*z),2*(x*x+y*y)-1),dim=1)


def native_gravity(env):
    return packet_gravity(env.action_manager.get_term('joint_pos').native_packet.current()[:,61:65])
