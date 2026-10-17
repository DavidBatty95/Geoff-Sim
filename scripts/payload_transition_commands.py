"""Opt-in move/stop curriculum with control-tick-spaced transition coverage."""
from dataclasses import dataclass

import torch

from mjlab.tasks.velocity.mdp.velocity_command import StandCreepStopVelocityCommand,StandCreepStopVelocityCommandCfg


def sample_phase_ticks(standing,moving_range,standing_range):
    moving = torch.randint(moving_range[0],moving_range[1]+1,standing.shape,device=standing.device)
    stopped = torch.randint(standing_range[0],standing_range[1]+1,standing.shape,device=standing.device)
    return torch.where(standing,stopped,moving)


def phase_deadlines(ticks,dt):
    # CommandManager decrements a float timer before comparing with zero.
    # Center the deadline between ticks so roundoff cannot add an extra update.
    return (ticks.to(torch.float32)-.5)*dt


class TransitionVelocityCommand(StandCreepStopVelocityCommand):
    def __init__(self,cfg,env):
        super().__init__(cfg,env)
        self.transition_counts = torch.zeros(3,dtype=torch.long,device=self.device)
        self.duration_counts = torch.zeros(max(cfg.moving_ticks[1],cfg.standing_ticks[1])+1,dtype=torch.long,device=self.device)

    def _resample_command(self,env_ids):
        was_moving = (~self.is_standing_env[env_ids])&(self.command_counter[env_ids]>0)
        super()._resample_command(env_ids)
        command = self.vel_command_b[env_ids].clone()
        lateral = (command[:,0]==0.)&(command[:,2]==0.)&(command[:,1]!=0.)
        endpoint = lateral&(torch.rand(len(env_ids),device=self.device)<self.cfg.lateral_endpoint_probability)
        command[endpoint,1] = command[endpoint,1].sign()*self.cfg.lateral_speed_max
        self.vel_command_b[env_ids] = command
        self.vel_command_w[env_ids] = command
        standing = self.is_standing_env[env_ids]
        ticks = sample_phase_ticks(standing,self.cfg.moving_ticks,self.cfg.standing_ticks)
        self.time_left[env_ids] = phase_deadlines(ticks,self._env.step_dt)
        self.duration_counts += torch.bincount(ticks,minlength=len(self.duration_counts))
        self.transition_counts += torch.stack(((lateral&(command[:,1]>0.)).sum(),
                                               (lateral&(command[:,1]<0.)).sum(),
                                               (was_moving&standing).sum()))


@dataclass(kw_only=True)
class TransitionVelocityCommandCfg(StandCreepStopVelocityCommandCfg):
    moving_ticks: tuple[int,int] = (75,175)
    standing_ticks: tuple[int,int] = (250,300)
    lateral_endpoint_probability: float = .5

    def build(self,env):
        return TransitionVelocityCommand(self,env)

    def __post_init__(self):
        super().__post_init__()
        for bounds in (self.moving_ticks,self.standing_ticks):
            if len(bounds)!=2 or any(type(v) is not int for v in bounds) or not 1<=bounds[0]<=bounds[1]:
                raise ValueError('transition durations must be positive ordered integer ticks')
        if not 0.<=self.lateral_endpoint_probability<=1.:
            raise ValueError('lateral endpoint probability must be in [0, 1]')
