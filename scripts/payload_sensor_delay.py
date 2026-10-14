"""Coherent per-environment sensor lag for the existing deployed actor layout."""
import torch


class SensorFrameDelay:
    def __init__(self,num_envs,width,max_lag,device,dtype=torch.float32):
        if max_lag<0:
            raise ValueError('sensor lag must be nonnegative')
        self.history = torch.zeros(num_envs,max_lag+1,width,device=device,dtype=dtype)
        self.pending_reset = torch.ones(num_envs,device=device,dtype=torch.bool)
        self.rows = torch.arange(num_envs,device=device)
        self.last_step = None

    def reset(self,env_ids=None):
        if env_ids is None:
            self.pending_reset.fill_(True)
        else:
            self.pending_reset[env_ids] = True

    def push(self,values,lags,step):
        if self.last_step is not None and step<self.last_step:
            self.pending_reset.fill_(True)
        if step!=self.last_step:
            self.history = torch.roll(self.history,1,dims=1)
            self.history[:,0] = values
            self.last_step = step
        # Seed every history slot for reset environments from their new sensor
        # state. Other environments must not advance on repeated same-step reads.
        self.history = torch.where(self.pending_reset[:,None,None],values[:,None,:],self.history)
        self.pending_reset.fill_(False)
        return self.history[self.rows,lags]


class DelayedSensorTerm:
    def __init__(self,cfg,env):
        self.max_lag = cfg.params['max_lag']
        self.buffer = None

    def reset(self,env_ids=None):
        if self.buffer is not None:
            self.buffer.reset(env_ids)

    def __call__(self,env,source,source_params,max_lag):
        values = source(env,**source_params)
        if self.buffer is None:
            self.buffer = SensorFrameDelay(env.num_envs,values.shape[-1],max_lag,values.device,values.dtype)
        # All IMU/joint terms use one per-episode lag vector, representing a
        # single LowState packet. Commands and published-action feedback stay current.
        lags = env.action_manager.get_term('joint_pos').sensor_delay_steps
        return self.buffer.push(values,lags,env.common_step_counter)
