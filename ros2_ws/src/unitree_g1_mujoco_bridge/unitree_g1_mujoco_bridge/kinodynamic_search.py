"""Small vectorized control-sequence search; callers recheck selected plans."""
import math
import time

import numpy as np
from .policy_response import NATIVE_CONTACT_PROFILE, COMMANDS, RATES, pivot_yaw_rate


def scalar_response_step(state,command,envelope,dt,active_dt):
    """Same response equations without NumPy setup overhead for small batches."""
    x,y,yaw,vx,vy,wz,fx,fy,fw=map(float,state)
    cx,cy,cw=map(float,command)
    initial=(fx,fy,fw)
    fx+=max(-envelope.accel_x*active_dt,min(envelope.accel_x*active_dt,cx-fx))
    fy+=max(-envelope.accel_y*active_dt,min(envelope.accel_y*active_dt,cy-fy))
    fw+=max(-envelope.accel_yaw*active_dt,min(envelope.accel_yaw*active_dt,cw-fw))
    if active_dt>0. and cx==cy==cw==0.:
        fx=fy=fw=0.
    fx,fy,fw=envelope.limit_planar((fx,fy,fw),initial)
    tx=fx*envelope.forward_gain
    ty=fy*envelope.lateral_response_gain(fy)
    moving=min(1.,math.hypot(tx,ty)/.25)
    walking=envelope.walking_yaw_gain or envelope.yaw_gain
    tw=fw*(envelope.yaw_gain+moving*(walking-envelope.yaw_gain))
    if envelope.response_profile==NATIVE_CONTACT_PROFILE:
        tw=(1.-moving)*pivot_yaw_rate(fw,envelope.payload_active)+moving*fw*walking
    tw+=fy*envelope.lateral_yaw_gain
    lateral_tau=envelope.linear_time_constant
    direction=fy if abs(fy)>1e-9 else vy
    if direction>=0. and envelope.positive_lateral_time_constant>0.:
        lateral_tau=envelope.positive_lateral_time_constant
    nx=vx+(tx-vx)*(1.-math.exp(-dt/max(.01,envelope.linear_time_constant)))
    ny=vy+(ty-vy)*(1.-math.exp(-dt/max(.01,lateral_tau)))
    nw=wz+(tw-wz)*(1.-math.exp(-dt/max(.01,envelope.yaw_time_constant)))
    mx,my,mw=.5*(vx+nx),.5*(vy+ny),.5*(wz+nw)
    heading=yaw+.5*dt*mw
    c,s=math.cos(heading),math.sin(heading)
    return (x+dt*(c*mx-s*my),y+dt*(s*mx+c*my),yaw+dt*mw,nx,ny,nw,fx,fy,fw)


def response_step(states, commands, envelope, dt, active_dt):
    """Advance [x,y,yaw,vx,vy,wz,filtered_x,filtered_y,filtered_wz] batches."""
    if len(states)<=12:
        active=np.broadcast_to(np.asarray(active_dt),(len(states),))
        return np.asarray([scalar_response_step(state,command,envelope,float(dt),float(interval))
                           for state,command,interval in zip(states,commands,active)]).reshape(states.shape)
    result = states.copy()
    active_dt=np.asarray(active_dt)
    limits = np.array((envelope.accel_x,envelope.accel_y,envelope.accel_yaw))*active_dt[...,None]
    filtered = states[:,6:9]+np.clip(commands-states[:,6:9],-limits,limits)
    filtered[np.all(commands == 0.,axis=1) & np.broadcast_to(active_dt>0.,(len(states),))] = 0.
    if math.isfinite(envelope.max_planar_speed):
        limit = max(0.,envelope.max_planar_speed)
        speed = np.linalg.norm(filtered[:,:2],axis=1)
        over = speed > limit
        prior = states[:,6:8]
        inside = np.linalg.norm(prior,axis=1) <= limit+1e-9
        delta = filtered[:,:2]-prior
        length = np.sum(delta*delta,axis=1)
        dot = np.sum(prior*delta,axis=1)
        discriminant = dot*dot+length*(limit*limit-np.sum(prior*prior,axis=1))
        fraction = np.clip((-dot+np.sqrt(np.maximum(0.,discriminant)))/np.maximum(1e-12,length),0.,1.)
        filtered[over & inside,:2] = (prior+fraction[:,None]*delta)[over & inside]
        filtered[over & ~inside,:2] *= (limit/np.maximum(1e-12,speed[over & ~inside]))[:,None]
    target = filtered*np.array((envelope.forward_gain,envelope.lateral_gain,envelope.yaw_gain))
    if envelope.negative_lateral_gain > 0.:
        negative = filtered[:,1] < 0.
        target[negative,1] = filtered[negative,1]*envelope.negative_lateral_gain
    moving = np.minimum(1.,np.hypot(target[:,0],target[:,1])/.25)
    yaw_gain = envelope.yaw_gain+moving*((envelope.walking_yaw_gain or envelope.yaw_gain)-envelope.yaw_gain)
    target[:,2] = filtered[:,2]*yaw_gain+filtered[:,1]*envelope.lateral_yaw_gain
    if envelope.response_profile == NATIVE_CONTACT_PROFILE:
        magnitude=np.abs(filtered[:,2])
        pivot=np.copysign(np.where(filtered[:,2]<0.,
            np.interp(magnitude,COMMANDS,RATES[bool(envelope.payload_active),True]),
            np.interp(magnitude,COMMANDS,RATES[bool(envelope.payload_active),False])),filtered[:,2])
        target[:,2] = ((1.-moving)*pivot+moving*filtered[:,2]*(envelope.walking_yaw_gain or envelope.yaw_gain)
                        +filtered[:,1]*envelope.lateral_yaw_gain)
    response = 1.-np.exp(-dt/np.maximum(.01,(envelope.linear_time_constant,envelope.linear_time_constant,envelope.yaw_time_constant)))
    fractions = np.broadcast_to(response,(len(states),3)).copy()
    direction = np.where(np.abs(filtered[:,1])>1e-9,filtered[:,1],states[:,4])
    if envelope.positive_lateral_time_constant > 0.:
        fractions[direction >= 0.,1] = 1.-math.exp(-dt/max(.01,envelope.positive_lateral_time_constant))
    velocity = states[:,3:6]+(target-states[:,3:6])*fractions
    midpoint = .5*(states[:,3:6]+velocity)
    heading = states[:,2]+.5*dt*midpoint[:,2]
    c,s = np.cos(heading),np.sin(heading)
    result[:,0] += dt*(c*midpoint[:,0]-s*midpoint[:,1])
    result[:,1] += dt*(s*midpoint[:,0]+c*midpoint[:,1])
    result[:,2] += dt*midpoint[:,2]
    result[:,3:6],result[:,6:9] = velocity,filtered
    return result


def remaining_schedule(schedule, elapsed):
    result = []
    for duration,vx,vy,wz in schedule:
        if elapsed >= duration:
            elapsed -= duration
            continue
        result.append((duration-max(0.,elapsed),vx,vy,wz))
        elapsed = 0.
    return tuple(result)


def rollout_schedule(envelope,pose,velocity,filtered,schedule,horizon,dt,delay):
    steps = max(1,math.ceil(horizon/max(.02,dt)))
    dt = horizon/steps
    state = np.array([[*pose,*velocity,*filtered]],dtype=float)
    delay = max(0.,delay)
    boundaries = delay+np.cumsum([item[0] for item in schedule])
    commands = [envelope.normalize(item[1:]) for item in schedule]
    trajectory = [(*pose,0.)]
    for step in range(steps):
        start,end = step*dt,(step+1)*dt
        cuts = [start]+sorted({float(t) for t in (delay,*boundaries) if start<t<end})+[end]
        for begin,finish in zip(cuts,cuts[1:]):
            index = int(np.searchsorted(boundaries,begin,side='right'))
            command = commands[index] if index < len(commands) else (0.,0.,0.)
            interval = finish-begin
            state = response_step(state,np.array([command]),envelope,interval,interval if begin>=delay else 0.)
        trajectory.append((*map(float,state[0,:3]),(step+1)*dt))
    return trajectory


def search_sequence(envelope,pose,velocity,filtered,actions,people,radii,goal,
                    horizon,dt,delay,static_query=None,static_clearance=.48,
                    goal_radius=.4,goal_weight=12.,beam_width=28,budget_ms=30.,
                    stationary=None,onset_radius=0.,onset_weight=0.,enclosure_weight=0.,terminal_hold_s=2.,diagnostics=None):
    """Search checked piecewise commands, keeping diversity in the first action.

    `people` is [time,person,xy], `radii` is [time,person]. A final 0.9-second
    braking tail is reserved at the default step size. This routine uses sensed
    forecasts only, and never supplies an unchecked partial plan on timeout.
    """
    started = time.perf_counter()
    def record_outcome(reason):
        if diagnostics is not None:
            diagnostics.update(reason=reason,elapsed_ms=(time.perf_counter()-started)*1000.)
    steps = max(1,math.ceil(horizon/max(.02,dt)))
    dt = horizon/steps
    delay = max(0.,delay)
    actions = np.asarray([envelope.normalize(command) for command in actions],dtype=float)
    people,radii = np.asarray(people),np.asarray(radii)
    if radii.ndim != 2 or people.shape != (steps+1,radii.shape[1],2) or radii.shape != people.shape[:2]:
        raise ValueError('forecast grid does not match the rollout grid')
    stationary = np.zeros(radii.shape[1],dtype=bool) if stationary is None else np.asarray(stationary)
    states = np.array([[*pose,*velocity,*filtered]],dtype=float)
    costs = np.zeros(1)
    first_actions = np.array([-1],dtype=int)
    histories = [[]]
    phase_steps = 3
    phases = math.ceil(steps/phase_steps)
    step_index = 0
    for phase in range(phases):
        if budget_ms > 0. and (time.perf_counter()-started)*1000. > budget_ms:
            record_outcome('deadline')
            return None
        choices = np.zeros((1,3)) if phase >= phases-2 else actions
        count = len(choices)
        parents = np.repeat(np.arange(len(states)),count)
        commands = np.tile(choices,(len(states),1))
        expanded = states[parents].copy()
        candidate_cost = costs[parents]+.25*np.sum((commands-expanded[:,6:9])**2,axis=1)
        origins = np.tile(np.arange(count),len(states)) if phase == 0 else first_actions[parents]
        valid = np.ones(len(expanded),dtype=bool)
        duration = min(phase_steps,steps-step_index)*dt
        arrivals = [delay]+list(delay+np.cumsum([item[0] for item in histories[0]]))
        phase_commands = [np.array([histories[parent][index][1:] for parent in parents])
                          for index in range(phase)]+[commands]
        for _ in range(min(phase_steps,steps-step_index)):
            before = expanded[:,:2].copy()
            start,end = step_index*dt,(step_index+1)*dt
            cuts = [start]+sorted({t for t in arrivals if start<t<end})+[end]
            for begin,finish in zip(cuts,cuts[1:]):
                interval = finish-begin
                index = max(0,int(np.searchsorted(arrivals,begin,side='right'))-1)
                applied = phase_commands[index]
                expanded = response_step(expanded,applied,envelope,interval,interval if begin>=delay else 0.)
            if len(radii[0]):
                relative = before[:,None,:]-people[step_index]
                next_relative = expanded[:,None,:2]-people[step_index+1]
                delta = next_relative-relative
                fraction = np.clip(-np.sum(relative*delta,axis=2)/np.maximum(1e-12,np.sum(delta*delta,axis=2)),0.,1.)
                separation = np.linalg.norm(relative+fraction[:,:,None]*delta,axis=2)
                valid &= np.all(separation >= np.maximum(radii[step_index],radii[step_index+1]),axis=1)
                candidate_cost += dt*6.*np.sum(np.maximum(0.,1.8-separation)**2,axis=1)
                if onset_weight > 0.:
                    onset = np.maximum(0.,onset_radius-separation)/np.maximum(.1,onset_radius-radii[step_index+1])
                    candidate_cost += dt*onset_weight*math.exp(-(step_index+1)*dt)*np.sum(onset**2*stationary,axis=1)
                if enclosure_weight > 0.:
                    distance = np.linalg.norm(next_relative,axis=2)
                    unit = next_relative/np.maximum(1e-6,distance[:,:,None])
                    influence = np.clip((3.-distance)/np.maximum(.1,3.-radii[step_index+1]),0.,1.)
                    for i in range(len(radii[0])):
                        for j in range(i+1,len(radii[0])):
                            squeeze = influence[:,i]*influence[:,j]*np.maximum(0.,-np.sum(unit[:,i]*unit[:,j],axis=1))
                            candidate_cost += dt*enclosure_weight*math.exp(-.25*(step_index+1)*dt)*squeeze
            if static_query is not None:
                bound = np.minimum(static_query(before),static_query(expanded[:,:2]))
                valid &= bound >= static_clearance+.5*np.linalg.norm(expanded[:,:2]-before,axis=1)
                candidate_cost += dt*8.*np.maximum(0.,(1.25-bound)/1.25)**2
            step_index += 1
        indexes = np.flatnonzero(valid)
        if not len(indexes):
            record_outcome('no_feasible_prefix')
            return None
        remaining = np.maximum(0.,np.linalg.norm(expanded[:,:2]-goal,axis=1)-goal_radius)
        ranking = candidate_cost+goal_weight*remaining
        ordered = indexes[np.argsort(ranking[indexes])]
        selected,seen_origins,seen_cells = [],set(),set()
        # Keep at least one route for each initially feasible escape direction.
        for index in ordered:
            if origins[index] not in seen_origins:
                selected.append(int(index))
                seen_origins.add(origins[index])
        selected = selected[:beam_width]
        for index in selected:
            seen_cells.add((*np.rint(expanded[index,:2]/.15).astype(int),
                            *np.rint(expanded[index,3:5]/.15).astype(int),origins[index]))
        for index in ordered:
            if len(selected) >= beam_width:
                break
            key = (*np.rint(expanded[index,:2]/.15).astype(int),
                   *np.rint(expanded[index,3:5]/.15).astype(int),origins[index])
            if key not in seen_cells:
                selected.append(int(index))
                seen_cells.add(key)
        histories = [histories[parents[index]]+[(duration,*map(float,commands[index]))] for index in selected]
        states,costs,first_actions = expanded[selected],candidate_cost[selected],origins[selected]
    # A low-cost endpoint is not a refuge if a pedestrian walks into it just
    # after the planning horizon. Require measured-model settling and a swept
    # zero-command hold; reject conflicts rather than assigning a soft penalty.
    valid = (np.linalg.norm(states[:,3:5],axis=1)<.08) & (np.abs(states[:,5])<.12)
    held = states.copy()
    human_velocity = (people[-1]-people[-2])/dt
    hold_steps = max(1,math.ceil(terminal_hold_s/dt))
    hold_dt = terminal_hold_s/hold_steps
    for index in range(hold_steps):
        if budget_ms>0. and (time.perf_counter()-started)*1000.>budget_ms:
            record_outcome('deadline')
            return None
        before = held[:,:2].copy()
        held = response_step(held,np.zeros((len(held),3)),envelope,hold_dt,hold_dt)
        if len(radii[0]):
            relative = before[:,None,:]-(people[-1]+index*hold_dt*human_velocity)
            following = held[:,None,:2]-(people[-1]+(index+1)*hold_dt*human_velocity)
            delta = following-relative
            fraction = np.clip(-np.sum(relative*delta,axis=2)/np.maximum(1e-12,np.sum(delta*delta,axis=2)),0.,1.)
            distance = np.linalg.norm(relative+fraction[:,:,None]*delta,axis=2)
            valid &= np.all(distance>=radii[-1],axis=1)
        if static_query is not None:
            bound = np.minimum(static_query(before),static_query(held[:,:2]))
            valid &= bound>=static_clearance+.5*np.linalg.norm(held[:,:2]-before,axis=1)
    if not np.any(valid):
        record_outcome('unsafe_terminal_state_or_hold')
        return None
    ranking = costs+goal_weight*np.maximum(0.,np.linalg.norm(states[:,:2]-goal,axis=1)-goal_radius)
    best = int(np.argmin(np.where(valid,ranking,np.inf)))
    record_outcome('complete')
    return tuple(histories[best])
