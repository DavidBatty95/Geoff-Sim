"""Small vectorized control-sequence search; callers recheck selected plans."""
import math
import time

import numpy as np


def response_step(states, commands, envelope, dt, active_dt):
    """Advance [x,y,yaw,vx,vy,wz,filtered_x,filtered_y,filtered_wz] batches."""
    result = states.copy()
    limits = np.array((envelope.accel_x,envelope.accel_y,envelope.accel_yaw))*active_dt
    filtered = states[:,6:9]+np.clip(commands-states[:,6:9],-limits,limits)
    if active_dt > 0.:
        filtered[np.all(commands == 0.,axis=1)] = 0.
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
                    stationary=None,onset_radius=0.,onset_weight=0.,enclosure_weight=0.):
    """Search checked piecewise commands, keeping diversity in the first action.

    `people` is [time,person,xy], `radii` is [time,person]. A final 0.9-second
    braking tail is reserved at the default step size. This routine uses sensed
    forecasts only, and never supplies an unchecked partial plan on timeout.
    """
    started = time.perf_counter()
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
    if len(radii[0]):
        c,s = np.cos(states[:,2]),np.sin(states[:,2])
        robot_velocity = np.column_stack((c*states[:,3]-s*states[:,4],s*states[:,3]+c*states[:,4]))
        human_velocity = (people[-1]-people[-2])/dt
        relative = states[:,None,:2]-people[-1]
        relative_velocity = robot_velocity[:,None,:]-human_velocity
        tcpa = -np.sum(relative*relative_velocity,axis=2)/np.maximum(1e-12,np.sum(relative_velocity**2,axis=2))
        closest = np.linalg.norm(relative+np.maximum(0.,tcpa)[:,:,None]*relative_velocity,axis=2)
        costs += 100.*np.sum((tcpa > 0.) & (tcpa < 2.*horizon) & (closest < radii[-1]),axis=1)
    best = int(np.argmin(costs+goal_weight*np.maximum(0.,np.linalg.norm(states[:,:2]-goal,axis=1)-goal_radius)))
    return tuple(histories[best])
