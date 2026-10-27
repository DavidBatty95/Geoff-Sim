"""Reusable command templates, freshly rolled out and checked on every request."""
from functools import lru_cache
from dataclasses import dataclass,field
import math
import time
from types import MappingProxyType

import numpy as np

from .kinodynamic_search import response_step


@dataclass(frozen=True)
class PrimitiveSnapshot:
    inputs: object
    context: tuple
    diagnostics: dict=field(default_factory=dict)

    def __post_init__(self):
        values=dict(self.inputs)
        for name in ('people','radii'):
            values[name].setflags(write=False)
        object.__setattr__(self,'inputs',MappingProxyType(values))

    def __reduce__(self):
        return type(self),(dict(self.inputs),self.context,self.diagnostics)


def warmup():
    """Load the numeric/search modules before the first real-time request."""
    return True


def process_worker():
    from concurrent.futures import ProcessPoolExecutor
    import multiprocessing
    from .escape_search import EscapeSearchWorker
    executor=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'))
    worker=EscapeSearchWorker(search=search_snapshot,executor=executor)
    worker.warmup=executor.submit(warmup)
    return worker


def snapshot(inputs,context):
    """Copy forecast arrays; the worker never reads a ROS node or mutable cache."""
    values=dict(inputs)
    for name in ('people','radii'):
        values[name]=np.array(values[name],copy=True)
        values[name].setflags(write=False)
    return PrimitiveSnapshot(MappingProxyType(values),context)


def prioritized_programs(inputs,limit=8):
    """Cheap refuge-directed ordering, never a clearance or execution decision."""
    model=inputs['envelope']
    pose,goal=inputs['pose'],inputs['goal']
    heading=pose[2]+max(-.35,min(.35,inputs['velocity'][2]*model.yaw_time_constant))
    heading+=.5*inputs['filtered'][2]*(model.walking_yaw_gain or model.yaw_gain)*min(
        .7,abs(inputs['filtered'][2])/max(.01,model.accel_yaw))
    c,s=math.cos(heading),math.sin(heading)
    ranked=[]
    for index,program in enumerate(inputs['programs']):
        x=y=0.
        for duration,vx,vy,_ in program:
            x+=duration*vx*model.forward_gain
            y+=duration*vy*model.lateral_response_gain(vy)
        error=math.hypot(pose[0]+c*x-s*y-goal[0],pose[1]+s*x+c*y-goal[1])
        direction=tuple(round(value,4) for value in program[0][1:])
        ranked.append((error+.1*max(0,len(program)-2),index,direction))
    ranked.sort()
    directions=[]
    for _,_,direction in ranked:
        if direction not in directions:
            directions.append(direction)
        if len(directions)>=max(1,limit//2):
            break
    counts={direction:0 for direction in directions}
    selected=[]
    for _,index,direction in ranked:
        if direction in counts and counts[direction]<2:
            selected.append(inputs['programs'][index])
            counts[direction]+=1
        if len(selected)>=limit:
            break
    return tuple(selected)


def search_snapshot(request):
    inputs=dict(request.inputs)
    lags=inputs.pop('admission_delays',None)
    prioritize=inputs.pop('prioritize',True)
    if lags is None:
        return check_programs(**inputs,diagnostics=request.diagnostics)
    if not lags or any(not math.isfinite(lag) or lag<0. for lag in lags):
        raise ValueError('invalid admission delays')
    started=time.perf_counter()
    budget=inputs.pop('budget_ms',80.)
    attempts=[]
    plan=None
    batches=[('full',inputs['programs'])]
    if prioritize:
        batches.insert(0,('priority',prioritized_programs(inputs)))
    for kind,candidate_programs in batches:
        remaining=budget-(time.perf_counter()-started)*1000.
        nominal={}
        if budget>0. and remaining<=0.:
            nominal['reason']='deadline'
            break
        plan=check_programs(**{**inputs,'programs':candidate_programs},
            budget_ms=remaining if budget>0. else 0.,ranked_limit=4,diagnostics=nominal)
        attempts.append(dict(kind=kind,diagnostics=nominal))
        if plan is not None or len(candidate_programs)==len(inputs['programs']):
            break
    request.diagnostics['attempts']=attempts
    request.diagnostics.update(nominal=nominal,stage='nominal',reason=nominal['reason'])
    if plan is None:
        request.diagnostics['elapsed_ms']=(time.perf_counter()-started)*1000.
        return None
    remaining=budget-(time.perf_counter()-started)*1000.
    if budget>0. and remaining<=0.:
        request.diagnostics.update(reason='deadline',stage='admission')
        return None
    # Admission latency shifts the whole motion, not just the first waypoint.
    # Extend the forecast so every delayed variant keeps its full braking/hold.
    horizon=inputs['horizon']
    extended=horizon+max(lags)
    old_times=np.linspace(0.,horizon,len(inputs['people']))
    new_times=np.linspace(0.,extended,math.ceil(extended/max(.02,inputs['dt']))+1)
    people=inputs['people']
    flat=people.reshape(len(people),-1)
    resized=np.column_stack([np.interp(new_times,old_times,flat[:,i]) for i in range(flat.shape[1])]) if flat.shape[1] else np.empty((len(new_times),0))
    resized=resized.reshape(len(new_times),people.shape[1],2)
    velocity=(people[-1]-people[-2])/(old_times[-1]-old_times[-2])
    resized+=np.maximum(0.,new_times-horizon)[:,None,None]*velocity
    radii=inputs['radii']
    expanded_radii=np.column_stack([np.interp(new_times,old_times,radii[:,i]) for i in range(radii.shape[1])]) if radii.shape[1] else np.empty((len(new_times),0))
    programs=tuple(candidate_programs[i] for i in nominal['ranked_candidates'] for _ in lags)
    delays=np.tile(np.asarray(lags)+inputs['delay'],len(nominal['ranked_candidates']))
    remaining=budget-(time.perf_counter()-started)*1000.
    if budget>0. and remaining<=0.:
        request.diagnostics.update(reason='deadline',stage='admission')
        return None
    details={}
    plan=check_programs(**{**inputs,'programs':programs,'people':resized,'radii':expanded_radii,
        'horizon':extended,'delay':delays},budget_ms=remaining if budget>0. else 0.,
        admission_group_size=len(lags),diagnostics=details)
    request.diagnostics.update(details,stage='admission',admission_delays_s=list(lags),admission_horizon_s=extended,
                               elapsed_ms=(time.perf_counter()-started)*1000.)
    if plan is None and kind=='priority' and len(candidate_programs)<len(inputs['programs']):
        remaining=budget-(time.perf_counter()-started)*1000.
        if budget<=0. or remaining>0.:
            full=snapshot({**inputs,'admission_delays':lags,'prioritize':False,
                           'budget_ms':remaining if budget>0. else 0.},request.context)
            plan=search_snapshot(full)
            initial=dict(request.diagnostics)
            request.diagnostics.update(full.diagnostics,priority_attempt=initial,
                                       elapsed_ms=(time.perf_counter()-started)*1000.)
    if budget>0. and request.diagnostics['elapsed_ms']>budget:
        request.diagnostics['reason']='deadline'
        return None
    return plan


@lru_cache(maxsize=16)
def templates(envelope,horizon,forward_limit=None,reverse_limit=None,lateral_limit=None):
    """Straight and dogleg escapes with a final braking phase; no cached poses."""
    duration=horizon/5.
    speed=min(.45,envelope.max_planar_speed)
    forward_limit=envelope.max_x if forward_limit is None else forward_limit
    reverse_limit=envelope.max_x if reverse_limit is None else reverse_limit
    lateral_limit=envelope.max_y if lateral_limit is None else lateral_limit
    commands=[envelope.normalize((np.clip(speed*math.cos(a),-reverse_limit,forward_limit),
                                  np.clip(speed*math.sin(a),-lateral_limit,lateral_limit),0.))
              for a in np.linspace(-math.pi,math.pi,16,endpoint=False)]
    zero=(0.,0.,0.)
    programs=[((horizon,*zero),)]
    for index,command in enumerate(commands):
        for phases in (1,2,3,4):
            programs.append(((duration*phases,*command),(horizon-duration*phases,*zero)))
        for split in (1,2):
            for direction in (-4,4):
                programs.append(((duration*split,*command),
                    (duration*(4-split),*commands[(index+direction)%len(commands)]),(duration,*zero)))
    return tuple(programs)


@lru_cache(maxsize=1024)
def compiled_program(envelope,program):
    """Cache only constant command normalization and relative phase times."""
    return np.cumsum([part[0] for part in program]),np.asarray([envelope.normalize(part[1:]) for part in program])


def check_programs(envelope,pose,velocity,filtered,programs,people,radii,goal,
                   horizon,dt,delay,*,static_query=None,static_clearance=.48,
                   hold_s=2.,budget_ms=12.,diagnostics=None,minimum_margin=0.,
                   admission_group_size=1,ranked_limit=0):
    """Batch-check response, swept clearance, settling and a two-second hold.

    Returned schedules refer to this invocation's measured state and forecasts.
    A timeout never returns a partially checked candidate.
    """
    started=time.perf_counter()
    def finish(reason,**details):
        if diagnostics is not None:
            diagnostics.update(reason=reason,elapsed_ms=(time.perf_counter()-started)*1000.,**details)
    def expired():
        return budget_ms>0 and (time.perf_counter()-started)*1000.>budget_ms
    steps=max(1,math.ceil(horizon/max(.02,dt)))
    dt=horizon/steps
    people,radii=np.asarray(people),np.asarray(radii)
    if radii.ndim!=2 or people.shape!=(steps+1,radii.shape[1],2) or radii.shape!=people.shape[:2]:
        raise ValueError('forecast grid does not match rollout grid')
    if (not all(np.all(np.isfinite(value)) for value in (people,radii,pose,velocity,filtered,goal)) or
            np.any(radii<0.) or any(not all(math.isfinite(v) for v in part) or part[0]<=0.
                                    for program in programs for part in program)):
        raise ValueError('invalid escape input')
    if not programs:
        finish('empty_library')
        return None
    count=len(programs)
    if admission_group_size<1 or count%admission_group_size or not math.isfinite(minimum_margin) or minimum_margin<0.:
        raise ValueError('invalid admission group or reserve')
    delays=np.broadcast_to(np.asarray(delay,dtype=float),(count,))
    if not np.all(np.isfinite(delays)) or np.any(delays<0.):
        raise ValueError('invalid command delays')
    phases=max(map(len,programs))
    boundaries=np.full((count,phases),np.inf)
    commands=np.zeros((count,phases+1,3))
    for i,program in enumerate(programs):
        phase_times,normalized=compiled_program(envelope,tuple(program))
        boundaries[i,:len(program)]=delays[i]+phase_times
        commands[i,:len(program)]=normalized
    cuts=np.unique(np.r_[delays,boundaries[np.isfinite(boundaries)]])
    states=np.tile([*pose,*velocity,*filtered],(count,1)).astype(float)
    valid=np.ones(count,dtype=bool)
    margins=np.full(count,np.inf)
    indexes=np.arange(count)
    human_rejected=np.zeros(count,dtype=bool)
    static_rejected=np.zeros(count,dtype=bool)
    static_segments=[]
    if diagnostics is not None:
        diagnostics['preparation_ms']=(time.perf_counter()-started)*1000.
        diagnostics['required_margin_m']=minimum_margin
    def check(before,after,h0,h1,radius):
        if len(radius):
            if count<=12:
                humans=list(zip(h0.tolist(),h1.tolist(),radius.tolist()))
                for i,(a,b) in enumerate(zip(before.tolist(),after.tolist())):
                    if not valid[i]:
                        continue
                    for p,q,r in humans:
                        x,y=a[0]-p[0],a[1]-p[1]
                        dx,dy=b[0]-q[0]-x,b[1]-q[1]-y
                        fraction=max(0.,min(1.,-(x*dx+y*dy)/max(1e-12,dx*dx+dy*dy)))
                        separation=math.hypot(x+fraction*dx,y+fraction*dy)-r
                        margins[i]=min(margins[i],separation)
                        human_rejected[i]|=separation<minimum_margin
            else:
                relative=before[:,None,:]-h0
                delta=after[:,None,:]-h1-relative
                fraction=np.clip(-np.sum(relative*delta,axis=2)/np.maximum(1e-12,np.sum(delta*delta,axis=2)),0.,1.)
                separation=np.linalg.norm(relative+fraction[:,:,None]*delta,axis=2)-radius
                minimum=np.min(separation,axis=1)
                margins[:]=np.minimum(margins,minimum)
                human_rejected[:]|=minimum<minimum_margin
        if static_query is not None:
            static_segments.append((before,after.copy()))
        valid[:]&=~human_rejected & ~static_rejected
    def check_static():
        if not static_segments:
            return
        before=np.concatenate([pair[0] for pair in static_segments])
        after=np.concatenate([pair[1] for pair in static_segments])
        clearance=np.minimum(static_query(before),static_query(after))
        rejected=clearance<static_clearance+.5*np.linalg.norm(after-before,axis=1)
        static_rejected[:]|=np.any(rejected.reshape(len(static_segments),count),axis=0)
        valid[:]&=~static_rejected
        static_segments.clear()
    for step in range(steps):
        if diagnostics is not None:
            diagnostics['rollout_steps']=step
        if expired():
            finish('deadline')
            return None
        begin,end=step*dt,(step+1)*dt
        boundaries_here=[begin,*cuts[(cuts>begin)&(cuts<end)],end]
        for a,b in zip(boundaries_here,boundaries_here[1:]):
            before=states[:,:2].copy()
            phase=np.sum(boundaries<=a,axis=1)
            states=response_step(states,commands[indexes,phase],envelope,b-a,np.where(a>=delays,b-a,0.))
            h0=people[step]+(people[step+1]-people[step])*((a-begin)/dt)
            h1=people[step]+(people[step+1]-people[step])*((b-begin)/dt)
            check(before,states[:,:2],h0,h1,np.maximum(radii[step],radii[step+1]))
        if not np.any(valid):
            finish('no_feasible_prefix',human_rejections=int(human_rejected.sum()),static_rejections=int(static_rejected.sum()))
            return None
    # Static geometry is an immutable snapshot: batch identical conservative
    # segment checks instead of issuing hundreds of tiny grid queries.
    check_static()
    if not np.any(valid):
        finish('no_feasible_prefix',human_rejections=int(human_rejected.sum()),static_rejections=int(static_rejected.sum()))
        return None
    moving=(np.linalg.norm(states[:,3:5],axis=1)>=.08) | (np.abs(states[:,5])>=.12)
    valid &= ~moving
    endpoint=states[:,:2].copy()
    human_velocity=(people[-1]-people[-2])/dt
    hold_steps=max(1,math.ceil(hold_s/dt))
    hold_dt=hold_s/hold_steps
    for step in range(hold_steps):
        if diagnostics is not None:
            diagnostics['hold_steps']=step
        if expired():
            finish('deadline')
            return None
        before=states[:,:2].copy()
        states=response_step(states,np.zeros((count,3)),envelope,hold_dt,hold_dt)
        check(before,states[:,:2],people[-1]+step*hold_dt*human_velocity,
              people[-1]+(step+1)*hold_dt*human_velocity,radii[-1])
    check_static()
    if expired():
        finish('deadline')
        return None
    if not np.any(valid):
        finish('unsafe_terminal_state_or_hold',human_rejections=int(human_rejected.sum()),
               static_rejections=int(static_rejected.sum()),unsettled=int(moving.sum()))
        return None
    if admission_group_size>1:
        valid &= np.repeat(np.all(valid.reshape(-1,admission_group_size),axis=1),admission_group_size)
        margins=np.repeat(np.min(margins.reshape(-1,admission_group_size),axis=1),admission_group_size)
        if not np.any(valid):
            finish('no_complete_admission_group')
            return None
    # Stay near a selected refuge while preferring clearance over a small gain
    # in endpoint distance. Infinity means the scenario contains no people.
    score=2.*np.linalg.norm(endpoint-goal,axis=1)-np.minimum(margins,2.)
    chosen=int(np.argmin(np.where(valid,score,np.inf)))
    if expired():
        finish('deadline')
        return None
    if diagnostics is not None and ranked_limit:
        diagnostics['ranked_candidates']=np.flatnonzero(valid)[np.argsort(score[valid])][:ranked_limit].tolist()
    finish('complete',evaluated=count,valid=int(valid.sum()),selected=chosen,
           minimum_margin_m=float(margins[chosen]) if math.isfinite(margins[chosen]) else None,
           endpoint=endpoint[chosen].tolist())
    return tuple(programs[chosen])
