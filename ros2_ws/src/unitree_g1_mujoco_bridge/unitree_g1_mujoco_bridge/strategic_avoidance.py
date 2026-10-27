"""Sensed-crowd pincer prediction and refuge hints, never executable commands."""
import math

import numpy as np


def crowd_forecast(tracks,times,robot_radius,margin):
    people=np.array([[[p.x+p.vx*t,p.y+p.vy*t] for p in tracks] for t in times],dtype=float)
    radii=np.array([robot_radius+p.body_radius+margin for p in tracks],dtype=float)
    return people,radii


def predict_pincer(reference_xy,people,radii,times):
    """Find simultaneously closing, opposed pedestrian envelopes along a route.

    The seven-second caller horizon is an early warning, not permission to move.
    Parallel/diverging or distant pedestrians do not constitute a pincer.
    """
    reference_xy,people,radii,times=map(np.asarray,(reference_xy,people,radii,times))
    if len(radii)<2:
        return dict(active=False,time_to_enclosure_s=None,pairs=[])
    if not all(np.all(np.isfinite(value)) for value in (reference_xy,people,radii,times)):
        raise ValueError('nonfinite crowd forecast')
    relative=people-reference_xy[:,None,:]
    distance=np.linalg.norm(relative,axis=2)
    direction=relative/np.maximum(1e-6,distance[:,:,None])
    closing=distance<distance[0]-.1
    near=distance<radii[None,:]+1.
    pairs=[]
    earliest=math.inf
    for i in range(len(radii)):
        for j in range(i+1,len(radii)):
            opposed=np.sum(direction[:,i]*direction[:,j],axis=1)<-.2
            risky=opposed & near[:,i] & near[:,j] & closing[:,i] & closing[:,j]
            indexes=np.flatnonzero(risky)
            if len(indexes):
                earliest=min(earliest,float(times[indexes[0]]))
                pairs.append([i,j])
    return dict(active=bool(pairs),time_to_enclosure_s=earliest if pairs else None,pairs=pairs)


def predict_strategic_risk(reference_xy,people,radii,times,lead_margin=.8):
    """Start planning for a closing corridor, before two-sided enclosure forms.

    This is an early planning trigger only; it cannot admit a motion. Requiring
    actual closing and meaningful relative motion avoids chasing stationary
    jitter or distant/diverging crowds. Existing collision envelopes stay intact.
    """
    status=predict_pincer(reference_xy,people,radii,times)
    status['pincer_active']=status['active']
    status.update(reason='pincer' if status['active'] else 'clear',watch_indices=[],time_to_reserve_s=None)
    if len(radii)<2:
        return status
    distance=np.linalg.norm(people-reference_xy[:,None,:],axis=2)
    closing=distance<distance[0]-.25
    speeds=np.linalg.norm(people[-1]-people[0],axis=1)/max(.01,times[-1]-times[0])
    robot_moving=np.linalg.norm(reference_xy[-1]-reference_xy[0])>.25
    approaching=closing & ((speeds>.2) | robot_moving)[None,:]
    warning=approaching & (distance-radii[None,:]<lead_margin)
    indexes=np.argwhere(warning)
    if len(indexes):
        status['active']=True
        status['watch_indices']=np.flatnonzero(np.any(warning,axis=0)).tolist()
        status['time_to_reserve_s']=float(times[indexes[0,0]])
        if not status['pincer_active']:
            status['reason']='closing_corridor'
    return status


def refuge_candidates(pose,people,radii,times,*,static_query=None,static_clearance=.48,
                      previous=None,speed=.35,delay=.3,hold_s=2.):
    """Rank coarse routes against every person, including their terminal hold.

    Straight travel is a strategic approximation only. The caller must use a
    response-aware sequence search and final collision checks before execution.
    A previously selected clear refuge is preferred within a 12 cm score band.
    """
    origin=np.asarray(pose[:2],dtype=float)
    goals=[origin.copy()]
    if previous is not None:
        goals.append(np.asarray(previous,dtype=float))
    for radius in (.7,1.4):
        for angle in np.linspace(-math.pi,math.pi,16,endpoint=False):
            goals.append(origin+radius*np.array((math.cos(angle),math.sin(angle))))
    # Retain the original per-person sidestep choices in the search set, but
    # evaluate them against the entire crowd rather than the nearest walker.
    velocities=(people[-1]-people[0])/max(.01,times[-1]-times[0])
    for vx,vy in velocities:
        magnitude=math.hypot(vx,vy)
        if magnitude>.25:
            normal=np.array((-vy,vx))/magnitude
            goals.extend((origin+1.5*normal,origin-1.5*normal))
    # Batch the coarse geometry and static queries instead of allocating and
    # querying a distance field separately for each refuge on the control tick.
    goals=np.asarray(goals)
    distances=np.linalg.norm(goals-origin,axis=1)
    durations=distances/max(.01,speed)
    valid=durations+delay<=times[-1]-hold_s
    alpha=np.clip((times[None,:]-delay)/np.maximum(.001,durations[:,None]),0.,1.)
    paths=origin+alpha[:,:,None]*(goals-origin)[:,None,:]
    if static_query is not None:
        clearance=np.asarray(static_query(paths.reshape(-1,2))).reshape(paths.shape[:2])
        step=np.linalg.norm(np.diff(paths,axis=1),axis=2)
        valid &= np.all(np.minimum(clearance[:,:-1],clearance[:,1:])>=static_clearance+.5*step,axis=1)
    relative=paths[:,:,None,:]-people[None,:,:,:]
    delta=np.diff(relative,axis=1)
    fraction=np.clip(-np.sum(relative[:,:-1]*delta,axis=3)/np.maximum(1e-12,np.sum(delta*delta,axis=3)),0.,1.)
    swept=np.linalg.norm(relative[:,:-1]+fraction[:,:,:,None]*delta,axis=3)-radii
    margins=np.min(swept,axis=(1,2))
    holding=times[None,1:]>=durations[:,None]+delay
    terminals=np.min(np.where(holding[:,:,None],swept,np.inf),axis=(1,2))
    results=[dict(goal=goals[i].tolist(),minimum_margin_m=float(margins[i]),
                  terminal_margin_m=float(terminals[i]),travel_m=float(distances[i]),retained=False)
             for i in np.flatnonzero(valid)]
    results.sort(key=lambda row:(-row['minimum_margin_m'],row['travel_m']))
    if results and previous is not None:
        retained=next((row for row in results if np.linalg.norm(np.asarray(row['goal'])-previous)<1e-6
                       and row['minimum_margin_m']>=0.
                       and row['minimum_margin_m']>=results[0]['minimum_margin_m']-.12),None)
        if retained is not None:
            retained['retained']=True
            results.remove(retained)
            results.insert(0,retained)
    return results
