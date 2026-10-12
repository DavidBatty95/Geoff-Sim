"""Offline crowd motion and planner load evidence; never an executive input."""
from collections import Counter
import math

import numpy as np


def crowd_summary(rows):
    actors,previous = {},{}
    states = Counter()
    planning = []
    max_tracks = 0
    overtaking_samples = 0
    gaps = 0
    for row in rows:
        data = row.get('data',{})
        if not isinstance(data,dict):
            continue
        if row.get('topic')=='planner_debug':
            states[data.get('state','unknown')] += 1
            max_tracks = max(max_tracks,int(data.get('track_count',0)))
            overtaking_samples += int(any(candidate.get('score_terms',{}).get('overtaking_conflict',0.)>0.
                                         for candidate in data.get('candidate_results',[])))
            elapsed = data.get('planning_ms')
            if isinstance(elapsed,(int,float)) and math.isfinite(elapsed):
                planning.append(elapsed)
        if row.get('topic')!='evaluation_aligned':
            continue
        stamp = data.get('sim_time')
        if not isinstance(stamp,(int,float)) or not math.isfinite(stamp):
            continue
        for person in data.get('people',[]):
            identifier = str(person['id'])
            point = float(person['x']),float(person['y'])
            speed = math.hypot(person['vx'],person['vy'])
            if not all(map(math.isfinite,(*point,speed))):
                continue
            last = previous.get(identifier)
            if last is not None and stamp<=last[0]:
                continue
            actor = actors.setdefault(identifier,dict(first_position=list(point),samples=0,moving_samples=0,
                                                       path_length_m=0.,max_displacement_m=0.,maximum_speed_m_s=0.))
            actor['samples'] += 1
            actor['moving_samples'] += int(speed>.05)
            actor['maximum_speed_m_s'] = max(actor['maximum_speed_m_s'],speed)
            actor['max_displacement_m'] = max(actor['max_displacement_m'],math.dist(point,actor['first_position']))
            if last is not None:
                if stamp-last[0]<=1.:
                    actor['path_length_m'] += math.dist(point,last[1])
                else:
                    gaps += 1  # Do not count an unobserved interval as measured travel.
            previous[identifier] = stamp,point
    return dict(source='offline_evaluation_aligned_and_planner_trace',actors=actors,
                moving_actor_count=sum(actor['path_length_m']>.5 and actor['moving_samples']>0 for actor in actors.values()),
                unmeasured_motion_gaps=gaps,max_observed_tracks=max_tracks,planner_states=dict(states),
                overtaking_penalty_samples=overtaking_samples,
                planner_planning_ms=dict(count=len(planning),
                    median=float(np.median(planning)) if planning else None,
                    p95=float(np.quantile(planning,.95)) if planning else None,
                    maximum=max(planning) if planning else None))
