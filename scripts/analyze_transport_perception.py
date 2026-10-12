#!/usr/bin/env python3
"""Summarize acquisition timing and outcomes from retained transport runs."""
import argparse
import json
from pathlib import Path

import numpy as np
from transport_crowd import crowd_summary


def distribution(values):
    if not values:
        return dict(count=0)
    return dict(count=len(values),median=float(np.median(values)),
                p95=float(np.quantile(values,.95)),maximum=float(max(values)))


def search_summary(events):
    """Use mission discoveries, never evaluator-visible object IDs, as evidence."""
    requests = []
    active = None
    plans = []
    for event in events:
        if event.get('phase')=='search_request':
            active = dict(object_id=event['object_id'],requested_at_s=event['elapsed_s'],
                          initially_visible=event['initially_visible'],first_goal_after_s=None,
                          first_dispatch_after_s=None,found_after_s=None)
            requests.append(active)
        if event.get('phase')=='search_viewpoint' and 'viewpoint_map' in event:
            plans.append(event)
            if active is not None and active['object_id']==event.get('object_id') and active['first_goal_after_s'] is None:
                active['first_goal_after_s'] = event['elapsed_s']-active['requested_at_s']
        if event.get('phase')=='search_found' and active is not None and active['object_id']==event.get('object_id'):
            active['found_after_s'] = event['elapsed_s']-active['requested_at_s']
        if (active is not None and active['first_dispatch_after_s'] is None and
                event.get('mode')=='navigate' and event.get('phase') in ('search_viewpoint','search_route')):
            active['first_dispatch_after_s'] = event['elapsed_s']-active['requested_at_s']
    rooms = [event for event in events if event.get('phase')=='search_room_decision']
    return dict(requests=requests,selected_viewpoints=len(plans),
                planning_ms=distribution([e['planning_ms'] for e in plans if 'planning_ms' in e]),
                selection_wait_ms=distribution([e['selection_wait_ms'] for e in plans if 'selection_wait_ms' in e]),
                room_decisions=len(rooms),region_count=distribution([e['regions'] for e in rooms]),
                deferred_region_selections=sum(bool((e.get('selected') or {}).get('deferred')) for e in rooms),
                partial_turns=sum(e.get('phase')=='search_partial_turn' for e in events),
                route_planning_ms=distribution([e['planning_ms'] for e in events
                    if e.get('phase') in ('search_route','pickup_route','delivery_route') and 'checkpoint_odom' in e]))


def pickup_summary(events):
    """Keep grasp request, held acknowledgement and camera-verified lift distinct."""
    pickups = []
    active = None
    for event in events:
        phase = event.get('phase')
        if phase=='search_found' and event.get('object_id','').startswith('box_'):
            active = dict(object_id=event['object_id'],discovered_at_s=event['elapsed_s'],
                          grasp_request_after_s=None,held_ack_after_s=None,verified_lift_after_s=None,
                          prepare_after_s=None,discovery_to_placement_s=None,grasp_to_placement_s=None)
            pickups.append(active)
        if active is None:
            continue
        field = None
        if phase=='prepare_pickup':
            field = 'prepare_after_s'
        elif phase=='grasp' and event.get('attachment_state')=='held':
            field = 'held_ack_after_s'
        elif phase=='grasp' and event.get('perception_source')=='registered_rgbd':
            field = 'grasp_request_after_s'
        elif phase=='retreat_pickup':
            field = 'verified_lift_after_s'
        if field is not None and active[field] is None:
            active[field] = event['elapsed_s']-active['discovered_at_s']
        if phase=='delivered' and event.get('object_id')==active['object_id']:
            active['discovery_to_placement_s'] = event['elapsed_s']-active['discovered_at_s']
            if active['held_ack_after_s'] is not None:
                active['grasp_to_placement_s'] = active['discovery_to_placement_s']-active['held_ack_after_s']
    return pickups


def summarize(run):
    history = json.loads((run/'perception_history.json').read_text()) if (run/'perception_history.json').exists() else []
    attempts = json.loads((run/'perception_attempts.json').read_text()) if (run/'perception_attempts.json').exists() else []
    result = json.loads((run/'result.json').read_text())
    manifest = json.loads((run/'manifest.json').read_text())
    arrivals = {}
    for item in attempts:
        arrivals.setdefault(item['stamp'],item['processed_at'])
    stamps = sorted(set(item['stamp'] for item in history))
    summary = dict(run=run.name,policy=manifest['settings'].get('policy_path'),
                seed=manifest['seed'],
                people=manifest['people'],contact_samples=result['contact_samples'],
                minimum_person_clearance_m=result.get('minimum_person_clearance_m'),
                source_sha256=manifest['source_sha256'],
                localization_recoveries=sum(bool(item.get('localization_recovered')) for item in result.get('events',[])),
                object_timestamp_scope=result.get('evaluation_health',{}).get('object_timestamp_scope'),
                history_available=(run/'perception_history.json').exists(),
                camera=[manifest['settings'][key] for key in ('camera_width','camera_height')],
                duration_s=manifest['duration_s'],
                acquisition_interval_s=distribution(np.diff(stamps).tolist()),
                first_attempt_age_s=distribution([arrival-stamp for stamp,arrival in arrivals.items()]),
                processed_age_s=distribution([item['processed_at']-item['stamp'] for item in history]),
                confirmed_ids=sorted({identifier for item in history for identifier in item['confirmed']}),
                success=result['success'],data_valid=result['data_valid'],fallen=result['fallen'],
                 deliveries=result['deliveries'],dropped_objects=result['dropped_objects'],error=result.get('error'))
    if result.get('mission_kind')=='explore_pick_find_place':
        summary.update(world=manifest['world'],layout_seed=manifest.get('layout_seed'),
                       early_overtaking_avoidance=manifest.get('early_overtaking_avoidance'),
                       tracker_wall_evidence=manifest.get('tracker_wall_evidence'),
                       mission_kind=result['mission_kind'],search=search_summary(result.get('events',[])),
                       search_summary=result.get('search_summary'),pickup=pickup_summary(result.get('events',[])),
                       objective=result.get('objective',manifest.get('objective')),
                       delivery_approaches=[event for event in result.get('events',[]) if event.get('phase')=='delivery_approach'])
        provenance = run/'source_provenance.json'
        if provenance.exists():
            captured = json.loads(provenance.read_text())
            summary['source_archive_sha256'] = captured['source_archive_sha256']
            summary['search_source_sha256'] = {name:sha for name,sha in captured['source_sha256'].items()
                if Path(name).name in {'exploration_search.py','exploration_mission.py','search_visualization.py','room_search.py',
                                      'dynamic_obstacle_tracker.py','wall_evidence.py','localization_recovery.py',
                                      'human_aware_motion_planner.py','bridge_safety.py'}}
    if manifest['people'] and (run/'trace.jsonl').exists():
        with (run/'trace.jsonl').open() as stream:
            summary['crowd'] = crowd_summary(json.loads(line) for line in stream if line.strip())
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--quiet',action='store_true',help='Write --output without printing the full JSON')
    args = parser.parse_args()
    if args.quiet and args.output is None:
        parser.error('--quiet requires --output')
    text = json.dumps([summarize(run) for run in args.runs],indent=2)+'\n'
    if args.output:
        args.output.write_text(text)
    if not args.quiet:
        print(text,end='')


if __name__=='__main__':
    main()
