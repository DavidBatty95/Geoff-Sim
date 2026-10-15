#!/usr/bin/env python3
"""Portable outcomes/probe summaries; retain failures and incomplete artifacts."""
import argparse
import hashlib
import json
from pathlib import Path

from performance_outcome import strict_success

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT/'ros2_ws/src/dynamic_obstacle_benchmarks/results'


def load(path):
    return json.loads(path.read_text()) if path.is_file() else {}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def training_attempt(directory):
    return dict(run=directory.name,manifest=load(directory/'manifest.json'),
        manifest_sha256=digest(directory/'manifest.json'),
        artifact_complete=(directory/'final.pt').is_file() and (directory/'policy.onnx').is_file(),
        checkpoint_sha256=digest(directory/'final.pt'),policy_sha256=digest(directory/'policy.onnx'),
        source_sha256={name:digest(directory/name) for name in ('training_source.py','payload_sensor_delay.py','payload_smoothness.py')},
        exported_candidates=[dict(source=path.name,sha256=digest(path),metadata=load(path))
                             for path in sorted(directory.glob('policy_*.manifest.json'))])


def attempt(run):
    result,manifest = load(run/'result.json'),load(run/'manifest.json')
    provenance = load(run/'source_provenance.json')
    application = load(run/'command_application_diagnostics.json')
    events = result.get('events',[])
    complete = next((item['elapsed_s'] for item in reversed(events) if item.get('phase')=='complete'),None)
    settings = manifest.get('settings',{})
    policy = settings.get('policy_path','').removeprefix('/workspace/')
    strict = strict_success(result)
    return dict(run=run.name,world=manifest.get('world'),people=manifest.get('people'),
        layout_seed=manifest.get('layout_seed'),seed=manifest.get('seed'),objective=manifest.get('objective'),
        policy=policy,policy_sha256=manifest.get('source_sha256',{}).get(policy),
        configuration=dict(policy_device_clock=settings.get('policy_use_device_tick'),
            numeric_thread_environment=manifest.get('numeric_thread_environment'),
            application_timing=settings.get('mujoco_application_timing'),
            arm_executor_threads=settings.get('arm_compositor_executor_threads'),
            arm_repeat_period=settings.get('arm_compositor_repeat_publish_period'),
            slam_loop_closing=settings.get('slam_loop_closing'),
            tracker_body_wall_sheets=manifest.get('tracker_body_wall_sheets'),
            predictive_refuge=manifest.get('predictive_refuge'),
            early_overtaking_avoidance=manifest.get('early_overtaking_avoidance'),
            navigation_profile_sha256=manifest.get('navigation_profile_sha256')),
        success=strict,mission_complete=result.get('mission_complete'),data_valid=result.get('data_valid'),
        artifact_complete=(run/'result.json').is_file(),fallen=result.get('fallen'),
        contacts=result.get('contact_samples'),error=result.get('error','missing_result' if not result else None),
        deliveries=result.get('deliveries',[]),dropped_objects=result.get('dropped_objects'),
        completion_s=complete,last_recorded_event_s=events[-1].get('elapsed_s') if events else None,
        deadline_s=manifest.get('duration_s'),search=result.get('search_summary'),
        discoveries=[item for item in events if item.get('phase')=='search_found'],
        placement_alignment=[item for item in events if 'payload_alignment_error_m' in item],
        route_blockage=load(run/'route_blockage.json'),
        result_sha256=digest(run/'result.json'),manifest_sha256=digest(run/'manifest.json'),
        source_archive_sha256=provenance.get('source_archive_sha256'),
        effective_parameters_sha256=digest(run/'effective_parameters.json'),
        runtime_metrics=load(run/'response_diagnostics.json').get('metrics',{}),
        policy_hold_events=load(run/'response_diagnostics.json').get('policy_hold_events',[]),
        command_application={key:value for key,value in application.items() if key!='samples'},
        command_application_sha256=digest(run/'command_application_diagnostics.json'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--date',default='20261012')
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    if len(args.date)!=8 or not args.date.isdigit():
        parser.error('--date must be YYYYMMDD')
    attempts = [attempt(run) for run in sorted((RESULTS/'transport_demo').glob(args.date+'T*')) if run.is_dir()]
    probes = []
    directories = list(RESULTS.glob('turn_calibration_*'+args.date))
    directories.append(RESULTS/('turn_stop_'+args.date))
    for directory in sorted(directories):
        for path in sorted(directory.rglob('*.json')):
            if path.name=='summary.json':
                continue
            data = load(path)
            if isinstance(data,dict) and 'policy_sha256' in data:
                probes.append(dict(source=str(path.relative_to(ROOT)),sha256=digest(path),
                                   **{key:value for key,value in data.items() if key!='samples'}))
    matrices = []
    for path in sorted((RESULTS/'performance_matrix').glob(args.date+'T*/summary.json')):
        data = load(path)
        records = [{key:value for key,value in row.items() if key!='command'} for row in data.get('attempts',[])]
        matrices.append(dict(source=str(path.relative_to(ROOT)),sha256=digest(path),complete=data.get('complete'),
                             policy_sha256=data.get('policy_sha256'),attempts=records))
    report = dict(schema_version=1,role='native_performance_development_evidence',date=args.date,
                  scope='Evolving implementation and asynchronous trajectories; not a controlled speedup or acceptance cohort',
                  attempts=attempts,probes=probes,matrices=matrices,
                  training_runs=[training_attempt(path.parent) for path in sorted(
                      (ROOT/'vendor/mjlab/logs/payload_training').glob('*'+args.date+'/manifest.json'))])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(attempts=len(attempts),complete_passes=sum(row['success'] for row in attempts),
                          native_probes=len(probes),output=str(args.output))))


if __name__=='__main__':
    main()
