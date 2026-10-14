#!/usr/bin/env python3
"""Offline native application timing; repeated content yields bounds, not IDs."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path

from analyze_response_trace import distribution


def correlate(applications,publications,policies,window_s=1.):
    by_crc,by_target = defaultdict(list),defaultdict(list)
    for row in publications:
        by_crc[row['crc']].append(row)
    for row in policies:
        if row.get('timing_valid') and row.get('leg_target_sha256'):
            by_target[row['leg_target_sha256']].append(row)
    samples = []
    missing = ambiguous = policy_missing = 0
    for application in applications:
        applied,received = application['applied_monotonic_ns'],application['received_monotonic_ns']
        if applied<received or received<=0:
            raise ValueError('invalid native application clock ordering')
        candidates = [row for row in by_crc[application['crc']]
                      if 0<=received-row['publish_before_monotonic_ns']<=window_s*1e9 and
                      row['publish_after_monotonic_ns']>=row['publish_before_monotonic_ns']]
        sample = dict(application,receipt_to_apply_ms=(applied-received)/1e6,
                      matching_publications=len(candidates))
        if not candidates:
            missing += 1
        else:
            ambiguous += len(candidates)>1
            sample['lowcmd_to_apply_lower_ms'] = min(max(0,applied-row['publish_after_monotonic_ns'])/1e6 for row in candidates)
            sample['lowcmd_to_apply_upper_ms'] = max((applied-row['publish_before_monotonic_ns'])/1e6 for row in candidates)
            identities = {row['leg_target_sha256'] for row in candidates}
            preceding = [row for key in identities for row in by_target[key]
                         if 0<=applied-row['published_monotonic_s']*1e9<=window_s*1e9]
            sample['matching_policy_publications'] = len(preceding)
            if preceding:
                latencies = [(applied-row['published_monotonic_s']*1e9)/1e6 for row in preceding]
                sample['policy_to_apply_lower_ms'],sample['policy_to_apply_upper_ms'] = min(latencies),max(latencies)
                # Add paired per-event intervals, never population percentiles.
                observed = [latency+row['lowstate_dds_source_to_target_publish_ms']
                            for row,latency in zip(preceding,latencies)
                            if row.get('context_matched') is True and
                            isinstance(row.get('lowstate_dds_source_to_target_publish_ms'),(int,float)) and
                            row['lowstate_dds_source_to_target_publish_ms']>=0.]
                if observed:
                    sample['dds_source_to_apply_lower_ms'],sample['dds_source_to_apply_upper_ms'] = min(observed),max(observed)
                tick_ages = [((application['device_tick_ms']-row['device_tick_ms'])&0xffffffff)
                             for row in preceding if row.get('context_matched') is True and
                             row.get('observation_clock')=='device_tick' and
                             type(row.get('device_tick_ms')) is int and type(application.get('device_tick_ms')) is int]
                tick_ages = [age for age in tick_ages if age<0x80000000]
                if tick_ages:
                    sample['device_observation_to_apply_lower_ms'],sample['device_observation_to_apply_upper_ms'] = min(tick_ages),max(tick_ages)
            else:
                policy_missing += 1
        samples.append(sample)
    metrics = {key:distribution([row[key] for row in samples if key in row]) for key in
               ('receipt_to_apply_ms','lowcmd_to_apply_lower_ms','lowcmd_to_apply_upper_ms',
                'policy_to_apply_lower_ms','policy_to_apply_upper_ms',
                'dds_source_to_apply_lower_ms','dds_source_to_apply_upper_ms',
                'device_observation_to_apply_lower_ms','device_observation_to_apply_upper_ms')}
    stamps = [row['publish_before_monotonic_ns'] for row in publications]
    span = (max(stamps)-min(stamps))/1e9 if len(stamps)>1 else 0.
    repeated = sum(a['crc']==b['crc'] for a,b in zip(publications,publications[1:]))
    sequences = [row.get('sequence') for row in publications]
    gaps = (sum(max(0,b-a-1) for a,b in zip(sequences,sequences[1:]))
            if sequences and all(type(value) is int for value in sequences) else None)
    traffic = dict(recorded_publications=len(publications),span_s=span,
                   recorded_rate_hz=(len(stamps)-1)/span if span>0. else None,
                   consecutive_repeat_fraction=repeated/(len(stamps)-1) if len(stamps)>1 else None,
                   sequence_gaps=gaps)
    return dict(role='sampled_native_application_content_lineage',matching_window_s=window_s,
                publication_traffic=traffic,
                content_transitions_only=bool(samples) and all(row.get('content_transition') is True for row in samples),
                applications=len(samples),unmatched_lowcmd=missing,ambiguous_lowcmd=ambiguous,
                matched_lowcmd_without_policy=policy_missing,metrics=metrics,samples=samples,
                limitations=[
                    'Same-host Linux monotonic clocks are required.',
                    'Device-tick intervals use simulation milliseconds; wall intervals must not be equated with injected simulation delay.',
                    'Native first-application records are sampled at 10 Hz, not every command.',
                    'Legacy records without content_transition include unchanged-target refreshes; their policy ages are not transition latency.',
                    'CRC and leg-target hashes identify content, not unique publication IDs.',
                    'Bounds are conditional on recorded matching content within the configured window; missing messages and CRC collisions cannot be excluded.',
                    'Identical targets and composed arms can make policy lineage ambiguous.',
                    'This measures control assignment before a physics step, not physical actuator hardware or motion onset.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    applications,publications,policies = [],[],[]
    malformed = 0
    prefix = '[native_command_application] '
    log,trace = args.run/'simulation.log',args.run/'trace.jsonl'
    for line in log.open():
        if prefix in line:
            try:
                applications.append(json.loads(line.split(prefix,1)[1]))
            except ValueError:
                malformed += 1
    for line in trace.open():
        row = json.loads(line)
        if row.get('topic')=='lowcmd_timing':
            publications.append(row['data'])
        elif row.get('topic')=='policy_timing':
            policies.append(row['data'])
    report = correlate(applications,publications,policies)
    report.update(malformed_native_records=malformed,
                  simulation_log_sha256=hashlib.sha256(log.read_bytes()).hexdigest(),
                  trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
                  analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({key:value for key,value in report.items() if key!='samples'},indent=2))


if __name__=='__main__':
    main()
