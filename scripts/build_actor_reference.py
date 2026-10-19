"""Build a provenance-linked rehearsal corpus from strictly passing native probes."""
import argparse
import hashlib
import json
from pathlib import Path

from score_native_stopping import motion_progress, score_probe


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output exists')
    observations, actions, sources, policies = [], [], [], set()
    moving = stopped = 0
    for path in args.inputs:
        data = json.loads(path.read_text())
        if not score_probe(data)['passed'] or not motion_progress(data)['passed']:
            raise ValueError(f'unqualified reference source: {path}')
        policies.add(data['policy_sha256'])
        sources.append(dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        for row in data['samples']:
            # Include movement and post-stop states, not just initial standing.
            if row['time'] < data['actual_command_start_s']:
                continue
            observations.append(row['actor_observation'])
            actions.append(row['published_action'])
            moving += int(any(row['command']))
            stopped += int(not any(row['command']))
    if len(policies) != 1 or not moving or not stopped:
        raise ValueError('one reference policy and both motion phases required')
    args.output.write_text(json.dumps(dict(observations=observations, actions=actions,
        sources=sources, policy_sha256=policies.pop(), all_sources_passed=True,
        moving_samples=moving, stopped_samples=stopped), allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
