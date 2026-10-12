"""Read-only replay of logged filtered commands, not inference acknowledgements."""
from bisect import bisect_right
import json
import math


def load_commands(path):
    samples = []
    empty_state_seen = False
    arm_phases = {'prepare_pickup','dock_pickup','reach','grasp','lift','lower','release','retract','restore_arm'}
    with path.open() as stream:
        for line in stream:
            row = json.loads(line)
            data = row.get('data',{})
            if row.get('topic')=='transport_state' and isinstance(data,dict):
                if data.get('attachment_state')!='empty' or data.get('object_may_be_held') or data.get('phase') in arm_phases:
                    raise ValueError('command replay supports empty-hand navigation only')
                empty_state_seen = True
            if row.get('topic')!='policy_cmd' or not isinstance(data,dict):
                continue
            stamp = float(data['source_stamp'])
            command = tuple(float(data[key]) for key in ('x','y','z'))
            if not all(map(math.isfinite,(stamp,*command))):
                raise ValueError('nonfinite policy command sample')
            if samples and stamp<=samples[-1][0]:
                if (stamp,command)==samples[-1]:
                    continue
                raise ValueError('conflicting or reordered policy command samples')
            samples.append((stamp,command))
    if not empty_state_seen:
        raise ValueError('empty-hand state evidence unavailable')
    first = next((index for index,(_,cmd) in enumerate(samples) if max(map(abs,cmd))>.03),None)
    if first is None:
        raise ValueError('no motion commands in trace')
    origin = samples[first][0]
    return [stamp-origin for stamp,_ in samples[first:]],[cmd for _,cmd in samples[first:]]


def command_at(times,commands,elapsed):
    if elapsed<0. or elapsed>times[-1]:
        return (0.,0.,0.)
    return commands[bisect_right(times,elapsed)-1]
