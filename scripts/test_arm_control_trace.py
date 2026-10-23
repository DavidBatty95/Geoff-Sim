import json

import pytest

from analyze_arm_control_trace import analyze


@pytest.mark.parametrize('fault',[None,'missing_commands','receipt_gap','frozen_ticks'])
def test_capture_requires_complete_advancing_control_streams(tmp_path,fault):
    rows = [dict(topic='transport_state',data=dict(phase='probe_extension_hold'),receipt_monotonic_s=0.)]
    for index in range(101):
        t = index/10
        data = {
            'evaluation_odom':dict(x=0.,y=0.,qx=0.,qy=0.,qz=0.,qw=1.),
            'actor_targets':[0.]*29,'composed_targets':[0.]*29,
            'lowstate_control':dict(device_tick_ms=0 if fault=='frozen_ticks' else index*100,q=[0.]*29),
            'lowcmd_control':dict(motors=[dict(q=0.,kp=160.,kd=4.) for _ in range(29)])}
        for topic,payload in data.items():
            if topic=='lowcmd_control' and (fault=='missing_commands' or (fault=='receipt_gap' and 30<index<50)):
                continue
            rows.append(dict(topic=topic,data=payload,receipt_monotonic_s=t))
    path = tmp_path/'trace.jsonl'
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    assert analyze(path)['capture_valid'] == (fault is None)
