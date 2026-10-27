import json

from analyze_contact_trace import analyze


def fixture(path,frozen=False):
    contacts=[]
    rows=[]
    for i in range(31):
        tick=i*50
        contacts.append(dict(sim_time=tick*.001,device_tick_ms=tick,physics_steps=25,
            contact_steps=1 if i==20 else 0,peak_normal_force_n=12. if i==20 else 0.,normal_impulse_ns=.024 if i==20 else 0.))
        for topic,data in [('lowstate_control',dict(device_tick_ms=0 if frozen else tick)),
                           ('lowcmd_control',{}),('policy_cmd',dict(x=0.,y=0.,z=0.))]:
            rows.append(dict(topic=topic,data=data,receipt_monotonic_s=10.+tick*.001))
    (path/'native_contacts.jsonl').write_text('\n'.join(map(json.dumps,contacts)))
    (path/'trace.jsonl').write_text('\n'.join(map(json.dumps,rows)))


def test_native_forces_are_distinct_from_zero_policy_command(tmp_path):
    fixture(tmp_path)
    result=analyze(tmp_path)
    assert result['capture_valid']
    assert result['peak_normal_force_n']==12.
    assert result['normal_impulse_ns']==.024
    assert len(result['force_windows'])==1
    assert result['force_windows'][0]['policy_command']==dict(x=0.,y=0.,z=0.)


def test_frozen_device_clock_does_not_validate_capture(tmp_path):
    fixture(tmp_path,frozen=True)
    assert not analyze(tmp_path)['capture_valid']
