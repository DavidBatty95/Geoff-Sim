import json
import pytest

from policy_command_replay import load_commands,command_at


def test_replay_preserves_source_timing_and_never_extrapolates_commands(tmp_path):
    rows = [dict(topic='policy_cmd',data=dict(source_stamp=t,x=x,y=0.,z=0.))
            for t,x in ((10.,0.),(11.,.4),(11.,.4),(11.2,0.))]
    rows.insert(0,dict(topic='transport_state',data=dict(attachment_state='empty',phase='search_route')))
    path = tmp_path/'trace.jsonl'
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    times,commands = load_commands(path)
    assert times==pytest.approx([0.,.2])
    assert command_at(times,commands,-1.)==(0.,0.,0.)
    assert command_at(times,commands,.1)==(.4,0.,0.)
    assert command_at(times,commands,1.)==(0.,0.,0.)


def test_replay_rejects_external_arm_activity_and_conflicting_samples(tmp_path):
    path = tmp_path/'trace.jsonl'
    path.write_text(json.dumps(dict(topic='transport_state',data=dict(phase='reach')))+'\n')
    with pytest.raises(ValueError,match='empty-hand'):
        load_commands(path)
    rows = [dict(topic='policy_cmd',data=dict(source_stamp=1.,x=x,y=0.,z=0.)) for x in (.4,.2)]
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    with pytest.raises(ValueError,match='conflicting'):
        load_commands(path)


def test_commands_without_possession_evidence_are_not_replayed(tmp_path):
    path = tmp_path/'trace.jsonl'
    path.write_text(json.dumps(dict(topic='policy_cmd',data=dict(source_stamp=1.,x=.4,y=0.,z=0.)))+'\n')
    with pytest.raises(ValueError,match='state evidence'):
        load_commands(path)


def test_missing_command_interval_is_not_silently_held(tmp_path):
    rows = [dict(topic='transport_state',data=dict(attachment_state='empty',phase='search_route'))]
    rows += [dict(topic='policy_cmd',data=dict(source_stamp=t,x=x,y=0.,z=0.))
             for t,x in ((10.,.4),(10.5,0.))]
    path = tmp_path/'trace.jsonl'
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    with pytest.raises(ValueError,match='coverage gap'):
        load_commands(path)
