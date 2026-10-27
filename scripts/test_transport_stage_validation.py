import json

import pytest

from publish_transport_stage_validation import summarize


@pytest.mark.parametrize('explicit_end',[None,1.15])
def test_publications_are_scoped_without_search_diagnostics_overwriting_mission_phase(tmp_path,explicit_end):
    result={'events':[],'success':False}
    if explicit_end is not None:
        result['task_end_monotonic_s']=explicit_end
    (tmp_path/'result.json').write_text(json.dumps(result))
    (tmp_path/'manifest.json').write_text('{}')
    rows=[]
    def add(topic,stamp,data):
        rows.append(dict(topic=topic,stamp=stamp,receipt_monotonic_s=stamp,data=data))
    def command(stamp,sequence):
        add('planner_debug',stamp,dict(selected='escape_primitive',
            command_sequence=dict(publisher_id='planner',sequence_id=sequence,first=[.3,0.,0.]),
            strategic_avoidance={'active':True},selected_cmd={'x':.3,'y':0.,'z':0.},
            kinodynamic_details={'beam':{'reason':'complete'}}))
    add('transport_state',1.,{'phase':'discover'})
    command(1.1,1)
    command(1.2,2)
    add('transport_state',2.,{'phase':'survey_complete'})
    command(2.1,3)
    add('safety_debug',2.12,dict(command_publisher_id='planner',command_sequence_id=3,
        command_envelope_valid=True,output_cmd={'x':.3,'y':0.,'z':0.},mode='pass_through'))
    (tmp_path/'trace.jsonl').write_text('\n'.join(map(json.dumps,rows)))
    report=summarize(tmp_path)['primitive_supervision']
    assert report['planner_phases']=={'discover':2,'survey_complete':1}
    assert report['planner_periods']==({'before_task_end':2,'after_task':1} if explicit_end is None else
                                         {'before_task_end':1,'after_task':2})
    assert report['counters']['preserved_translation']==1
