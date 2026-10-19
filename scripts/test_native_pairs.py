import pytest
from summarize_native_pairs import compare


def report():
    rows = [dict(policy=p,case_id=f'{p}{i}',load='payload',delay_steps=3,linear_y=.45,
                 command_duration_s=i+1,passed=passed)
            for p,verdicts in [('candidate',[True,False]),('baseline',[False,True])]
            for i,passed in enumerate(verdicts)]
    return dict(complete=True,plan=rows.copy(),attempts=rows.copy())


def test_equal_totals_cannot_hide_a_regression():
    result = compare(report(),'candidate','baseline')
    assert result['fixes']==result['regressions']==1
    assert [row['kind'] for row in result['changes']]==['fix','regression']


@pytest.mark.parametrize('fault',['missing','duplicate','incomplete'])
def test_unmatched_evidence_is_rejected(fault):
    data = report()
    if fault=='missing':
        data['attempts'].pop()
    elif fault=='duplicate':
        data['attempts'].append(data['attempts'][0])
    else:
        data['complete'] = False
    with pytest.raises(ValueError):
        compare(data,'candidate','baseline')
