import pytest

from run_transport_demo import select_transport_task


PAIRS = [dict(object_id=f'box_{color}',destination_id=f'target_{color}',color=color)
         for color in ('red','blue')]


def test_explicit_cross_color_objective_contains_no_layout_and_keeps_source_color():
    objective = select_transport_task(PAIRS,'red:blue')
    assert objective['pairs']==[dict(object_id='box_red',destination_id='target_blue',
                                     color='red',destination_color='blue')]
    assert objective['description']=='Put the red box on the blue goal'
    assert objective['pairs'][0] is not PAIRS[0]


def test_seeded_random_objectives_replay_and_cover_all_color_combinations():
    assert select_transport_task(PAIRS,seed=48)==select_transport_task(PAIRS,seed=48)
    outcomes = {tuple(select_transport_task(PAIRS,seed=seed)['pairs'][0][key]
                      for key in ('object_id','destination_id')) for seed in range(100)}
    assert outcomes=={(f'box_{box}',f'target_{goal}') for box in ('red','blue') for goal in ('red','blue')}
    generated = select_transport_task(PAIRS)
    assert select_transport_task(PAIRS,seed=generated['task_seed'])==generated


def test_invalid_or_unavailable_objective_is_rejected():
    for task in ('red','red:green','green:blue','red:blue:green'):
        with pytest.raises(ValueError):
            select_transport_task(PAIRS,task)
    assert select_transport_task(PAIRS,'all')['pairs']==PAIRS
