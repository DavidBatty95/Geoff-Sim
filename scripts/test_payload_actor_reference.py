import json
from types import SimpleNamespace

import pytest
import torch

from payload_actor_reference import ActorReferenceLoss


def test_rehearsal_gradient_reduces_reference_error_without_rng_changes(tmp_path):
    actor = torch.nn.Module()
    actor.mlp = torch.nn.Linear(480,29,bias=False)
    actor.is_recurrent = actor.obs_normalization = False
    with torch.no_grad():
        actor.mlp.weight.fill_(.01)
    path = tmp_path/'corpus.json'
    path.write_text(json.dumps(dict(observations=[[1.]*480]*4,actions=[[0.]*29]*4,
        all_sources_passed=True,moving_samples=2,stopped_samples=2)))
    reference = ActorReferenceLoss(actor,path,1.)
    state = torch.random.get_rng_state().clone()
    loss = reference.compute_loss(actor,None,4)
    loss.backward()
    assert actor.mlp.weight.grad.abs().sum()>0
    torch.optim.SGD(actor.parameters(),lr=.001).step()
    assert reference.compute_loss(actor,None,4)<loss
    assert torch.equal(torch.random.get_rng_state(),state)
    assert not reference.actions.requires_grad and not reference.observations.requires_grad
    assert reference.audit()['updates']==2


@pytest.mark.parametrize('weight',[0.,-1.,float('nan')])
def test_invalid_weight_is_rejected(weight):
    with pytest.raises(ValueError):
        ActorReferenceLoss(SimpleNamespace(),None,weight)


@pytest.mark.parametrize('fault',['shape','nonfinite','no_motion','unqualified'])
def test_invalid_reference_evidence_is_rejected(tmp_path,fault):
    actor = torch.nn.Module()
    actor.mlp = torch.nn.Linear(480,29)
    actor.is_recurrent = actor.obs_normalization = False
    data = dict(observations=[[1.]*480]*2,actions=[[0.]*29]*2,
                all_sources_passed=True,moving_samples=1,stopped_samples=1)
    if fault=='shape':
        data['actions'] = [[0.]*28]*2
    elif fault=='nonfinite':
        data['actions'][0][0] = float('nan')
    elif fault=='no_motion':
        data['moving_samples'] = 0
    else:
        data['all_sources_passed'] = False
    path = tmp_path/'corpus.json'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        ActorReferenceLoss(actor,path,10.)
