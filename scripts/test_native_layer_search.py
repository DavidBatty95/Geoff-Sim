import json
import random

import pytest

from iterate_native_layers import proposals,screen_reason,initial_weights,proposal_batch,digest


def test_screen_cannot_accept_ties_falls_or_setup_errors():
    passed = dict(passed=True,metadata={'fallen':False})
    failed = dict(passed=False,metadata={'fallen':False})
    assert screen_reason([passed,failed],2) is None
    assert screen_reason([failed,failed],2)=='cannot_improve_failure_count'
    assert screen_reason([dict(passed=False,metadata={'fallen':True})],10)=='fall_or_missing_fall_status'
    assert screen_reason([dict(passed=False,error='probe_timeout')],10)=='process_or_protocol_error'


def test_layer_proposals_are_bounded_and_change_one_layer():
    baseline = [.95]*4
    candidates = list(proposals(baseline,.1))
    assert len(candidates)==8
    assert baseline==[.95]*4
    for candidate in candidates:
        assert all(0<=v<=1 for v in candidate)
        assert sum(a!=b for a,b in zip(candidate,baseline))==1


def test_resume_uses_actual_layer_weights_and_rejects_wrong_endpoints(tmp_path):
    policy,left,right = [tmp_path/name for name in ('policy.onnx','left.pt','right.pt')]
    for path in (policy,left,right):
        path.write_bytes(path.name.encode())
    weights = [.927732,.92538,.92884,.959266]
    (tmp_path/'manifest.json').write_text(json.dumps(dict(layer_weights=weights,right_weight=.95,
        left={'sha256':digest(left)},right={'sha256':digest(right)})))
    (tmp_path/'policy.manifest.json').write_text(json.dumps(dict(policy_sha256=digest(policy))))
    assert initial_weights(policy,left,right)==weights
    right.write_bytes(b'changed checkpoint')
    with pytest.raises(ValueError,match='provenance'):
        initial_weights(policy,left,right)


def test_plateau_search_cycles_small_scales_and_is_reproducible():
    rng = random.Random(12)
    sigmas = []
    for sweep in range(6):
        batch,sigma = proposal_batch([.93]*4,.00015625,sweep,rng)
        sigmas.append(sigma)
        assert len(batch)==8 and all(0<=v<=1 for weights in batch for v in weights)
    assert sigmas==[.0005,.001,.002,.005,.01,.0005]
    assert proposal_batch([.93]*4,.00015625,0,random.Random(12))==proposal_batch([.93]*4,.00015625,0,random.Random(12))
    batch,sigma = proposal_batch([.93]*4,.01,0,random.Random(12))
    assert sigma is None and len(batch)==8
