from iterate_native_layers import proposals,screen_reason


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
