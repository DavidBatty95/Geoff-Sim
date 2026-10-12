import threading
import time
from dataclasses import FrozenInstanceError
from types import SimpleNamespace

import numpy as np
import pytest

from unitree_g1_mujoco_bridge.g1_policy_cmd_vel_controller import (
    PolicyInferenceCandidate,
    PolicyInferenceWorker,
    G1PolicyCmdVelController,
)


def _candidate(value: float, obs_dim: int = 2, action_dim: int = 2):
    return PolicyInferenceCandidate(
        obs_dim=obs_dim,
        action_dim=action_dim,
        observation=(value,) * obs_dim,
    )


def _wait_for_result(worker: PolicyInferenceWorker, timeout: float = 1.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        result = worker.take_result()
        if result is not None:
            return result
        time.sleep(0.005)
    pytest.fail("Timed out waiting for policy inference result")


def _wait_until(predicate, timeout: float = 1.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.005)
    pytest.fail("Timed out waiting for worker state")


def test_worker_is_daemon_and_requests_results_are_immutable():
    worker = PolicyInferenceWorker(lambda obs: np.zeros((1, 2)))
    try:
        mutable_observation = [1.0, 1.0]
        candidate = PolicyInferenceCandidate(2, 2, mutable_observation)
        request = worker.submit([candidate], time.monotonic() + 1.0)
        mutable_observation[0] = 9.0
        result = _wait_for_result(worker)

        assert worker.thread.daemon
        assert isinstance(request.candidates, tuple)
        assert request.candidates[0].observation == (1.0, 1.0)
        assert isinstance(result.actions, tuple)
        with pytest.raises(FrozenInstanceError):
            request.sequence = 4
        with pytest.raises(FrozenInstanceError):
            result.actions = (1.0,)
    finally:
        assert worker.shutdown(1.0)


def test_startup_upright_gate_does_not_replace_an_established_balance_policy():
    from rclpy.time import Time
    params = {'startup_require_upright':True,'startup_current_pose_hold_time':0.,
              'startup_blend_time':0.,'startup_hold_time':0.,'startup_max_recovery_time':0.}
    controller = SimpleNamespace(first_lowstate_time=Time(seconds=1.),first_policy_time=Time(seconds=2.),
        upright_since=None,get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10.)),
        get_parameter=lambda name:SimpleNamespace(value=params[name]),
        get_logger=lambda:SimpleNamespace(warn=lambda *args,**kwargs:None))
    assert not G1PolicyCmdVelController._startup_active(controller)
    # A reset or explicit safety hold rearms the original upright startup gate.
    controller.first_policy_time = None
    assert G1PolicyCmdVelController._startup_active(controller)


def test_ready_poll_consumes_without_resubmitting_and_invalidation_clears_readiness():
    worker = PolicyInferenceWorker(lambda obs: obs)
    consumed = []
    def control_tick(*,consume_only):
        assert consume_only
        consumed.append(worker.take_result())
    controller = SimpleNamespace(policy_worker=worker,_control_tick=control_tick)
    try:
        worker.submit([_candidate(.6)],time.monotonic()+1.)
        _wait_until(worker.has_result)
        G1PolicyCmdVelController._result_tick(controller)
        assert consumed[0].actions == pytest.approx((.6,.6))
        assert worker.diagnostics().submitted == 1
        assert not worker.has_result()
        worker.submit([_candidate(.4)],time.monotonic()+1.)
        _wait_until(worker.has_result)
        worker.invalidate()
        G1PolicyCmdVelController._result_tick(controller)
        assert len(consumed) == 1
    finally:
        worker.shutdown(1.)


def test_stop_transition_invalidates_a_running_movement_inference():
    entered,release = threading.Event(),threading.Event()
    def policy(observation):
        entered.set()
        assert release.wait(1.)
        return observation
    worker = PolicyInferenceWorker(policy)
    controller = SimpleNamespace(policy_worker=worker,inference_context={'old':True},stop_request_invalidations=0)
    try:
        first = worker.submit([_candidate(.6)],time.monotonic()+1.)
        assert entered.wait(1.)
        G1PolicyCmdVelController._invalidate_stopped_inference(controller,[.6,0.,0.],[0.,0.,0.],True)
        release.set()
        _wait_until(lambda:worker.diagnostics().discarded == 1)
        assert worker.take_result() is None
        assert controller.inference_context is None
        assert controller.stop_request_invalidations == 1
        next_request = worker.submit([_candidate(0.)],time.monotonic()+1.)
        assert next_request.generation > first.generation
        assert _wait_for_result(worker).actions == (0.,0.)
    finally:
        release.set()
        worker.shutdown(1.)


def test_repeated_stop_and_normal_slew_do_not_starve_balance_inference():
    calls = []
    controller = SimpleNamespace(policy_worker=SimpleNamespace(invalidate=lambda:calls.append(1)),
                                 inference_context={'current':True},stop_request_invalidations=0)
    for previous,command,stop in (([0.,0.,0.],[0.,0.,0.],True),([.6,0.,0.],[.3,0.,0.],False)):
        G1PolicyCmdVelController._invalidate_stopped_inference(controller,previous,command,stop)
    assert not calls
    assert controller.inference_context == {'current':True}


def test_candidate_fallback_and_output_normalization_run_in_worker():
    policy_thread = []

    def policy(obs):
        policy_thread.append(threading.current_thread().name)
        if obs.shape == (1, 2):
            raise ValueError("wrong candidate")
        return np.array([[1.0, 2.0, 3.0, 99.0]], dtype=np.float64)

    worker = PolicyInferenceWorker(policy)
    try:
        worker.submit(
            (_candidate(1.0), _candidate(2.0, obs_dim=3, action_dim=3)),
            time.monotonic() + 1.0,
        )
        result = _wait_for_result(worker)

        assert result.error is None
        assert result.obs_dim == 3
        assert result.action_dim == 3
        assert result.actions == (1.0, 2.0, 3.0)
        assert policy_thread == ["g1-policy-inference", "g1-policy-inference"]
    finally:
        worker.shutdown(1.0)


def test_single_flight_rejects_requests_until_result_is_consumed():
    entered = threading.Event()
    release = threading.Event()
    calls = []

    def policy(obs):
        calls.append(float(obs[0, 0]))
        if len(calls) == 1:
            entered.set()
            assert release.wait(1.0)
        return obs

    worker = PolicyInferenceWorker(policy)
    try:
        deadline = time.monotonic() + 2.0
        worker.submit((_candidate(1.0),), deadline)
        assert entered.wait(1.0)
        assert worker.submit((_candidate(2.0),), deadline) is None
        assert worker.submit((_candidate(3.0),), deadline) is None
        release.set()

        result = _wait_for_result(worker)
        assert calls == [1.0]
        assert result.sequence == 1

        latest = worker.submit((_candidate(3.0),), deadline)
        assert latest is not None
        next_result = _wait_for_result(worker)
        assert calls == [1.0, 3.0]
        assert next_result.sequence == latest.sequence
    finally:
        release.set()
        worker.shutdown(1.0)


@pytest.mark.parametrize(
    "policy, expected_error",
    [
        (lambda obs: np.array([[np.nan, 0.0]]), "non-finite"),
        (lambda obs: np.array([[1.0]]), "expected 2"),
    ],
)
def test_invalid_outputs_fail_and_invalidate_generation(policy, expected_error):
    worker = PolicyInferenceWorker(policy)
    try:
        first = worker.submit((_candidate(1.0),), time.monotonic() + 1.0)
        result = _wait_for_result(worker)

        assert expected_error in result.error
        assert worker.diagnostics().failed == 1
        assert worker.diagnostics().invalidated == 1
        second = worker.submit((_candidate(2.0),), time.monotonic() + 1.0)
        assert second.generation == first.generation + 1
    finally:
        worker.shutdown(1.0)


def test_deadline_uses_monotonic_time_and_returns_no_action():
    release = threading.Event()

    def policy(obs):
        release.wait(1.0)
        return obs

    worker = PolicyInferenceWorker(policy)
    try:
        request = worker.submit(
            (_candidate(1.0),), deadline=time.monotonic() + 0.02
        )
        _wait_until(lambda: worker.deadline_missed(), timeout=0.5)
        release.set()
        result = _wait_for_result(worker)

        assert result.sequence == request.sequence
        assert result.deadline_missed
        assert result.actions == ()
        diagnostics = worker.diagnostics()
        assert diagnostics.expired == 1
        assert diagnostics.invalidated == 1
    finally:
        release.set()
        worker.shutdown(1.0)


def test_invalidation_discards_active_result_and_shutdown_join_is_bounded():
    entered = threading.Event()
    release = threading.Event()

    def policy(obs):
        entered.set()
        release.wait(2.0)
        return obs

    worker = PolicyInferenceWorker(policy)
    worker.submit((_candidate(1.0),), time.monotonic() + 2.0)
    assert entered.wait(1.0)
    worker.invalidate()

    started = time.monotonic()
    assert not worker.shutdown(0.02)
    assert time.monotonic() - started < 0.2
    assert worker.thread.daemon

    release.set()
    assert worker.shutdown(1.0)
    assert worker.take_result() is None
    assert worker.diagnostics().discarded == 1
