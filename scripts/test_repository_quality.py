"""Exercise fail-closed CI routing and its final required-check gate."""
import subprocess

import pytest

from check_repository import workflow_errors
from ci_scope import check_results, classify, event_scope


def test_known_documentation_can_skip_runtime_build():
    assert classify(['README.md', 'docs/media/banner.svg', 'ros2_ws/src/g1_msgs/README.md',
                     'scripts/readme_navigation.py', 'CITATION.cff']) == {'runtime': False, 'review': False}


@pytest.mark.parametrize('path', ['ros2_ws/src/bridge.py', 'docs/results/cohort.json',
                                 'patches/unitree_mujoco.patch', '.gitmodules', 'new-tool.toml'])
def test_code_data_pins_and_unknown_changes_require_runtime(path):
    assert classify([path])['runtime']


def test_review_sources_request_a_rebuild_without_ros():
    assert classify(['docs/literature_review/fundamentals.tex']) == {'runtime': False, 'review': True}


def test_workflow_and_classifier_changes_exercise_all_jobs():
    for path in ('.github/workflows/verification.yml', 'scripts/ci_scope.py'):
        assert classify([path]) == {'runtime': True, 'review': True}


def test_rename_keeps_old_runtime_path_in_classification():
    def git(args):
        assert '--no-renames' in args
        return b'scripts/runtime.py\0docs/example.md\0'
    assert event_scope('push', {'before': 'a'*40}, 'b'*40, git)['runtime']


def test_missing_invalid_or_unavailable_base_falls_back_to_full_checks():
    def unavailable(args):
        raise subprocess.CalledProcessError(128, args)
    for event, head in (({'before': '0'*40}, 'b'*40), ({'before': '--bad-ref'}, 'b'*40),
                        ({'before': 'a'*41}, 'b'*40), ({'before': 42}, 'b'*40), ({}, '')):
        assert event_scope('push', event, head) == {'runtime': True, 'review': True}
    assert event_scope('push', {'before': 'a'*40}, 'b'*40, unavailable) == {'runtime': True, 'review': True}
    assert event_scope('workflow_dispatch', {}, '') == {'runtime': True, 'review': True}
    assert event_scope('merge_group', {}, '') == {'runtime': True, 'review': True}


def results(**updates):
    values = dict(CHANGES_RESULT='success', QUALITY_RESULT='success', RUNTIME_REQUIRED='false',
                  REVIEW_REQUIRED='false', RUNTIME_RESULT='skipped', REVIEW_RESULT='skipped')
    values.update(updates)
    return values


def test_gate_accepts_only_expected_skips():
    assert not check_results(results())
    assert check_results(results(RUNTIME_REQUIRED='true'))
    assert check_results(results(REVIEW_REQUIRED='true', REVIEW_RESULT='cancelled'))
    assert check_results(results(RUNTIME_RESULT='failure'))
    assert check_results(results(CHANGES_RESULT='failure'))
    assert check_results(results(QUALITY_RESULT='skipped'))
    assert check_results(results(REVIEW_REQUIRED=''))


def test_workflow_policy_requires_pins_and_read_only_defaults():
    good = {'permissions': {'contents': 'read'}, 'on': {'pull_request': None},
            'jobs': {'test': {'steps': [{'uses': 'actions/checkout@'+'a'*40}]}}}
    assert workflow_errors(good, 'test.yml') == []
    good['jobs']['test']['steps'][0]['uses'] = 'actions/checkout@main'
    assert workflow_errors(good, 'test.yml')
    good['permissions'] = 'write-all'
    good['on'] = {'pull_request_target': None}
    assert len(workflow_errors(good, 'test.yml')) == 3
