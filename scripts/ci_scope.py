#!/usr/bin/env python3
"""Fail-closed CI routing: known presentation changes can skip native builds."""
import argparse
import json
import os
from pathlib import Path, PurePosixPath
import re
import subprocess

QUALITY_TOOLS = {
    'scripts/check_docs.py', 'scripts/check_repository.py',
    'scripts/generate_readme_graphics.py', 'scripts/readme_navigation.py',
    'scripts/resource_library.py', 'scripts/test_readme_navigation.py',
    'scripts/test_check_docs.py', 'scripts/test_repository_quality.py',
    'scripts/test_pdf_provenance.py', 'scripts/install_quality_tool.py',
    'tools/requirements-quality.txt',
}
METADATA = {'CITATION.cff', '.editorconfig', '.gitattributes', '.pre-commit-config.yaml',
            '.github/CODEOWNERS', '.github/dependabot.yml'}


def classify(paths, force=False):
    runtime = review = force
    for raw in paths:
        path = PurePosixPath(raw).as_posix()
        if path.startswith('.github/workflows/') or path == 'scripts/ci_scope.py':
            runtime = review = True
            continue
        if path.startswith('docs/literature_review/') and not path.endswith('.md'):
            review = True
        if path == 'tools/requirements-quality.txt':
            review = True
        documentation = (
            path.endswith('.md') or path in METADATA or path in QUALITY_TOOLS
            or path.startswith(('docs/media/', 'docs/literature_review/', 'profile/assets/',
                                '.github/ISSUE_TEMPLATE/'))
        )
        # Result JSON, simulator pins, configuration and unknown paths need native CI.
        if not documentation:
            runtime = True
    return {'runtime': runtime, 'review': review}


def event_scope(event_name, event, head, run_git=subprocess.check_output):
    if event_name == 'pull_request':
        base = event.get('pull_request', {}).get('base', {}).get('sha', '')
    elif event_name == 'push':
        base = event.get('before', '')
    else:
        return classify([], force=True)
    if not all(isinstance(value, str) and re.fullmatch(r'(?:[0-9a-fA-F]{40}|[0-9a-fA-F]{64})', value)
               for value in (base, head)):
        return classify([], force=True)
    if set(base) == {'0'}:
        return classify([], force=True)
    try:
        changed = run_git(['git', 'diff', '--no-renames', '--name-only', '-z', base, head, '--'])
    except subprocess.CalledProcessError:
        return classify([], force=True)
    return classify(changed.decode().split('\0')[:-1])


def check_results(environment):
    errors = []
    for name in ('CHANGES', 'QUALITY'):
        if environment.get(name+'_RESULT') != 'success':
            errors.append(name+' did not succeed')
    for name in ('RUNTIME', 'REVIEW'):
        required = environment.get(name+'_REQUIRED')
        outcome = environment.get(name+'_RESULT')
        if required not in ('true', 'false'):
            errors.append(name+' classification missing or invalid')
        elif outcome != 'success' and not (required == 'false' and outcome == 'skipped'):
            errors.append(name+' did not satisfy its required verification')
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-results', action='store_true')
    args = parser.parse_args()
    if args.check_results:
        errors = check_results(os.environ)
        if errors:
            raise SystemExit('\n'.join(errors))
        print('All required verification jobs passed.')
        return
    event = json.loads(Path(os.environ['GITHUB_EVENT_PATH']).read_text())
    result = event_scope(os.environ['GITHUB_EVENT_NAME'], event, os.environ.get('GITHUB_SHA', ''))
    with Path(os.environ['GITHUB_OUTPUT']).open('a') as stream:
        for key, value in result.items():
            stream.write(f'{key}={str(value).lower()}\n')
    print(json.dumps(result, sort_keys=True))


if __name__ == '__main__':
    main()
