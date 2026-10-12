#!/usr/bin/env python3
"""Check repository metadata, issue forms, workflow pinning and tracked-file hygiene."""
from pathlib import Path
import re
import subprocess

import yaml

ROOT = Path(__file__).resolve().parents[1]
REQUIRED = ('README.md', 'CONTRIBUTING.md', 'SECURITY.md', 'CODE_OF_CONDUCT.md',
            'CHANGELOG.md', 'CITATION.cff', 'LICENSES.md', '.github/CODEOWNERS',
            '.github/ISSUE_TEMPLATE/config.yml', '.github/pull_request_template.md')


def workflow_errors(data, name):
    errors = []
    permissions = data.get('permissions', {})
    if not isinstance(permissions, dict) or permissions.get('contents') != 'read':
        errors.append(f'{name}: declare read-only contents permission at workflow scope')
    events = data.get('on', data.get(True, {}))
    if isinstance(events, dict) and 'pull_request_target' in events:
        errors.append(f'{name}: privileged pull_request_target requires a separately reviewed design')
    for job in data.get('jobs', {}).values():
        for step in job.get('steps', []):
            action = step.get('uses', '')
            if action and not action.startswith('./') and not re.fullmatch(r'[^@]+@[0-9a-f]{40}', action):
                errors.append(f'{name}: action must use a full commit pin: {action}')
    return errors


def main():
    errors = [f'Missing repository file: {name}' for name in REQUIRED if not (ROOT/name).is_file()]
    paths = [ROOT/'.github/dependabot.yml', ROOT/'.pre-commit-config.yaml']
    paths += list((ROOT/'.github/ISSUE_TEMPLATE').glob('*.yml'))
    paths += list((ROOT/'.github/workflows').glob('*.yml'))
    for path in paths:
        try:
            data = yaml.safe_load(path.read_text())
            if not isinstance(data, dict):
                raise ValueError('Expected a YAML mapping')
            if path.parent.name == 'workflows':
                errors.extend(workflow_errors(data, str(path.relative_to(ROOT))))
            if path.parent.name == 'ISSUE_TEMPLATE' and path.name != 'config.yml':
                if not data.get('name') or not data.get('description') or not isinstance(data.get('body'), list):
                    raise ValueError('Issue form needs name, description and body')
        except (ValueError, yaml.YAMLError) as error:
            errors.append(f'{path.relative_to(ROOT)}: {error}')
    tracked = subprocess.check_output(['git', 'ls-files', '-z'], cwd=ROOT).decode().split('\0')
    for filename in filter(None, tracked):
        path = Path(filename)
        if (path.name == '.env' or path.name.startswith('.env.')) and path.name != '.env.example':
            errors.append('Local environment file is tracked: '+filename)
        if filename.startswith('docs/literature_review/build/'):
            errors.append('Primary-paper/build cache is tracked: '+filename)
    if errors:
        raise SystemExit('\n'.join(errors))
    print('Repository metadata, issue forms, workflow action pins and tracked-file hygiene passed.')


if __name__ == '__main__':
    main()
