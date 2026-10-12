#!/usr/bin/env python3
"""Install a checksum-pinned Linux x86-64 CI tool into an explicit directory."""
import argparse
import hashlib
import io
from pathlib import Path
import platform
import tarfile
from urllib.request import urlopen

TOOLS = {
    'actionlint': (
        'https://github.com/rhysd/actionlint/releases/download/v1.7.7/actionlint_1.7.7_linux_amd64.tar.gz',
        '023070a287cd8cccd71515fedc843f1985bf96c436b7effaecce67290e7e0757',
    ),
    'tectonic': (
        'https://github.com/tectonic-typesetting/tectonic/releases/download/tectonic%400.17.0/tectonic-0.17.0-x86_64-unknown-linux-musl.tar.gz',
        '8533d07f9ccbd7a65824b9e0459041bca34af1eb33daba48f59215593753a3b7',
    ),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('tool', choices=TOOLS)
    parser.add_argument('--directory', type=Path, required=True)
    args = parser.parse_args()
    if platform.system() != 'Linux' or platform.machine() not in ('x86_64', 'AMD64'):
        raise SystemExit('This pinned installer supports Linux x86-64 only.')
    url, expected = TOOLS[args.tool]
    with urlopen(url, timeout=120) as response:
        data = response.read(32 * 1024 * 1024 + 1)
    if hashlib.sha256(data).hexdigest() != expected:
        raise SystemExit('Tool archive checksum mismatch; nothing was installed.')
    with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as archive:
        member = archive.getmember(args.tool)
        if not member.isfile():
            raise SystemExit('Expected a regular executable in the tool archive.')
        executable = archive.extractfile(member).read()
    args.directory.mkdir(parents=True, exist_ok=True)
    target = args.directory/args.tool
    target.write_bytes(executable)
    target.chmod(0o755)
    print(f'Installed verified {args.tool}: {target}')


if __name__ == '__main__':
    main()
