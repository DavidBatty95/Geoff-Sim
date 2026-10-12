#!/usr/bin/env python3
"""Check maintained Markdown links, heading anchors, fences and SVG syntax."""
import argparse
import configparser
from html import unescape
import re
import unicodedata
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]


def maintained_documents(root=ROOT):
    files = list(root.glob('*.md')) + list((root / 'docs').glob('*.md'))
    files += list((root / 'docs/literature_review').glob('*.md')) + list((root / 'profile').glob('*.md'))
    files += list((root / 'docs/resources').glob('*.md'))
    # Check the project-owned navigation surfaces, without treating historical
    # archive contents or upstream submodule documentation as maintained guides.
    for directory in ('scripts', 'patches', 'docker', 'tools', 'integrations',
                      'integrations/grove_g1', 'archive', 'docs/results', 'ros2_ws/src'):
        files.extend((root / directory).glob('README.md'))
    external = {'unitree_mujoco', 'unitree_ros2', 'unitree_sdk2'}
    files.extend(path for path in (root / 'ros2_ws/src').glob('*/README.md')
                 if path.parent.name not in external)
    return sorted(set(files))


def prose_lines(text):
    """Exclude fenced examples from live link/heading checks."""
    lines, fence = [], None
    for line in text.splitlines():
        match = re.match(r'^ {0,3}(`{3,}|~{3,})(.*)$', line)
        if match:
            marker, rest = match.groups()
            if fence is None:
                fence = marker
            elif marker[0] == fence[0] and len(marker) >= len(fence) and not rest.strip():
                fence = None
            continue
        if fence is None:
            lines.append(line)
    return lines, fence is not None


def heading_anchors(text):
    lines, _ = prose_lines(text)
    headings, used, result = {}, set(), set()
    for index, line in enumerate(lines):
        match = re.match(r'^ {0,3}#{1,6}\s+(.+?)\s*#*$', line)
        heading = match[1] if match else None
        if heading is None and index and re.fullmatch(r' {0,3}(?:=+|-+)\s*', line) and lines[index-1].strip():
            heading = lines[index-1].strip()
        if heading is None:
            continue
        heading = re.sub(r'!?\[([^]]*)\]\([^)]*\)', r'\1', heading)
        heading = unescape(re.sub(r'<[^>]+>', '', heading)).lower()
        slug = ''.join(c for c in heading if c in ' -_' or unicodedata.category(c)[0] in 'LN').replace(' ', '-')
        number = headings.get(slug, 0)
        candidate = slug if number == 0 else f'{slug}-{number}'
        while candidate in used:
            number += 1
            candidate = f'{slug}-{number}'
        headings[slug] = number + 1
        used.add(candidate)
        result.add(candidate)
    result.update(unescape(value) for _, value in re.findall(r'\b(?:id|name)=([\"\x27])([^\"\x27]+)\1', '\n'.join(lines)))
    return result


def submodule_roots(root):
    config = configparser.ConfigParser()
    config.read(root/'.gitmodules')
    return [(root/config[section]['path']).resolve() for section in config.sections()
            if section.startswith('submodule ') and 'path' in config[section]]


def check_documents(root, files, allow_uninitialized=False):
    root = root.resolve()
    failures = []
    deferred = set()
    modules = submodule_roots(root)
    anchor_cache = {}
    for path in files:
        text = path.read_text()
        lines, unclosed = prose_lines(text)
        if unclosed:
            failures.append(f'{path.relative_to(root)}: unclosed fenced code block')
        prose = '\n'.join(lines)
        links = re.findall(r'\]\(([^)]+)\)', prose) + re.findall(r'(?:href|src)="([^"]+)"', prose)
        links += [a or b for a, b in re.findall(r'^ {0,3}\[[^]]+\]:\s*(?:<([^>]+)>|(\S+))', prose, re.M)]
        for link in links:
            url = urlsplit(link.split(' "', 1)[0])
            if url.scheme or url.netloc:
                continue
            target = (path.parent / unquote(url.path)).resolve() if url.path else path.resolve()
            if not target.is_relative_to(root):
                failures.append(f'{path.relative_to(root)}: link escapes repository: {link}')
                continue
            if not target.exists():
                module = next((item for item in modules if target == item or target.is_relative_to(item)), None)
                if allow_uninitialized and module is not None and not (module/'.git').exists():
                    deferred.add(str(target.relative_to(root)))
                    continue
                failures.append(f'{path.relative_to(root)}: missing {link}')
                continue
            markdown = target/'README.md' if target.is_dir() else target
            if url.fragment and markdown.is_file() and markdown.suffix.lower() == '.md':
                if markdown not in anchor_cache:
                    anchor_cache[markdown] = heading_anchors(markdown.read_text())
                if unquote(url.fragment) not in anchor_cache[markdown]:
                    failures.append(f'{path.relative_to(root)}: missing heading/anchor {link}')
        if re.search(r'bitbucket\.org|vec-uol', text, re.I):
            failures.append(f'{path.relative_to(root)}: work-account reference')
    for directory in (root / 'docs/media', root / 'profile/assets'):
        for path in directory.rglob('*.svg'):
            try:
                ET.parse(path)
            except ET.ParseError as error:
                failures.append(f'{path.relative_to(root)}: invalid SVG: {error}')
    return failures, sorted(deferred)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--allow-uninitialized-submodules', action='store_true',
                        help='Report deferred upstream links in a documentation-only checkout')
    args = parser.parse_args()
    files = maintained_documents()
    failures, deferred = check_documents(ROOT, files, args.allow_uninitialized_submodules)
    if failures:
        raise SystemExit('\n'.join(failures))
    print(f'Local links, heading anchors and code fences checked in {len(files)} maintained/root documents')
    if deferred:
        print(f'Deferred {len(deferred)} links into uninitialized submodules; native CI checks these after checkout.')
        for target in deferred:
            print('  '+target)


if __name__ == '__main__':
    main()
