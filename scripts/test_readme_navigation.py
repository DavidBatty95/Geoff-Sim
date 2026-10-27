"""Protect navigation destinations and ownership of generated Markdown blocks."""
from html.parser import HTMLParser
from urllib.parse import urlsplit

import pytest

from readme_navigation import END, LINKS, START, menu, update_navigation


class MenuLinks(HTMLParser):
    def __init__(self, markup):
        super().__init__()
        self.links = []
        self.images = []
        self.feed(markup)

    def handle_starttag(self, tag, attrs):
        if tag == 'a':
            self.links.append(dict(attrs))
        elif tag == 'img':
            self.images.append(dict(attrs))


@pytest.mark.parametrize('page', ['README.md', 'docs/README.md',
                                 'ros2_ws/src/g1_msgs/README.md'])
def test_all_links_resolve_to_same_destinations_at_every_depth(tmp_path, page):
    source = tmp_path / page
    parsed = MenuLinks(menu(source, tmp_path))
    assert len(parsed.links) == len(parsed.images) == len(LINKS)
    for anchor, image, (_, label, destination, _) in zip(parsed.links, parsed.images, LINKS):
        url = urlsplit(anchor['href'])
        expected = urlsplit(destination)
        assert (source.parent / url.path).resolve() == (tmp_path / expected.path).resolve()
        assert url.fragment == expected.fragment
        assert (source.parent / image['src']).resolve().parent == tmp_path / 'docs/media/navigation'
        assert image['alt'].startswith(label)
    current = [image for image in parsed.images if '(current page)' in image['alt']]
    assert len(current) == (1 if page in ('README.md', 'docs/README.md') else 0)


def test_regeneration_preserves_authored_content_and_is_idempotent(tmp_path):
    before = '# Title\n\nAuthored introduction.\n\n'
    after = '\n\n## Evidence\nKeep this exact source and formatting.\n'
    text = before + START + '\nstale generated menu\n' + END + after
    updated = update_navigation(text, tmp_path / 'docs/guide.md', tmp_path)
    assert updated.startswith(before + START)
    assert updated.endswith(END + after)
    assert 'stale generated menu' not in updated
    assert update_navigation(updated, tmp_path / 'docs/guide.md', tmp_path) == updated


@pytest.mark.parametrize('text', [START, END, END + START, START + START + END])
def test_ambiguous_markers_never_overwrite_page_content(tmp_path, text):
    with pytest.raises(ValueError, match='Invalid navigation markers'):
        update_navigation(text, tmp_path / 'README.md', tmp_path)


def test_pages_without_generated_navigation_are_untouched(tmp_path):
    text = '# Upstream document\n\nOriginal content.\n'
    assert update_navigation(text, tmp_path / 'upstream/README.md', tmp_path) == text
