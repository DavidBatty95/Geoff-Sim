"""Document checks must catch broken links without interpreting code examples."""
from check_docs import check_documents, heading_anchors


def test_github_heading_duplicates_unicode_and_explicit_ids():
    text = '# A\n# A\n# A-1\n## Café & *robots*\n<a id="custom-anchor"></a>\n'
    assert heading_anchors(text) == {'a', 'a-1', 'a-1-1', 'café--robots', 'custom-anchor'}


def test_fenced_examples_do_not_create_live_links_or_headings(tmp_path):
    page = tmp_path/'README.md'
    page.write_text('# Guide\n\n```md\n# Fake\n[example](missing.md)\n```\n[real](#guide)\n')
    assert check_documents(tmp_path, [page]) == ([], [])
    assert heading_anchors(page.read_text()) == {'guide'}


def test_bad_heading_and_reference_style_target_are_reported(tmp_path):
    page = tmp_path/'README.md'
    page.write_text('# Guide\n\n[wrong](#absent)\n[ref]: missing.md\n')
    failures, _ = check_documents(tmp_path, [page])
    assert any('missing heading/anchor' in failure for failure in failures)
    assert any('missing missing.md' in failure for failure in failures)


def test_links_cannot_escape_repository_and_unclosed_fences_fail(tmp_path):
    page = tmp_path/'README.md'
    page.write_text('[outside](../outside.md)\n\n~~~python\npass\n')
    failures, _ = check_documents(tmp_path, [page])
    assert any('escapes repository' in failure for failure in failures)
    assert any('unclosed fenced' in failure for failure in failures)


def test_uninitialized_submodule_deferral_is_explicit_and_bounded(tmp_path):
    (tmp_path/'.gitmodules').write_text('[submodule "upstream"]\n path = vendor/upstream\n url = https://example.com/upstream.git\n')
    page = tmp_path/'README.md'
    page.write_text('[upstream](vendor/upstream/README.md)\n')
    assert check_documents(tmp_path, [page])[0]
    assert check_documents(tmp_path, [page], True) == ([], ['vendor/upstream/README.md'])
    module = tmp_path/'vendor/upstream'
    module.mkdir(parents=True)
    (module/'.git').write_text('gitdir: /unused\n')
    assert check_documents(tmp_path, [page], True)[0]


def test_directory_readme_and_encoded_heading_links(tmp_path):
    directory = tmp_path/'docs'
    directory.mkdir()
    (directory/'README.md').write_text('# Café\n')
    page = tmp_path/'README.md'
    page.write_text('[directory](docs/#caf%C3%A9)\n')
    assert check_documents(tmp_path, [page]) == ([], [])
