"""A stale review source/PDF must not be blessed by an old validation report."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'docs/literature_review'))
from pdf_provenance import DATA_INPUTS, check_build_receipt, write_build_receipt
from verify_pdf import compare_reports


def review_fixture(root):
    for name in DATA_INPUTS:
        (root/name).write_text('input\n')
    (root/'literature_review.tex').write_text('review source\n')
    (root/'references.bib').write_text('bibliography\n')
    (root/'literature_review.pdf').write_bytes(b'%PDF-fixture')
    write_build_receipt(root, 'test engine')


def test_receipt_matches_exact_inputs_and_pdf(tmp_path):
    review_fixture(tmp_path)
    assert check_build_receipt(tmp_path) == []
    (tmp_path/'literature_review.tex').write_text('changed review\n')
    assert any('inputs changed' in error for error in check_build_receipt(tmp_path))


def test_new_chapter_and_changed_pdf_invalidate_receipt(tmp_path):
    review_fixture(tmp_path)
    (tmp_path/'new_chapter.tex').write_text('new text\n')
    assert any('new_chapter.tex' in error for error in check_build_receipt(tmp_path))
    (tmp_path/'literature_review.pdf').write_bytes(b'%PDF-changed')
    assert any('PDF differs' in error for error in check_build_receipt(tmp_path))


def test_missing_receipt_requires_a_build(tmp_path):
    assert 'rebuild' in check_build_receipt(tmp_path)[0]


def test_semantic_comparison_ignores_pdf_timestamp_changes_but_not_content():
    original = {'pdf_sha256': 'first', 'pages': 74, 'text_sha256': 'same', 'outline': []}
    rebuilt = dict(original, pdf_sha256='second')
    assert compare_reports(rebuilt, original) == ['pdf_sha256']
    assert compare_reports(rebuilt, original, exact_pdf=False) == []
    rebuilt['text_sha256'] = 'changed'
    assert 'text_sha256' in compare_reports(rebuilt, original, exact_pdf=False)
