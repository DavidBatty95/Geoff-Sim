#!/usr/bin/env python3
"""Inspect the delivered PDF's bibliography links, bookmarks and page structure.

Requires the optional pypdf package. This checks document integrity, not whether
the scientific interpretations are correct. Use --write-report to save evidence.
"""
import argparse
from datetime import datetime, timezone
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
from pathlib import Path
import re
from urllib.parse import unquote

from pdf_provenance import check_build_receipt

ROOT = Path(__file__).resolve().parent


def inspect_pdf(root=ROOT):
    from pypdf import PdfReader

    pdf = root/'literature_review.pdf'
    reader = PdfReader(pdf)
    records = json.loads((root/'verified_sources.json').read_text())
    named = reader.named_destinations
    uris, destinations = set(), []
    errors = []
    for number, page in enumerate(reader.pages, 1):
        for reference in page.get('/Annots', []):
            annotation = reference.get_object()
            if annotation.get('/Subtype') != '/Link':
                continue
            if '/Rect' in annotation:
                x1, y1, x2, y2 = map(float, annotation['/Rect'])
                if min(x1, y1) < -1 or x2 > float(page.mediabox.width)+1 or y2 > float(page.mediabox.height)+1:
                    errors.append(f'Link outside page {number}')
            if '/A' in annotation:
                action = annotation['/A'].get_object()
                if '/URI' in action:
                    uris.add(unquote(str(action['/URI'])).rstrip('/'))
                if '/D' in action and isinstance(action['/D'], str):
                    destinations.append(str(action['/D']))
            if '/Dest' in annotation and isinstance(annotation['/Dest'], str):
                destinations.append(str(annotation['/Dest']))
    for destination in destinations:
        if destination not in named:
            errors.append('Unresolved internal destination: '+destination)
    for item in records:
        key = item['key']
        if 'cite.'+key not in named or 'cite.'+key not in destinations:
            errors.append('Missing linked bibliography entry/citation: '+key)
        if unquote(item['url']).rstrip('/') not in uris:
            errors.append('Missing publication URL: '+key)

    outline = []
    def flatten(items):
        for item in items:
            if isinstance(item, list):
                flatten(item)
            else:
                outline.append({'title': item.title, 'page': reader.get_destination_page_number(item)+1})
    flatten(reader.outline)
    required = ('what makes navigation social', 'Deeper thematic synthesis',
                'ranked research gaps', 'A novelty roadmap', 'Detailed paper-by-paper', 'Linked bibliography')
    for title in required:
        if not any(title.lower() in item['title'].lower() for item in outline):
            errors.append('Missing bookmark: '+title)
    if not any(destination.startswith('page.') for destination in destinations):
        errors.append('No page back-reference links found')
    if errors:
        raise SystemExit('\n'.join(errors))
    text = '\n'.join(page.extract_text() or '' for page in reader.pages)
    try:
        fonttools_version = version('fonttools')
    except PackageNotFoundError:
        fonttools_version = None
    report = {
        'checked_at': datetime.now(timezone.utc).isoformat(),
        'pdf': 'literature_review.pdf', 'pdf_sha256': hashlib.sha256(pdf.read_bytes()).hexdigest(),
        'pages': len(reader.pages), 'extracted_word_count': len(text.split()),
        'text_sha256': hashlib.sha256(re.sub(r'\s+', ' ', text).strip().encode()).hexdigest(),
        'validator_versions': {'pypdf': version('pypdf'), 'fonttools': fonttools_version},
        'bibliographic_identities_with_linked_citations_and_publication_urls': len(records),
        'unique_external_urls': len(uris), 'internal_link_annotations': len(destinations),
        'bookmark_count': len(outline), 'outline': outline,
        'external_link_sha256': hashlib.sha256('\n'.join(sorted(uris)).encode()).hexdigest(),
        'internal_link_sha256': hashlib.sha256('\n'.join(sorted(destinations)).encode()).hexdigest(),
        'scope': 'PDF structure and link integrity; not independent scientific peer review',
    }
    return report


def compare_reports(actual, expected, exact_pdf=True):
    fields = ('pages', 'text_sha256', 'bibliographic_identities_with_linked_citations_and_publication_urls',
              'unique_external_urls', 'internal_link_annotations', 'bookmark_count', 'outline',
              'external_link_sha256', 'internal_link_sha256', 'validator_versions')
    if exact_pdf:
        fields += ('pdf_sha256',)
    return [field for field in fields if actual.get(field) != expected.get(field)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--write-report', action='store_true')
    mode.add_argument('--check-report', action='store_true', help='Require the published report and build receipt to match')
    mode.add_argument('--compare-report', type=Path, help='Compare rebuilt content/links to a saved report, ignoring PDF metadata timestamps')
    args = parser.parse_args()
    if args.write_report or args.check_report or args.compare_report:
        errors = check_build_receipt(ROOT)
        if errors:
            raise SystemExit('\n'.join(errors))
    report = inspect_pdf()
    if args.write_report:
        (ROOT/'pdf_validation.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    if args.check_report or args.compare_report:
        expected = json.loads((args.compare_report or ROOT/'pdf_validation.json').read_text())
        differences = compare_reports(report, expected, exact_pdf=not bool(args.compare_report))
        if differences:
            raise SystemExit('PDF/report mismatch: '+', '.join(differences))
    print(f"PDF verified: {report['pages']} pages; "
          f"{report['bibliographic_identities_with_linked_citations_and_publication_urls']} linked references; "
          f"{report['internal_link_annotations']} resolved internal links; {report['bookmark_count']} bookmarks.")
    required = ('what makes navigation social', 'Deeper thematic synthesis',
                'ranked research gaps', 'A novelty roadmap', 'Detailed paper-by-paper', 'Linked bibliography')
    for item in report['outline']:
        if any(term.lower() in item['title'].lower() for term in (*required, 'Gap 1:', 'Gap 2:', 'Gap 3:', 'Gap 4:', 'Gap 5:')):
            print(f"  p. {item['page']}: {item['title']}")


if __name__ == '__main__':
    main()
