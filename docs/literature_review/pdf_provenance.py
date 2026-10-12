"""Small, dependency-free receipt for the exact review inputs used by a build."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

DATA_INPUTS = ('sources.json', 'verified_sources.json', 'paper_appraisals.json',
               'fulltext_retrieval.json', 'generate_review_appendix.py',
               'verify_sources.py', 'pdf_provenance.py', 'build.sh')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_fingerprints(root):
    root = Path(root)
    paths = set(root.glob('*.tex')) | set(root.glob('*.bib'))
    paths.update(root/name for name in DATA_INPUTS)
    return {path.name: digest(path) for path in sorted(paths)}


def write_build_receipt(root, engine):
    root = Path(root)
    receipt = {
        'schema_version': 1,
        'built_at': datetime.now(timezone.utc).isoformat(),
        'engine': engine,
        'pdf_sha256': digest(root/'literature_review.pdf'),
        'source_sha256': source_fingerprints(root),
        'scope': 'Review TeX/BibTeX and declared data/build inputs; not a bit-for-bit environment lock',
    }
    (root/'build_provenance.json').write_text(json.dumps(receipt, indent=2)+'\n')


def check_build_receipt(root):
    root = Path(root)
    path = root/'build_provenance.json'
    if not path.exists():
        return ['Missing build_provenance.json; rebuild the review first']
    receipt = json.loads(path.read_text())
    errors = []
    if receipt.get('schema_version') != 1:
        errors.append('Unsupported PDF build receipt schema')
    if receipt.get('pdf_sha256') != digest(root/'literature_review.pdf'):
        errors.append('Published PDF differs from its successful-build receipt')
    current = source_fingerprints(root)
    recorded = receipt.get('source_sha256', {})
    changed = sorted(key for key in set(current) | set(recorded) if current.get(key) != recorded.get(key))
    if changed:
        errors.append('Review inputs changed since PDF build: '+', '.join(changed))
    return errors
