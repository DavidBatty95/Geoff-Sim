#!/usr/bin/env bash
# Build from any working directory; bibliography checks are offline by default.
set -euo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--help" ]]; then
    printf 'Usage: bash %s [--check-only]\n' "$0"
    printf 'Builds build/literature_review.pdf using Tectonic, latexmk, or LuaLaTeX + BibTeX.\n'
    printf 'Set TECTONIC=/path/to/tectonic to select an external binary.\n'
    exit 0
fi
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != "--check-only" ) ]]; then
    printf 'Unknown argument. Use --help.\n' >&2
    exit 2
fi
python3 "$HERE/generate_review_appendix.py" --check
python3 "$HERE/verify_sources.py"
if [[ "${1:-}" == "--check-only" ]]; then
    exit 0
fi

mkdir -p "$HERE/build"
pushd "$HERE" >/dev/null
if [[ -n "${TECTONIC:-}" ]] || command -v tectonic >/dev/null 2>&1; then
    export REVIEW_ENGINE="$("${TECTONIC:-tectonic}" --version)"
    "${TECTONIC:-tectonic}" --keep-logs --keep-intermediates --outdir build literature_review.tex
elif command -v latexmk >/dev/null 2>&1 && command -v lualatex >/dev/null 2>&1; then
    export REVIEW_ENGINE="latexmk + LuaLaTeX"
    latexmk -lualatex -interaction=nonstopmode -halt-on-error -outdir=build literature_review.tex
elif command -v lualatex >/dev/null 2>&1 && command -v bibtex >/dev/null 2>&1; then
    export REVIEW_ENGINE="LuaLaTeX + BibTeX"
    lualatex -interaction=nonstopmode -halt-on-error -output-directory=build literature_review.tex
    BIBINPUTS="$HERE:${BIBINPUTS:-}" bibtex build/literature_review
    lualatex -interaction=nonstopmode -halt-on-error -output-directory=build literature_review.tex
    lualatex -interaction=nonstopmode -halt-on-error -output-directory=build literature_review.tex
else
    printf '%s\n' 'No supported TeX toolchain found.' \
      'Install Tectonic, or on Ubuntu/Debian:' \
      '  sudo apt-get install latexmk texlive-luatex texlive-latex-extra texlive-fonts-recommended' >&2
    exit 1
fi
python3 - <<'PY'
from pathlib import Path
import os
import re
import shutil
from pdf_provenance import write_build_receipt
log = Path('build/literature_review.log').read_text(errors='replace')
patterns = [r'Citation .* undefined', r'Reference .* undefined',
            r'There were undefined', r'Missing character:', r'^! ',
            r'Rerun to get cross-references right', r'Overfull \\[hv]box']
issues = [line for line in log.splitlines() if any(re.search(p, line) for p in patterns)]
if issues:
    raise SystemExit('Document checks failed:\n' + '\n'.join(issues))
pdf = Path('build/literature_review.pdf')
if not pdf.is_file() or pdf.stat().st_size == 0:
    raise SystemExit('PDF missing or empty')
shutil.copyfile(pdf, 'literature_review.pdf')
write_build_receipt(Path('.'), os.environ['REVIEW_ENGINE'])
print(f'PDF and log checks passed: {pdf.resolve()}')
PY
popd >/dev/null
