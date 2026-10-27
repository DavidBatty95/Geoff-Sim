"""Shared, GitHub-compatible navigation assets and generated Markdown blocks."""
from html import escape
import os
from pathlib import Path

START = '<!-- geoff-nav:start -->'
END = '<!-- geoff-nav:end -->'
ASSETS = Path('docs/media/navigation')

# Slug, visible label, repository-relative destination, line icon.
LINKS = (
    ('home', 'Home', 'README.md', 'M3 10 12 3l9 7M5 9v12h5v-7h4v7h5V9'),
    ('start', 'Quick start', 'README.md#quick-start', 'm13 2-9 12h7l-1 8 10-13h-7z'),
    ('demos', 'Demos', 'docs/DEMO_GUIDE.md', 'M5 3h14v18H5zM10 8l6 4-6 4z'),
    ('docs', 'Docs', 'docs/README.md', 'M3 4h6l3 2 3-2h6v16h-6l-3 2-3-2H3zM12 6v16'),
    ('folders', 'Folders', 'docs/REPOSITORY_GUIDE.md', 'M2 5h7l3 3h10v13H2z'),
    ('progress', 'Progress', 'docs/PROJECT_STATUS.md', 'M3 3v18h18M7 16v-4m5 4V8m5 8V4'),
    ('roadmap', 'Roadmap', 'docs/PROJECT_PLAN.md', 'M4 3v18m0-16h10l-2 4 2 4H4M9 21h12'),
    ('results', 'Results', 'docs/RESULTS_REVIEW.md', 'M5 2h14v20H5zM8 7h8m-8 5h8m-8 5h4'),
    ('research', 'Research', 'docs/literature_review/README.md', 'M9 2h6m-5 0v7L4 20q-1 2 2 2h12q3 0 2-2l-6-11V2M7 16h10'),
    ('develop', 'Develop', 'docs/DEVELOPMENT.md', 'm8 5-7 7 7 7m8-14 7 7-7 7m-3-17-2 20'),
)


def button(label, icon, current=False):
    accent = '#7de39a' if current else '#43d2ec'
    border = '#7de39a' if current else '#38566f'
    state = ' — current page' if current else ''
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="132" height="44" viewBox="0 0 132 44" role="img" aria-labelledby="title desc">
  <title id="title">{escape(label + state)}</title>
  <desc id="desc">Geoff-Sim navigation: {escape(label)}{escape(state)}. The surrounding link opens this section.</desc>
  <defs>
    <linearGradient id="surface" x2="1" y2="1"><stop stop-color="#172e43"/><stop offset="1" stop-color="#0b1727"/></linearGradient>
  </defs>
  <style>
    .trace {{animation:travel 8s linear infinite;stroke-dasharray:22 314}}
    .signal {{animation:breathe 4s ease-in-out infinite}}
    @keyframes travel {{to {{stroke-dashoffset:-336}}}}
    @keyframes breathe {{0%,100% {{opacity:.55}} 50% {{opacity:1}}}}
    @media (prefers-reduced-motion:reduce) {{.trace,.signal {{animation:none}}}}
  </style>
  <rect x="1" y="3" width="130" height="38" rx="9" fill="url(#surface)" stroke="{border}"/>
  <rect class="trace" x="1" y="3" width="130" height="38" rx="9" pathLength="336" fill="none" stroke="{accent}" stroke-width="1.2" opacity=".75"/>
  <path d="{icon}" transform="translate(12 12) scale(.78)" fill="none" stroke="{accent}" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
  <text x="39" y="27" fill="#edf6ff" font-family="Arial,Helvetica,sans-serif" font-size="13" font-weight="600">{escape(label)}</text>
  <circle class="signal" cx="121" cy="11" r="2" fill="{accent}"/>
</svg>
'''


def graphics():
    return {str(ASSETS / f'{slug}{suffix}.svg'): button(label, icon, current)
            for slug, label, _, icon in LINKS
            for current, suffix in ((False, ''), (True, '-current'))}


def menu(page, root):
    """Normal HTML anchors make SVG images clickable in GitHub Markdown."""
    page = Path(page)
    root = Path(root)
    lines = [START, '<p align="center">']
    for index, (slug, label, destination, _) in enumerate(LINKS):
        target, _, fragment = destination.partition('#')
        current = not fragment and page.resolve() == (root / target).resolve()
        suffix = '-current' if current else ''
        href = Path(os.path.relpath(root / target, page.parent)).as_posix()
        if fragment:
            href += '#' + fragment
        image = Path(os.path.relpath(root / ASSETS / f'{slug}{suffix}.svg', page.parent)).as_posix()
        alt = label + (' (current page)' if current else '')
        lines.append(f'  <a href="{escape(href, quote=True)}"><img src="{escape(image, quote=True)}" width="132" height="44" alt="{escape(alt, quote=True)}"></a>')
        if index == 4:
            lines.append('  <br>')
    lines.extend(('</p>', END))
    return '\n'.join(lines)


def update_navigation(text, page, root):
    """Replace only the owned block; incomplete/duplicate markers are errors."""
    if START not in text and END not in text:
        return text
    if text.count(START) != 1 or text.count(END) != 1 or text.index(END) < text.index(START):
        raise ValueError(f'Invalid navigation markers: {page}')
    before, _, rest = text.partition(START)
    _, _, after = rest.partition(END)
    return before + menu(page, root) + after
