"""Verify complete-run GIF coverage and provenance for every README GIF."""
import argparse
import hashlib
import json
from pathlib import Path
import re

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contact-sheet',type=Path)
    args = parser.parse_args()
    paths = sorted(set(re.findall(r'docs/media/[^\s\)"<>]+\.gif',(ROOT/'README.md').read_text())))
    if not paths:
        raise ValueError('README contains no GIFs')
    thumbnails = []
    for name in paths:
        path = ROOT/name
        metadata = json.loads(path.with_suffix('.json').read_text())
        if hashlib.sha256(path.read_bytes()).hexdigest()!=metadata['gif_sha256']:
            raise ValueError('GIF hash mismatch')
        if (abs(metadata['simulation_start_s'])>1e-8 or abs(metadata['simulation_end_s']-15)>1e-8 or
                not metadata['stopping']['passed'] or not metadata['motion_progress']['passed'] or metadata['fallen']):
            raise ValueError('incomplete or unqualified recording')
        with Image.open(path) as gif:
            duration = 0
            if gif.n_frames!=151 or gif.n_frames!=metadata['frame_count']:
                raise ValueError('GIF frame count mismatch')
            for index in range(gif.n_frames):
                gif.seek(index)
                duration += gif.info['duration']
                if index in (0,50,70,150):
                    image = gif.convert('RGB')
                    image.thumbnail((320,200))
                    thumbnails.append(image)
            if duration!=16000 or duration!=metadata['playback_duration_ms']:
                raise ValueError('GIF playback duration mismatch')
        print(f'{name}: 151 frames, full 0–15 s, movement/stopping PASS, hash verified')
    if args.contact_sheet:
        if args.contact_sheet.exists():
            raise ValueError('contact sheet exists')
        sheet = Image.new('RGB',(1280,200*len(paths)))
        for i,image in enumerate(thumbnails):
            sheet.paste(image,((i%4)*320,(i//4)*200))
        sheet.save(args.contact_sheet)


if __name__=='__main__':
    main()
