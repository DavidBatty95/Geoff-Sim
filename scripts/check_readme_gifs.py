"""Verify README recording provenance and explicitly labelled architecture GIFs."""
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
        if metadata.get('kind')=='architecture_diagram':
            if metadata['scope']!='illustrated architecture, not simulation footage':
                raise ValueError('architecture scope missing')
            generator=ROOT/metadata['generator']
            if hashlib.sha256(generator.read_bytes()).hexdigest()!=metadata['generator_sha256']:
                raise ValueError('architecture generator changed; regenerate diagrams')
            with Image.open(path) as gif:
                if gif.n_frames!=metadata['frame_count'] or list(gif.size)!=metadata['dimensions']:
                    raise ValueError('architecture dimensions/frame count mismatch')
                duration=0
                for index in range(gif.n_frames):
                    gif.seek(index)
                    duration+=gif.info['duration']
                    if index in (0,24,48,71):
                        image=gif.convert('RGB')
                        image.thumbnail((320,200))
                        thumbnails.append(image)
                if duration!=metadata['playback_duration_ms']:
                    raise ValueError('architecture playback duration mismatch')
            print(f'{name}: illustrated architecture, {metadata["frame_count"]} frames, hash verified')
            continue
        dex3 = metadata.get('kind')=='dex3_contact_bench'
        expected_end,expected_count,expected_duration = (10.,101,11000) if dex3 else (15.,151,16000)
        if dex3:
            grasp=metadata['physical_grasp']
            passed=(grasp['success'] and grasp['object_constraints']==0 and not grasp['object_external_force']
                    and grasp['opposed_contact'] and grasp['minimum_held_lift_m']>.1 and grasp['released'])
        else:
            passed=metadata['stopping']['passed'] and metadata['motion_progress']['passed'] and not metadata['fallen']
        if (abs(metadata['simulation_start_s'])>1e-8 or abs(metadata['simulation_end_s']-expected_end)>1e-8 or not passed):
            raise ValueError('incomplete or unqualified recording')
        with Image.open(path) as gif:
            duration = 0
            if gif.n_frames!=expected_count or gif.n_frames!=metadata['frame_count']:
                raise ValueError('GIF frame count mismatch')
            for index in range(gif.n_frames):
                gif.seek(index)
                duration += gif.info['duration']
                if index in ((0,30,55,100) if dex3 else (0,50,70,150)):
                    image = gif.convert('RGB')
                    image.thumbnail((320,200))
                    thumbnails.append(image)
            if duration!=expected_duration or duration!=metadata['playback_duration_ms']:
                raise ValueError('GIF playback duration mismatch')
        print(f'{name}: {expected_count} frames, full 0–{expected_end:g} s, qualification PASS, hash verified')
    if args.contact_sheet:
        if args.contact_sheet.exists():
            raise ValueError('contact sheet exists')
        sheet = Image.new('RGB',(1280,200*len(paths)))
        for i,image in enumerate(thumbnails):
            sheet.paste(image,((i%4)*320,(i//4)*200))
        sheet.save(args.contact_sheet)


if __name__=='__main__':
    main()
