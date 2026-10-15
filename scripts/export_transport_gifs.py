#!/usr/bin/env python3
"""Export labelled, source-linked GIFs from actual transport window captures."""
import argparse
import hashlib
import json
import math
from datetime import datetime
from pathlib import Path

from PIL import Image,ImageDraw,ImageFont

from record_transport_preview import ROOT,RUNS
from performance_outcome import strict_success


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def export_clip(recording,output,title,phases,crop,max_frames=100):
    capture = json.loads((recording/'capture.json').read_text())
    if capture.get('media_valid') is False:
        raise ValueError('capture is marked unusable: '+capture.get('media_invalid_reason','review failed'))
    manifest = json.loads((ROOT/capture['run']/'manifest.json').read_text())
    date_label = datetime.fromisoformat(capture['recorded_at']).strftime('%d %b %Y').upper()
    people_label = f'{manifest["people"]} PEOPLE'
    frames = [row for row in capture['frames'] if
              (row['phase'] not in ('startup','initializing') if phases is None else row['phase'] in phases)]
    if not frames:
        raise ValueError(f'no captured frames for {title}')
    stride = max(1,math.ceil(len(frames)/max_frames))
    chosen = frames[::stride]
    if chosen[-1] is not frames[-1]:
        chosen.append(frames[-1])
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    title_font = ImageFont.truetype(font_path,22)
    small = ImageFont.truetype(font_path,14)
    rendered = []
    width = 640 if title[:2] in ('01','02','03') else 800
    origin = capture['frames'][0]['monotonic_s']
    for row in chosen:
        with Image.open(recording/row['file']) as image:
            image = image.crop(crop)
            image = image.resize((width,round(image.height*width/image.width)),getattr(Image,'Resampling',Image).LANCZOS)
            canvas = Image.new('RGB',(width,image.height+106),'#081725')
            canvas.paste(image,(0,70))
        draw = ImageDraw.Draw(canvas)
        draw.text((20,10),title,font=title_font,fill='#eef6fc')
        draw.text((20,42),f'RECORDED SIMULATION  /  {date_label}  /  {people_label}',font=small,fill='#72dcd4')
        draw.text((20,canvas.height-26),row['phase'].replace('_',' ').upper(),font=small,fill='#eef6fc')
        draw.text((width-230,canvas.height-26),f'T+{row["monotonic_s"]-origin:05.1f}s | time-lapse',font=small,fill='#b5c6d4')
        rendered.append(canvas)
    result = json.loads((ROOT/capture['run']/'result.json').read_text())
    # Keep the outcome visible in the animation, not only in external prose.
    card = Image.new('RGB',rendered[-1].size,'#081725')
    draw = ImageDraw.Draw(card)
    passed = strict_success(result)
    draw.text((30,60),'CAPTURED ATTEMPT: COMPLETED' if passed else 'CAPTURED ATTEMPT: INCOMPLETE',font=title_font,fill='#f2bf79')
    draw.text((30,110),'Excerpt from '+recording.name,font=small,fill='#eef6fc')
    error = ('Verified delivery and mission completion; see source evidence.' if passed else
             result.get('error') or 'See result and provenance manifest.')
    import textwrap
    for index,line in enumerate(textwrap.wrap(error,72 if width==640 else 85)):
        draw.text((30,155+index*24),line,font=small,fill='#b5c6d4')
    draw.text((30,240),'Magnetic attachment fixture; not a physical finger grasp.',font=small,fill='#b5c6d4')
    rendered.append(card)
    # Shared palette sampled across the sequence avoids colour flicker.
    sheet = Image.new('RGB',(160*10,100))
    for index in range(10):
        tile = rendered[round(index*(len(rendered)-1)/9)].copy()
        tile.thumbnail((160,100))
        sheet.paste(tile,(160*index,0))
    palette = sheet.quantize(colors=192)
    quantized = [frame.quantize(palette=palette,dither=getattr(Image,'Dither',Image).NONE) for frame in rendered]
    durations = [170]*(len(rendered)-1)+[3000]
    quantized[0].save(output,save_all=True,append_images=quantized[1:],duration=durations,
                      loop=0,optimize=True,disposal=1)
    rendered[0].save(output.with_suffix('.png'))
    return dict(file=output.name,sha256=digest(output),bytes=output.stat().st_size,
        run=capture['run'],capture_sha256=digest(recording/'capture.json'),crop=list(crop),output_dimensions=list(rendered[0].size),
        playback='sampled time-lapse with 3-second outcome card',frame_stride=stride,
        excluded_initialization=phases is None,
        source_frames=[dict(row,sha256=digest(recording/row['file'])) for row in chosen],
        displayed_frames=len(rendered),duration_ms=sum(durations),success=passed,
        mission_complete=result.get('mission_complete'),error=result.get('error'),
        result_sha256=digest(ROOT/capture['run']/'result.json'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pickup-run',required=True)
    parser.add_argument('--carry-run',required=True)
    parser.add_argument('--placement-run',help='Optional recorded lower/release/verify/restoration excerpt')
    parser.add_argument('--output',type=Path,default=ROOT/'docs/media/locomanipulation')
    args = parser.parse_args()
    captures = RUNS.parent/'transport_recordings'
    args.output.mkdir(parents=True,exist_ok=True)
    clips = [
        export_clip(captures/args.carry_run,args.output/'mission-overview.gif','GEOFF / RECORDED LOCOMANIPULATION',None,(303,0,960,600),110),
        export_clip(captures/args.pickup_run,args.output/'pickup-and-lift.gif','01 / REACH, ATTACH, LIFT',{'reach','grasp','lift','retreat_pickup'},(420,240,750,500),85),
        export_clip(captures/args.carry_run,args.output/'loaded-transport.gif','02 / RETREAT AND CARRY',{'retreat_pickup','remembered_destination','delivery_approach','navigate_delivery'},(400,150,820,510),90),
    ]
    if args.placement_run:
        clips.append(export_clip(captures/args.placement_run,args.output/'placement-and-verify.gif',
            '03 / PLACE, VERIFY, RESTORE',{'lower','release','retract','verify_place','delivered','restore_arm','complete'},
            (540,120,850,400),90))
    attempts = []
    for recording in sorted(captures.glob('*-readme-*')):
        capture = json.loads((recording/'capture.json').read_text())
        run = ROOT/capture['run']
        result = json.loads((run/'result.json').read_text()) if (run/'result.json').exists() else {}
        manifest = json.loads((run/'manifest.json').read_text())
        attempts.append(dict(run=capture['run'],recorded_at=capture['recorded_at'],command=capture['command'],
            capture_sha256=digest(recording/'capture.json'),frame_count=len(capture['frames']),
            capture_errors=capture['capture_errors'],returncode=capture['returncode'],
            media_valid=capture.get('media_valid'),media_invalid_reason=capture.get('media_invalid_reason'),
            window_id=capture.get('window_id'),window_title=capture.get('window_title'),
            success=result.get('success'),mission_complete=result.get('mission_complete'),
            fallen=result.get('fallen'),error=result.get('error'),deliveries=result.get('deliveries'),
            events=result.get('events'),source_manifest=manifest,
            result_sha256=digest(run/'result.json') if (run/'result.json').exists() else None,
            source_provenance_sha256=digest(run/'source_provenance.json')))
    metadata = dict(schema_version=1,kind='edited actual MuJoCo window recordings',
        scope='Fresh development attempts, not a controlled acceptance cohort; all capture attempts retained',
        clips=clips,attempts=attempts)
    (args.output/'recordings.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps([dict(file=clip['file'],bytes=clip['bytes'],frames=clip['displayed_frames']) for clip in clips],indent=2))


if __name__=='__main__':
    main()
