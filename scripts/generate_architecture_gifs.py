#!/usr/bin/env python3
"""Render reproducible animated architecture diagrams and static reading copies."""
import hashlib
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'docs/media'
BG='#0b1220'
PANEL='#152238'
TEXT='#eef5ff'
MUTED='#b3c6df'
ACCENT='#55e0c0'
BLUE='#70b9ff'

SYSTEM=dict(
    name='architecture-system', title='Geoff-Sim / the connected system',
    subtitle='MuJoCo physics  +  ROS 2 Humble  +  Unitree DDS  |  architectural animation',
    nodes=[
        ('01  Physical world', 'MuJoCo native simulator', ['G1 body + articulated Dex3 hands','Contacts, cylinders and pedestrians','Procedural rooms and furnishings']),
        ('02  Onboard sensing', 'g1_sim_sensors', ['LiDAR points + RGB-D images','IMU + joint encoders + LowState','Acquisition-stamped sensor packets']),
        ('03  State & perception', 'onboard_lidar_odometry / SLAM', ['Scan registration + measured attitude','Online map + observed free space','RGB-D object and support observations']),
        ('06  Collision supervision', 'human_collision_avoidance', ['Freshness + command lineage','Swept-body and static clearance','Checked velocity command to policy']),
        ('05  Social planning', 'human_aware_motion_planner', ['Tracked people + motion forecasts','Response-aware trajectories + braking','Optional process-isolated escape search']),
        ('04  Mission & route', 'transport_demo / Nav2', ['Survey, approach and task sequencing','Observed-map goals + route updates','Payload and manipulation context']),
        ('07  Learned locomotion', 'g1_policy_cmd_vel_controller', ['ONNX inference + measured joints','Device-tick cadence + watchdogs','29 body-joint action targets']),
        ('08  Command composition', 'arm_command_compositor / DDS', ['Policy body + owned arm commands','Validated LowCmd + hand commands','Physics receives actuator targets']),
        ('09  Contact manipulation', 'arm_kinematics / dex3_grasp', ['Cartesian waypoints + continuous IK','Opposing tactile contact + load dwell','Measured opening + release services'])],
    edges=[(0,1),(1,2),(2,5),(5,4),(4,3),(3,6),(6,7),(5,8),(8,7)],
    phases=[([0,1], 'SENSE', 'Native physics produces the sensor streams used by the robot-side stack.'),
            ([2], 'ESTIMATE', 'LiDAR, attitude and encoders build pose, online maps and object observations.'),
            ([5,4], 'PLAN', 'Mission intent and route context combine with tracked pedestrian forecasts.'),
            ([3], 'CHECK', 'Independent collision supervision validates the command before locomotion.'),
            ([6,7,8], 'ACTUATE', 'Body policy and manipulation ownership meet in the DDS command path.'),
            ([0,7], 'CLOSE THE LOOP', 'Actuator targets change physics; measured state returns on the next cycle.')],
    footer='Separate evaluation branch: native truth → aligned metrics → JSON traces → browser dashboard',
    boundary='Evaluation truth is used for scoring and offline diagnosis; robot decisions use onboard observations.')

CONTROL=dict(
    name='architecture-control', title='Geoff-Sim / mission, contact and command ownership',
    subtitle='Detailed transport implementation  |  architectural sequence, not a recorded mission',
    nodes=[
        ('01  Observe the task', 'transport_perception', ['RGB-D colour and depth registration','Object, destination and support IDs','Observation timestamps + confidence']),
        ('02  Survey & approach', 'transport_demo', ['Viewing goals + fresh re-observation','Policy-controlled arms while walking','Measured range, heading and settling']),
        ('03  Plan the reach', 'arm_kinematics', ['Tool-centred targets + continuous IK','Cartesian segments up to 25 mm','Bounded joint speed + arm ownership']),
        ('06  Contact release', 'dex3_grasp / perception', ['Measured slow finger opening','Post-release object observations','Placement verification in task logic']),
        ('05  Payload motion', 'mission + social planner', ['Carry posture and payload context','Observed routes + predicted people','Full trajectory and terminal-hold checks']),
        ('04  Physical grasp', 'native Dex3 tactile feedback', ['14 finger DOFs across both hands','Opposing contact + joint-load evidence','Close ramp + native-tick dwell']),
        ('07  Command contract', 'planner → independent supervisor', ['Publisher and sequence identity','Source time, expiry and context','Fresh revalidation at command admission']),
        ('08  Execution contract', 'policy → compositor → DDS', ['Device-tick observations + inference','Arm/body authority composition','Validated actuator messages']),
        ('09  Reproducibility', 'runner + logger + test suite', ['Seed, policy hash and source snapshot','Mission milestones + sensor traces','Layered physics / ROS / stage checks'])],
    edges=[(0,1),(1,2),(2,5),(5,4),(4,3),(4,7),(6,7),(7,8)],
    phases=[([0,1], 'OBSERVE & POSITION', 'Fresh sensed geometry establishes the viewing and approach targets.'),
            ([2,5], 'REACH & GRASP', 'Arm ownership and tactile feedback connect planning to physical fingers.'),
            ([4,6], 'PLAN WITH CONTEXT', 'Payload state and predicted people inform checked motion commands.'),
            ([7], 'EXECUTE', 'The low-level path composes owned arm motion with the locomotion policy.'),
            ([3], 'RELEASE & OBSERVE', 'Finger opening and post-release observations support placement assessment.'),
            ([8], 'REPRODUCE', 'Source-linked artifacts connect each result to its exact experiment setup.')],
    footer='Three evidence layers: physical hand bench • integrated ROS/DDS services • mobile stage experiments',
    boundary='The diagram describes implemented modules and contracts; dated reports provide experiment scope.')


def font(size,bold=False):
    suffix='-Bold' if bold else ''
    return ImageFont.truetype(f'/usr/share/fonts/truetype/dejavu/DejaVuSans{suffix}.ttf',size)


def render(model,index):
    image=Image.new('RGB',(1440,1030),BG)
    draw=ImageDraw.Draw(image)
    phase=index//12
    active,title,caption=model['phases'][phase]
    draw.text((32,22),model['title'],font=font(36,True),fill=TEXT)
    draw.text((34,77),model['subtitle'],font=font(19),fill=MUTED)
    boxes=[(32+(i%3)*476,145+(i//3)*214,460+(i%3)*476,309+(i//3)*214) for i in range(9)]
    for a,b in model['edges']:
        ra,rb=boxes[a],boxes[b]
        ca=((ra[0]+ra[2])/2,(ra[1]+ra[3])/2)
        cb=((rb[0]+rb[2])/2,(rb[1]+rb[3])/2)
        dx,dy=cb[0]-ca[0],cb[1]-ca[1]
        if abs(dx)>abs(dy):
            start=(ra[2] if dx>0 else ra[0],ca[1]);end=(rb[0] if dx>0 else rb[2],cb[1])
        else:
            start=(ca[0],ra[3] if dy>0 else ra[1]);end=(cb[0],rb[1] if dy>0 else rb[3])
        colour=ACCENT if a in active or b in active else '#405777'
        draw.line([start,end],fill=colour,width=3)
        angle=math.atan2(end[1]-start[1],end[0]-start[0])
        draw.polygon([end,(end[0]-10*math.cos(angle-.5),end[1]-10*math.sin(angle-.5)),
                      (end[0]-10*math.cos(angle+.5),end[1]-10*math.sin(angle+.5))],fill=colour)
        if a in active or b in active:
            t=(index%12)/12
            x,y=start[0]+t*(end[0]-start[0]),start[1]+t*(end[1]-start[1])
            draw.ellipse((x-5,y-5,x+5,y+5),fill=TEXT)
    # Explicit outer return path closes body commands back into physics.
    if model is SYSTEM:
        draw.line([(722,737),(722,766),(15,766),(15,124),(246,124),(246,145)],fill=BLUE,width=2)
        draw.text((36,773),'Actuator feedback loop',font=font(15),fill=BLUE)
    for i,(heading,module,lines) in enumerate(model['nodes']):
        x,y,right,bottom=boxes[i]
        draw.rounded_rectangle((x,y,right,bottom),radius=14,fill=PANEL,
                               outline=ACCENT if i in active else '#31465f',width=3 if i in active else 1)
        draw.text((x+18,y+13),heading,font=font(23,True),fill=ACCENT if i in active else TEXT)
        draw.text((x+18,y+47),module,font=font(16),fill=BLUE)
        for j,line in enumerate(lines):
            assert draw.textlength(line,font=font(18))<right-x-36,line
            draw.text((x+18,y+77+j*25),line,font=font(18),fill=MUTED)
    draw.rounded_rectangle((32,813,1412,881),radius=12,fill='#102b34')
    draw.text((50,823),model['footer'],font=font(20,True),fill=ACCENT)
    draw.text((50,852),model['boundary'],font=font(17),fill=MUTED)
    draw.text((34,909),f'{phase+1:02d} / 06   {title}',font=font(23,True),fill=TEXT)
    draw.text((34,948),caption,font=font(20),fill=MUTED)
    for i in range(6):
        draw.rounded_rectangle((34+i*230,1001,244+i*230,1007),radius=3,
                               fill=ACCENT if i==phase else '#31465f')
    return image


def main():
    for model in (SYSTEM,CONTROL):
        frames=[render(model,i) for i in range(72)]
        path=OUT/f"{model['name']}.gif"
        palette=frames[0].quantize(colors=128)
        encoded=[frame.quantize(palette=palette,dither=0) for frame in frames]
        encoded[0].save(path,save_all=True,append_images=encoded[1:],duration=200,loop=0,optimize=True,disposal=1)
        frames[0].save(path.with_suffix('.png'))
        metadata=dict(kind='architecture_diagram',scope='illustrated architecture, not simulation footage',
            frame_count=72,playback_duration_ms=14400,dimensions=[1440,1030],
            gif_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            generator='scripts/generate_architecture_gifs.py',
            generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            model=model)
        path.with_suffix('.json').write_text(json.dumps(metadata,indent=2)+'\n')
        print(f'{path.relative_to(ROOT)}: 72 frames, 14.4 seconds, {path.stat().st_size/1024:.0f} KiB')


if __name__=='__main__':
    main()
