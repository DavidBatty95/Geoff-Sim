#!/usr/bin/env python3
"""Generate physical multi-table transport fixtures; layouts are evaluator data."""
import argparse
import json
import math
import copy
import xml.etree.ElementTree as ET

from avoidance_worlds import ROOT, SCENES, generate as generate_navigation_worlds, scene
from world_appearance import sign_png

COLORS = {'red': '0.85 0.08 0.06 1', 'blue': '0.05 0.20 0.85 1', 'green': '0.06 0.65 0.18 1'}
TRANSPORT_WORLDS = {
    'transport_medium': {'base':'medium_open','size':[16,12],'label':'ROOM LAB','camera_pitch_deg':25.,
                         'building':'medium','stations':[((5.6,3.8),(-5.6,-3.8)),
                                                        ((-5.6,3.8),(5.6,-3.8))]},
    'transport_atrium': {'base':'medium_open','size':[28,24],'label':'ATRIUM','camera_pitch_deg':25.,
                         'building':'atrium','stations':[((11.,8.),(-11.,-8.))],
                         'distractor_tables':[(11.,0.),(11.,-8.),(-11.,0.),(-11.,8.),(0.,10.),(0.,-10.)]},
    'transport_corridors': {'base':'medium_open','size':[32,24],'label':'ROOM WING','camera_pitch_deg':25.,
                            'building':'corridors','stations':[((13.,8.),(-13.,-8.))],
                            'distractor_tables':[(x,y) for x in (-13.,-5.,5.,13.) for y in (-8.,0.,8.)
                                                 if (x,y) not in ((13.,8.),(-13.,-8.))]},
    'transport_search': {'base':'medium_open','size':[12,10],'label':'SEARCH LAB','camera_pitch_deg':25.,
                         'stations': [((3.5,2.8),(-3.5,-2.8))],
                         'distractor_tables':[(3.5,-2.8),(-3.5,2.8)],
                         'search_partitions':[(1.5,1.5),(-1.5,-1.5)]},
    'transport_dual': {'base':'medium_open','size':[12,10],'label':'TWO CUBE DELIVERY','camera_pitch_deg':25.,
                       # 3.9 m central aisle and 2.0 m between adjacent table
                       # edges leave room to pass/yield without crowding a dock.
                       'stations': [((2.5,0.),(-2.5,0.)), ((2.5,3.),(-2.5,3.))]},
    'transport_aisle': {'base':'medium_open','size':[12,10],'label':'CARRY LAB',
                        'stations': [((2.,0.),(4.,1.4))]},
    'transport_lab': {'base':'medium_open','size':[12,10],'label':'TRANSPORT LAB',
                      'stations': [((2.,0.),(-2.,1.4))]},
    'workshop': {'base': 'medium_open', 'size': [12,10], 'label': 'WORKSHOP',
                 'stations': [((-4.5,-2.2),(4.5,2.2)), ((-4.5,2.2),(4.5,-2.2))]},
    'office_delivery': {'base': 'large_office', 'size': [16,14], 'label': 'OFFICES',
                        'stations': [((-6.,-4.7),(6.,4.7)), ((-6.,0.),(6.,0.)), ((-6.,4.7),(6.,-4.7))]},
    'plaza_delivery': {'base': 'plaza', 'size': [26,24], 'label': 'DELIVERY',
                       'stations': [((-7.3,.9),(6.7,-2.4)), ((-7.3,-6.4),(7.3,4.5))]},
}
TABLE_HEIGHT = .95
TABLE_HALF_SIZE = (.55,.50,.04)
BOX_HALF_SIZE = .055
PAD_HALF_SIZE = .18
INITIAL_KEYFRAME = 'transport_unitree_rl_lab_stand'


def building_walls(kind):
    """Simulator-only (x,y,half_x,half_y) walls with 3 m door openings."""
    if kind=='medium':
        # Four rooms around a compact cross-corridor/atrium, with two 2.8 m
        # entrances per room. Table locations remain simulator/scorer-only.
        walls = []
        for side in (-1,1):
            for low,high in ((-6.,-5.2),(-2.4,-1.6),(1.6,2.4),(5.2,6.)):
                walls.append((side*2.,(low+high)/2,.12,(high-low)/2))
            for y in (-1.6,1.6):
                for low,high in ((2.,3.4),(6.2,8.)):
                    walls.append((side*(low+high)/2,y,(high-low)/2,.12))
        return walls
    walls = []
    boundaries = (7.,) if kind=='atrium' else (2.,9.)
    outer = 14. if kind=='atrium' else 16.
    for side in (-1,1):
        for x in boundaries:
            for low,high in ((-12.,-9.5),(-6.5,-1.5),(1.5,6.5),(9.5,12.)):
                walls.append((side*x,(low+high)/2,.12,(high-low)/2))
        for y in (-4.,4.):
            walls.append((side*(outer+boundaries[0])/2,y,(outer-boundaries[0])/2,.12))
    if kind=='atrium':
        for side in (-1,1):
            for x in (-4.,4.):
                walls.append((x,side*9.5,.12,2.5))
            for x in (-2.75,2.75):
                walls.append((x,side*7.,1.25,.12))
        # Four physical columns frame an open central atrium and circulation loop.
        walls.extend((x,y,.3,.3) for x in (-3.,3.) for y in (-3.,3.))
    return walls


def camera_robot_xml(pitch_deg):
    """Declared simulation camera mounting, separate from object/table layout."""
    robot = ET.parse(SCENES/'g1_29dof.xml').getroot()
    site = robot.find(".//site[@name='d435i_color_site']")
    w,x,y,z = map(float,site.get('quat').split())
    c,s = math.cos(math.radians(pitch_deg)/2),math.sin(math.radians(pitch_deg)/2)
    site.set('quat',' '.join(map(str,(c*w-s*y,c*x+s*z,c*y+s*w,c*z-s*x))))
    ET.indent(robot)
    return ET.tostring(robot,encoding='unicode')+'\n'


def add_station(root, color, location, pickup):
    x,y = location
    identifier = ('empty_' if pickup is None else 'pickup_' if pickup else 'delivery_') + color
    world = root.find('worldbody')
    station = ET.SubElement(world,'body',name='station_'+identifier,pos=f'{x} {y} 0')
    ET.SubElement(station,'geom',name=identifier+'_top',type='box',
                  pos=f'0 0 {TABLE_HEIGHT-TABLE_HALF_SIZE[2]}',size=' '.join(map(str,TABLE_HALF_SIZE)),
                  rgba='.46 .31 .18 1',contype='1',conaffinity='1',friction='1 .01 .001')
    leg_half_height=(TABLE_HEIGHT-2*TABLE_HALF_SIZE[2])/2
    for i,(lx,ly) in enumerate(((-.44,-.39),(-.44,.39),(.44,-.39),(.44,.39))):
        ET.SubElement(station,'geom',name=f'{identifier}_leg_{i}',type='box',
                      pos=f'{lx} {ly} {leg_half_height}',size=f'.035 .035 {leg_half_height}',
                      rgba='.12 .14 .16 1',contype='1',conaffinity='1')
    # Boxes and pads face the central aisle; their centers remain supported.
    if pickup is None:
        return {'station':identifier,'table_center':[x,y],'table_top_m':TABLE_HEIGHT}
    inward = 1. if x < 0 else -1.
    offset = .47 if pickup else TABLE_HALF_SIZE[0]-PAD_HALF_SIZE-.025
    center = [x+inward*offset,y,TABLE_HEIGHT]
    if pickup:
        center[2] += BOX_HALF_SIZE+.002
        body=ET.SubElement(world,'body',name=f'manip_object_box_{color}',pos=' '.join(map(str,center)))
        ET.SubElement(body,'freejoint',name=f'box_{color}_freejoint')
        ET.SubElement(body,'geom',name=f'box_{color}_geom',type='box',
                      size=f'{BOX_HALF_SIZE} {BOX_HALF_SIZE} {BOX_HALF_SIZE}',mass='.15',
                      rgba=COLORS[color],friction='1 .01 .001',contype='1',conaffinity='1')
    else:
        # A filled colour square remains visible in the low-resolution RGB-D
        # camera; a one-pixel outline fragments at ordinary viewing distances.
        marker=ET.SubElement(world,'body',name=f'manip_surface_target_{color}',
                             pos=f'{center[0]} {center[1]} {TABLE_HEIGHT+.002}')
        ET.SubElement(marker,'geom',name=f'target_{color}_region',type='box',
                      size=f'{PAD_HALF_SIZE} {PAD_HALF_SIZE} .002',rgba=COLORS[color],contype='0',conaffinity='0')
        # Keep the scoring region as the body's sole geom: the object-state
        # protocol reports one box per body. Decoration belongs to a child.
        outline = ET.SubElement(marker,'body',name=f'target_{color}_outline')
        for i,(px,py,sx,sy) in enumerate(((-PAD_HALF_SIZE,0,.012,PAD_HALF_SIZE),
                                         (PAD_HALF_SIZE,0,.012,PAD_HALF_SIZE),
                                         (0,-PAD_HALF_SIZE,PAD_HALF_SIZE,.012),
                                         (0,PAD_HALF_SIZE,PAD_HALF_SIZE,.012))):
            ET.SubElement(outline,'geom',name=f'target_{color}_edge_{i}',type='box',
                          pos=f'{px} {py} .001',size=f'{sx} {sy} .002',rgba=COLORS[color],contype='0',conaffinity='0')
    return {'station':identifier,'table_center':[x,y], 'table_top_m':TABLE_HEIGHT,
            'object_or_pad_center':center}


def build_world(name,layout_seed=0):
    definition=copy.deepcopy(TRANSPORT_WORLDS[name])
    if (name=='transport_search' or definition.get('building')) and layout_seed%2:
        # The alternate evaluation layout mirrors every station and occluder;
        # no coordinate or layout seed is passed to the search executive.
        definition['stations'] = [tuple((x,-y) for x,y in pair) for pair in definition['stations']]
        for key in ('distractor_tables','search_partitions'):
            definition[key] = [(x,-y) for x,y in definition.get(key,())]
    if definition['base']=='plaza':
        root=ET.parse(SCENES/'scene_29dof_pedestrians.xml').getroot()
        ET.SubElement(root,'include',file='natural_crowd_assets.xml')
        ET.SubElement(root,'include',file='natural_crowd_bodies.xml')
        custom=ET.SubElement(root,'custom')
        ET.SubElement(custom,'numeric',name='avoidance_world_half_extents',data='8 7')
    else:
        geometry = dict(size=definition['size'],boxes=[]) if definition.get('building') else None
        root=ET.fromstring(scene(definition['base'],geometry))
        root.find("asset/texture[@name='world_sign']").set('file',f'transport_sign_{name}.png')
    root.set('model','G1 transport '+name)
    if definition.get('camera_pitch_deg',0.):
        root.find("include[@file='g1_29dof.xml']").set('file',f'g1_29dof_{name}_camera.xml')
    custom = root.find('custom')
    if custom is None:
        custom = ET.SubElement(root,'custom')
    ET.SubElement(custom,'numeric',name='transport_hand_capture',data='.12 .01 0 .09')
    root.find('visual/global').set('elevation','-48')
    equalities=ET.SubElement(root,'equality')
    keys=ET.SubElement(root,'keyframe')
    robot=ET.parse(SCENES/'g1_29dof.xml').getroot()
    qpos=robot.find("keyframe/key[@name='unitree_rl_lab_stand']").get('qpos').split()
    pairs=[]
    layout=[]
    for index,(x,y,sx,sy) in enumerate(building_walls(definition['building']) if definition.get('building') else ()):
        ET.SubElement(root.find('worldbody'),'geom',name=f'building_wall_{index}',type='box',
                      pos=f'{x} {y} 1.6',size=f'{sx} {sy} 1.6',material='painted_wall',contype='1',conaffinity='1')
    for index,position in enumerate(definition.get('distractor_tables',())):
        add_station(root,f'distractor_{index}',position,None)
    for index,(x,y) in enumerate(definition.get('search_partitions',())):
        ET.SubElement(root.find('worldbody'),'geom',name=f'search_partition_{index}',type='box',
                      pos=f'{x} {y} 1.05',size='.08 .55 1.05',rgba='.45 .45 .45 1',contype='1',conaffinity='1')
    for color,(pickup,delivery) in zip(COLORS,definition['stations']):
        source=add_station(root,color,pickup,True)
        destination=add_station(root,color,delivery,False)
        qpos += [str(v) for v in source['object_or_pad_center']] + ['1','0','0','0']
        for side in ('left','right'):
            ET.SubElement(equalities,'weld',name=f'grasp_{side}_box_{color}',
                          body1=f'{side}_wrist_yaw_link',body2=f'manip_object_box_{color}',active='false')
        pairs.append({'object_id':f'box_{color}','destination_id':f'target_{color}','color':color})
        layout.append({'color':color,'pickup':source,'delivery':destination})
    ET.SubElement(keys,'key',name=INITIAL_KEYFRAME,qpos=' '.join(qpos))
    ET.indent(root)
    return ET.tostring(root,encoding='unicode')+'\n', pairs, layout


def generate(layout_seed=0):
    generate_navigation_worlds()
    manifests={}
    for name,definition in TRANSPORT_WORLDS.items():
        if definition.get('camera_pitch_deg',0.):
            (SCENES/f'g1_29dof_{name}_camera.xml').write_text(camera_robot_xml(definition['camera_pitch_deg']))
        xml,pairs,layout=build_world(name,layout_seed)
        (SCENES/f'scene_29dof_transport_{name}.xml').write_text(xml)
        if definition['base']!='plaza':
            (SCENES/f'transport_sign_{name}.png').write_bytes(sign_png(definition['label']))
        # Pair files contain no locations; layout files are simulator/scorer-only.
        (SCENES/f'transport_pairs_{name}.json').write_text(json.dumps({'pairs':pairs},indent=2)+'\n')
        manifests[name]={'initial_keyframe':INITIAL_KEYFRAME,'size_m':definition['size'],
                          'layout_seed':layout_seed if name=='transport_search' or definition.get('building') else None,
                          'camera_pitch_deg':definition.get('camera_pitch_deg',0.),
                         'evaluation_layout':layout,'grasp_model':'capture_radius_weld', 'base_pin_available':False}
    (SCENES/'transport_evaluation_layouts.json').write_text(json.dumps(manifests,indent=2)+'\n')
    print(f'Generated {len(manifests)} physical transport fixtures')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layout-seed',type=int,default=0)
    args = parser.parse_args()
    generate(args.layout_seed)
