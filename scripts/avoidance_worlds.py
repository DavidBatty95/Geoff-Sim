#!/usr/bin/env python3
"""Generate bounded MuJoCo avoidance worlds from versioned geometric definitions."""
import argparse
import copy
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from world_appearance import THEMES, dress, sign_png

ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / 'ros2_ws/src/unitree_mujoco/unitree_robots/g1'
WORLD_VERSION = 'representative-v2'
WORLDS = {
    'small_open': dict(size=[8, 8], complexity='low', boxes=[]),
    'small_clutter': dict(size=[8, 8], complexity='medium', boxes=[[-2, 2, .45, .6], [2, -2, .5, .5], [2.4, 2.2, .5, .4]]),
    'medium_open': dict(size=[12, 10], complexity='low', boxes=[]),
    'medium_doorway': dict(size=[12, 10], complexity='high', boxes=[[2, 3, .18, 2], [2, -3, .18, 2]]),
    'large_pillars': dict(size=[16, 14], complexity='medium', boxes=[[x, y, .45, .45] for x in [-4, 0, 4] for y in [-3, 3]]),
    'large_office': dict(size=[16, 14], complexity='high', boxes=[[-3, 3.6, .18, 2.4], [3, -3.6, .18, 2.4], [-5, -3, 1, .5], [5, 3, 1, .5], [0, 4.8, .8, .5], [0, -4.8, .8, .5]]),
}


def scene(world, definition=None,hand_model='dex3'):
    definition = WORLDS[world] if definition is None else definition
    source = ET.parse(SCENES / 'scene_29dof_pedestrians.xml').getroot()
    root = ET.Element('mujoco', model='Avoidance ' + world)
    ET.SubElement(root, 'include', file='g1_29dof_dex3.xml' if hand_model=='dex3' else 'g1_29dof.xml')
    for tag in ('visual', 'asset'):
        root.append(copy.deepcopy(source.find(tag)))
    root.find('visual/global').set('elevation', '-50')
    ET.SubElement(root, 'include', file='natural_crowd_assets.xml')
    ET.SubElement(root, 'include', file='natural_crowd_bodies.xml')
    x, y = (n / 2 for n in definition['size'])
    ET.SubElement(root, 'statistic', center='0 0 1', extent=str(max(x, y) + 2))
    custom = ET.SubElement(root, 'custom')
    ET.SubElement(custom, 'numeric', name='avoidance_world_half_extents', data=f'{x} {y}')
    body = ET.SubElement(root, 'worldbody')
    ET.SubElement(body, 'light', pos='0 0 8', dir='0 0 -1', diffuse='.8 .8 .8')
    ET.SubElement(body, 'geom', name='floor', type='plane', size='0 0 .05', material='paving', friction='1.0 .01 .001')
    boxes = [[0, y+.15, x+.3, .15], [0, -y-.15, x+.3, .15],
             [x+.15, 0, .15, y], [-x-.15, 0, .15, y]] + definition['boxes']
    for i, (px, py, sx, sy) in enumerate(boxes):
        ET.SubElement(body, 'geom', name=f'world_box_{i}', type='box',
                      pos=f'{px} {py} 1', size=f'{sx} {sy} 1',
                      material='building_concrete' if i < 4 else 'brick')
    for child in source.find('worldbody'):
        if child.tag == 'body' and child.get('name', '').startswith('ped'):
            body.append(copy.deepcopy(child))
    dress(root, world, definition)
    ET.indent(root)
    return ET.tostring(root, encoding='unicode') + '\n'


def generate():
    for name in WORLDS:
        (SCENES / f'scene_29dof_avoidance_{name}.xml').write_text(scene(name))
        (SCENES / f'avoidance_sign_{name}.png').write_bytes(sign_png(THEMES[name][1]))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    if args.list:
        print(json.dumps(WORLDS, indent=2))
    else:
        generate()
        print(f'Generated {len(WORLDS)} avoidance worlds in {SCENES}')
