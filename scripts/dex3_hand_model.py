#!/usr/bin/env python3
"""Compose official seven-joint Dex3 hands onto the existing native 29-DOF body."""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'ros2_ws/src/unitree_g1_description'
SCENES = ROOT/'ros2_ws/src/unitree_mujoco/unitree_robots/g1'
SOURCE = PACKAGE/'dex3_sources/g1_29dof_with_hand_rev_1_0.xml'
CHANNELS = ('thumb_0','thumb_1','thumb_2','middle_0','middle_1','index_0','index_1')


def joint_names(side):
    return [f'{side}_hand_{channel}_joint' for channel in CHANNELS]


def add_hand(root,wrist,side):
    source = ET.parse(SOURCE).getroot()
    native = source.find(f".//body[@name='{side}_wrist_yaw_link']")
    asset = root.find('asset')
    for mesh in source.findall('asset/mesh'):
        if mesh.get('name','').startswith(side+'_hand_'):
            mesh = copy.deepcopy(mesh)
            filename = mesh.get('file')
            if root.find('compiler').get('meshdir')=='meshes':
                mesh.set('file','../../../../unitree_g1_description/meshes/dex3/'+filename)
            else:
                mesh.set('file',str(PACKAGE/'meshes/dex3'/filename))
            asset.append(mesh)
    for geom in list(wrist.findall('geom')):
        if 'rubber_hand' in geom.get('mesh',''):
            wrist.remove(geom)
    old = wrist.find('inertial')
    if old is not None:
        wrist.remove(old)
    wrist.insert(0,copy.deepcopy(native.find('inertial')))
    for geom in native.findall('geom'):
        if '_hand_palm_' in geom.get('mesh',''):
            wrist.append(copy.deepcopy(geom))
    for body in native.findall('body'):
        if body.get('name','').startswith(side+'_hand_'):
            wrist.append(copy.deepcopy(body))
    contact = root.find('contact')
    if contact is None:
        contact = ET.SubElement(root,'contact')
    # Adjacent CAD housings overlap at hinges. Explicit exclusions are needed
    # on the fixed/mocap bench as well as on a moving robot wrist.
    for parent in wrist.iter('body'):
        for child in parent.findall('body'):
            ET.SubElement(contact,'exclude',body1=parent.get('name'),body2=child.get('name'))
    for body in wrist.iter('body'):
        for joint in body.findall('joint'):
            if '_hand_' in joint.get('name',''):
                joint.set('damping','.025')
                joint.set('armature','.0001')
                joint.set('frictionloss','.002')
        for i,geom in enumerate(body.findall('geom')):
            if body is wrist and '_hand_' not in geom.get('mesh',''):
                continue
            geom.set('name',f'dex3_{body.get("name")}_{i}')
            if geom.get('contype','1')!='0':
                geom.set('friction','1 .005 .0001')
                geom.set('condim','4')
                geom.set('solref','.008 1')
    return joint_names(side)


def build_robot():
    root = ET.parse(SCENES/'g1_29dof.xml').getroot()
    old_names = [j.get('name') for j in root.findall('worldbody//joint') if j.get('type')!='free']
    keys = {}
    for key in root.findall('keyframe/key'):
        values = key.get('qpos').split()
        keys[key.get('name')] = (values[:7],dict(zip(old_names,values[7:])))
    for side in ('left','right'):
        add_hand(root,root.find(f".//body[@name='{side}_wrist_yaw_link']"),side)
    names = [j.get('name') for j in root.findall('worldbody//joint') if j.get('type')!='free']
    for key in root.findall('keyframe/key'):
        prefix,positions = keys[key.get('name')]
        key.set('qpos',' '.join(prefix+[positions.get(name,'0') for name in names]))
    root.set('model','g1_29dof_dex3_contact')
    ET.indent(root)
    return root


def generate_urdf():
    root = ET.parse(PACKAGE/'urdf/g1_29dof_mode_15_brainco_hand.urdf').getroot()
    source = ET.parse(PACKAGE/'dex3_sources/g1_29dof_with_hand_rev_1_0.urdf').getroot()
    for side in ('left','right'):
        descendants = {f'{side}_base_link',f'{side}_base2_link'}
        for _ in range(20):
            descendants.update(j.find('child').get('link') for j in root.findall('joint')
                               if j.find('parent').get('link') in descendants)
        for item in list(root):
            if ((item.tag=='link' and item.get('name') in descendants) or
                    (item.tag=='joint' and item.find('child').get('link') in descendants)):
                root.remove(item)
        # Official hand links include their wrist-fixed palm; retain the body's
        # established kinematics and substitute the Dex3 wrist inertial only.
        old_wrist = root.find(f"link[@name='{side}_wrist_yaw_link']")
        new_wrist = source.find(f"link[@name='{side}_wrist_yaw_link']")
        old_wrist.remove(old_wrist.find('inertial'))
        old_wrist.insert(0,copy.deepcopy(new_wrist.find('inertial')))
        for item in source:
            if item.tag in ('joint','link') and item.get('name','').startswith(side+'_hand_'):
                item = copy.deepcopy(item)
                for mesh in item.findall('.//mesh'):
                    mesh.set('filename','package://unitree_g1_description/meshes/dex3/'+Path(mesh.get('filename')).name)
                root.append(item)
    root.set('name','g1_29dof_dex3')
    ET.indent(root)
    ET.ElementTree(root).write(PACKAGE/'urdf/g1_29dof_dex3.urdf',encoding='unicode')


def generate():
    ET.ElementTree(build_robot()).write(SCENES/'g1_29dof_dex3.xml',encoding='unicode')
    generate_urdf()
    scene = ET.parse(SCENES/'scene_29dof.xml')
    scene.getroot().find('include').set('file','g1_29dof_dex3.xml')
    scene.write(SCENES/'scene_29dof_dex3.xml',encoding='unicode')
    for name in ('scene_29dof_extreme_lab','scene_29dof_natural_crowd','scene_29dof_pedestrians'):
        scene = ET.parse(SCENES/(name+'.xml'))
        include = scene.getroot().find("include[@file='g1_29dof.xml']")
        if include is not None:
            include.set('file','g1_29dof_dex3.xml')
        scene.write(SCENES/(name+'_dex3.xml'),encoding='unicode')
    # Migrate the stationary manipulation fixture without retaining its base
    # pin or object welds. Joint names, not numeric insertion offsets, map poses.
    scene = ET.parse(SCENES/'scene_29dof_manipulation.xml')
    root = scene.getroot()
    root.find('include').set('file','g1_29dof_dex3.xml')
    root.remove(root.find('equality'))
    geom = root.find(".//geom[@name='red_cube_geom']")
    geom.set('type','cylinder');geom.set('size','.02 .045');geom.set('mass','.08')
    old_robot = ET.parse(SCENES/'g1_29dof.xml').getroot()
    old_names = [j.get('name') for j in old_robot.findall('worldbody//joint') if j.get('type')!='free']
    new_names = [j.get('name') for j in build_robot().findall('worldbody//joint') if j.get('type')!='free']
    for key in root.findall('keyframe/key'):
        values = key.get('qpos').split()
        positions = dict(zip(old_names,values[7:36]))
        key.set('qpos',' '.join(values[:7]+[positions.get(name,'0') for name in new_names]+values[36:]))
    scene.write(SCENES/'scene_29dof_manipulation_dex3.xml',encoding='unicode')
    srdf = ET.parse(ROOT/'ros2_ws/src/g1_moveit_config/config/g1.srdf')
    srdf.getroot().set('name','g1_29dof_dex3')
    for chain in srdf.getroot().findall('group/chain'):
        for side in ('left','right'):
            if chain.get('tip_link')==side+'_base_link':
                chain.set('tip_link',side+'_wrist_yaw_link')
    description = ET.parse(PACKAGE/'urdf/g1_29dof_dex3.urdf').getroot()
    links = {link.get('name') for link in description.findall('link')}
    for collision in list(srdf.getroot().findall('disable_collisions')):
        if collision.get('link1') not in links or collision.get('link2') not in links:
            srdf.getroot().remove(collision)
    existing = {frozenset((c.get('link1'),c.get('link2'))) for c in srdf.getroot().findall('disable_collisions')}
    for joint in description.findall('joint'):
        a,b = joint.find('parent').get('link'),joint.find('child').get('link')
        if '_hand_' in b and frozenset((a,b)) not in existing:
            ET.SubElement(srdf.getroot(),'disable_collisions',link1=a,link2=b,reason='Adjacent')
    ET.indent(srdf)
    srdf.write(ROOT/'ros2_ws/src/g1_moveit_config/config/g1_dex3.srdf',encoding='unicode')


if __name__=='__main__':
    generate()
