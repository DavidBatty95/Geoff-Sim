import xml.etree.ElementTree as ET
import math
from collections import deque

import numpy as np

from transport_worlds import TRANSPORT_WORLDS, build_world, camera_robot_xml, INITIAL_KEYFRAME, TABLE_HALF_SIZE, BOX_HALF_SIZE, PAD_HALF_SIZE


def test_transport_fixtures_have_supported_pairs_and_no_base_pin():
    for name, definition in TRANSPORT_WORLDS.items():
        text,pairs,layout = build_world(name)
        root = ET.fromstring(text)
        assert len(pairs) == len(definition['stations'])
        assert len({p['object_id'] for p in pairs}) == len(pairs)
        assert len({p['destination_id'] for p in pairs}) == len(pairs)
        assert all(set(p)=={'object_id','destination_id','color'} for p in pairs)
        assert root.find("equality/weld[@name='manipulation_base_pin']") is None
        assert not root.find('equality').findall('weld')
        assert root.find("custom/numeric[@name='dex3_contact_only']") is not None
        for pair in pairs:
            marker = root.find(f"worldbody/body[@name='manip_surface_{pair['destination_id']}']")
            assert len(marker.findall('geom')) == 1  # Object-state protocol contract.
            assert marker.find('geom').get('rgba') != '0 0 0 0'
        assert len(root.find(f"keyframe/key[@name='{INITIAL_KEYFRAME}']").get('qpos').split()) == 50+7*len(pairs)
        for entry in layout:
            for station in ('pickup','delivery'):
                record=entry[station]
                dx=abs(record['object_or_pad_center'][0]-record['table_center'][0])
                extent=BOX_HALF_SIZE if station=='pickup' else PAD_HALF_SIZE+.012
                assert dx+extent < TABLE_HALF_SIZE[0]
                top=root.find(f"worldbody/body[@name='station_{record['station']}']/geom")
                assert top.get('contype') == top.get('conaffinity') == '1'


def test_dual_demo_contains_two_graspable_cylinders_and_four_physical_tables():
    text,pairs,layout = build_world('transport_dual')
    root = ET.fromstring(text)
    assert [pair['color'] for pair in pairs] == ['red','blue']
    assert len([body for body in root.findall('worldbody/body') if body.get('name','').startswith('station_')]) == 4
    assert len(root.findall('worldbody/body/freejoint')) == 2
    for color in ('red','blue'):
        geom = root.find(f".//geom[@name='box_{color}_geom']")
        assert geom.get('type')=='cylinder' and geom.get('size')=='.02 .045'
        assert geom.get('mass')=='.08'
    assert len(layout) == 2
    assert all('position' not in pair and 'table_center' not in pair for pair in pairs)


def test_magnetic_fixture_requires_explicit_legacy_selection():
    text,pairs,_ = build_world('transport_dual',grasp_model='legacy_magnetic')
    root = ET.fromstring(text)
    assert len(root.find('equality').findall('weld'))==2*len(pairs)
    assert len(root.find(f"keyframe/key[@name='{INITIAL_KEYFRAME}']").get('qpos').split())==36+7*len(pairs)


def test_tabletop_camera_pitch_matches_declared_optical_axis():
    pitch = TRANSPORT_WORLDS['transport_dual']['camera_pitch_deg']
    root = ET.fromstring(camera_robot_xml(pitch))
    w,x,y,z = map(float,root.find(".//site[@name='d435i_color_site']").get('quat').split())
    forward = (2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y))
    assert abs(forward[0]-math.cos(math.radians(pitch))) < 1e-12
    assert abs(forward[1]) < 1e-12
    assert abs(forward[2]+math.sin(math.radians(pitch))) < 1e-12


def test_dual_layout_leaves_passing_space_between_tables_and_walls():
    definition = TRANSPORT_WORLDS['transport_dual']
    red,blue = definition['stations']
    assert abs(red[0][0]-red[1][0])-2*TABLE_HALF_SIZE[0] >= 3.9
    assert abs(blue[0][1]-red[0][1])-2*TABLE_HALF_SIZE[1] >= 2.
    for pair in definition['stations']:
        for x,y in pair:
            assert definition['size'][0]/2-abs(x)-TABLE_HALF_SIZE[0] >= 1.2
            assert definition['size'][1]/2-abs(y)-TABLE_HALF_SIZE[1] >= 1.2


def test_search_fixture_has_one_task_among_four_tables_and_hidden_start_views():
    for seed in (0,1):
        text,pairs,layout = build_world('transport_search',seed)
        root = ET.fromstring(text)
        assert pairs==[{'object_id':'box_red','destination_id':'target_red','color':'red'}]
        assert len([body for body in root.findall('worldbody/body') if body.get('name','').startswith('station_')])==4
        assert len(root.findall('worldbody/body/freejoint'))==1
        start = list(map(float,root.find(f"keyframe/key[@name='{INITIAL_KEYFRAME}']").get('qpos').split()[:2]))
        for role,index in (('pickup',0),('delivery',1)):
            point = layout[0][role]['object_or_pad_center']
            wall = root.find(f"worldbody/geom[@name='search_partition_{index}']")
            x,y,_ = map(float,wall.get('pos').split())
            fraction = (x-start[0])/(point[0]-start[0])
            crossing_y = start[1]+fraction*(point[1]-start[1])
            half_y = float(wall.get('size').split()[1])
            assert 0.<fraction<1. and abs(crossing_y-y)<half_y


def test_large_buildings_have_connected_clearance_and_hidden_task_tables():
    # Offline fixture validation only: these coordinates never reach the mission.
    for name in ('transport_medium','transport_atrium','transport_corridors'):
        for seed in (0,1):
            text,pairs,layout = build_world(name,seed)
            root = ET.fromstring(text)
            hx,hy = (n/2 for n in TRANSPORT_WORLDS[name]['size'])
            resolution,radius = .1,.7
            xs = np.arange(-hx,hx,resolution)+resolution/2
            ys = np.arange(-hy,hy,resolution)+resolution/2
            xx,yy = np.meshgrid(xs,ys)
            free = (np.abs(xx)<hx-radius)&(np.abs(yy)<hy-radius)
            walls = [geom for geom in root.findall('worldbody/geom') if geom.get('name','').startswith('building_wall_')]
            assert len(walls)>=16
            for geom in walls:
                x,y,_ = map(float,geom.get('pos').split())
                sx,sy,_ = map(float,geom.get('size').split())
                free &= (np.abs(xx-x)>sx+radius)|(np.abs(yy-y)>sy+radius)
            tables = [body for body in root.findall('worldbody/body') if body.get('name','').startswith('station_')]
            assert len(tables)>=(4 if name=='transport_medium' else 8)
            for table in tables:
                x,y,_ = map(float,table.get('pos').split())
                free &= (np.abs(xx-x)>TABLE_HALF_SIZE[0]+radius)|(np.abs(yy-y)>TABLE_HALF_SIZE[1]+radius)
            def cell(point):
                return int((point[0]+hx)/resolution),int((point[1]+hy)/resolution)
            start = cell((0.,0.))
            seen,queue = {start},deque([start])
            while queue:
                x,y = queue.popleft()
                for nx,ny in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                    if 0<=ny<len(ys) and 0<=nx<len(xs) and free[ny,nx] and (nx,ny) not in seen:
                        seen.add((nx,ny)); queue.append((nx,ny))
            for table in tables:
                x,y,_ = map(float,table.get('pos').split())
                assert cell((x-math.copysign(1.8,x) if x else x+1.8,y)) in seen
            for role in ('pickup','delivery'):
                point = layout[0][role]['object_or_pad_center']
                ray = np.linspace((0.,0.),point[:2],1000)
                assert any(np.any((np.abs(ray[:,0]-float(g.get('pos').split()[0]))<float(g.get('size').split()[0])) &
                                     (np.abs(ray[:,1]-float(g.get('pos').split()[1]))<float(g.get('size').split()[1]))) for g in walls)
            assert all(set(pair)=={'object_id','destination_id','color'} for pair in pairs)


def test_medium_fixture_has_two_boxes_and_two_goals_in_four_rooms():
    text,pairs,layout = build_world('transport_medium')
    root = ET.fromstring(text)
    assert TRANSPORT_WORLDS['transport_medium']['size']==[16,12]
    assert {pair['object_id'] for pair in pairs}=={'box_red','box_blue'}
    assert {pair['destination_id'] for pair in pairs}=={'target_red','target_blue'}
    assert len(root.findall('worldbody/body/freejoint'))==2
    assert len(root.findall("worldbody/body[@name='station_pickup_red']"))==1
