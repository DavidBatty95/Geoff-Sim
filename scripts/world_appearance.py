"""Procedural environment dressing using original geometry and bitmap signage.

Furniture stays within the declared collision footprints; wall details remain
at the boundary. No downloaded textures or new third-party asset licenses.
"""
import math
import struct
import zlib
import xml.etree.ElementTree as ET

THEMES = {
    'small_open': ('Residential courtyard', 'COURTYARD', 'outdoor'),
    'small_clutter': ('Cafe courtyard', 'CAFE', 'outdoor'),
    'medium_open': ('Station entrance hall', 'STATION', 'indoor'),
    'medium_doorway': ('Clinic waiting area', 'CLINIC', 'indoor'),
    'large_pillars': ('Station concourse', 'PLATFORM', 'indoor'),
    'large_office': ('Office lobby', 'OFFICES', 'indoor'),
}
FONT = {
    'X': [17,17,10,4,10,17,17], 'J': [7,2,2,2,18,18,12],
    '3': [30,1,1,14,1,1,30],
    ' ': [0,0,0,0,0,0,0],
    'B': [30,17,17,30,17,17,30], 'G': [14,17,16,23,17,17,14],
    'H': [17,17,17,31,17,17,17], 'K': [17,18,20,24,20,18,17],
    'V': [17,17,17,17,17,10,4], 'W': [17,17,17,21,21,27,17],
    'A': [14,17,17,31,17,17,17], 'C': [14,17,16,16,16,17,14],
    'D': [30,17,17,17,17,17,30], 'E': [31,16,16,30,16,16,31],
    'F': [31,16,16,30,16,16,16], 'I': [31,4,4,4,4,4,31],
    'L': [16,16,16,16,16,16,31], 'M': [17,27,21,21,17,17,17],
    'N': [17,25,25,21,19,19,17], 'O': [14,17,17,17,17,17,14],
    'P': [30,17,17,30,16,16,16], 'R': [30,17,17,30,20,18,17],
    'S': [15,16,16,14,1,1,30], 'T': [31,4,4,4,4,4,4],
    'U': [17,17,17,17,17,17,14], 'Y': [17,17,10,4,4,4,4],
}


def sign_png(text):
    width, height, scale = 512, 96, 8
    pixels = bytearray([18, 53, 66] * width * height)
    left = (width - len(text)*6*scale)//2
    for letter, char in enumerate(text):
        for row, mask in enumerate(FONT[char]):
            for col in range(5):
                if mask & (1 << (4-col)):
                    for dy in range(scale):
                        for dx in range(scale):
                            x, y = left+(letter*6+col)*scale+dx, 20+row*scale+dy
                            offset = (y*width+x)*3
                            pixels[offset:offset+3] = b'\xee\xf1\xdb'
    def chunk(kind, content):
        return struct.pack('>I', len(content)) + kind + content + struct.pack('>I', zlib.crc32(kind+content) & 0xffffffff)
    raw = b''.join(b'\0'+pixels[y*width*3:(y+1)*width*3] for y in range(height))
    return b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width,height,8,2,0,0,0)) + chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b'')


def dress(root, name, definition):
    _, label, setting = THEMES[name]
    x, y = [n/2 for n in definition['size']]
    asset, body = root.find('asset'), root.find('worldbody')
    ET.SubElement(asset, 'texture', name='world_sign', type='2d', file=f'avoidance_sign_{name}.png')
    ET.SubElement(asset, 'material', name='world_sign', texture='world_sign', texuniform='false')
    ET.SubElement(asset, 'texture', name='world_tiles', type='2d', builtin='checker', mark='edge',
                  rgb1='.65 .67 .66' if setting=='outdoor' else '.76 .78 .8',
                  rgb2='.57 .59 .58' if setting=='outdoor' else '.66 .69 .72',
                  markrgb='.4 .43 .45', width='512', height='512')
    ET.SubElement(asset, 'material', name='world_tiles', texture='world_tiles', texuniform='true', texrepeat='2 2')
    body.find("geom[@name='floor']").set('material', 'world_tiles')
    counter = 0
    def geom(kind, pos, size, material=None, rgba=None, **extra):
        nonlocal counter
        counter += 1
        attrs = dict(name=f'detail_{counter}', type=kind, pos=' '.join(map(str,pos)),
                     size=' '.join(map(str,size)), contype='0', conaffinity='0')
        if material:
            attrs['material'] = material
        if rgba:
            attrs['rgba'] = rgba
        attrs.update(extra)
        return ET.SubElement(body, 'geom', **attrs)
    # Shopfront/reception glazing, lintels, doors and wall-mounted signage.
    for side in (-1, 1):
        for index in range(-int(x)+1, int(x), 2):
            geom('box', (index, side*(y-.008), 1.7), (.65,.012,.65), 'blue_window')
            for edge in (-1,1):
                geom('box', (index+edge*.67, side*(y-.03), 1.7), (.025,.025,.69), 'metal_dark')
            geom('box', (index, side*(y-.03), 1.7), (.65,.025,.025), 'metal_dark')
        geom('box', (0, side*(y-.08), 2.85), (x,.16,.08), 'stone')
        geom('box', (side*(x-.015), 0, 1.3), (.025,.58,1.3), 'bench_wood')
        geom('box', (side*(x-.05), .35, 1.1), (.025,.025,.1), 'metal_dark')
    geom('box', (0,y-.05,2.65), (min(2.,x*.55),.04,.3), 'world_sign')
    # Floor guides and threshold strips add readable circulation structure.
    for offset in (-.8,.8):
        geom('box', (offset,0,.003), (.025,y-.15,.003), 'white_marking' if setting=='outdoor' else 'yellow_line')
    for side in (-1,1):
        geom('box', (0,side*(y-.12),.005), (x-.15,.06,.005), 'stone')
    if setting == 'indoor':
        # High beams and strip lights leave the viewer's top-down view open.
        for px in range(-int(x)+1,int(x),3):
            geom('box', (px,0,3.3), (.08,y,.08), 'building_concrete')
            geom('box', (px,0,3.21), (.055,y-.3,.018), 'lamp_glass')
    else:
        for side in (-1,1):
            geom('box', (side*(x-.035),0,2.3), (.025,y,.12), 'brick')
            for py in range(-int(y)+1,int(y),2):
                geom('sphere', (side*(x-.10),py,2.5), (.1,), 'lamp_glass')
    for index, (px,py,sx,sy) in enumerate(definition['boxes']):
        proxy = body.find(f"geom[@name='world_box_{index+4}']")
        if name == 'medium_doorway' or (name == 'large_office' and index < 2):
            proxy.set('material', 'painted_wall')
            proxy.set('pos', f'{px} {py} 1.5')
            proxy.set('size', f'{sx} {sy} 1.5')
            for side in (-1,1):
                geom('box',(px+side*(sx+.012),py,1.65),(.015,max(.1,sy-.2),.6),'blue_window')
                geom('box',(px+side*(sx+.015),py,.25),(.02,sy,.03),'stone')
        elif name == 'large_pillars':
            proxy.set('rgba', '0 0 0 0')
            proxy.attrib.pop('material', None)
            geom('cylinder',(px,py,1.65),(min(sx,sy),1.65),'building_concrete')
            geom('cylinder',(px,py,.12),(min(sx,sy),.12),'metal_dark')
            geom('cylinder',(px,py,1.4),(min(sx,sy)+.002,.09),'sign_blue')
        elif name == 'small_clutter' and index == 2:
            proxy.set('pos',f'{px} {py} .3')
            proxy.set('size',f'{sx} {sy} .3')
            proxy.set('material','curb')
            geom('cylinder',(px,py,1.1),(.09,.65),'tree_bark')
            geom('sphere',(px,py,1.9),(min(sx,sy),),'tree_leaf')
            geom('sphere',(px+.1,py,2.15),(min(sx,sy)*.7,),'tree_leaf_light')
        else:
            # Low benches/tables: visible legs, wooden slats and backing.
            proxy.set('pos',f'{px} {py} .42')
            proxy.set('size',f'{sx} {sy} .42')
            proxy.set('rgba','0 0 0 0')
            proxy.attrib.pop('material',None)
            for a in (-1,1):
                for b in (-1,1):
                    geom('box',(px+a*sx*.75,py+b*sy*.7,.36),(.035,.035,.36),'metal_dark')
            for slat in range(5):
                geom('box',(px,py+(slat-2)*sy*.35,.78),(sx,sy*.15,.04),'bench_wood')
            if name == 'large_office':
                geom('box',(px,py+sy*.85,1.0),(sx,.045,.22),'bench_wood')
            else:
                geom('cylinder',(px,py,.85),(.09,.045),'curb')
    # Opaque building boundary is higher than all doors/glazing details.
    for index in range(4):
        wall=body.find(f"geom[@name='world_box_{index}']")
        position=wall.get('pos').split(); position[2]='1.6'; wall.set('pos',' '.join(position))
        size=wall.get('size').split(); size[2]='1.6'; wall.set('size',' '.join(size))
        wall.set('material','brick' if setting=='outdoor' else 'painted_wall')
