#!/usr/bin/env python3
"""Install pinned official Unitree Dex3 geometry and source descriptions."""
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'ros2_ws/src/unitree_g1_description'
REVISION = '5994d4faef0a9cadd3287f8de0199a67eeb2a259'
BASE = f'https://raw.githubusercontent.com/unitreerobotics/unitree_ros/{REVISION}/'


def main():
    sources = PACKAGE/'dex3_sources'
    meshes = PACKAGE/'meshes/dex3'
    manifest_path = sources/'manifest.json'
    if manifest_path.is_file():
        existing = json.loads(manifest_path.read_text())
        if existing.get('revision')==REVISION and all((PACKAGE/name).is_file() and
                hashlib.sha256((PACKAGE/name).read_bytes()).hexdigest()==entry['sha256']
                for name,entry in existing['files'].items()):
            print('Pinned Dex3 assets verified')
            return
    sources.mkdir(parents=True,exist_ok=True)
    meshes.mkdir(parents=True,exist_ok=True)
    files = {}
    def fetch(remote,destination):
        payload = urlopen(BASE+remote,timeout=60).read()
        if destination.exists() and destination.read_bytes()!=payload:
            raise ValueError(f'refusing to replace changed upstream asset: {destination}')
        if not destination.exists():
            destination.write_bytes(payload)
        files[str(destination.relative_to(PACKAGE))] = dict(source=remote,sha256=hashlib.sha256(payload).hexdigest())
        return payload
    model = 'robots/g1_description/g1_29dof_with_hand_rev_1_0'
    xml = fetch(model+'.xml',sources/'g1_29dof_with_hand_rev_1_0.xml')
    fetch(model+'.urdf',sources/'g1_29dof_with_hand_rev_1_0.urdf')
    fetch('LICENSE',sources/'LICENSE')
    for mesh in ET.fromstring(xml).findall('asset/mesh'):
        if '_hand_' in mesh.get('name',''):
            filename = mesh.get('file')
            fetch('robots/g1_description/meshes/'+filename,meshes/filename)
            print(filename,flush=True)
    (sources/'manifest.json').write_text(json.dumps(dict(repository='unitreerobotics/unitree_ros',
        revision=REVISION,files=files),indent=2)+'\n')


if __name__=='__main__':
    main()
