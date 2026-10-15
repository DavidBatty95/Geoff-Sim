#!/usr/bin/env python3
"""Transport source snapshots and actual runtime identities (no environment dump)."""
import hashlib
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import zipfile

EXCLUDED = {'.git','.venv','__pycache__','.pytest_cache','build','install','log','logs','results'}
SOURCE_SUFFIXES = {'.py','.sh','.cc','.cpp','.h','.hpp','.xml','.xacro','.urdf',
                   '.yaml','.yml','.json','.toml','.cfg','.ini','.cmake','.txt','.patch'}


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            value.update(block)
    return value.hexdigest()


def inventory(root,locations,excluded=EXCLUDED):
    files = {}
    for location in locations:
        path = root/location
        candidates = path.rglob('*') if path.is_dir() else [path]
        for candidate in candidates:
            relative = candidate.relative_to(root)
            if (not candidate.is_file() or any(part in excluded for part in relative.parts) or
                    candidate.name.startswith('.env') or not candidate.resolve().is_relative_to(root.resolve())):
                continue
            files[str(relative)] = digest(candidate)
    return dict(sorted(files.items()))


def command(args,cwd=None):
    result = subprocess.run(args,cwd=cwd,capture_output=True,text=True,timeout=30)
    return dict(returncode=result.returncode,stdout=result.stdout.strip(),stderr=result.stderr.strip())


def capture_source(root,run,policy=None):
    locations = ['scripts','patches','ros2_ws/src','docker','docker-compose.yml']
    if policy is not None:
        locations.append(str(policy.relative_to(root)))
    files = inventory(root,locations)
    archive = run/'source_snapshot.zip'
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED) as snapshot:
        for name in files:
            path = root/name
            if path.suffix in SOURCE_SUFFIXES or path.name == 'CMakeLists.txt':
                snapshot.write(path,name)
    metadata = dict(schema_version=1,source_sha256=files,
        source_archive_sha256=digest(archive),
        archive_scope='source/config/model text; binary assets and policies are hashed separately',
        git_head=command(['git','rev-parse','HEAD'],root),
        git_status=command(['git','status','--short'],root),
        submodules=command(['git','submodule','status','--recursive'],root),
        image=command(['docker','image','inspect','unitree-g1-ros2-mujoco:humble','--format','{{.Id}}'],root))
    (run/'source_provenance.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return metadata


def capture_runtime(root,output):
    """Run inside the sourced launch container; record what Python really imports."""
    packages = {}
    for name in ('unitree_g1_mujoco_bridge','dynamic_obstacle_benchmarks','g1_msgs',
                 'unitree_social_navigation_msgs'):
        module = importlib.import_module(name)
        directory = Path(module.__file__).parent
        packages[name] = dict(path=str(directory),files={
            str(path.relative_to(directory)):digest(path) for path in sorted(directory.rglob('*.py'))})
    artifacts = inventory(root,['ros2_ws/.verification/install/share/unitree_g1_sim_bringup',
        'ros2_ws/src/unitree_mujoco/simulate/build/unitree_mujoco'],excluded={'.git','__pycache__'})
    # Build directories are deliberately excluded from source inventories.
    binary = root/'ros2_ws/src/unitree_mujoco/simulate/build/unitree_mujoco'
    if binary.is_file():
        artifacts[str(binary.relative_to(root))] = digest(binary)
    result = dict(schema_version=1,python=sys.version,platform=platform.platform(),packages=packages,
        numeric_thread_limits={name:os.environ.get(name) for name in
            ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS')},
        artifacts_sha256=artifacts,dependencies={dist.metadata['Name']:dist.version
            for dist in importlib.metadata.distributions() if dist.metadata['Name']})
    source = json.loads((output.parent/'source_provenance.json').read_text())
    result['python_source_alignment'] = compare_python_sources(source['source_sha256'],packages)
    output.write_text(json.dumps(result,indent=2)+'\n')
    if not result['python_source_alignment']['matched']:
        raise RuntimeError('installed Python differs from captured source; rebuild before transport')


def compare_python_sources(source,packages):
    mismatches = []
    checked = 0
    # Generated ROS message modules do not have one-to-one source .py files.
    for package in ('unitree_g1_mujoco_bridge','dynamic_obstacle_benchmarks'):
        prefix = f'ros2_ws/src/{package}/{package}/'
        expected = {key[len(prefix):]:value for key,value in source.items()
                    if key.startswith(prefix) and key.endswith('.py')}
        actual = packages[package]['files']
        for name in sorted(expected.keys() | actual.keys()):
            checked += 1
            if expected.get(name) != actual.get(name):
                mismatches.append(f'{package}/{name}')
    return dict(matched=not mismatches and checked>0,checked_files=checked,mismatches=mismatches,
                scope='Python autonomy packages only; native binary identity recorded separately')


if __name__ == '__main__':
    capture_runtime(Path('/workspace'),Path(sys.argv[1]))
