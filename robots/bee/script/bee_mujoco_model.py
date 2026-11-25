#!/usr/bin/env python3
"""Build the shared Bee meshes once and select each vehicle's MJCF visuals."""
import argparse
import fcntl
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import uuid
import xml.etree.ElementTree as ET

import rospkg
import yaml


CAD_FILES = ('base_link.dae', 'foot_ball.dae', 'livox_module.dae', 'rotor_module.dae', 'carbon_base.png', 'carbon_livox.jpg')


def vehicle_colors(bee, robot_id):
    colors = yaml.safe_load((Path(bee) / 'config/body_colors.yaml').read_text())
    blades = colors['propeller']
    return {
        'body_trim': colors.get(robot_id, colors['default']),
        'propeller': blades.get(robot_id, blades['default']),
    }


def set_vehicle_colors(root, prefix, colors):
    for mesh, rgba in colors.items():
        geoms = root.findall(f".//geom[@mesh='{prefix}{mesh}']")
        expected = 4 if mesh == 'propeller' else 1
        if len(geoms) != expected:
            raise ValueError(f'Expected {expected} {prefix}{mesh} geoms, found {len(geoms)}')
        for geom in geoms:
            geom.attrib.pop('material', None)
            geom.set('rgba', rgba)


def validate_shared(tree, config):
    meshes = {geom.get('mesh', '') for geom in tree.findall('.//geom')}
    for stem in ('base_link', 'livox_module', 'rotor_module', 'body_trim'):
        if not any(name == stem or name.startswith(stem + '_') for name in meshes):
            raise ValueError('Missing MuJoCo mesh ' + stem)
    if len(tree.findall(".//geom[@mesh='propeller']")) != 4:
        raise ValueError('Expected four MuJoCo propeller geoms')
    feet = config.get('compliant_feet', {})
    for foot in feet.get('feet', []):
        name = "{}_{}_ball".format(feet.get('name_prefix', 'spring_foot'), foot['name'])
        if tree.find(f".//geom[@name='{name}']") is None:
            raise ValueError('Missing MuJoCo contact geom ' + name)


def attach_foot_visuals(tree, feet):
    base = tree.find(".//body[@name='base_link']")
    visuals = base.findall("geom[@mesh='foot_ball']")
    if len(visuals) != len(feet):
        raise ValueError('Expected one CAD ball per compliant foot')
    for foot in feet:
        name = 'spring_foot_' + foot['name']
        body = base.find(f"body[@name='{name}']")
        ball = body.find(f"geom[@name='{name}_ball']")
        center = [a + float(b) for a, b in zip(foot['pos'], ball.get('pos').split())]
        matches = [v for v in visuals if all(abs(a - float(b)) < 1e-5
                   for a, b in zip(center, v.get('pos').split()))]
        if len(matches) != 1:
            raise ValueError(f'CAD and collision ball centers disagree for {name}')
        visual = matches[0]
        base.remove(visual)
        visual.attrib.update(name=name + '_visual', pos=ball.get('pos'),
                             contype='0', conaffinity='0', mass='0')
        body.append(visual)


def build_shared(bee, backend, output):
    import trimesh
    # The upstream generator writes into its input package and clears its
    # mujoco directory. Give it a private build package, including scratch meshes.
    with tempfile.TemporaryDirectory(prefix='bee-cad-') as tmp:
        pkg = Path(tmp) / 'bee_model_build'
        meshes = pkg / 'urdf/mesh'
        meshes.mkdir(parents=True)
        manifest = ET.parse(bee / 'package.xml')
        manifest.find('name').text = pkg.name
        manifest.write(pkg / 'package.xml')
        config = yaml.safe_load((bee / 'config/mujoco/mujoco_model.yaml').read_text())['bee']
        (pkg / 'input.xacro').write_text(
            '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="bee">'
            '<xacro:arg name="livox" default="true"/>'
            f'<xacro:include filename="$(find bee)/{config["input"][0]}"/></robot>')
        config.update(input=['input.xacro'], filename=['bee'])
        (pkg / 'build.yaml').write_text(yaml.safe_dump({'package': [pkg.name], pkg.name: config}))
        for asset in CAD_FILES:
            (meshes / asset).symlink_to(bee / 'urdf/mesh' / asset)
        for name in ('body_trim', 'propeller'):
            trimesh.load_mesh(bee / 'urdf/mesh' / (name + '.stl')).export(meshes / (name + '.obj'))
        env = dict(os.environ, ROS_PACKAGE_PATH=tmp + os.pathsep + os.environ['ROS_PACKAGE_PATH'])
        subprocess.run([sys.executable, str(backend / 'scripts/mujoco_model_generator.py'),
                        str(pkg / 'build.yaml')], env=env, stdout=sys.stderr, check=True)
        build = pkg / 'mujoco/bee'
        tree = ET.parse(build / 'robot.xml')
        validate_shared(tree, config)
        attach_foot_visuals(tree, config['compliant_feet']['feet'])
        shutil.copyfile(backend / 'config/world.xml', build / 'world.xml')
        tree.find('include').set('file', 'world.xml')
        for geom in tree.findall('.//geom'):
            if geom.get('mesh', '').startswith('livox_module'):
                geom.set('contype', '0')
                geom.set('conaffinity', '0')
                geom.set('mass', '0')
        tree.write(build / 'robot.xml')
        shutil.copytree(build, output)


def robot_model(bee, backend, robot_id=1, livox=False, regenerate=False, force_test_wall=False):
    bee, backend = Path(bee), Path(backend)
    inputs = [Path(__file__), bee / 'robots/bee.urdf.xacro', bee / 'robots/bee_sim.urdf.xacro',
              bee / 'config/mujoco/mujoco_model.yaml', bee / 'config/contact_point.yaml',
              backend / 'scripts/mujoco_model_generator.py', backend / 'scripts/convert.py',
              backend / 'config/world.xml']
    inputs += sorted((bee / 'urdf').glob('*.xacro'))
    inputs += [bee / 'urdf/mesh' / name for name in (*CAD_FILES, 'body_trim.stl', 'propeller.stl')]
    signature = [(str(p.resolve()), p.stat().st_mtime_ns, p.stat().st_size) for p in inputs]
    cache = Path(rospkg.get_ros_home()) / 'bee_models'
    cache.mkdir(parents=True, exist_ok=True)
    shared = cache / hashlib.sha256(repr(signature).encode()).hexdigest()[:20]
    if regenerate:
        shared = cache / (shared.name + '-' + uuid.uuid4().hex)
    with (cache / '.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if not shared.is_dir():
            with tempfile.TemporaryDirectory(dir=cache) as stage:
                generated = Path(stage) / 'model'
                build_shared(bee, backend, generated)
                generated.rename(shared)
        colors = vehicle_colors(bee, robot_id)
        color_key = hashlib.sha256('|'.join(colors.values()).encode()).hexdigest()[:10]
        wall_suffix = '-wall' if force_test_wall else ''
        variant = shared / f'bee{robot_id}-livox{int(livox)}-{color_key}{wall_suffix}.xml'
        if not variant.is_file():
            tree = ET.parse(shared / 'robot.xml')
            if not livox:
                for parent in tree.iter():
                    for child in list(parent):
                        if child.get('mesh', '').startswith('livox_module') or (
                                parent.tag == 'asset' and 'livox_module' in child.get('name', '')):
                            parent.remove(child)
            set_vehicle_colors(tree, '', colors)
            if force_test_wall:
                ET.SubElement(tree.find('worldbody'), 'geom', name='force_test_wall', type='box',
                              pos='-0.775 0 0.75', size='0.025 0.6 0.75', friction='1 0.02 0.001',
                              rgba='0.25 0.52 0.65 1', contype='1', conaffinity='1')
            staging = variant.with_suffix('.tmp')
            tree.write(staging)
            staging.replace(variant)
    return str(variant)


if __name__ == '__main__':
    cli = argparse.ArgumentParser()
    cli.add_argument('robot_id', type=int)
    cli.add_argument('livox', type=lambda s: s.lower() in ('true', '1'))
    cli.add_argument('--force-test-wall', action='store_true')
    args = cli.parse_args()
    packages = rospkg.RosPack()
    print(robot_model(packages.get_path('bee'), packages.get_path('mujoco_ros_control'),
                      args.robot_id, args.livox, force_test_wall=args.force_test_wall), end='')
