"""CAD payload in body axes at the centre of mass (SI units)."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import rospkg
import yaml
from scipy.spatial.transform import Rotation


def mesh_path(resource):
    if resource.startswith('package://'):
        package, rel = resource[10:].split('/', 1)
        path = Path(rospkg.RosPack().get_path(package)) / rel
    else:
        path = Path(resource)
    if not path.is_absolute() or not path.is_file() or path.suffix.lower() not in ('.obj', '.stl'):
        raise ValueError('Expected an existing package:// or absolute OBJ / binary STL: ' + resource)
    return str(path)


def vector(value, count):
    v = np.asarray(value, dtype=float)
    if v.shape != (count,) or not np.all(np.isfinite(v)):
        raise ValueError('Expected {} finite numbers: {}'.format(count, value))
    return v


def load_model(file, scene):
    with open(file) as src:
        model = yaml.safe_load(src)
    if model['object_shape'] != 'triangular_prism' or model['robot_names'] != ['bee1', 'bee2', 'bee3'] or scene.get('robot_count', 3) != 3:
        raise ValueError('CAD model requires a three-Bee triangular prism')
    keys = ('object_shape', 'object_mass', 'object_inertia', 'object_height', 'face_angles', 'face_radii',
            'object_bottom_offset', 'contact_height', 'marker_to_model_xyz', 'marker_to_model_rpy',
            'object_mesh', 'collision_meshes', 'mesh_scale', 'mesh_xyz', 'mesh_rpy')
    scene.update({k: model[k] for k in keys if k in model})
    scene['grasp_staging_clearance'] = model['staging_clearance']
    for name in ('object_mass', 'object_height', 'staging_clearance'):
        if not np.isfinite(model[name]) or model[name] <= 0:
            raise ValueError('Expected positive ' + name)
    vector(model['face_angles'], 3)
    if min(vector(model['face_radii'], 3)) <= 0:
        raise ValueError('Expected positive face radii')
    bottom = model.get('object_bottom_offset', model['object_height']/2)
    if not 0 < bottom <= model['object_height']:
        raise ValueError('Invalid object_bottom_offset')
    I = vector(model['object_inertia'], 9).reshape(3, 3)
    moments = np.linalg.eigvalsh(I)
    if not np.allclose(I, I.T, atol=1e-12, rtol=1e-9) or moments[0] <= 0 or moments[2] > moments[0]+moments[1]+1e-12:
        raise ValueError('Invalid inertia tensor about the object centre of mass')
    if not model.get('object_mesh'):
        raise ValueError('Set object_mesh in the CAD model YAML')
    for key in ('mesh_xyz', 'mesh_rpy', 'marker_to_model_xyz', 'marker_to_model_rpy'):
        vector(model.get(key, [0]*3), 3)
    if min(vector(model.get('mesh_scale', [1]*3), 3)) <= 0:
        raise ValueError('Expected positive mesh_scale')


def apply_mesh(tree, model):
    body = tree.find(".//body[@name='grasp_prism']")
    asset = tree.find('asset')
    old = body.findall('geom')
    geom = old[0].attrib.copy()
    for g in old:
        body.remove(g)
        asset.remove(asset.find("mesh[@name='{}']".format(g.get('mesh'))))
    def numbers(values):
        return ' '.join(str(float(x)) for x in values)
    I = np.asarray(model['object_inertia']).reshape(3, 3)
    inertial = body.find('inertial')
    inertial.attrib.clear()
    inertial.attrib.update(pos='0 0 0', mass=str(model['object_mass']),
                           fullinertia=numbers([*I.diagonal(), I[0,1], I[0,2], I[1,2]]))
    quat = Rotation.from_euler('xyz', model.get('mesh_rpy', [0]*3)).as_quat()
    geom.update(pos=numbers(model.get('mesh_xyz', [0]*3)), quat=numbers(np.roll(quat,1)))
    parts = model.get('collision_meshes', [])
    if not isinstance(parts, list):
        raise ValueError('collision_meshes must be a list of mesh paths')
    for i, resource in enumerate([model['object_mesh']] + parts):
        name = 'grasp_cad_' + str(i)
        ET.SubElement(asset, 'mesh', name=name, file=mesh_path(resource), scale=numbers(model.get('mesh_scale', [1]*3)))
        attributes = dict(geom, mesh=name, name='grasp_prism_geom' if i == 0 else name+'_collision')
        if parts and i == 0:
            attributes.update(contype='0', conaffinity='0')
        if i:
            attributes.update(rgba='0 0 0 0')
        ET.SubElement(body, 'geom', attributes)
