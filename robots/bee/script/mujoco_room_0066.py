"""Import the Gazebo 0066 visual into the Bee MuJoCo scene."""
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import rospkg


def convert_mesh(dae, obj):
    root = ET.parse(dae)
    mesh = root.find('.//{*}geometry/{*}mesh')
    triangles = mesh.find('{*}triangles')
    inputs = {e.get('semantic'): e for e in triangles.findall('{*}input')}
    indices = np.fromstring(triangles.findtext('{*}p'), dtype=np.int32, sep=' ').reshape(-1, 3)
    position = mesh.find('{*}vertices/{*}input').get('source')
    uv = inputs['TEXCOORD'].get('source')

    def values(source, width):
        array = mesh.find(f"{{*}}source[@id='{source[1:]}']/{{*}}float_array")
        return np.fromstring(array.text, sep=' ').reshape(-1, width)

    node = root.find('.//{*}node[{*}instance_geometry]')
    transform = np.fromstring(node.findtext('{*}matrix'), sep=' ').reshape(4, 4)
    xyz = values(position, 3) @ transform[:3, :3].T + transform[:3, 3]
    faces = indices[:, [int(inputs[key].get('offset')) for key in ('VERTEX', 'TEXCOORD')]] + 1
    with obj.open('w') as out:
        np.savetxt(out, xyz, fmt='v %.8g %.8g %.8g')
        np.savetxt(out, values(uv, 2), fmt='vt %.8g %.8g')
        np.savetxt(out, faces.reshape(-1, 6), fmt='f %d/%d %d/%d %d/%d')


def add_room(tree, output_dir):
    room = Path(rospkg.RosPack().get_path('aerial_robot_simulation')) / 'gazebo_model/models/0066'
    visual = ET.parse(room / 'model.sdf').find('.//visual')
    dae = room / visual.findtext('geometry/mesh/uri').split('model://0066/')[1]
    jpg = dae.with_name('texture.jpg')
    cache = Path(output_dir).resolve()
    obj, png = cache / '0066.obj', cache / '0066.png'
    with TemporaryDirectory(dir=cache) as temp:
        if not obj.exists() or obj.stat().st_mtime < max(dae.stat().st_mtime, Path(__file__).stat().st_mtime):
            tmp = Path(temp) / obj.name
            convert_mesh(dae, tmp)
            tmp.replace(obj)
        if not png.exists() or png.stat().st_mtime < jpg.stat().st_mtime:
            tmp = Path(temp) / png.name
            with Image.open(jpg) as image:
                image.thumbnail((4096, 4096), Image.LANCZOS)
                image.save(tmp)
            tmp.replace(png)
    asset = tree.find('asset')
    ET.SubElement(asset, 'mesh', name='room0066', file=str(obj))
    ET.SubElement(asset, 'texture', name='room0066', type='2d', file=str(png))
    ET.SubElement(asset, 'material', name='room0066', texture='room0066', emission='0.5')
    pose = visual.findtext('pose').split()
    ET.SubElement(tree.find('worldbody'), 'geom', name='room0066', type='mesh',
                  mesh='room0066', material='room0066', pos=' '.join(pose[:3]),
                  euler=' '.join(pose[3:]), contype='0', conaffinity='0', group='2')
