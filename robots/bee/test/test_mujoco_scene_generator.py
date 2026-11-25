#!/usr/bin/env python3
"""Offline geometry/manifest tests: no ROS master, simulator or flight output."""
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import patch
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'script'))
from mujoco_scene_generator import MujocoSceneGenerator
from grasp_geometry import extra_shape_geometry, surface_triangles


class SceneGeneratorTest(unittest.TestCase):
    def test_measured_triangle_uses_mesh_parts_and_full_tensor(self):
        import xml.etree.ElementTree as ET
        from grasp_cad import apply_mesh, load_model
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            mesh = folder / 'payload.obj'
            mesh.touch()
            model = yaml.safe_load((Path(__file__).resolve().parents[1] /
                                    'config/grasp/object_triangle_example.yaml').read_text())
            model.update(object_mesh=str(mesh), collision_meshes=[str(mesh)], mesh_scale=[.001]*3,
                         mesh_xyz=[.01, .02, .03], object_mass=.81, face_radii=[.14,.16,.18],
                         object_inertia=[.03,.001,0,.001,.028,.0005,0,.0005,.026])
            file = folder/'model.yaml'
            file.write_text(yaml.safe_dump(model))
            params = {}
            load_model(file, params)
            xml = ET.fromstring('<mujoco><asset><mesh name="old"/></asset><worldbody>'
                                '<body name="grasp_prism"><inertial/><geom mesh="old" type="mesh"/>'
                                '</body></worldbody></mujoco>')
            apply_mesh(xml, params)
            body = xml.find('.//body')
            self.assertEqual(float(body.find('inertial').get('mass')), .81)
            self.assertEqual(list(map(float,body.find('inertial').get('fullinertia').split())),
                             [.03,.028,.026,.001,0,.0005])
            visual, collision = body.findall('geom')
            self.assertEqual(visual.get('contype'), '0')
            self.assertIsNone(collision.get('contype'))
            self.assertEqual(collision.get('pos'), '0.01 0.02 0.03')
            self.assertEqual(params['face_radii'], model['face_radii'])
            self.assertEqual(xml.find('asset/mesh').get('scale'), '0.001 0.001 0.001')
            model['object_inertia'][0] = 1
            file.write_text(yaml.safe_dump(model))
            with self.assertRaises(ValueError):
                load_model(file, {})

    def test_rock_mesh_mass_and_contact_faces(self):
        import numpy as np
        from collections import Counter
        g = extra_shape_geometry(dict(object_shape='rock'))
        vertices, normals = np.array(g['vertices']), np.array(g['normals'])
        offsets = np.array(g['offsets'])
        self.assertLess(np.max(vertices @ normals.T - offsets), 1e-8)
        edges = Counter(tuple(sorted((face[i], face[(i+1) % 3]))) for face in g['faces'] for i in range(3))
        self.assertTrue(all(count == 2 for count in edges.values()))
        self.assertEqual(len(vertices) - len(edges) + len(g['faces']), 2)
        volume, first = 0., np.zeros(3)
        for face in g['faces']:
            a, b, c = vertices[face]
            dv = np.dot(a, np.cross(b, c)) / 6
            self.assertGreater(dv, 0)
            volume += dv
            first += dv * (a+b+c) / 4
        self.assertAlmostEqual(volume, g['volume'])
        self.assertLess(np.linalg.norm(first), 1e-12)
        self.assertAlmostEqual(np.ptp(vertices[:, 2]), .6)
        for i, reach in enumerate(g['radii']):
            p = np.array([reach * math.cos(i*math.pi/2), reach * math.sin(i*math.pi/2), 0])
            self.assertAlmostEqual(normals[i] @ p, offsets[i])
            self.assertGreater(np.min(np.delete(offsets - normals @ p, i)), .08)
        # Independent volume integration using a fixed Monte Carlo seed.
        lo, hi = vertices.min(axis=0), vertices.max(axis=0)
        samples = np.random.default_rng(61113).uniform(lo, hi, (200000, 3))
        inside = samples[np.all(samples @ normals.T <= offsets, axis=1)]
        self.assertAlmostEqual(len(inside)/len(samples)*np.prod(hi-lo), volume, delta=.002)
        moment = inside.T @ inside / len(inside)
        inertia = np.trace(moment)*np.eye(3)-moment
        expected = np.array(g['unit_inertia'])
        measured = np.array([inertia[i,j] for i,j in ((0,0),(1,1),(2,2),(0,1),(0,2),(1,2))])
        self.assertTrue(np.allclose(measured, expected, atol=.0005))

    def test_rock_manifest_v3_and_invalid_contact(self):
        generator = self.builder(object_shape='rock', object_mass=.5, spawn_destination_pedestal=True)
        objects = generator.build_grasp_objects()
        m = generator.control_manifest(generator.build_robot_layout(4), objects)
        self.assertEqual(m['schema_version'], 3)
        self.assertEqual(len(m['contact_normals']), 12)
        self.assertEqual(len(m['surface_vertices']), 3*len(objects[-1]['vertices']))
        self.assertEqual(len(objects[-1]['unit_inertia']), 6)
        self.assertAlmostEqual(m['object_position'][2], .65 + m['object_bottom_offset'])
        for contact in (-.5, .5, float('nan')):
            with self.assertRaises(RuntimeError):
                extra_shape_geometry(dict(object_shape='rock', contact_height=contact))

    def builder(self, **params):
        return MujocoSceneGenerator(params, '/unused/bee', '/unused/mujoco')

    def test_density_mass_is_resolved_without_mutating_input(self):
        params = {'object_mass': -1.0, 'object_density': 10.4166666667}
        generator = MujocoSceneGenerator(params, '/unused/bee', '/unused/mujoco')
        objects = generator.build_grasp_objects()
        self.assertAlmostEqual(generator.resolved_mass, .5, places=8)
        self.assertEqual(params['object_mass'], -1.0)
        manifest = generator.control_manifest(generator.build_robot_layout(3), objects)
        self.assertAlmostEqual(manifest['object_mass'], .5, places=8)
        self.assertAlmostEqual(manifest['object_position'][2], .95)

    def test_shape_layout_and_destination_contract(self):
        for shape, count in [('triangular_prism', 3), ('box', 4), ('pentagonal_prism', 5), ('sphere', 4), ('ellipsoid', 4)]:
            generator = self.builder(object_shape=shape, object_yaw=.4, object_mass=.8,
                                     spawn_destination_pedestal=True,
                                     destination_pedestal_height=.7, object_ground_clearance=.002)
            robots = generator.build_robot_layout(count)
            objects = generator.build_grasp_objects()
            manifest = generator.control_manifest(robots, objects)
            self.assertTrue(manifest['available'])
            self.assertEqual(manifest['schema_version'], 1)
            self.assertEqual(len(manifest['face_angles']), count)
            self.assertEqual(manifest['object_mass'], .8)
            self.assertAlmostEqual(manifest['destination_position'][2], 1.002)
            for robot, angle, radius in zip(robots, manifest['face_angles'], manifest['face_radii']):
                self.assertAlmostEqual(robot['pos'][0], (radius + .8) * math.cos(angle + .4))
                self.assertAlmostEqual(robot['pos'][1], 1 + (radius + .8) * math.sin(angle + .4))

    def test_transport_distance_in_shipped_scenes(self):
        scenes = Path(__file__).resolve().parents[1] / 'config' / 'mujoco'
        for path in sorted(scenes.glob('scene_*.yaml')):
            with self.subTest(scene=path.name):
                params = yaml.safe_load(path.read_text())
                generator = self.builder(**params)
                manifest = generator.control_manifest(generator.build_robot_layout(params['robot_count']),
                                                      generator.build_grasp_objects())
                self.assertTrue(manifest['has_destination'])
                distance = math.dist(manifest['object_position'][:2], manifest['destination_position'][:2])
                if path.stem == 'scene_triangle':
                    self.assertEqual(manifest['object_position'][:2], [-1.093, -0.282])
                    self.assertEqual(manifest['destination_position'][:2], [0.680, -0.234])
                    self.assertEqual(params['object_pedestal_height'], params['destination_pedestal_height'])
                else:
                    self.assertAlmostEqual(distance, 1.5)

    def test_disabled_payload_or_extra_robots_unavailable_to_controller(self):
        generator = self.builder(spawn_object=False)
        self.assertEqual(generator.build_grasp_objects(), [])
        self.assertFalse(generator.control_manifest(generator.build_robot_layout(3), [])['available'])
        generator = self.builder()
        self.assertFalse(generator.control_manifest(generator.build_robot_layout(4), generator.build_grasp_objects())['available'])

    def test_grid_layout_still_supported(self):
        generator = self.builder(grasp_staging_layout=False, grid_cols=2)
        robots = generator.build_robot_layout(4)
        self.assertEqual([r['pos'][:2] for r in robots], [[-2., 0.], [-1., 0.], [-2., 1.], [-1., 1.]])

    def test_incomplete_generated_geometry_is_rejected_before_caching(self):
        from bee_mujoco_model import validate_shared
        tree = ET.fromstring('<mujoco/>')
        for mesh in ('base_link_0', 'rotor_module_1', 'livox_module_0', 'body_trim'):
            ET.SubElement(tree, 'geom', mesh=mesh)
        for _ in range(4):
            ET.SubElement(tree, 'geom', mesh='propeller')
        config = {'compliant_feet': {'feet': [{'name': 'front'}]}}
        with self.assertRaisesRegex(ValueError, 'spring_foot_front_ball'):
            validate_shared(tree, config)
        ET.SubElement(tree, 'geom', name='spring_foot_front_ball')
        validate_shared(tree, config)
        tree.remove(tree.find("geom[@mesh='livox_module_0']"))
        with self.assertRaisesRegex(ValueError, 'livox_module'):
            validate_shared(tree, config)

    def test_model_and_manifest_written_to_requested_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            generator = self.builder(robot_count=5, object_shape='pentagonal_prism')
            generator.output_dir = directory
            generator.bee_path = str(Path(__file__).resolve().parents[1])
            def write_model(command):
                config = yaml.safe_load(Path(command[-1]).read_text())
                scene = ET.Element('mujoco')
                world = ET.SubElement(scene, 'worldbody')
                for robot in config['robots']:
                    body = ET.SubElement(world, 'body', name=robot['name'])
                    ET.SubElement(body, 'geom', mesh=robot['name'] + '_body_trim')
                    for _ in range(4):
                        ET.SubElement(body, 'geom', mesh=robot['name'] + '_propeller')
                payload = ET.SubElement(world, 'body', name='grasp_prism')
                ET.SubElement(payload, 'inertial', mass='0.5', diaginertia='0.02 0.03 0.04')
                ET.ElementTree(scene).write(config['output_model'])
            with patch.object(generator, 'ensure_single_robot_model', return_value='source.xml'), patch('mujoco_scene_generator.subprocess.check_call', side_effect=write_model) as composer:
                robots, model, manifest, manifest_path = generator.generate()
            scene = ET.parse(model)
            for index, color in enumerate(['1 1 0 1', '1 0 0 1', '0 0 0 1', '0 0 0 1', '0 0 0 1'], start=1):
                geoms = scene.findall(f".//geom[@mesh='bee{index}_propeller']")
                self.assertEqual([g.get('rgba') for g in geoms], [color] * 4)
            colors = [g.get('rgba') for g in scene.findall('.//geom') if g.get('mesh').endswith('_body_trim')]
            self.assertEqual(colors[1:3], ['1 1 1 1', '0.035 0.075 0.22 1'])
            self.assertEqual(colors[0], colors[3])
            self.assertEqual(manifest['object_inertia'], [.02, 0., 0., 0., .03, 0., 0., 0., .04])
            self.assertEqual(len(robots), 5)
            self.assertTrue(model.startswith(directory))
            self.assertEqual(yaml.safe_load(Path(manifest_path).read_text()), manifest)
            config_path = composer.call_args.args[0][-1]
            config = yaml.safe_load(Path(config_path).read_text())
            self.assertEqual(config['output_model'], model)
            self.assertEqual(config['jacobian'], 'dense')
            self.assertEqual(config['option']['impratio'], 500.)

    def test_invalid_shape_and_dimensions_rejected(self):
        for params in ({'object_shape': 'unknown'}, {'object_height': -1},
                       {'object_shape': 'sphere', 'object_size_x': .7},
                       {'object_inner_triangle_side': .7}, {'object_sliding_friction': -1}):
            with self.assertRaises(RuntimeError):
                self.builder(**params).build_grasp_objects()

    def test_ellipsoid_manifest_and_mass(self):
        generator = self.builder(object_shape='ellipsoid', object_size_x=.7, object_size_y=.5,
                                 object_height=.6, object_density=10, contact_height=-.025)
        objects = generator.build_grasp_objects()
        m = generator.control_manifest(generator.build_robot_layout(4), objects)
        self.assertEqual(m['surface_radii'], [.35, .25, .3])
        self.assertEqual(m['contact_height'], -.025)
        self.assertAlmostEqual(m['object_mass'], 10 * math.pi * .7 * .5 * .6 / 6)

    def test_new_profiles_geometry_layout_and_com_height(self):
        for shape in ('cylinder', 'frustum', 'capsule'):
            path = Path(__file__).resolve().parents[1] / 'config/mujoco' / ('scene_' + shape + '.yaml')
            params = yaml.safe_load(path.read_text())
            params['object_yaw'] = .37
            generator = self.builder(**params)
            g = extra_shape_geometry(params)
            robots = generator.build_robot_layout(4)
            objects = generator.build_grasp_objects()
            payload = objects[-1]
            manifest = generator.control_manifest(robots, objects)
            self.assertTrue(payload['free'])
            self.assertEqual(manifest['schema_version'], 2)
            self.assertEqual(payload['size'], g['size'])
            self.assertEqual(manifest['surface_radius'], g['radius'])
            self.assertEqual(manifest['surface_length'], g['length'])
            self.assertEqual(manifest['face_radii'], g['radii'])
            self.assertAlmostEqual(manifest['object_position'][2], .65 + g['bottom'])
            self.assertAlmostEqual(manifest['destination_position'][2], .65 + g['bottom'])
            for robot, angle, radius in zip(robots, manifest['face_angles'], manifest['face_radii']):
                self.assertAlmostEqual(robot['pos'][0], (radius + .8) * math.cos(angle + .37))
                self.assertAlmostEqual(robot['pos'][1], 1 + (radius + .8) * math.sin(angle + .37))
            if shape == 'capsule':
                self.assertAlmostEqual(g['radii'][0] - g['radii'][1], .2)
            if shape == 'frustum':
                self.assertLess(g['bottom'], params['object_height'] / 2)

    def test_new_shape_density_and_bad_dimensions(self):
        for shape in ('cylinder', 'frustum', 'capsule'):
            g = extra_shape_geometry(dict(object_shape=shape))
            generator = self.builder(object_shape=shape, object_density=10)
            generator.build_grasp_objects()
            self.assertAlmostEqual(generator.resolved_mass, 10 * g['volume'])
        for params in (dict(object_shape='cylinder', object_radius=0),
                       dict(object_shape='frustum', object_radius_top=-.1),
                       dict(object_shape='frustum', contact_height=-.29),
                       dict(object_shape='capsule', object_length=.5),
                       dict(object_shape='capsule', object_height=.5),
                       dict(object_shape='cylinder', object_radius=float('nan'))):
            with self.assertRaises(RuntimeError):
                self.builder(**params).build_grasp_objects()

    def test_rendered_mesh_bounds_match_physics(self):
        for shape in ('frustum', 'capsule'):
            g = extra_shape_geometry(dict(object_shape=shape))
            vertices = surface_triangles(g)
            self.assertEqual(len(vertices) % 3, 0)
            self.assertAlmostEqual(min(p[2] for p in vertices), -g['bottom'])
            self.assertAlmostEqual(max(p[2] for p in vertices), g['height'] - g['bottom'])
            half_x = (g['length'] / 2 + g['radius']) if shape == 'capsule' else max(g['radius'], g['top_radius'])
            self.assertAlmostEqual(max(p[0] for p in vertices), half_x)
            self.assertTrue(all(math.isfinite(c) for v in vertices for c in v))


if __name__ == '__main__':
    unittest.main()
