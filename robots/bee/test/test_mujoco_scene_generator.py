#!/usr/bin/env python3
"""Offline geometry/manifest tests: no ROS master, simulator or flight output."""
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'script'))
from mujoco_scene_generator import MujocoSceneGenerator


class SceneGeneratorTest(unittest.TestCase):
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
        for shape, count in [('triangular_prism', 3), ('box', 4), ('pentagonal_prism', 5)]:
            generator = self.builder(object_shape=shape, object_yaw=.4, object_mass=.8,
                                     spawn_destination_pedestal=True,
                                     destination_pedestal_height=.7, object_ground_clearance=.002)
            robots = generator.build_robot_layout(count)
            objects = generator.build_grasp_objects()
            manifest = generator.control_manifest(robots, objects)
            self.assertTrue(manifest['available'])
            self.assertEqual(len(manifest['face_angles']), count)
            self.assertEqual(manifest['object_mass'], .8)
            self.assertAlmostEqual(manifest['destination_position'][2], 1.002)
            for robot, angle, radius in zip(robots, manifest['face_angles'], manifest['face_radii']):
                self.assertAlmostEqual(robot['pos'][0], (radius + .8) * math.cos(angle + .4))
                self.assertAlmostEqual(robot['pos'][1], 1 + (radius + .8) * math.sin(angle + .4))

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

    def test_model_and_manifest_written_to_requested_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            generator = self.builder(robot_count=5, object_shape='pentagonal_prism')
            generator.output_dir = directory
            with patch.object(generator, 'ensure_single_robot_model'), patch('mujoco_scene_generator.subprocess.check_call') as composer:
                robots, model, manifest, manifest_path = generator.generate()
            self.assertEqual(len(robots), 5)
            self.assertTrue(model.startswith(directory))
            self.assertEqual(yaml.safe_load(Path(manifest_path).read_text()), manifest)
            config_path = composer.call_args.args[0][-1]
            config = yaml.safe_load(Path(config_path).read_text())
            self.assertEqual(config['output_model'], model)
            self.assertEqual(config['jacobian'], 'dense')
            self.assertEqual(config['option']['impratio'], 50.)

    def test_invalid_shape_and_dimensions_rejected(self):
        for params in ({'object_shape': 'sphere'}, {'object_height': -1},
                       {'object_inner_triangle_side': .7}, {'object_sliding_friction': -1}):
            with self.assertRaises(RuntimeError):
                self.builder(**params).build_grasp_objects()


if __name__ == '__main__':
    unittest.main()
