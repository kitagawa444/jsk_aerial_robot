#!/usr/bin/env python3
"""Check scene/vehicle consistency and startup gates without a ROS master."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'script'))
import prepare_mujoco_scene as preparation


class ScenePreparationTest(unittest.TestCase):
    def read_scene(self, contents, count=3):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'scene.yaml'
            config.write_text(contents)
            return preparation.load_scene(config, count)

    def test_launch_and_yaml_count_must_agree(self):
        for contents in ('robot_count: 4', 'robot_count: true', 'robot_count: 3.0', '- not_a_mapping'):
            with self.subTest(contents=contents), self.assertRaises(ValueError):
                self.read_scene(contents)
        self.assertEqual(self.read_scene('robot_count: 3')['robot_count'], 3)
        self.assertEqual(self.read_scene('object_mass: 0.5')['robot_count'], 3)

    def test_all_vehicles_must_be_configured(self):
        plugin = 'mujoco_ros_control/AerialRobotHWSim'
        with patch.object(preparation.rospy, 'is_shutdown', return_value=False), \
                patch.object(preparation.rospy, 'get_param', return_value=plugin), \
                patch.object(preparation.rospy, 'has_param', side_effect=lambda key: not key.startswith('/bee3/')), \
                patch.object(preparation.time, 'monotonic', side_effect=[0, 2]):
            with self.assertRaisesRegex(RuntimeError, '/bee3/robot_description'):
                preparation.wait_for_vehicle_parameters(['bee1', 'bee2', 'bee3'], timeout=1)

    def test_wrong_plugin_is_not_treated_as_ready(self):
        with patch.object(preparation.rospy, 'is_shutdown', return_value=False), \
                patch.object(preparation.rospy, 'get_param', return_value='mujoco_ros_control/DefaultRobotHWSim'), \
                patch.object(preparation.rospy, 'has_param', return_value=True), \
                patch.object(preparation.time, 'monotonic', side_effect=[0, 2]):
            with self.assertRaisesRegex(RuntimeError, 'AerialRobotHWSim'):
                preparation.wait_for_vehicle_parameters(['bee1'], timeout=1)

    def test_ready_configuration_proceeds(self):
        with patch.object(preparation.rospy, 'is_shutdown', return_value=False), \
                patch.object(preparation.rospy, 'get_param', return_value='mujoco_ros_control/AerialRobotHWSim'), \
                patch.object(preparation.rospy, 'has_param', return_value=True):
            preparation.wait_for_vehicle_parameters(['bee1', 'bee2', 'bee3'], timeout=1)

    def test_shutdown_does_not_proceed_to_backend(self):
        with patch.object(preparation.rospy, 'is_shutdown', return_value=True):
            with self.assertRaises(preparation.rospy.ROSInterruptException):
                preparation.wait_for_vehicle_parameters(['bee1'], timeout=1)


if __name__ == '__main__':
    unittest.main()
