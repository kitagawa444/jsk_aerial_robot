#!/usr/bin/env python3
"""Build RViz messages offline: no ROS master or display is needed."""
from pathlib import Path
import sys
import unittest
import yaml
import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'script'))
from grasp_geometry import extra_shape_geometry, surface_triangles
from mujoco_scene_visualizer import MujocoSceneVisualizer


class AdditionalMarkersTest(unittest.TestCase):
    def marker(self, shape):
        config = Path(__file__).resolve().parents[1] / 'config/mujoco' / ('scene_' + shape + '.yaml')
        params = yaml.safe_load(config.read_text())
        vis = MujocoSceneVisualizer.__new__(MujocoSceneVisualizer)
        vis.object_shape = shape
        vis.object_size_x = vis.object_size_y = .6
        vis.object_height = params['object_height']
        vis.extra_geometry = extra_shape_geometry(params)
        vis.extra_points = ([vis.point(v) for v in surface_triangles(vis.extra_geometry)]
                            if shape != 'cylinder' else [])
        pose = Pose()
        pose.position.x = 1.2
        pose.orientation.w = 1
        return vis.object_marker(rospy.Time(1), pose), vis.extra_geometry

    def test_cylinder_marker_uses_physical_dimensions(self):
        m, g = self.marker('cylinder')
        self.assertEqual(m.type, Marker.CYLINDER)
        self.assertEqual(m.scale.x, .6)
        self.assertEqual(m.scale.y, .6)
        self.assertEqual(m.scale.z, .6)
        self.assertEqual(m.pose.position.x, 1.2)

    def test_mesh_markers_use_com_frame(self):
        for shape in ('frustum', 'capsule', 'rock'):
            m, g = self.marker(shape)
            self.assertEqual(m.type, Marker.TRIANGLE_LIST)
            self.assertEqual(m.scale.x, 1)
            self.assertGreater(len(m.points), 100)
            self.assertEqual(len(m.points) % 3, 0)
            self.assertAlmostEqual(min(p.z for p in m.points), -g['bottom'])
            self.assertAlmostEqual(max(p.z for p in m.points), g['height'] - g['bottom'])
            self.assertEqual(m.pose.position.x, 1.2)
            self.assertEqual(m.pose.orientation.w, 1)
            self.assertEqual(m.header.frame_id, 'world')


if __name__ == '__main__':
    unittest.main()
