#!/usr/bin/env python3
"""Parse models/launches only; never start hardware or a flight controller."""
from pathlib import Path
import unittest
import numpy as np
import yaml
import xml.etree.ElementTree as ET

from test_livox_launch import load


class ContactPointModelTest(unittest.TestCase):
    def test_separating_the_feet_preserves_the_cad_mass_properties(self):
        def base_assembly(args):
            graph = load('robots/bee/launch/bringup.launch', args)
            root = ET.fromstring(graph.params['/bee1/robot_description'].value)
            mass = 0.
            first_moment = np.zeros(3)
            tensor_at_origin = np.zeros((3, 3))
            for link in root.findall('link'):
                name = link.get('name')
                if name != 'base_link' and not name.startswith('spring_foot_'):
                    continue
                inertial = link.find('inertial')
                m = float(inertial.find('mass').get('value'))
                origin = inertial.find('origin')
                self.assertEqual(origin.get('rpy'), '0 0 0')
                c = np.fromstring(origin.get('xyz'), sep=' ')
                if name != 'base_link':
                    mount = root.find(f"joint[@name='{name}_joint']/origin")
                    self.assertEqual(mount.get('rpy'), '0 0 0')
                    c += np.fromstring(mount.get('xyz'), sep=' ')
                inertia = inertial.find('inertia')
                xx, xy, xz, yy, yz, zz = (float(inertia.get(k)) for k in
                                          ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'))
                tensor = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
                mass += m
                first_moment += m * c
                tensor_at_origin += tensor + m * (c.dot(c) * np.eye(3) - np.outer(c, c))
            return np.r_[mass, first_moment, tensor_at_origin.ravel()]

        assembled = base_assembly([])
        split = base_assembly(['simulation:=true', 'real_machine:=false'])
        np.testing.assert_allclose(split, assembled, rtol=0, atol=1e-10)

    def test_cad_visual_and_both_simulators_use_the_same_ball_centers(self):
        package = Path(__file__).resolve().parents[1]
        feet = yaml.safe_load((package / 'config/mujoco/mujoco_model.yaml').read_text())['bee']['compliant_feet']
        graph = load('robots/bee/launch/bringup.launch',
                     ['simulation:=true', 'real_machine:=false', 'headless:=true'])
        root = ET.fromstring(graph.params['/bee1/robot_description'].value)
        for foot in feet['feet']:
            name = foot['name']
            with self.subTest(foot=name):
                mount = root.find(f"joint[@name='spring_foot_{name}_joint']/origin")
                for v, p in zip(mount.get('xyz').split(), foot['pos']):
                    self.assertAlmostEqual(float(v), p)
                link = root.find(f"link[@name='spring_foot_{name}']")
                self.assertEqual(len(link.findall('visual')), 0)
                self.assertEqual(len(link.findall('collision')), 1)
                collision = link.find('collision')
                self.assertEqual(float(collision.find('geometry/sphere').get('radius')), feet['ball_radius'])
                offset = list(map(float, collision.find('origin').get('xyz').split()))
                self.assertEqual(offset, [0, 0, -feet['free_length']])
                visual = root.find(f"link[@name='base_link']/visual[@name='foot_ball_{name}']")
                self.assertTrue(visual.find('geometry/mesh').get('filename').endswith('/foot_ball.dae'))
                for v, p, d in zip(visual.find('origin').get('xyz').split(), foot['pos'], offset):
                    self.assertAlmostEqual(float(v), p + d)

    def test_single_vehicle_and_central_use_same_geometry(self):
        central = load('robots/bee/launch/cooperative_grasp.launch')
        offset = central.params['/bee_cooperative_grasp/controller/contact_offset'].value
        for args in [[], ['simulation:=true', 'real_machine:=false'],
                     ['simulation:=true', 'real_machine:=false', 'mujoco:=true']]:
            with self.subTest(args=args):
                c = load('robots/bee/launch/bringup.launch', ['robot_id:=2'] + args)
                self.assertFalse(any(n.type == 'bee_cooperative_grasp_node' for n in c.nodes))
                model = ET.fromstring(c.params['/bee2/robot_description'].value)
                links = model.findall("link[@name='grasp_contact_point']")
                self.assertEqual(len(links), 1)
                self.assertEqual(len(links[0]), 0)  # No mass/collision added.
                joint = model.find("joint[@name='grasp_contact_point_joint']")
                self.assertEqual(joint.attrib['type'], 'fixed')
                self.assertEqual(joint.find('parent').attrib['link'], 'fc')
                self.assertEqual(joint.find('child').attrib['link'], 'grasp_contact_point')
                self.assertEqual([float(x) for x in joint.find('origin').attrib['xyz'].split()], offset)
                self.assertEqual(joint.find('origin').attrib['rpy'], '0 0 0')
                fc = model.find("joint[@name='fc_joint']/origin")
                base_offset = [float(x) + y for x, y in zip(fc.attrib['xyz'].split(), offset)]
                for actual, expected in zip(base_offset, [0, 0, -.089]):
                    self.assertAlmostEqual(actual, expected)
                publisher = next(n for n in c.nodes if n.package == 'robot_state_publisher')
                self.assertEqual(c.params[publisher.namespace+publisher.name+'/tf_prefix'].value, 'bee2')
                # Legacy assembly point is independent of the foot reference.
                legacy = model.find("joint[@name='cp_base_joint']/origin")
                self.assertEqual([float(x) for x in legacy.attrib['xyz'].split()], [.26, 0, 0])


if __name__ == '__main__':
    unittest.main()
