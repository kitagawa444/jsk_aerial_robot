#!/usr/bin/env python3
"""Parse launch graphs only. Does not start a master, sensor, simulator or vehicle."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import roslaunch

REPO = Path(__file__).resolve().parents[3]


def load(path, args=()):
    config = roslaunch.ROSLaunchConfig()
    roslaunch.XmlLoader().load(str(REPO/path), config, argv=list(args), verbose=False)
    return config


class LivoxLaunchTest(unittest.TestCase):
    def test_livox_estimator_configuration(self):
        c = load('robots/bee/launch/bringup.launch', ['estimate_mode:=0', 'livox_enable:=true'])
        def param(name):
            return c.params['/bee1/'+name].value
        # SensorBase uses both the common vo and indexed vo1 namespaces.
        self.assertEqual(param('estimation/mode'), 0)
        self.assertEqual(param('estimation/sensor_list'),
                         ['sensor_plugin/imu', 'sensor_plugin/mocap', 'sensor_plugin/vo'])
        self.assertEqual(param('sensor_plugin/imu/estimate_mode'), 3)
        self.assertEqual(param('sensor_plugin/mocap/estimate_mode'), 6)
        self.assertEqual(param('sensor_plugin/vo1/estimate_mode'), 1)
        self.assertEqual([param('estimation/fuser_egomotion_id'+str(i)) for i in range(1, 4)], [8, 16, 32])
        self.assertEqual(param('sensor_plugin/imu/imu_topic_name'), 'imu')
        self.assertEqual(param('common/imu_topic'), 'livox/imu')
        self.assertEqual(param('common/lid_topic'), 'livox/lidar')
        self.assertEqual(param('sensor_plugin/vo/vo_sub_topic_name'), 'Odometry_precede')
        self.assertEqual(param('sensor_plugin/vo/sensor_frame'), 'lidar_imu')
        self.assertEqual(param('sensor_plugin/vo/fusion_mode'), 2)
        self.assertTrue(param('sensor_plugin/vo/vio_mode'))
        # The local FAST-LIO implementation publishes velocity in its world frame.
        self.assertFalse(param('sensor_plugin/vo/local_vel_mode'))
        self.assertFalse(param('sensor_plugin/vo/z_vel_mode'))
        self.assertFalse(param('sensor_plugin/vo/time_sync'))

    def test_bee_bringup_model_contains_connected_livox_frame(self):
        # Validate the actual robot_description produced by bringup, not just
        # whether a link declaration occurs in an included xacro source.
        for simulation, enabled in [(False, False), (False, True), (True, False), (True, True)]:
            with self.subTest(simulation=simulation, enabled=enabled):
                args = ['robot_id:=2', 'headless:=true',
                        'simulation:='+str(simulation).lower(),
                        'real_machine:='+str(not simulation).lower(),
                        ('lidar_sim:=' if simulation else 'livox_enable:=')+str(enabled).lower()]
                c = load('robots/bee/launch/bringup.launch', args)
                model = ET.fromstring(c.params['/bee2/robot_description'].value)
                visuals = model.findall("link[@name='base_link']/visual[@name='livox_module']")
                self.assertEqual(len(visuals), int(enabled))
                color = model.find("link[@name='body_trim']/visual/material/color")
                self.assertEqual(color.get('rgba'), '1 1 1 1')
                links = [link.attrib['name'] for link in model.findall('link')]
                self.assertEqual(links.count('lidar_imu'), 1)
                self.assertEqual(len(links), len(set(links)))
                joints = {joint.find('child').attrib['link']: joint for joint in model.findall('joint')}
                for child, parent in [('lidar_imu', 'base_link'), ('base_link', 'root'), ('fc', 'base_link')]:
                    self.assertEqual(joints[child].attrib['type'], 'fixed')
                    self.assertEqual(joints[child].find('parent').attrib['link'], parent)
                publisher = next(n for n in c.nodes if n.package == 'robot_state_publisher')
                self.assertEqual(c.params[publisher.namespace+publisher.name+'/tf_prefix'].value, 'bee2')
                lidar_plugins = [p for p in model.findall('.//gazebo/sensor/plugin')
                                 if p.attrib.get('filename') == 'liblivox_laser_simulation.so']
                self.assertEqual(len(lidar_plugins), int(simulation and enabled))
                if enabled:
                    self.assertEqual(c.params['/bee2/sensor_plugin/vo/sensor_frame'].value, 'lidar_imu')
                    self.assertEqual(c.params['/bee2/pointcloud_to_laserscan/target_frame'].value, 'bee2/lidar_imu')

    def test_sensor_only_hardware_namespaces_and_single_scan_converter(self):
        c = load('aerial_robot_base/launch/external_module/livox_mid360.launch', ['robot_ns:=bee2'])
        self.assertEqual(len([n for n in c.nodes if n.package == 'livox_ros_driver2']), 1)
        self.assertEqual(len([n for n in c.nodes if n.package == 'fast_lio']), 1)
        self.assertEqual(len([n for n in c.nodes if n.package == 'pointcloud_to_laserscan']), 1)
        self.assertTrue(all(n.namespace == '/bee2/' for n in c.nodes))
        self.assertFalse(any(n.package in ['spinal', 'aerial_robot_base'] for n in c.nodes))
        self.assertEqual(c.params['/bee2/frame_id'].value, 'bee2/lidar_imu')
        self.assertEqual(c.params['/bee2/tf_prefix'].value, 'bee2')
        self.assertEqual(c.params['/bee2/pointcloud_to_laserscan/target_frame'].value, 'bee2/lidar_imu')

    def test_simulated_sensor_does_not_start_hardware_driver(self):
        c = load('aerial_robot_base/launch/external_module/livox_mid360.launch', ['simulation:=true'])
        self.assertFalse(any(n.package == 'livox_ros_driver2' for n in c.nodes))
        self.assertEqual(len([n for n in c.nodes if n.package == 'fast_lio']), 1)

    def test_bee_defaults_and_mujoco_do_not_start_livox(self):
        for args in [[], ['simulation:=true', 'lidar_sim:=true', 'mujoco:=true']]:
            c = load('robots/bee/launch/include/sensors.launch.xml', args)
            self.assertFalse(any(n.package in ['livox_ros_driver2', 'fast_lio'] for n in c.nodes))

    def test_bee_gazebo_and_gimbalrotor_keep_mapping(self):
        for package in ['bee', 'gimbalrotor']:
            c = load('robots/'+package+'/launch/include/sensors.launch.xml',
                     ['simulation:=true', 'real_machine:=false', 'lidar_sim:=true', 'robot_ns:=testbee'])
            self.assertTrue(any(n.package == 'fast_lio' and n.namespace == '/testbee/' for n in c.nodes))
            self.assertFalse(any(n.package == 'livox_ros_driver2' for n in c.nodes))

    def test_navigation_uses_shared_parameters_without_duplicate_scan(self):
        for launch in ['aerial_robot_base/launch/external_module/livox_navigation.launch',
                       'robots/gimbalrotor/launch/navigation.launch']:
            c = load(launch, ['robot_ns:=testbee'])
            self.assertTrue(any(n.package == 'move_base' for n in c.nodes))
            self.assertFalse(any(n.package == 'pointcloud_to_laserscan' for n in c.nodes))
            self.assertEqual(c.params['/testbee/move_base/global_costmap/robot_base_frame'].value, 'testbee/root')


if __name__ == '__main__':
    unittest.main()
