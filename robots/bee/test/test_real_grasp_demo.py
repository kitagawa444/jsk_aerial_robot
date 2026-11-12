#!/usr/bin/env python3
"""No ROS master, actuator or simulator is used by these tests."""
import math
import os
import sys
import time
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'script'))
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
import rospy
import yaml

from grasp_hardware_safety import packet_error, freshness_error
from real_grasp_demo import RealGraspDemo
from mujoco_triple_press_keyboard_demo import MujocoTriplePressKeyboardDemo


class SafetyTest(unittest.TestCase):
    def pose(self):
        msg = PoseStamped()
        msg.header.frame_id = '/world'
        msg.header.stamp = rospy.Time.from_sec(10)
        msg.pose.orientation.w = 1
        return msg

    def test_valid_and_wrong_frame(self):
        msg = self.pose()
        self.assertIsNone(packet_error(msg, 'pose', 'world', 10.1, .3))
        self.assertIsNotNone(packet_error(msg, 'pose', 'map', 10.1, .3))

    def test_stale_future_zero_stamp(self):
        msg = self.pose()
        for now in (11, 9):
            self.assertIsNotNone(packet_error(msg, 'pose', 'world', now, .3))
        msg.header.stamp = rospy.Time()
        self.assertIsNotNone(packet_error(msg, 'pose', 'world', 10, .3))

    def test_invalid_values(self):
        msg = self.pose()
        msg.pose.position.x = math.nan
        self.assertIsNotNone(packet_error(msg, 'pose', 'world', 10, .3))
        msg = self.pose()
        msg.pose.orientation.w = 0
        self.assertIsNotNone(packet_error(msg, 'pose', 'world', 10, .3))
        msg = WrenchStamped()
        msg.header = self.pose().header
        msg.wrench.force.z = math.inf
        self.assertIsNotNone(packet_error(msg, 'wrench', 'world', 10, .3))
        msg = Odometry()
        msg.header = self.pose().header
        msg.pose.pose.orientation.w = 1
        msg.twist.twist.linear.x = math.nan
        self.assertIsNotNone(packet_error(msg, 'odom', 'world', 10, .3))

    def test_receipt_and_source_age(self):
        self.assertIsNotNone(freshness_error({}, ['object'], 10, 10, .3))
        self.assertIsNotNone(freshness_error({'object': (9, 10, None)}, ['object'], 10, 10, .3))
        self.assertIsNotNone(freshness_error({'object': (10, 9, None)}, ['object'], 10, 10, .3))
        self.assertIsNone(freshness_error({'object': (10, 10, None)}, ['object'], 10, 10, .3))


class HardwareInterfaceTest(unittest.TestCase):
    def setUp(self):
        config = os.path.join(os.path.dirname(__file__), '..', 'config', 'real_grasp_demo.yaml')
        with open(config) as source:
            self.params = {'~' + k: v for k, v in yaml.safe_load(source).items()}
        def get_param(key, *default):
            if key in self.params:
                return self.params[key]
            if default:
                return default[0]
            raise KeyError(key)
        self.patches = [patch.object(rospy, 'get_param', side_effect=get_param),
                        patch.object(rospy, 'has_param', side_effect=lambda k: k in self.params),
                        patch.object(rospy, 'get_time', return_value=10),
                        patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(10)),
                        patch.object(rospy, 'is_shutdown', return_value=False)]
        self.patches += [patch.object(rospy, method) for method in
                         ('Publisher', 'Subscriber', 'loginfo', 'logwarn', 'logerr', 'logwarn_throttle')]
        for p in self.patches:
            p.start()
            self.addCleanup(p.stop)

    def test_no_simulator_inputs_or_wrench_output(self):
        demo = RealGraspDemo()
        topics = [call.args[0] for call in rospy.Subscriber.call_args_list]
        outputs = [call.args[0] for call in rospy.Publisher.call_args_list]
        self.assertFalse(any('/mujoco/' in t or 'ground_truth' in t for t in topics + outputs))
        self.assertIn('/bee1/uav/baselink/odom', topics)
        self.assertIn('/mocap/grasp_object/pose', topics)
        self.assertEqual(demo.grasp_object_mass, .2)
        self.assertEqual(demo.wrench_pubs, {})

    def test_simulation_interfaces_remain_available(self):
        self.params.pop('~object_pose_topic')
        demo = MujocoTriplePressKeyboardDemo()
        topics = [call.args[0] for call in rospy.Subscriber.call_args_list]
        self.assertIn('/bee1/ground_truth', topics)
        self.assertIn('/bee1/mujoco/grasp_contact_states', topics)
        self.assertIn('/bee1/mujoco/grasp_object_pose', topics)
        self.assertEqual(set(demo.wrench_pubs), {'bee1', 'bee2', 'bee3'})

    def test_check_only_never_sends(self):
        demo = RealGraspDemo()
        demo.input_reason = MagicMock(return_value=None)
        self.assertTrue(demo.run())
        for pubs in (demo.nav_pubs, demo.start_pubs, demo.takeoff_pubs, demo.baselink_rpy_pubs):
            for pub in pubs.values():
                pub.publish.assert_not_called()

    def test_external_force_and_sim_time_rejected(self):
        self.params['~use_external_wrench'] = True
        with self.assertRaises(ValueError):
            RealGraspDemo()
        self.params['~use_external_wrench'] = False
        self.params['/use_sim_time'] = True
        with self.assertRaises(ValueError):
            RealGraspDemo()

    def test_namespace_mapping(self):
        self.params['~robot_namespaces'] = {'bee1': 'lab/beeA', 'bee2': 'lab/beeB', 'bee3': 'lab/beeC'}
        demo = RealGraspDemo()
        self.assertEqual(demo.robot_topic('bee1', 'uav/nav'), '/lab/beeA/uav/nav')

    def test_stale_command_latches_abort(self):
        demo = RealGraspDemo()
        demo.execute = demo.command_active = True
        demo.input_reason = MagicMock(return_value='stale object')
        demo.stop_commands = MagicMock()
        with self.assertRaises(RuntimeError):
            demo.publish_accel_nav('bee1', 0, 0, 0)
        self.assertEqual(demo.abort_reason, 'stale object')
        self.assertTrue(demo.stop_requested)
        demo.input_reason.return_value = None
        with self.assertRaises(RuntimeError):
            demo.publish_accel_nav('bee1', 0, 0, 0)
        demo.stop_commands.assert_called()

    def test_acceleration_limit_aborts(self):
        demo = RealGraspDemo()
        demo.execute = demo.command_active = True
        demo.input_reason = MagicMock(return_value=None)
        demo.stop_commands = MagicMock()
        with self.assertRaises(RuntimeError):
            demo.publish_accel_nav('bee1', 100, 0, 0)
        self.assertIn('limit', demo.abort_reason)

    def test_no_auto_takeoff(self):
        demo = RealGraspDemo()
        self.assertFalse(demo.arm_and_takeoff())
        for pub in demo.start_pubs.values():
            pub.publish.assert_not_called()

    def test_stop_respects_non_hover_state(self):
        demo = RealGraspDemo()
        demo.nav_pubs = {name: MagicMock() for name in demo.names}
        demo.flight_states = {'bee1': 5, 'bee2': 6, 'bee3': 0}
        demo.stop_commands()
        demo.nav_pubs['bee1'].publish.assert_called_once()
        demo.nav_pubs['bee2'].publish.assert_not_called()
        demo.nav_pubs['bee3'].publish.assert_not_called()

    def test_watchdog_control_loop_timeout(self):
        demo = RealGraspDemo()
        demo.command_active = demo.execute = True
        demo.last_command_wall = time.monotonic() - 2
        demo.input_reason = MagicMock(return_value=None)
        demo.stop_commands = MagicMock()
        demo.watchdog.start()
        try:
            deadline = time.monotonic() + 1
            while demo.abort_reason is None and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertEqual(demo.abort_reason, 'control loop heartbeat lost')
        finally:
            demo.watchdog_exit.set()
            demo.watchdog.join(timeout=1)


if __name__ == '__main__':
    unittest.main()
