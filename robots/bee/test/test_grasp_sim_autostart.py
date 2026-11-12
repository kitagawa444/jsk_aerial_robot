"""Offline tests: no connection to a ROS master."""
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'script'))
import grasp_sim_autostart as startup


class StartupTest(unittest.TestCase):
    def setUp(self):
        self.runner = startup.AutoStart.__new__(startup.AutoStart)
        self.runner.names = ['bee1', 'bee2', 'bee3']
        self.runner.states = {n: (0, 100.) for n in self.runner.names}
        self.runner.publishers = {a: {n: MagicMock() for n in self.runner.names}
                                  for a in ('start', 'takeoff')}

    def test_state_freshness(self):
        with patch.object(startup.time, 'monotonic', return_value=100.):
            self.assertTrue(self.runner.all_states({0}))
            self.assertFalse(self.runner.all_states({5}))
        with patch.object(startup.time, 'monotonic', return_value=103.):
            self.assertFalse(self.runner.all_states({0}))

    def test_simulation_gates(self):
        params = {'/use_sim_time': True, '/bee_cooperative_grasp/execute': True,
                  '/bee_grasp/scene/available': True,
                  '/bee_grasp/scene/robot_names': self.runner.names}
        for n in self.runner.names:
            params['/%s/simulation/robot_hw_sim_plugin_name' % n] = 'mujoco_ros_control/AerialRobotHWSim'
        with patch.object(startup.rospy, 'get_param', side_effect=lambda k, d=None: params.get(k, d)), \
                patch.object(startup.time, 'monotonic', return_value=100.):
            self.assertTrue(self.runner.simulation_ready())
            params['/use_sim_time'] = False
            self.assertFalse(self.runner.simulation_ready())
            params['/use_sim_time'] = True
            params['/bee1/simulation/robot_hw_sim_plugin_name'] = 'wrong'
            self.assertFalse(self.runner.simulation_ready())

    def test_timeout_uses_wall_clock(self):
        with patch.object(startup.rospy, 'is_shutdown', return_value=False), \
                patch.object(startup.rospy, 'loginfo'), \
                patch.object(startup.time, 'monotonic', side_effect=[0., 2.]):
            with self.assertRaisesRegex(RuntimeError, 'Timeout'):
                self.runner.wait(lambda: False, 'ready', 1.)

    def test_object_pose_required_and_checked(self):
        self.runner.object_pose = None
        self.assertFalse(self.runner.object_ready())
        message = startup.PoseStamped()
        message.header.frame_id = 'world'
        message.header.stamp = startup.rospy.Time.from_sec(10.)
        message.pose.orientation.w = 1.
        self.runner.object_pose = (message, 100.)
        with patch.object(startup.rospy, 'get_time', return_value=10.1), \
                patch.object(startup.time, 'monotonic', return_value=100.1):
            self.assertTrue(self.runner.object_ready())
            message.header.frame_id = 'wrong'
            self.assertFalse(self.runner.object_ready())
            message.header.frame_id = 'world'
            message.pose.position.x = float('nan')
            self.assertFalse(self.runner.object_ready())

    def test_stale_object_rejected(self):
        message = startup.PoseStamped()
        message.header.frame_id = 'world'
        message.header.stamp = startup.rospy.Time.from_sec(10.)
        message.pose.orientation.w = 1.
        self.runner.object_pose = (message, 100.)
        with patch.object(startup.rospy, 'get_time', return_value=11.), \
                patch.object(startup.time, 'monotonic', return_value=100.1):
            self.assertFalse(self.runner.object_ready())


if __name__ == '__main__':
    unittest.main()
