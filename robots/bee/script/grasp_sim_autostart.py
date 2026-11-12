#!/usr/bin/env python3
"""Simulation startup only; all grasp/transport control stays in C++."""
import math
import time

import rospy
from std_msgs.msg import Empty, UInt8
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped


class AutoStart:
    def __init__(self):
        count = int(rospy.get_param('~robot_count', 3))
        if count not in (3, 4, 5):
            raise RuntimeError('robot_count must be 3, 4 or 5')
        self.names = ['bee%d' % i for i in range(1, count + 1)]
        self.states = {}
        self.object_pose = None
        self.subscribers = [rospy.Subscriber('/%s/flight_state' % name, UInt8,
                                            self.state_callback, callback_args=name)
                            for name in self.names]
        self.subscribers.append(rospy.Subscriber('/bee1/mujoco/grasp_object_pose',
                                                PoseStamped, self.object_callback))
        self.publishers = {action: {name: rospy.Publisher(
            '/%s/teleop_command/%s' % (name, action), Empty, queue_size=1)
            for name in self.names} for action in ('start', 'takeoff')}

    def state_callback(self, message, name):
        self.states[name] = (message.data, time.monotonic())

    def object_callback(self, message):
        self.object_pose = (message, time.monotonic())

    def object_ready(self):
        if self.object_pose is None:
            return False
        message, received = self.object_pose
        p, q = message.pose.position, message.pose.orientation
        age = rospy.get_time() - message.header.stamp.to_sec()
        return (message.header.frame_id.lstrip('/') == 'world'
                and message.header.stamp.to_sec() > 0
                and -.05 <= age <= .5 and time.monotonic() - received <= .5
                and all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) < .05)

    def all_states(self, allowed):
        now = time.monotonic()
        return all(name in self.states and self.states[name][0] in allowed
                   and now - self.states[name][1] < 2.0 for name in self.names)

    def wait(self, predicate, label, timeout=120.0):
        rospy.loginfo('Automatic C++ demo: waiting for %s', label)
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown():
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise RuntimeError('Timeout waiting for ' + label)
            time.sleep(0.1)  # Wall time: a stopped /clock must not hang startup.
        raise rospy.ROSInterruptException('shutdown')

    def simulation_ready(self):
        return (rospy.get_param('/use_sim_time', False)
                and rospy.get_param('/bee_cooperative_grasp/execute', False)
                and rospy.get_param('/bee_grasp/scene/available', False)
                and rospy.get_param('/bee_grasp/scene/robot_names', []) == self.names
                and all(rospy.get_param('/%s/simulation/robot_hw_sim_plugin_name' % n, '')
                        == 'mujoco_ros_control/AerialRobotHWSim' for n in self.names)
                and all(p.get_num_connections() for group in self.publishers.values()
                        for p in group.values()) and self.all_states({0}))

    def run(self):
        self.wait(self.simulation_ready, 'MuJoCo, controller and disarmed vehicles')
        initial_time = rospy.get_time()
        self.wait(lambda: rospy.get_time() > initial_time, 'advancing simulation clock', 10)
        self.wait(self.object_ready, 'fresh world-frame /bee1/mujoco/grasp_object_pose', 30)
        for p in self.publishers['start'].values():
            p.publish(Empty())
        self.wait(lambda: self.all_states({2}), 'all vehicles ARMED', 30)
        for p in self.publishers['takeoff'].values():
            p.publish(Empty())
        self.wait(lambda: self.all_states({5}), 'all vehicles HOVER', 120)
        rospy.wait_for_service('/bee_cooperative_grasp/start', timeout=30)
        start = rospy.ServiceProxy('/bee_cooperative_grasp/start', Trigger)

        def request_start():
            if not self.all_states({5}):
                raise RuntimeError('Lost HOVER before grasp start')
            result = start()
            if not result.success:
                rospy.logwarn_throttle(2.0, 'Grasp start rejected: ' + result.message)
            return result.success

        self.wait(request_start, 'C++ controller to accept start', 30)
        rospy.loginfo('C++ grasp mission started. Ctrl-C stops the entire simulation.')
        # Keep the required node alive; successful startup must not stop roslaunch.
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('bee_grasp_sim_autostart')
    try:
        AutoStart().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as error:
        rospy.logfatal('Automatic demo stopped: %s', error)
        raise SystemExit(1)
