#!/usr/bin/env python3

import csv
import math
import os

import rospy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float32MultiArray, UInt8


class FlipTest:
    def __init__(self):
        self.robot_ns = rospy.get_param("~robot_ns", "bee1").strip("/")
        self.output = rospy.get_param("~output", "/tmp/bee_flip_test.csv")
        self.step_angle = float(rospy.get_param("~step_angle", math.pi / 4.0))
        self.step_duration = float(rospy.get_param("~step_duration", 5.0))
        self.hover_timeout = float(rospy.get_param("~hover_timeout", 50.0))
        self.start_delay = float(rospy.get_param("~start_delay", 1.0))
        self.final_hold = float(rospy.get_param("~final_hold", 5.0))
        self.terminal_attitude_tolerance = float(rospy.get_param(
            "~terminal_attitude_tolerance", 0.25))
        self.odom = None
        self.flight_state = None
        self.debug = []
        self.joints = {}
        self.joint_velocities = {}
        self.gimbal_commands = {}
        self.debug_sequence = 0
        self.debug_stamp = float('nan')
        self.phase = 'takeoff'
        self.start_time = None
        self.previous_wrapped_roll = None
        self.unwrapped_roll = 0.0
        self.minimum_z = float("inf")
        self.maximum_z = -float("inf")
        self.maximum_pitch = 0.0
        self.minimum_scale = 1.0
        self.rows = []
        self.maximum_terminal_error = 0.0

        prefix = "/" + self.robot_ns
        rospy.Subscriber(prefix + "/ground_truth", Odometry, self.odom_callback,
                         queue_size=1)
        rospy.Subscriber(prefix + "/flight_state", UInt8,
                         lambda msg: setattr(self, "flight_state", msg.data),
                         queue_size=1)
        rospy.Subscriber(prefix + "/joint_states", JointState,
                         self.joint_callback, queue_size=1)
        rospy.Subscriber(prefix + "/debug/differential_allocation",
                         Float32MultiArray,
                         self.debug_callback,
                         queue_size=1)
        rospy.Subscriber(prefix + "/gimbals_ctrl", JointState,
                         self.command_callback, queue_size=1)
        self.start_pub = rospy.Publisher(prefix + "/teleop_command/start", Empty,
                                         queue_size=1)
        self.takeoff_pub = rospy.Publisher(prefix + "/teleop_command/takeoff",
                                           Empty, queue_size=1)
        self.halt_pub = rospy.Publisher(prefix + "/teleop_command/halt", Empty,
                                        queue_size=1)
        self.rotation_pub = rospy.Publisher(
            prefix + "/final_target_baselink_rpy", Vector3Stamped,
            queue_size=1)

    @staticmethod
    def quaternion_to_rpy(quaternion):
        sin_roll = 2.0 * (quaternion.w * quaternion.x +
                          quaternion.y * quaternion.z)
        cos_roll = 1.0 - 2.0 * (quaternion.x * quaternion.x +
                                quaternion.y * quaternion.y)
        sin_pitch = 2.0 * (quaternion.w * quaternion.y -
                           quaternion.z * quaternion.x)
        sin_yaw = 2.0 * (quaternion.w * quaternion.z +
                         quaternion.x * quaternion.y)
        cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y +
                               quaternion.z * quaternion.z)
        return (math.atan2(sin_roll, cos_roll),
                math.asin(max(-1.0, min(1.0, sin_pitch))),
                math.atan2(sin_yaw, cos_yaw))

    def odom_callback(self, message):
        self.odom = message

    def joint_callback(self, message):
        self.joints = dict(zip(message.name, message.position))
        self.joint_velocities = dict(zip(message.name, message.velocity))

    def command_callback(self, message):
        names = message.name or ['gimbal{}'.format(i + 1)
                                 for i in range(len(message.position))]
        self.gimbal_commands = dict(zip(names, message.position))

    def debug_callback(self, message):
        self.debug = list(message.data)
        self.debug_stamp = rospy.Time.now().to_sec()
        self.debug_sequence += 1

    def publish_roll_rotation(self, angle):
        message = Vector3Stamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        message.vector.x = angle
        self.rotation_pub.publish(message)
        rospy.loginfo("flip test: target baselink roll %.1f deg",
                      math.degrees(angle))

    def sample(self, target_angle):
        if self.odom is None:
            return True
        roll, pitch, yaw = self.quaternion_to_rpy(
            self.odom.pose.pose.orientation)
        if self.previous_wrapped_roll is None:
            self.unwrapped_roll = roll
        else:
            delta = roll - self.previous_wrapped_roll
            if delta > math.pi:
                delta -= 2.0 * math.pi
            elif delta < -math.pi:
                delta += 2.0 * math.pi
            self.unwrapped_roll += delta
        self.previous_wrapped_roll = roll

        z = self.odom.pose.pose.position.z
        self.minimum_z = min(self.minimum_z, z)
        self.maximum_z = max(self.maximum_z, z)
        self.maximum_pitch = max(self.maximum_pitch, abs(pitch))
        scale = self.debug[37] if len(self.debug) >= 38 else float("nan")
        if math.isfinite(scale):
            self.minimum_scale = min(self.minimum_scale, scale)
        limiting_input = self.debug[62] if len(self.debug) >= 67 else float("nan")
        limiting_reason = self.debug[63] if len(self.debug) >= 67 else float("nan")
        sigma_min = self.debug[64] if len(self.debug) >= 67 else float("nan")
        sigma_max = self.debug[65] if len(self.debug) >= 67 else float("nan")
        condition = self.debug[66] if len(self.debug) >= 67 else float("nan")
        row = [rospy.Time.now().to_sec() - self.start_time, target_angle, z,
               roll, self.unwrapped_roll, pitch, yaw, scale, limiting_input,
               limiting_reason, sigma_min, sigma_max, condition]
        row.extend(self.joints.get("gimbal{}".format(index), float("nan"))
                   for index in range(1, 5))
        row.extend(self.debug[index] if len(self.debug) > index else float("nan")
                   for index in range(67))
        row.extend([self.phase, self.debug_sequence, self.debug_stamp,
                    self.odom.header.stamp.to_sec()])
        row.extend(self.joint_velocities.get('gimbal{}'.format(i), float('nan'))
                   for i in range(1, 5))
        row.extend(self.gimbal_commands.get('gimbal{}'.format(i), float('nan'))
                   for i in range(1, 5))
        self.rows.append(row)
        return z >= 0.08 and z <= 3.0 and abs(pitch) <= 1.2

    def wait_for_hover(self):
        deadline = rospy.Time.now().to_sec() + self.hover_timeout
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            self.sample(0.0)
            if self.flight_state == 5 and self.odom is not None:
                return True
            rate.sleep()
        return False

    def write_csv(self):
        directory = os.path.dirname(self.output)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        header = ["time", "target_roll", "z", "roll_wrapped",
                  "roll_unwrapped", "pitch", "yaw", "dld_scale",
                  "limiting_input", "limiting_reason", "sigma_min",
                  "sigma_max", "condition"]
        header.extend("gimbal{}_position".format(index)
                      for index in range(1, 5))
        header.extend("debug_{}".format(index) for index in range(67))
        header.extend(['phase', 'debug_sequence', 'debug_receive_time', 'odom_stamp'])
        header.extend('gimbal{}_velocity'.format(i) for i in range(1, 5))
        header.extend('gimbal{}_command'.format(i) for i in range(1, 5))
        with open(self.output, "w") as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(self.rows)

    def run(self):
        rospy.sleep(self.start_delay)
        self.start_time = rospy.Time.now().to_sec()
        self.start_pub.publish(Empty())
        rospy.sleep(2.0)
        self.takeoff_pub.publish(Empty())
        if not self.wait_for_hover():
            rospy.logerr("flip test: hover timeout")
            self.halt_pub.publish(Empty())
            return False

        target_angles = [index * self.step_angle for index in range(
            1, int(round(2.0 * math.pi / self.step_angle)) + 1)]
        self.phase = 'rotation'
        # Keep takeoff samples in CSV, but summarize the commanded maneuver.
        self.minimum_z = float('inf')
        self.maximum_z = -float('inf')
        self.maximum_pitch = 0.0
        self.minimum_scale = 1.0
        rate = rospy.Rate(50)
        for target_angle in target_angles:
            self.publish_roll_rotation(target_angle)
            deadline = rospy.Time.now().to_sec() + self.step_duration
            while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
                if not self.sample(target_angle):
                    rospy.logerr("flip test: safety stop at roll %.1f deg, z %.3f",
                                 math.degrees(self.unwrapped_roll),
                                 self.odom.pose.pose.position.z)
                    self.halt_pub.publish(Empty())
                    self.write_csv()
                    return False
                rate.sleep()

            roll, pitch, yaw = self.quaternion_to_rpy(
                self.odom.pose.pose.orientation)
            roll_error = math.atan2(math.sin(target_angle - roll),
                                    math.cos(target_angle - roll))
            attitude_error = math.sqrt(roll_error * roll_error +
                                       pitch * pitch + yaw * yaw)
            self.maximum_terminal_error = max(self.maximum_terminal_error,
                                              attitude_error)
            rospy.loginfo("flip test: terminal error %.1f deg at target %.1f deg",
                          math.degrees(attitude_error),
                          math.degrees(target_angle))

        deadline = rospy.Time.now().to_sec() + self.final_hold
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            if not self.sample(2.0 * math.pi):
                self.halt_pub.publish(Empty())
                self.write_csv()
                return False
            rate.sleep()
        self.write_csv()
        tracking_success = (
            self.maximum_terminal_error <= self.terminal_attitude_tolerance)
        rospy.loginfo("flip test: %s, roll %.1f deg, z %.3f--%.3f, "
                      "max pitch %.1f deg, max terminal error %.1f deg, "
                      "min DLD scale %.4f",
                      "COMPLETE" if tracking_success else "TRACKING_FAIL",
                      math.degrees(self.unwrapped_roll), self.minimum_z,
                      self.maximum_z, math.degrees(self.maximum_pitch),
                      math.degrees(self.maximum_terminal_error),
                      self.minimum_scale)
        return tracking_success


if __name__ == "__main__":
    rospy.init_node("differential_allocation_flip_test")
    test = FlipTest()
    success = False
    try:
        success = test.run()
    finally:
        test.write_csv()
    raise SystemExit(0 if success else 1)
