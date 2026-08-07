#!/usr/bin/env python

from __future__ import print_function

import os
import select
import sys
import termios
import time
import tty

import rospy
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32

from mujoco_dual_push_demo import MujocoDualPushDemo


HELP = """
Three-Bee observer force control
--------------------------------
  w : increase common normal force
  s : decrease common normal force
  [ : increase common world-Z velocity
  ] : decrease common world-Z velocity
SPACE: set world-Z velocity to zero
  r : reset force and Z velocity
  p : print current command
  x : stop demo
CTRL-C: stop demo
--------------------------------
"""


class TerminalKeyboard(object):
    """Non-blocking single-key input while the flight loop keeps running."""

    def __init__(self):
        self.settings = None

    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard demo requires an interactive terminal")
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def read_all(self):
        keys = []
        while select.select([sys.stdin], [], [], 0.0)[0]:
            # sys.stdin's text buffer can prefetch several characters and
            # make a following select() report no data even though buffered
            # keys remain.  Read directly from the terminal descriptor.
            key = os.read(sys.stdin.fileno(), 1)
            if not isinstance(key, str):
                key = key.decode("utf-8", "ignore")
            keys.append(key)
        return keys

    def __exit__(self, exception_type, exception, traceback):
        if self.settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


class MujocoTriplePressKeyboardDemo(MujocoDualPushDemo):
    """Press a triangular prism with three Bees and adjust commands by key."""

    ACTIVE_NAMES = ("bee1", "bee2", "bee3")

    def __init__(self):
        super(MujocoTriplePressKeyboardDemo, self).__init__()
        common_force = rospy.get_param("~target_normal_force", 1.0)
        self.initial_normal_force = common_force
        self.set_common_normal_force(common_force)

        self.force_step = rospy.get_param("~keyboard_force_step", 0.25)
        self.minimum_normal_force = rospy.get_param("~minimum_normal_force", 0.0)
        self.maximum_normal_force = rospy.get_param("~maximum_normal_force", 8.0)
        self.z_velocity_step = rospy.get_param("~keyboard_z_velocity_step", 0.02)
        self.maximum_abs_z_velocity = rospy.get_param(
            "~maximum_abs_z_velocity", 0.20)
        self.target_z_velocity = rospy.get_param("~initial_z_velocity", 0.0)
        self.normal_force_feedforward_scale = rospy.get_param(
            "~normal_force_feedforward_scale", 1.0)
        self.stop_requested = False

        self.target_force_pub = rospy.Publisher(
            "/mujoco_triple_press/target_normal_force", Float32, queue_size=1)
        self.target_z_velocity_pub = rospy.Publisher(
            "/mujoco_triple_press/target_z_velocity", Float32, queue_size=1)
        self.observer_forces_pub = rospy.Publisher(
            "/mujoco_triple_press/observer_normal_forces",
            Vector3Stamped, queue_size=1)

    @staticmethod
    def clamp_value(value, lower, upper):
        return max(lower, min(upper, value))

    def set_common_normal_force(self, force):
        for name in self.names:
            self.target_normal_forces[name] = force

    def common_normal_force(self):
        return self.target_normal_forces[self.names[0]]

    def publish_interactive_state(self, observer_forces):
        self.target_force_pub.publish(Float32(data=self.common_normal_force()))
        self.target_z_velocity_pub.publish(Float32(data=self.target_z_velocity))
        message = Vector3Stamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "bee1_bee2_bee3"
        message.vector.x = observer_forces.get("bee1", 0.0)
        message.vector.y = observer_forces.get("bee2", 0.0)
        message.vector.z = observer_forces.get("bee3", 0.0)
        self.observer_forces_pub.publish(message)

    def log_command(self):
        rospy.loginfo(
            "triple press command: common normal force %.2f N, "
            "world-Z velocity %+.3f m/s",
            self.common_normal_force(), self.target_z_velocity)

    def handle_key(self, key):
        force = self.common_normal_force()
        if key == "w":
            force = self.clamp_value(
                force + self.force_step,
                self.minimum_normal_force, self.maximum_normal_force)
            self.set_common_normal_force(force)
        elif key == "s":
            force = self.clamp_value(
                force - self.force_step,
                self.minimum_normal_force, self.maximum_normal_force)
            self.set_common_normal_force(force)
        elif key == "[":
            self.target_z_velocity = self.clamp_value(
                self.target_z_velocity + self.z_velocity_step,
                -self.maximum_abs_z_velocity, self.maximum_abs_z_velocity)
        elif key == "]":
            self.target_z_velocity = self.clamp_value(
                self.target_z_velocity - self.z_velocity_step,
                -self.maximum_abs_z_velocity, self.maximum_abs_z_velocity)
        elif key == " ":
            self.target_z_velocity = 0.0
        elif key == "r":
            self.set_common_normal_force(self.initial_normal_force)
            self.target_z_velocity = 0.0
        elif key == "p":
            self.log_command()
            return
        elif key == "x" or key == "\x03":
            self.stop_requested = True
            return
        else:
            return
        self.log_command()

    def wait_for_triple_data(self, timeout=20.0):
        if not self.wait_for_dual_data(timeout):
            return False
        rospy.loginfo("triple press: Bee1, Bee2, Bee3 and object data ready")
        return True

    def press_acceleration(self, name, odom, object_pose, measured_force,
                           desired_radius):
        """Track force with physical F/m feed-forward plus observer feedback."""
        normal, tangent, yaw = self.face_frame(name)
        position = odom.pose.pose.position
        velocity = odom.twist.twist.linear
        dx = position.x - object_pose.position.x
        dy = position.y - object_pose.position.y
        radius = dx * normal[0] + dy * normal[1]
        tangent_position = dx * tangent[0] + dy * tangent[1]
        radial_velocity = velocity.x * normal[0] + velocity.y * normal[1]
        tangent_velocity = velocity.x * tangent[0] + velocity.y * tangent[1]

        target_force = self.target_normal_forces[name]
        force_error = target_force - measured_force
        force_feedforward = (self.normal_force_feedforward_scale *
                             target_force / max(1.0e-6, self.robot_mass))
        radial = (-force_feedforward -
                  self.normal_force_kp * force_error -
                  self.radial_kd * radial_velocity)
        if radius < desired_radius - self.press_radius_safety_margin:
            radial = (self.press_radial_kp * (desired_radius - radius) -
                      self.radial_kd * radial_velocity)
        tangential = (-self.tangent_kp * tangent_position -
                      self.tangent_kd * tangent_velocity)
        radial = self.clamp(radial, self.press_accel_limit)
        tangential = self.clamp(tangential, self.approach_accel_limit)
        return (radial * normal[0] + tangential * tangent[0],
                radial * normal[1] + tangential * tangent[1], yaw)

    def stage_and_approach(self):
        inradius = self.triangle_side / (2.0 * (3.0 ** 0.5))
        staging_radius = inradius + self.staging_clearance
        contact_radius = (inradius + self.foot_contact_offset +
                          self.foot_ball_radius - self.foot_max_compression)

        rospy.loginfo(
            "triple press: recover three Bees to staging radius %.3f m",
            staging_radius)
        stage_deadline = time.monotonic() + self.staging_timeout
        while not rospy.is_shutdown() and time.monotonic() < stage_deadline:
            _, odometry, _, object_pose = self.dual_snapshot()
            settled = True
            for name in self.names:
                ax, ay, yaw, radius, tangent, radial_velocity, tangent_velocity = \
                    self.approach_acceleration(
                        name, odometry[name], object_pose, staging_radius)
                self.publish_accel_nav(name, ax, ay, yaw)
                settled = (settled and
                           abs(radius - staging_radius) < self.staging_tolerance and
                           abs(tangent) < self.staging_tolerance and
                           abs(radial_velocity) < 0.08 and
                           abs(tangent_velocity) < 0.08)
            if settled:
                break
            time.sleep(self.control_period)
        else:
            rospy.logerr("triple press: staging timed out")
            return None

        rospy.loginfo("triple press: rotate all physical foot planes toward prism")
        ramp_start = time.monotonic()
        while not rospy.is_shutdown():
            alpha = min(1.0, (time.monotonic() - ramp_start) /
                        max(0.001, self.attitude_ramp_duration))
            blend = alpha * alpha * (3.0 - 2.0 * alpha)
            self.commanded_roll = blend * self.approach_roll
            _, odometry, _, object_pose = self.dual_snapshot()
            for name in self.names:
                ax, ay, yaw, _, _, _, _ = self.approach_acceleration(
                    name, odometry[name], object_pose, staging_radius)
                self.publish_accel_nav(name, ax, ay, yaw)
            if alpha >= 1.0:
                break
            time.sleep(self.control_period)

        settle_deadline = time.monotonic() + self.attitude_settle_duration
        while not rospy.is_shutdown() and time.monotonic() < settle_deadline:
            _, odometry, _, object_pose = self.dual_snapshot()
            for name in self.names:
                ax, ay, yaw, _, _, _, _ = self.approach_acceleration(
                    name, odometry[name], object_pose, staging_radius)
                self.publish_accel_nav(name, ax, ay, yaw)
            time.sleep(self.control_period)

        observer_baseline = self.observer_baseline_dual(staging_radius)
        desired_radii = dict((name, staging_radius) for name in self.names)
        complete_count = 0
        last_sim = rospy.get_time()
        approach_start = last_sim
        rospy.loginfo("triple press: simultaneous geometric approach")
        while not rospy.is_shutdown():
            now = rospy.get_time()
            dt = max(0.0, min(0.5, now - last_sim))
            last_sim = now
            _, odometry, _, object_pose = self.dual_snapshot()
            forces = {}
            for name in self.names:
                desired_radii[name] = max(
                    contact_radius,
                    desired_radii[name] - self.approach_speed * dt)
                forces[name] = self.observer_normal_force(
                    name, observer_baseline)
                ax, ay, yaw, _, _, _, _ = self.approach_acceleration(
                    name, odometry[name], object_pose, desired_radii[name])
                self.publish_accel_nav(name, ax, ay, yaw)
            at_target = all(
                desired_radii[name] <= contact_radius + 1.0e-6
                for name in self.names)
            complete_count = complete_count + 1 if at_target else 0
            rospy.loginfo_throttle(
                0.5,
                "triple approach observer: F1 %.3f N, F2 %.3f N, F3 %.3f N",
                forces["bee1"], forces["bee2"], forces["bee3"])
            if complete_count >= self.contact_confirm_samples:
                return observer_baseline, contact_radius
            if now - approach_start > self.approach_timeout:
                rospy.logerr("triple press: approach timed out")
                return None
            time.sleep(self.control_period)
        return None

    def interactive_press(self, observer_baseline, contact_radius):
        filtered_forces = dict(
            (name, self.observer_normal_force(name, observer_baseline))
            for name in self.names)
        print(HELP)
        self.log_command()
        rospy.loginfo(
            "triple press keyboard steps: force %.2f N (range %.2f..%.2f), "
            "Z velocity %.3f m/s (limit +/-%0.3f)",
            self.force_step, self.minimum_normal_force,
            self.maximum_normal_force, self.z_velocity_step,
            self.maximum_abs_z_velocity)
        rospy.loginfo(
            "triple press: observer feedback active; keyboard control ready")

        with TerminalKeyboard() as keyboard:
            while not rospy.is_shutdown() and not self.stop_requested:
                for key in keyboard.read_all():
                    self.handle_key(key)

                _, odometry, _, object_pose = self.dual_snapshot()
                alpha = max(0.0, min(1.0, self.force_filter_alpha))
                for name in self.names:
                    measured = self.observer_normal_force(
                        name, observer_baseline)
                    filtered_forces[name] += alpha * (
                        measured - filtered_forces[name])
                    ax, ay, yaw = self.press_acceleration(
                        name, odometry[name], object_pose,
                        filtered_forces[name], contact_radius)
                    self.publish_accel_nav(
                        name, ax, ay, yaw,
                        FlightNav.VEL_MODE, self.target_z_velocity)

                self.publish_interactive_state(filtered_forces)
                rospy.loginfo_throttle(
                    0.5,
                    "triple press: target %.2f N, observer "
                    "[%.3f, %.3f, %.3f] N, vz %+.3f m/s, object z %.3f m",
                    self.common_normal_force(),
                    filtered_forces["bee1"], filtered_forces["bee2"],
                    filtered_forces["bee3"], self.target_z_velocity,
                    object_pose.position.z)
                time.sleep(self.control_period)
        return True

    def run(self):
        if not self.wait_for_triple_data():
            rospy.logerr("triple press: required Bee or object data is missing")
            return False
        if not self.arm_and_takeoff():
            rospy.logerr("triple press: all three Bees did not reach HOVER")
            return False
        if not self.wait_for_observers():
            rospy.logerr("triple press: momentum-observer topics did not start")
            return False
        prepared = self.stage_and_approach()
        if prepared is None:
            return False
        return self.interactive_press(prepared[0], prepared[1])


def main():
    rospy.init_node("bee_mujoco_triple_press_keyboard_demo")
    demo = MujocoTriplePressKeyboardDemo()
    try:
        if not demo.run():
            raise SystemExit(1)
    finally:
        if demo.odometry:
            demo.publish_stop()


if __name__ == "__main__":
    main()
