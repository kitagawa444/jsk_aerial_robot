#!/usr/bin/env python

from __future__ import print_function

import math
import time

import rospy

from mujoco_push_demo import MujocoPushDemo


class MujocoTiltLiftDemo(MujocoPushDemo):
    """Press Bee1 into the prism and lift one side with foot friction."""

    def __init__(self):
        super(MujocoTiltLiftDemo, self).__init__()
        # Use the same preload as the push test to produce useful upward
        # friction, then stop pushing as soon as a clear lift is detected.
        self.target_normal_force = rospy.get_param("~target_normal_force", 5.0)
        self.normal_force_kp = rospy.get_param("~normal_force_kp", 0.25)
        self.attitude_settle_duration = rospy.get_param(
            "~attitude_settle_duration", 1.5)
        self.lift_speed = rospy.get_param("~lift_speed", 0.02)
        self.lift_height = rospy.get_param("~lift_height", 0.08)
        self.lift_timeout = rospy.get_param("~lift_timeout", 20.0)
        self.tilt_success_angle = math.radians(
            rospy.get_param("~tilt_success_angle_deg", 5.0))
        self.maximum_center_drop = rospy.get_param(
            "~maximum_center_drop", 0.015)
        self.maximum_horizontal_displacement = rospy.get_param(
            "~maximum_horizontal_displacement", 0.30)

    @staticmethod
    def object_tilt(pose):
        """Return the angle between the object's local Z axis and world Z."""
        q = pose.orientation
        local_z_world_z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        local_z_world_z = max(-1.0, min(1.0, local_z_world_z))
        return math.acos(local_z_world_z)

    @staticmethod
    def upward_force(forces, baseline):
        return sum(max(0.0, force.z - zero[2])
                   for force, zero in zip(forces, baseline))

    def hold_staging(self, staging_radius, duration):
        deadline = time.monotonic() + duration
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            _, odom, _, object_pose = self.snapshot()
            ax, ay, yaw, _ = self.approach_acceleration(
                odom, object_pose, staging_radius)
            self.publish_nav(ax, ay, yaw, self.object_z)
            time.sleep(self.period)

    def run(self):
        if not self.wait_for_data():
            rospy.logerr("tilt-lift demo: required Bee1 or object topics are missing")
            return False
        if not self.arm_and_takeoff():
            rospy.logerr("tilt-lift demo: Bee1 did not reach HOVER")
            return False

        _, _, _, initial_object_pose = self.snapshot()
        initial_object_x = initial_object_pose.position.x
        initial_object_y = initial_object_pose.position.y
        initial_object_z = initial_object_pose.position.z
        initial_tilt = self.object_tilt(initial_object_pose)
        inradius = self.triangle_side / (2.0 * math.sqrt(3.0))
        staging_radius = inradius + self.staging_clearance
        contact_radius = (inradius + self.foot_contact_offset +
                          self.foot_ball_radius - self.foot_max_compression)

        rospy.loginfo(
            "tilt-lift demo: recover Bee1 to staging radius %.3f m",
            staging_radius)
        stage_deadline = time.monotonic() + 90.0
        while not rospy.is_shutdown() and time.monotonic() < stage_deadline:
            _, odom, _, object_pose = self.snapshot()
            ax, ay, yaw, radius = self.approach_acceleration(
                odom, object_pose, staging_radius)
            self.publish_nav(ax, ay, yaw, self.object_z)
            _, _, _, _, tangent_position, radial_velocity, tangent_velocity = \
                self.relative_state(odom, object_pose)
            if (abs(radius - staging_radius) < 0.06 and
                    abs(tangent_position) < 0.06 and
                    abs(radial_velocity) < 0.08 and
                    abs(tangent_velocity) < 0.08):
                break
            time.sleep(self.period)
        else:
            rospy.logerr("tilt-lift demo: staging timed out")
            return False

        rospy.loginfo("tilt-lift demo: rotate physical feet toward prism")
        ramp_start = time.monotonic()
        while not rospy.is_shutdown():
            alpha = min(1.0, (time.monotonic() - ramp_start) /
                        max(0.001, self.attitude_ramp_duration))
            blend = alpha * alpha * (3.0 - 2.0 * alpha)
            self.commanded_roll = blend * math.pi / 2.0
            _, odom, _, object_pose = self.snapshot()
            ax, ay, yaw, _ = self.approach_acceleration(
                odom, object_pose, staging_radius)
            self.publish_nav(ax, ay, yaw, self.object_z)
            if alpha >= 1.0:
                break
            time.sleep(self.period)

        # The push demo can start translating while the physical attitude is
        # still catching up.  Give this test a quiet interval so the following
        # upward motion is attributable to the diagonal contact force.
        rospy.loginfo(
            "tilt-lift demo: settle vertical foot plane for %.2f s",
            self.attitude_settle_duration)
        self.hold_staging(staging_radius, self.attitude_settle_duration)

        _, _, forces, _ = self.snapshot()
        baseline = [(force.x, force.y, force.z) for force in forces]
        desired_radius = staging_radius
        last_sim = rospy.get_time()
        approach_start = rospy.get_time()
        rospy.loginfo("tilt-lift demo: approach prism with Bee1")
        while not rospy.is_shutdown():
            now = rospy.get_time()
            # A one-Bee headless scene can run much faster than wall time.
            # Preserve the requested speed across that case while still
            # bounding a large /clock discontinuity.
            dt = max(0.0, min(0.5, now - last_sim))
            last_sim = now
            desired_radius = max(
                contact_radius, desired_radius - self.approach_speed * dt)
            _, odom, forces, object_pose = self.snapshot()
            ax, ay, yaw, _ = self.approach_acceleration(
                odom, object_pose, desired_radius)
            self.publish_nav(ax, ay, yaw, self.object_z)
            measured_force = self.normal_force(forces, baseline)
            if (desired_radius <= contact_radius + 1.0e-6 and
                    measured_force > 0.2):
                break
            if now - approach_start > 40.0:
                rospy.logerr("tilt-lift demo: approach timed out")
                return False
            time.sleep(self.period)

        _, contact_odom, _, _ = self.snapshot()
        contact_z = contact_odom.pose.pose.position.z
        lift_start = rospy.get_time()
        success = False
        rospy.loginfo(
            "tilt-lift demo: press at %.2f N and raise Bee1 %.3f m/s",
            self.target_normal_force, self.lift_speed)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            elapsed = now - lift_start
            _, odom, forces, object_pose = self.snapshot()
            measured_force = self.normal_force(forces, baseline)
            measured_upward_force = self.upward_force(forces, baseline)
            ax, ay, yaw = self.push_acceleration(
                odom, object_pose, measured_force, contact_radius)
            z_target = contact_z + min(
                self.lift_height, max(0.0, elapsed) * self.lift_speed)
            self.publish_nav(ax, ay, yaw, z_target)

            tilt_delta = max(0.0, self.object_tilt(object_pose) - initial_tilt)
            center_rise = object_pose.position.z - initial_object_z
            horizontal_displacement = math.hypot(
                object_pose.position.x - initial_object_x,
                object_pose.position.y - initial_object_y)
            rospy.loginfo_throttle(
                0.5, "tilt-lift demo: normal force %.3f / %.3f N, upward "
                "force %.3f N, Bee z target "
                "%.3f m, object tilt %.2f deg, center rise %.3f m, "
                "horizontal displacement %.3f m",
                measured_force, self.target_normal_force,
                measured_upward_force, z_target,
                math.degrees(tilt_delta), center_rise,
                horizontal_displacement)

            if (tilt_delta >= self.tilt_success_angle and
                    center_rise >= -self.maximum_center_drop and
                    horizontal_displacement <=
                    self.maximum_horizontal_displacement):
                success = True
                rospy.loginfo(
                    "tilt-lift demo: SUCCESS: one side lifted; tilt %.2f deg, "
                    "center rise %.3f m, displacement %.3f m",
                    math.degrees(tilt_delta), center_rise,
                    horizontal_displacement)
                break
            if elapsed > self.lift_timeout:
                rospy.logerr(
                    "tilt-lift demo: timed out; tilt %.2f deg, center rise %.3f m",
                    math.degrees(tilt_delta), center_rise)
                return False
            time.sleep(self.period)

        # One Bee cannot arrest the prism once contact is lost, because it can
        # push but cannot pull it back toward the pedestal.  Stop chasing the
        # object after the lift event and hold Bee1 safely in place.
        if success:
            hold_odom = odom
            rospy.loginfo(
                "tilt-lift demo: lift event complete; holding Bee1 position")
            while not rospy.is_shutdown():
                self.publish_position_hold(hold_odom, yaw)
                time.sleep(0.05)
        return success


def main():
    rospy.init_node("bee1_mujoco_tilt_lift_demo")
    demo = MujocoTiltLiftDemo()
    success = False
    try:
        success = demo.run()
    finally:
        _, odom, _, _ = demo.snapshot()
        if odom is not None:
            demo.publish_position_hold(odom, demo.face_frame()[2])
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
