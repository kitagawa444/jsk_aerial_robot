#!/usr/bin/env python

from __future__ import print_function

import csv
import math
import os
import time

import rospy
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from nav_msgs.msg import Odometry

from mujoco_grasp_demo import MujocoGraspDemo


class MujocoDualPushDemo(MujocoGraspDemo):
    """Verify that Bee1 and Bee2 move the prism along their force resultant."""

    ACTIVE_NAMES = ("bee1", "bee2")

    def __init__(self):
        super(MujocoDualPushDemo, self).__init__()
        # The parent creates all three interfaces.  ACTIVE_NAMES selects the
        # vehicles controlled by this demo (and is overridden by the triple
        # press variant).
        self.names = list(self.ACTIVE_NAMES)
        common_force = rospy.get_param("~target_normal_force", 3.0)
        self.target_normal_forces = dict(
            (name, rospy.get_param(
                "~target_normal_force_{}".format(name), common_force))
            for name in self.names)
        self.push_displacement = rospy.get_param("~push_displacement", 0.08)
        self.drop_height = rospy.get_param("~drop_height", 0.08)
        self.push_timeout = rospy.get_param("~push_timeout", 60.0)
        self.direction_tolerance = math.radians(
            rospy.get_param("~direction_tolerance_deg", 15.0))
        self.force_direction_tolerance = math.radians(
            rospy.get_param("~force_direction_tolerance_deg", 15.0))
        self.minimum_resultant_force = rospy.get_param(
            "~minimum_resultant_force", 0.20)
        # The contact reaction on each Bee points along the outward face
        # normal.  Keep the sign configurable for estimators with the opposite
        # convention, but use the physical external-force convention here.
        self.observer_force_sign = rospy.get_param("~observer_force_sign", 1.0)
        self.observer_force_scale = rospy.get_param("~observer_force_scale", 1.0)
        self.observer_timeout = rospy.get_param("~observer_timeout", 1.0)
        self.observer_baseline_duration = rospy.get_param(
            "~observer_baseline_duration", 1.0)
        self.direction_confirm_samples = rospy.get_param(
            "~direction_confirm_samples", 10)
        self.force_filter_alpha = rospy.get_param("~force_filter_alpha", 0.10)
        self.log_path = os.path.expanduser(rospy.get_param(
            "~log_path", "/tmp/bee_mujoco_dual_push_metrics.csv"))

        self.expected_pub = rospy.Publisher(
            "/mujoco_dual_push/expected_resultant", Vector3Stamped, queue_size=1)
        self.measured_pub = rospy.Publisher(
            "/mujoco_dual_push/measured_resultant", Vector3Stamped, queue_size=1)
        self.displacement_pub = rospy.Publisher(
            "/mujoco_dual_push/object_displacement", Vector3Stamped, queue_size=1)
        self.force_site_pub = rospy.Publisher(
            "/mujoco_dual_push/force_site_resultant",
            Vector3Stamped, queue_size=1)

        self.observer_wrenches = {}
        self.observer_receipt_times = {}
        self.cog_odometry = {}
        for name in self.names:
            rospy.Subscriber(
                "/{}/estimated_external_wrench".format(name),
                WrenchStamped, self.observer_cb, callback_args=name, queue_size=1)
            rospy.Subscriber(
                "/{}/uav/cog/odom".format(name), Odometry,
                self.cog_odom_cb, callback_args=name, queue_size=1)

    def observer_cb(self, message, name):
        with self.lock:
            self.observer_wrenches[name] = message
            self.observer_receipt_times[name] = time.monotonic()

    def cog_odom_cb(self, message, name):
        with self.lock:
            self.cog_odometry[name] = message

    @staticmethod
    def vector_angle(vector):
        return math.atan2(vector[1], vector[0])

    @staticmethod
    def vector_norm(vector):
        return math.hypot(vector[0], vector[1])

    @staticmethod
    def angle_difference(first, second):
        return math.atan2(math.sin(first - second), math.cos(first - second))

    def wait_for_dual_data(self, timeout=20.0):
        deadline = time.monotonic() + timeout
        rospy.loginfo(
            "push demo: waiting for %s odometry and object pose",
            ",".join(self.names))
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                ready = (all(name in self.flight_states for name in self.names) and
                         all(name in self.odometry for name in self.names) and
                         all(name in self.cog_odometry for name in self.names) and
                         self.object_pose is not None)
            if ready:
                return True
            time.sleep(0.05)
        return False

    def wait_for_observers(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        rospy.loginfo(
            "push demo: waiting for %s momentum observers",
            ",".join(self.names))
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                ready = all(name in self.observer_wrenches
                            for name in self.names)
            if ready:
                return True
            time.sleep(0.05)
        return False

    def dual_snapshot(self):
        with self.lock:
            states = dict((name, self.flight_states.get(name))
                          for name in self.names)
            odometry = dict((name, self.odometry.get(name))
                            for name in self.names)
            forces = dict((name, list(self.world_forces.get(name, [])))
                          for name in self.names)
            object_pose = self.object_pose
        return states, odometry, forces, object_pose

    def observer_snapshot(self):
        with self.lock:
            wrenches = dict(self.observer_wrenches)
            receipt_times = dict(self.observer_receipt_times)
            cog_odometry = dict(self.cog_odometry)
        return wrenches, receipt_times, cog_odometry

    @staticmethod
    def rotate_cog_to_world(vector, quaternion):
        # geometry_msgs uses quaternion order x,y,z,w.  The observer publishes
        # force in CoG coordinates; odometry carries the CoG-to-world attitude.
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1.0e-9:
            return vector
        x /= norm
        y /= norm
        z /= norm
        w /= norm
        vx, vy, vz = vector
        # q * [v,0] * conjugate(q), expanded to avoid a tf dependency.
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return (vx + w * tx + y * tz - z * ty,
                vy + w * ty + z * tx - x * tz,
                vz + w * tz + x * ty - y * tx)

    def observer_force_world(self, name):
        wrenches, receipt_times, cog_odometry = self.observer_snapshot()
        if name not in wrenches:
            raise RuntimeError("{} momentum observer is unavailable".format(name))
        age = time.monotonic() - receipt_times[name]
        if age > self.observer_timeout:
            raise RuntimeError(
                "{} momentum observer is stale ({:.2f} s)".format(name, age))
        force = wrenches[name].wrench.force
        return self.rotate_cog_to_world(
            (force.x, force.y, force.z),
            cog_odometry[name].pose.pose.orientation)

    def observer_normal_force(self, name, baseline):
        force = self.observer_force_world(name)
        normal, _, _ = self.face_frame(name)
        delta_x = force[0] - baseline[name][0]
        delta_y = force[1] - baseline[name][1]
        signed_force = self.observer_force_scale * self.observer_force_sign * (
            delta_x * normal[0] + delta_y * normal[1])
        return max(0.0, signed_force)

    def force_baseline_dual(self):
        _, _, forces, _ = self.dual_snapshot()
        return dict((name, [(force.x, force.y, force.z)
                            for force in forces[name]])
                    for name in self.names)

    def observer_baseline_dual(self, staging_radius):
        totals = dict((name, [0.0, 0.0, 0.0]) for name in self.names)
        samples = 0
        deadline = time.monotonic() + self.observer_baseline_duration
        rospy.loginfo(
            "dual push: calibrate momentum-observer baseline for %.2f s",
            self.observer_baseline_duration)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            _, odometry, _, object_pose = self.dual_snapshot()
            for name in self.names:
                force = self.observer_force_world(name)
                for axis in range(3):
                    totals[name][axis] += force[axis]
                ax, ay, yaw, _, _, _, _ = self.approach_acceleration(
                    name, odometry[name], object_pose, staging_radius)
                self.publish_accel_nav(name, ax, ay, yaw)
            samples += 1
            time.sleep(self.control_period)
        if samples == 0:
            raise RuntimeError("momentum-observer baseline has no samples")
        baseline = dict(
            (name, tuple(value / samples for value in totals[name]))
            for name in self.names)
        for name in self.names:
            rospy.loginfo(
                "dual push: %s observer baseline world [%.3f, %.3f, %.3f] N",
                name, baseline[name][0], baseline[name][1], baseline[name][2])
        return baseline

    def foot_normal_forces(self, name, forces, baseline):
        normal, _, _ = self.face_frame(name)
        values = []
        for force, zero in zip(forces[name], baseline[name]):
            delta_x = force.x - zero[0]
            delta_y = force.y - zero[1]
            values.append(abs(delta_x * normal[0] + delta_y * normal[1]))
        return values

    def expected_resultant(self):
        result_x = 0.0
        result_y = 0.0
        for name in self.names:
            normal, _, _ = self.face_frame(name)
            # face_frame normal points from the object toward the Bee.  The
            # force applied to the object is in the opposite direction.
            result_x -= self.target_normal_forces[name] * normal[0]
            result_y -= self.target_normal_forces[name] * normal[1]
        return result_x, result_y

    def measured_resultant(self, normal_forces):
        result_x = 0.0
        result_y = 0.0
        for name in self.names:
            normal, _, _ = self.face_frame(name)
            magnitude = normal_forces[name]
            result_x -= magnitude * normal[0]
            result_y -= magnitude * normal[1]
        return result_x, result_y

    def publish_vectors(self, expected, measured, force_site, displacement):
        stamp = rospy.Time.now()
        for publisher, vector in (
                (self.expected_pub, expected),
                (self.measured_pub, measured),
                (self.force_site_pub, force_site),
                (self.displacement_pub, displacement)):
            message = Vector3Stamped()
            message.header.stamp = stamp
            message.header.frame_id = "world"
            message.vector.x = vector[0]
            message.vector.y = vector[1]
            publisher.publish(message)

    def approach_acceleration(self, name, odom, object_pose, desired_radius):
        normal, tangent, yaw = self.face_frame(name)
        position = odom.pose.pose.position
        velocity = odom.twist.twist.linear
        dx = position.x - object_pose.position.x
        dy = position.y - object_pose.position.y
        radius = dx * normal[0] + dy * normal[1]
        tangent_position = dx * tangent[0] + dy * tangent[1]
        radial_velocity = velocity.x * normal[0] + velocity.y * normal[1]
        tangent_velocity = velocity.x * tangent[0] + velocity.y * tangent[1]
        radial = (self.radial_kp * (desired_radius - radius) -
                  self.radial_kd * radial_velocity)
        tangential = (-self.tangent_kp * tangent_position -
                      self.tangent_kd * tangent_velocity)
        radial = self.clamp(radial, self.approach_accel_limit)
        tangential = self.clamp(tangential, self.approach_accel_limit)
        return (radial * normal[0] + tangential * tangent[0],
                radial * normal[1] + tangential * tangent[1], yaw, radius,
                tangent_position, radial_velocity, tangent_velocity)

    def press_acceleration(self, name, odom, object_pose, measured_force,
                           desired_radius):
        normal, tangent, yaw = self.face_frame(name)
        position = odom.pose.pose.position
        velocity = odom.twist.twist.linear
        dx = position.x - object_pose.position.x
        dy = position.y - object_pose.position.y
        radius = dx * normal[0] + dy * normal[1]
        tangent_position = dx * tangent[0] + dy * tangent[1]
        radial_velocity = velocity.x * normal[0] + velocity.y * normal[1]
        tangent_velocity = velocity.x * tangent[0] + velocity.y * tangent[1]
        force_error = self.target_normal_forces[name] - measured_force
        radial = -self.normal_force_kp * force_error - self.radial_kd * radial_velocity
        if radius < desired_radius - self.press_radius_safety_margin:
            radial = (self.press_radial_kp * (desired_radius - radius) -
                      self.radial_kd * radial_velocity)
        tangential = (-self.tangent_kp * tangent_position -
                      self.tangent_kd * tangent_velocity)
        radial = self.clamp(radial, self.press_accel_limit)
        tangential = self.clamp(tangential, self.approach_accel_limit)
        return (radial * normal[0] + tangential * tangent[0],
                radial * normal[1] + tangential * tangent[1], yaw)

    def arm_and_takeoff(self):
        with self.lock:
            takeoff_names = [name for name in self.names
                             if self.flight_states.get(name) != 5]
        if not takeoff_names:
            return True
        rospy.loginfo("dual push: arm/takeoff %s only", ",".join(takeoff_names))
        self.broadcast_empty(self.start_pubs, takeoff_names)
        time.sleep(0.5)
        self.broadcast_empty(self.takeoff_pubs, takeoff_names)
        deadline = time.monotonic() + 45.0
        retry = time.monotonic() + 2.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                states = dict(self.flight_states)
            if all(states.get(name) == 5 for name in self.names):
                return True
            if time.monotonic() >= retry:
                stopped = [name for name in takeoff_names if states.get(name) == 0]
                if stopped:
                    self.broadcast_empty(self.start_pubs, stopped)
                    time.sleep(0.2)
                    self.broadcast_empty(self.takeoff_pubs, stopped)
                retry = time.monotonic() + 2.0
            time.sleep(0.05)
        return False

    def hold_current_positions(self):
        _, odometry, _, _ = self.dual_snapshot()
        rospy.loginfo("dual push: holding Bee1 and Bee2 positions")
        while not rospy.is_shutdown():
            for name in self.names:
                position = odometry[name].pose.pose.position
                message = self._position_message(name, position)
                self.nav_pubs[name].publish(message)
                baselink = Vector3Stamped()
                baselink.header.stamp = message.header.stamp
                baselink.header.frame_id = "world"
                baselink.vector.x = self.commanded_roll
                self.baselink_rpy_pubs[name].publish(baselink)
            time.sleep(0.05)
        return True

    def _position_message(self, name, position):
        message = FlightNav()
        message.header.stamp = rospy.Time.now()
        message.control_frame = FlightNav.WORLD_FRAME
        message.target = FlightNav.COG
        message.pos_xy_nav_mode = FlightNav.POS_MODE
        message.target_pos_x = position.x
        message.target_pos_y = position.y
        message.pos_z_nav_mode = FlightNav.POS_MODE
        message.target_pos_z = position.z
        message.yaw_nav_mode = FlightNav.POS_MODE
        message.target_yaw = self.face_frame(name)[2]
        message.roll_nav_mode = FlightNav.POS_MODE
        message.pitch_nav_mode = FlightNav.POS_MODE
        return message

    def open_dual_log(self):
        directory = os.path.dirname(self.log_path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        log_file = open(self.log_path, "w")
        fields = [
            "sim_time", "bee1_observer_normal_force",
            "bee2_observer_normal_force", "bee1_force_site_normal_force",
            "bee2_force_site_normal_force", "expected_x", "expected_y",
            "observer_resultant_x", "observer_resultant_y",
            "force_site_resultant_x", "force_site_resultant_y",
            "displacement_x", "displacement_y", "displacement", "drop",
            "target_force_angle_deg", "observer_force_angle_deg",
            "force_site_angle_deg",
            "displacement_angle_deg", "force_target_error_deg",
            "motion_force_error_deg",
        ]
        writer = csv.DictWriter(log_file, fieldnames=fields)
        writer.writeheader()
        return log_file, writer

    def run(self):
        if not self.wait_for_dual_data():
            rospy.logerr("dual push: required Bee1, Bee2, or object data is missing")
            return False
        if not self.arm_and_takeoff():
            rospy.logerr("dual push: Bee1 and Bee2 did not reach HOVER")
            return False
        if not self.wait_for_observers():
            rospy.logerr("dual push: momentum-observer topics did not start")
            return False

        inradius = self.triangle_side / (2.0 * math.sqrt(3.0))
        staging_radius = inradius + self.staging_clearance
        contact_radius = (inradius + self.foot_contact_offset +
                          self.foot_ball_radius - self.foot_max_compression)

        rospy.loginfo("dual push: recover both Bees to staging radius %.3f m",
                      staging_radius)
        stage_deadline = time.monotonic() + 180.0
        while not rospy.is_shutdown() and time.monotonic() < stage_deadline:
            _, odometry, _, object_pose = self.dual_snapshot()
            settled = True
            for name in self.names:
                ax, ay, yaw, radius, tangent, radial_velocity, tangent_velocity = \
                    self.approach_acceleration(
                        name, odometry[name], object_pose, staging_radius)
                self.publish_accel_nav(name, ax, ay, yaw)
                settled = (settled and abs(radius - staging_radius) < 0.06 and
                           abs(tangent) < 0.06 and
                           abs(radial_velocity) < 0.08 and
                           abs(tangent_velocity) < 0.08)
            if settled:
                break
            time.sleep(self.control_period)
        else:
            rospy.logerr("dual push: staging timed out")
            return False

        rospy.loginfo("dual push: rotate both physical foot planes toward prism")
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

        # From this point onward the observer is the only force-feedback
        # source.  The approach-to-press phase transition is based only on the
        # commanded geometry; force sites remain an independent MuJoCo-only
        # validation channel and never affect a command.
        force_site_baseline = self.force_baseline_dual()
        observer_baseline = self.observer_baseline_dual(staging_radius)
        desired_radii = dict((name, staging_radius) for name in self.names)
        approach_complete_count = 0
        last_sim = rospy.get_time()
        approach_start = last_sim
        rospy.loginfo("dual push: simultaneous approach")
        while not rospy.is_shutdown():
            now = rospy.get_time()
            dt = max(0.0, min(0.5, now - last_sim))
            last_sim = now
            _, odometry, forces, object_pose = self.dual_snapshot()
            approach_observer_forces = {}
            approach_force_site_forces = {}
            for name in self.names:
                desired_radii[name] = max(
                    contact_radius, desired_radii[name] - self.approach_speed * dt)
                observer_force = self.observer_normal_force(
                    name, observer_baseline)
                approach_observer_forces[name] = observer_force
                approach_force_site_forces[name] = sum(
                    self.foot_normal_forces(
                        name, forces, force_site_baseline))
                ax, ay, yaw, _, _, _, _ = self.approach_acceleration(
                    name, odometry[name], object_pose, desired_radii[name])
                self.publish_accel_nav(name, ax, ay, yaw)
            at_contact_target = all(
                desired_radii[name] <= contact_radius + 1.0e-6
                for name in self.names)
            approach_complete_count = (approach_complete_count + 1
                                       if at_contact_target else 0)
            rospy.loginfo_throttle(
                0.5, "dual push approach: observer F1 %.3f N, F2 %.3f N; "
                "force-site F1 %.3f N, F2 %.3f N",
                approach_observer_forces["bee1"],
                approach_observer_forces["bee2"],
                approach_force_site_forces["bee1"],
                approach_force_site_forces["bee2"])
            if approach_complete_count >= self.contact_confirm_samples:
                break
            if now - approach_start > self.approach_timeout:
                rospy.logerr("dual push: approach timed out")
                return False
            time.sleep(self.control_period)

        _, odometry, forces, _ = self.dual_snapshot()
        for name in self.names:
            observer_force = self.observer_normal_force(
                name, observer_baseline)
            force_site_force = sum(self.foot_normal_forces(
                name, forces, force_site_baseline))
            rospy.loginfo(
                "dual push: %s contact observer %.3f N, force-site %.3f N",
                name, observer_force, force_site_force)

        _, _, _, push_start_pose = self.dual_snapshot()
        start_x = push_start_pose.position.x
        start_y = push_start_pose.position.y
        start_z = push_start_pose.position.z
        expected = self.expected_resultant()
        if self.vector_norm(expected) < 1.0e-6:
            rospy.logerr("dual push: target forces cancel; resultant is undefined")
            return False
        expected_angle = self.vector_angle(expected)
        rospy.loginfo(
            "dual push: simultaneous press Bee1 %.2f N + Bee2 %.2f N; "
            "expected resultant %.2f deg",
            self.target_normal_forces["bee1"],
            self.target_normal_forces["bee2"], math.degrees(expected_angle))

        log_file, log_writer = self.open_dual_log()
        filtered_observer_forces = dict(
            (name, self.observer_normal_force(name, observer_baseline))
            for name in self.names)
        filtered_force_site_forces = dict((name, 0.0) for name in self.names)
        confirm_count = 0
        direction_verified = False
        verified_measured_angle = expected_angle
        verified_displacement_angle = expected_angle
        push_start = rospy.get_time()
        try:
            while not rospy.is_shutdown():
                now = rospy.get_time()
                _, odometry, forces, object_pose = self.dual_snapshot()
                observer_forces = {}
                force_site_forces = {}
                for name in self.names:
                    measured_observer = self.observer_normal_force(
                        name, observer_baseline)
                    measured_force_site = sum(self.foot_normal_forces(
                        name, forces, force_site_baseline))
                    alpha = max(0.0, min(1.0, self.force_filter_alpha))
                    filtered_observer_forces[name] += alpha * (
                        measured_observer - filtered_observer_forces[name])
                    filtered_force_site_forces[name] += alpha * (
                        measured_force_site - filtered_force_site_forces[name])
                    observer_forces[name] = filtered_observer_forces[name]
                    force_site_forces[name] = filtered_force_site_forces[name]
                    ax, ay, yaw = self.press_acceleration(
                        name, odometry[name], object_pose,
                        filtered_observer_forces[name],
                        contact_radius)
                    self.publish_accel_nav(name, ax, ay, yaw)

                measured_resultant = self.measured_resultant(observer_forces)
                force_site_resultant = self.measured_resultant(force_site_forces)
                displacement = (object_pose.position.x - start_x,
                                object_pose.position.y - start_y)
                displacement_norm = self.vector_norm(displacement)
                drop = start_z - object_pose.position.z
                measured_norm = self.vector_norm(measured_resultant)
                measured_angle = (self.vector_angle(measured_resultant)
                                  if measured_norm > 1.0e-6 else expected_angle)
                force_site_norm = self.vector_norm(force_site_resultant)
                force_site_angle = (self.vector_angle(force_site_resultant)
                                    if force_site_norm > 1.0e-6
                                    else expected_angle)
                displacement_angle = (self.vector_angle(displacement)
                                      if displacement_norm > 1.0e-6
                                      else expected_angle)
                force_target_error = self.angle_difference(
                    measured_angle, expected_angle)
                motion_force_error = self.angle_difference(
                    displacement_angle, measured_angle)
                self.publish_vectors(
                    expected, measured_resultant, force_site_resultant,
                    displacement)

                log_writer.writerow({
                    "sim_time": "{:.6f}".format(now),
                    "bee1_observer_normal_force": "{:.8f}".format(
                        observer_forces["bee1"]),
                    "bee2_observer_normal_force": "{:.8f}".format(
                        observer_forces["bee2"]),
                    "bee1_force_site_normal_force": "{:.8f}".format(
                        force_site_forces["bee1"]),
                    "bee2_force_site_normal_force": "{:.8f}".format(
                        force_site_forces["bee2"]),
                    "expected_x": "{:.8f}".format(expected[0]),
                    "expected_y": "{:.8f}".format(expected[1]),
                    "observer_resultant_x": "{:.8f}".format(
                        measured_resultant[0]),
                    "observer_resultant_y": "{:.8f}".format(
                        measured_resultant[1]),
                    "force_site_resultant_x": "{:.8f}".format(
                        force_site_resultant[0]),
                    "force_site_resultant_y": "{:.8f}".format(
                        force_site_resultant[1]),
                    "displacement_x": "{:.8f}".format(displacement[0]),
                    "displacement_y": "{:.8f}".format(displacement[1]),
                    "displacement": "{:.8f}".format(displacement_norm),
                    "drop": "{:.8f}".format(drop),
                    "target_force_angle_deg": "{:.5f}".format(
                        math.degrees(expected_angle)),
                    "observer_force_angle_deg": "{:.5f}".format(
                        math.degrees(measured_angle)),
                    "force_site_angle_deg": "{:.5f}".format(
                        math.degrees(force_site_angle)),
                    "displacement_angle_deg": "{:.5f}".format(
                        math.degrees(displacement_angle)),
                    "force_target_error_deg": "{:.5f}".format(
                        math.degrees(force_target_error)),
                    "motion_force_error_deg": "{:.5f}".format(
                        math.degrees(motion_force_error)),
                })
                log_file.flush()

                direction_ok = (
                    measured_norm >= self.minimum_resultant_force and
                    abs(force_target_error) <= self.force_direction_tolerance and
                    abs(motion_force_error) <= self.direction_tolerance)
                confirm_count = confirm_count + 1 if (
                    direction_ok and displacement_norm >= self.push_displacement) else 0
                rospy.loginfo_throttle(
                    0.5, "dual push: observer F1 %.3f N, F2 %.3f N, "
                    "resultant %.2f deg (target error %.2f deg), force-site "
                    "F1 %.3f N, F2 %.3f N at %.2f deg, displacement %.3f m "
                    "at %.2f deg (observer error %.2f deg), drop %.3f m",
                    observer_forces["bee1"], observer_forces["bee2"],
                    math.degrees(measured_angle),
                    math.degrees(force_target_error),
                    force_site_forces["bee1"], force_site_forces["bee2"],
                    math.degrees(force_site_angle), displacement_norm,
                    math.degrees(displacement_angle),
                    math.degrees(motion_force_error), drop)

                if (not direction_verified and
                        confirm_count >= self.direction_confirm_samples):
                    direction_verified = True
                    verified_measured_angle = measured_angle
                    verified_displacement_angle = displacement_angle
                    rospy.loginfo(
                        "dual push: direction verified; continue pushing until "
                        "the object falls. target %.2f deg, measured %.2f deg, "
                        "motion %.2f deg",
                        math.degrees(expected_angle), math.degrees(measured_angle),
                        math.degrees(displacement_angle))
                if drop >= self.drop_height:
                    if not direction_verified:
                        rospy.logerr(
                            "dual push: object fell before resultant direction "
                            "was verified")
                        return False
                    rospy.loginfo(
                        "dual push: SUCCESS: object followed the measured "
                        "resultant; target %.2f deg, measured %.2f deg, "
                        "motion %.2f deg, then moved %.3f m and fell %.3f m",
                        math.degrees(expected_angle),
                        math.degrees(verified_measured_angle),
                        math.degrees(verified_displacement_angle),
                        displacement_norm, drop)
                    return self.hold_current_positions()
                if now - push_start > self.push_timeout:
                    rospy.logerr(
                        "dual push: timed out; target %.2f deg, measured %.2f deg, "
                        "motion %.2f deg, displacement %.3f m, drop %.3f m",
                        math.degrees(expected_angle), math.degrees(measured_angle),
                        math.degrees(displacement_angle), displacement_norm, drop)
                    return False
                time.sleep(self.control_period)
        finally:
            log_file.close()
            rospy.loginfo("dual push: metrics written to %s", self.log_path)


def main():
    rospy.init_node("bee_mujoco_dual_push_demo")
    demo = MujocoDualPushDemo()
    try:
        if not demo.run():
            raise SystemExit(1)
    finally:
        if demo.odometry:
            demo.publish_stop()


if __name__ == "__main__":
    main()
