#!/usr/bin/env python

from __future__ import print_function

import csv
import math
import os
import threading
import time

import rospy
from aerial_robot_msgs.msg import FlightNav, ForceList
from geometry_msgs.msg import PoseStamped, Vector3Stamped, WrenchStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, UInt8


class MujocoGraspDemo(object):
    """Three-Bee friction-grasp test using only the flight controller."""

    FOOT_NAMES = [
        "spring_foot_front",
        "spring_foot_rear",
        "spring_foot_left",
        "spring_foot_right",
    ]

    def __init__(self):
        self.names = ["bee1", "bee2", "bee3"]
        self.lock = threading.Lock()
        self.flight_states = {}
        self.odometry = {}
        self.world_forces = {}
        self.contact_states = {}
        self.object_pose = None

        self.object_x = rospy.get_param("~object_x", 0.0)
        self.object_y = rospy.get_param("~object_y", 1.0)
        self.object_z = rospy.get_param("~object_z", 0.802)
        self.object_yaw = rospy.get_param("~object_yaw", 0.0)
        self.object_mass = rospy.get_param("~object_mass", 0.20)
        self.robot_mass = rospy.get_param("~robot_mass", 1.84199)
        self.triangle_side = rospy.get_param("~object_triangle_side", 0.80)
        self.staging_clearance = rospy.get_param("~grasp_staging_clearance", 0.80)

        self.control_period = rospy.get_param("~control_period", 0.01)
        self.staging_timeout = rospy.get_param("~staging_timeout", 150.0)
        self.staging_tolerance = rospy.get_param("~staging_tolerance", 0.08)
        self.approach_speed = rospy.get_param("~approach_speed", 0.035)
        self.approach_timeout = rospy.get_param("~approach_timeout", 35.0)
        self.radial_kp = rospy.get_param("~radial_kp", 3.0)
        self.radial_kd = rospy.get_param("~radial_kd", 3.0)
        self.tangent_kp = rospy.get_param("~tangent_kp", 1.2)
        self.tangent_kd = rospy.get_param("~tangent_kd", 3.0)
        self.approach_accel_limit = rospy.get_param("~approach_accel_limit", 0.80)
        self.press_accel_limit = rospy.get_param("~press_accel_limit", 15.0)
        self.approach_roll = rospy.get_param("~approach_roll", math.pi / 2.0)
        self.attitude_ramp_duration = rospy.get_param("~attitude_ramp_duration", 3.0)
        self.attitude_settle_duration = rospy.get_param("~attitude_settle_duration", 1.0)
        self.attitude_tolerance = rospy.get_param("~attitude_tolerance", 0.10)
        self.commanded_roll = 0.0
        # Physical-foot ball centre: base_link z=-0.083 m plus 24 mm
        # free-length offset along local -Z.
        self.foot_contact_offset = rospy.get_param("~foot_contact_offset", 0.107)
        self.foot_ball_radius = rospy.get_param("~foot_ball_radius", 0.012)
        self.foot_max_compression = rospy.get_param("~foot_max_compression", 0.015)
        # Bee's current MuJoCo attitude convention reflects local X but not
        # local Y.  Keep both axis conversions explicit for model variants.
        self.accel_command_sign_x = rospy.get_param("~accel_command_sign_x", 1.0)
        self.accel_command_sign_y = rospy.get_param("~accel_command_sign_y", 1.0)
        self.use_external_wrench = rospy.get_param("~use_external_wrench", False)
        self.force_trial_mode = rospy.get_param("~force_trial_mode", True)
        self.external_lift_force = rospy.get_param("~external_lift_force", 5.0)
        self.hover_handover_duration = rospy.get_param("~hover_handover_duration", 4.0)
        self.compression_target = rospy.get_param("~compression_target", 0.020)
        self.compression_gain = rospy.get_param("~compression_gain", 400.0)
        self.press_radial_kp = rospy.get_param("~press_radial_kp", 400.0)
        self.target_normal_force = rospy.get_param("~target_normal_force", 5.0)
        self.normal_force_kp = rospy.get_param("~normal_force_kp", 0.40)
        self.press_radius_safety_margin = rospy.get_param(
            "~press_radius_safety_margin", 0.05)
        self.touch_threshold = rospy.get_param("~touch_threshold", 0.02)
        self.normal_force_threshold = rospy.get_param("~normal_force_threshold", 0.10)
        self.compression_threshold = rospy.get_param("~compression_threshold", 0.0002)
        self.contact_confirm_samples = rospy.get_param("~contact_confirm_samples", 10)
        self.preload_duration = rospy.get_param("~preload_duration", 3.0)
        self.preload_hold_mode = rospy.get_param("~preload_hold_mode", False)

        self.lift_velocity = rospy.get_param("~lift_velocity", 0.05)
        self.lift_height = rospy.get_param("~lift_height", 0.08)
        self.lift_timeout = rospy.get_param("~lift_timeout", 12.0)
        self.force_margin = rospy.get_param("~force_margin", 1.02)
        self.force_confirm_samples = rospy.get_param("~force_confirm_samples", 10)
        self.log_path = os.path.expanduser(rospy.get_param(
            "~log_path", "/tmp/bee_mujoco_grasp_metrics.csv"))

        self.start_pubs = {}
        self.takeoff_pubs = {}
        self.nav_pubs = {}
        self.baselink_rpy_pubs = {}
        self.wrench_pubs = {}
        for name in self.names:
            self.start_pubs[name] = rospy.Publisher(
                "/{}/teleop_command/start".format(name), Empty, queue_size=1)
            self.takeoff_pubs[name] = rospy.Publisher(
                "/{}/teleop_command/takeoff".format(name), Empty, queue_size=1)
            self.nav_pubs[name] = rospy.Publisher(
                "/{}/uav/nav".format(name), FlightNav, queue_size=1)
            self.baselink_rpy_pubs[name] = rospy.Publisher(
                "/{}/final_target_baselink_rpy".format(name),
                Vector3Stamped,
                queue_size=1)
            self.wrench_pubs[name] = rospy.Publisher(
                "/{}/mujoco/external_wrench".format(name), WrenchStamped, queue_size=1)
            rospy.Subscriber("/{}/flight_state".format(name), UInt8,
                             self.flight_state_cb, callback_args=name, queue_size=1)
            rospy.Subscriber("/{}/ground_truth".format(name), Odometry,
                             self.odometry_cb, callback_args=name, queue_size=1)
            rospy.Subscriber("/{}/mujoco/grasp_forces_world".format(name), ForceList,
                             self.force_cb, callback_args=name, queue_size=1)
            rospy.Subscriber("/{}/mujoco/grasp_contact_states".format(name), JointState,
                             self.contact_state_cb, callback_args=name, queue_size=1)

        rospy.Subscriber("/bee1/mujoco/grasp_object_pose", PoseStamped,
                         self.object_pose_cb, queue_size=1)

        self.log_file = None
        self.log_writer = None

    def flight_state_cb(self, msg, name):
        with self.lock:
            self.flight_states[name] = msg.data

    def odometry_cb(self, msg, name):
        with self.lock:
            self.odometry[name] = msg

    def force_cb(self, msg, name):
        with self.lock:
            self.world_forces[name] = list(msg.forces)

    def contact_state_cb(self, msg, name):
        if (len(msg.name) != 4 or len(msg.position) != 4 or
                len(msg.velocity) != 4 or len(msg.effort) != 4):
            rospy.logwarn_throttle(
                1.0, "%s: expected four complete physical-foot contact states", name)
            return
        if list(msg.name) != self.FOOT_NAMES:
            rospy.logwarn_throttle(
                1.0, "%s: unexpected physical-foot order: %s",
                name, ", ".join(msg.name))
            return
        with self.lock:
            self.contact_states[name] = msg

    def object_pose_cb(self, msg):
        with self.lock:
            self.object_pose = msg.pose

    def wait_for_data(self, timeout=20.0):
        rospy.loginfo(
            "grasp demo: waiting for three Bee flight states, odometry, "
            "contact data, and object pose")
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                ready = (all(name in self.flight_states for name in self.names) and
                         all(name in self.odometry for name in self.names) and
                         all(len(self.world_forces.get(name, [])) == 4
                             for name in self.names) and
                         all(name in self.contact_states for name in self.names) and
                         self.object_pose is not None)
            if ready:
                rospy.loginfo("grasp demo: all required input topics are ready")
                return True
            time.sleep(0.05)
        return False

    def broadcast_empty(self, publishers, selected_names=None):
        selected_names = selected_names if selected_names is not None else self.names
        deadline = time.monotonic() + 10.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if all(publishers[name].get_num_connections() > 0 for name in selected_names):
                break
            time.sleep(0.05)
        for name in selected_names:
            publishers[name].publish(Empty())

    def all_hovering(self):
        with self.lock:
            # flight_navigation.h: HOVER_STATE=5.
            return all(self.flight_states.get(name) == 5 for name in self.names)

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    @staticmethod
    def vector_delta(force, baseline):
        return (force.x - baseline[0], force.y - baseline[1], force.z - baseline[2])

    @staticmethod
    def quaternion_to_rpy(q):
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def angle_error(target, actual):
        return math.atan2(math.sin(target - actual), math.cos(target - actual))

    def face_frame(self, name):
        index = self.names.index(name)
        angle = self.object_yaw + (math.pi / 3.0, math.pi, 5.0 * math.pi / 3.0)[index]
        normal = (math.cos(angle), math.sin(angle))       # object -> Bee
        tangent = (-normal[1], normal[0])
        inward_yaw = math.atan2(-normal[1], -normal[0])
        # With base_link roll=+pi/2, physical local -Z (the foot-ball
        # direction) maps to local +Y.  Rotate local +Y onto the inward face
        # normal, hence the additional -pi/2 yaw.
        approach_yaw = self.angle_error(inward_yaw - math.pi / 2.0, 0.0)
        return normal, tangent, approach_yaw

    def snapshot(self):
        with self.lock:
            odometry = dict(self.odometry)
            forces = dict((name, list(values))
                          for name, values in self.world_forces.items())
            contacts = dict(self.contact_states)
            object_pose = self.object_pose
        return odometry, forces, contacts, object_pose

    def force_baseline(self):
        _, forces, _, _ = self.snapshot()
        return dict((name, [(f.x, f.y, f.z) for f in forces[name]])
                    for name in self.names)

    def contact_metrics(self, name, forces, contacts, baseline):
        normal, _, _ = self.face_frame(name)
        contact = contacts[name]
        metrics = []
        for index, foot_name in enumerate(contact.name):
            delta = self.vector_delta(forces[name][index], baseline[name][index])
            metrics.append({
                "name": foot_name,
                "delta": delta,
                "normal": abs(delta[0] * normal[0] + delta[1] * normal[1]),
                "touch": contact.effort[index],
                "compression": contact.position[index],
                "compression_velocity": contact.velocity[index],
            })
        return metrics

    def four_point_contact(self, metrics):
        return all(metric["touch"] > self.touch_threshold and
                   metric["normal"] > self.normal_force_threshold and
                   metric["compression"] > self.compression_threshold
                   for metric in metrics)

    def publish_accel_nav(self, name, accel_x, accel_y, yaw,
                          z_mode=FlightNav.POS_MODE, z_value=None,
                          external_force_z=None):
        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG
        msg.pos_xy_nav_mode = FlightNav.ACC_MODE
        if self.use_external_wrench:
            # Clear the vehicle controller's XY acceleration while a world
            # force is integrated by MuJoCo below.
            msg.target_acc_x = 0.0
            msg.target_acc_y = 0.0
        else:
            msg.target_acc_x = self.accel_command_sign_x * accel_x
            msg.target_acc_y = self.accel_command_sign_y * accel_y
        msg.yaw_nav_mode = FlightNav.POS_MODE
        msg.target_yaw = yaw
        msg.roll_nav_mode = FlightNav.POS_MODE
        msg.target_roll = 0.0
        msg.pitch_nav_mode = FlightNav.POS_MODE
        msg.target_pitch = 0.0
        msg.pos_z_nav_mode = z_mode
        if z_mode == FlightNav.VEL_MODE:
            msg.target_vel_z = z_value if z_value is not None else 0.0
        else:
            msg.target_pos_z = z_value if z_value is not None else self.object_z
        self.nav_pubs[name].publish(msg)

        baselink_rpy = Vector3Stamped()
        baselink_rpy.header.stamp = msg.header.stamp
        baselink_rpy.header.frame_id = "world"
        baselink_rpy.vector.x = self.commanded_roll
        baselink_rpy.vector.y = 0.0
        baselink_rpy.vector.z = 0.0
        self.baselink_rpy_pubs[name].publish(baselink_rpy)

        if self.use_external_wrench:
            wrench = WrenchStamped()
            wrench.header.stamp = msg.header.stamp
            wrench.header.frame_id = "world"
            wrench.wrench.force.x = self.robot_mass * accel_x
            wrench.wrench.force.y = self.robot_mass * accel_y
            if external_force_z is not None:
                wrench.wrench.force.z = external_force_z
            elif z_mode == FlightNav.VEL_MODE:
                wrench.wrench.force.z = self.external_lift_force
            # Attitude is controlled only through final_target_baselink_rpy.
            # A second root torque would fight Bee's base-link controller.
            self.wrench_pubs[name].publish(wrench)

    def publish_stop(self):
        odometry, _, _, _ = self.snapshot()
        for name in self.names:
            msg = FlightNav()
            msg.header.stamp = rospy.Time.now()
            msg.control_frame = FlightNav.WORLD_FRAME
            msg.target = FlightNav.COG
            if name in odometry:
                position = odometry[name].pose.pose.position
                msg.pos_xy_nav_mode = FlightNav.POS_MODE
                msg.target_pos_x = position.x
                msg.target_pos_y = position.y
                msg.pos_z_nav_mode = FlightNav.POS_MODE
                msg.target_pos_z = position.z
            else:
                msg.pos_xy_nav_mode = FlightNav.ACC_MODE
                msg.target_acc_x = 0.0
                msg.target_acc_y = 0.0
                msg.pos_z_nav_mode = FlightNav.VEL_MODE
                msg.target_vel_z = 0.0
            msg.yaw_nav_mode = FlightNav.POS_MODE
            msg.target_yaw = self.face_frame(name)[2]
            msg.roll_nav_mode = FlightNav.POS_MODE
            msg.target_roll = 0.0
            msg.pitch_nav_mode = FlightNav.POS_MODE
            msg.target_pitch = 0.0
            self.nav_pubs[name].publish(msg)
            baselink_rpy = Vector3Stamped()
            baselink_rpy.header.stamp = msg.header.stamp
            baselink_rpy.header.frame_id = "world"
            baselink_rpy.vector.x = self.commanded_roll
            self.baselink_rpy_pubs[name].publish(baselink_rpy)
            wrench = WrenchStamped()
            wrench.header.stamp = msg.header.stamp
            wrench.header.frame_id = "world"
            self.wrench_pubs[name].publish(wrench)

    def ramp_attitude_at_staging(self, desired_radii, desired_tangents):
        """Tilt the physical feet toward the object without a step command."""
        duration = max(0.0, self.attitude_ramp_duration)
        start = time.monotonic()
        rospy.loginfo("grasp demo: ramp base-link roll to %.3f rad over %.2f s",
                      self.approach_roll, duration)
        while not rospy.is_shutdown():
            elapsed = time.monotonic() - start
            alpha = 1.0 if duration == 0.0 else min(1.0, elapsed / duration)
            # Smoothstep gives zero roll-rate at both ends of the transition.
            blend = alpha * alpha * (3.0 - 2.0 * alpha)
            self.commanded_roll = blend * self.approach_roll
            odometry, _, _, _ = self.snapshot()
            for name in self.names:
                ax, ay, yaw, _ = self.control_acceleration(
                    name, odometry[name], desired_radii[name], desired_tangents[name],
                    [], False)
                self.publish_accel_nav(name, ax, ay, yaw)
            if alpha >= 1.0:
                break
            time.sleep(self.control_period)

        settle_deadline = time.monotonic() + self.attitude_settle_duration
        while not rospy.is_shutdown() and time.monotonic() < settle_deadline:
            odometry, _, _, _ = self.snapshot()
            for name in self.names:
                ax, ay, yaw, _ = self.control_acceleration(
                    name, odometry[name], desired_radii[name], desired_tangents[name],
                    [], False)
                self.publish_accel_nav(name, ax, ay, yaw)
            time.sleep(self.control_period)

    def hold_lifted_object(self, desired_radii, desired_tangents, baseline):
        """Keep preload and hover at the achieved lift height until shutdown."""
        odometry, _, _, object_pose = self.snapshot()
        handover_start = rospy.get_time()
        handover_duration = max(0.0, self.hover_handover_duration)
        rospy.loginfo(
            "grasp demo: hand over lifted load to flight controller over %.2f s",
            handover_duration)
        while not rospy.is_shutdown():
            elapsed = rospy.get_time() - handover_start
            alpha = (1.0 if handover_duration == 0.0 else
                     min(1.0, elapsed / handover_duration))
            blend = alpha * alpha * (3.0 - 2.0 * alpha)
            target_velocity = self.lift_velocity * (1.0 - blend)
            trial_force = self.external_lift_force * (1.0 - blend)
            odometry, forces, contacts, object_pose = self.snapshot()
            all_metrics, upward_friction = self.log_metrics(
                "hover_handover", forces, contacts, baseline, object_pose)
            for name in self.names:
                ax, ay, yaw, _ = self.control_acceleration(
                    name, odometry[name], desired_radii[name], desired_tangents[name],
                    all_metrics[name], True)
                self.publish_accel_nav(
                    name, ax, ay, yaw, FlightNav.VEL_MODE, target_velocity,
                    external_force_z=trial_force)
            rospy.loginfo_throttle(
                1.0, "grasp demo: handover object z %.3f m, vz target %.3f m/s, "
                "trial force %.3f N/Bee, z friction %.3f N",
                object_pose.position.z, target_velocity, trial_force, upward_friction)
            if alpha >= 1.0:
                break
            time.sleep(self.control_period)

        rospy.loginfo(
            "grasp demo: HOLD: object z %.3f m; maintaining zero vertical velocity "
            "and inward preload "
            "until shutdown", object_pose.position.z)
        while not rospy.is_shutdown():
            odometry, forces, contacts, object_pose = self.snapshot()
            all_metrics, upward_friction = self.log_metrics(
                "hold", forces, contacts, baseline, object_pose)
            for name in self.names:
                ax, ay, yaw, _ = self.control_acceleration(
                    name, odometry[name], desired_radii[name], desired_tangents[name],
                    all_metrics[name], True, bounded_press=True)
                # VEL=0 is the flight controller's closed-loop hover mode here.
                self.publish_accel_nav(
                    name, ax, ay, yaw, FlightNav.VEL_MODE, 0.0,
                    external_force_z=0.0)
            rospy.loginfo_throttle(
                1.0, "grasp demo: HOLD object z %.3f m, upward z friction %.3f N",
                object_pose.position.z, upward_friction)
            time.sleep(self.control_period)
        return True

    def hold_preload(self, desired_radii, desired_tangents, baseline):
        """Hold the three Bees against the supported object without lifting."""
        rospy.loginfo(
            "grasp demo: PRELOAD HOLD: lift is disabled; maintaining face-normal "
            "preload until shutdown")
        while not rospy.is_shutdown():
            odometry, forces, contacts, object_pose = self.snapshot()
            all_metrics, upward_friction = self.log_metrics(
                "preload_hold", forces, contacts, baseline, object_pose)
            normal_force_sum = 0.0
            for name in self.names:
                normal_force_sum += sum(metric["normal"]
                                        for metric in all_metrics[name])
                ax, ay, yaw, _ = self.control_acceleration(
                    name, odometry[name], desired_radii[name], desired_tangents[name],
                    all_metrics[name], True)
                self.publish_accel_nav(
                    name, ax, ay, yaw, FlightNav.POS_MODE, self.object_z,
                    external_force_z=0.0)
            rospy.loginfo_throttle(
                1.0, "grasp demo: PRELOAD HOLD object z %.3f m, normal force sum "
                "%.3f N, upward z force %.3f N",
                object_pose.position.z, normal_force_sum, upward_friction)
            time.sleep(self.control_period)
        return True

    def open_log(self):
        directory = os.path.dirname(self.log_path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        self.log_file = open(self.log_path, "w")
        fields = [
            "phase", "sim_time", "bee", "foot", "force_x_world",
            "force_y_world", "force_z_world", "touch", "compression",
            "compression_velocity", "normal_force", "upward_friction_sum",
            "object_z",
        ]
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=fields)
        self.log_writer.writeheader()

    def log_metrics(self, phase, forces, contacts, baseline, object_pose):
        upward_friction = 0.0
        all_metrics = {}
        for name in self.names:
            all_metrics[name] = self.contact_metrics(name, forces, contacts, baseline)
            # Positive world-Z readings track the upward friction transmitted
            # from the physical feet to the object in this sensor convention.
            upward_friction += sum(max(0.0, m["delta"][2])
                                   for m in all_metrics[name])
        for name in self.names:
            for metric in all_metrics[name]:
                self.log_writer.writerow({
                    "phase": phase,
                    "sim_time": "{:.6f}".format(rospy.get_time()),
                    "bee": name,
                    "foot": metric["name"],
                    "force_x_world": "{:.8f}".format(metric["delta"][0]),
                    "force_y_world": "{:.8f}".format(metric["delta"][1]),
                    "force_z_world": "{:.8f}".format(metric["delta"][2]),
                    "touch": "{:.8f}".format(metric["touch"]),
                    "compression": "{:.8f}".format(metric["compression"]),
                    "compression_velocity": "{:.8f}".format(metric["compression_velocity"]),
                    "normal_force": "{:.8f}".format(metric["normal"]),
                    "upward_friction_sum": "{:.8f}".format(upward_friction),
                    "object_z": "{:.8f}".format(object_pose.position.z),
                })
        self.log_file.flush()
        return all_metrics, upward_friction

    def control_acceleration(self, name, odom, desired_radius, desired_tangent,
                             metrics, pressing, bounded_press=False):
        normal, tangent, yaw = self.face_frame(name)
        pos = odom.pose.pose.position
        vel = odom.twist.twist.linear
        dx = pos.x - self.object_x
        dy = pos.y - self.object_y
        radius = dx * normal[0] + dy * normal[1]
        tangent_position = dx * tangent[0] + dy * tangent[1]
        radial_velocity = vel.x * normal[0] + vel.y * normal[1]
        tangent_velocity = vel.x * tangent[0] + vel.y * tangent[1]

        if pressing:
            if bounded_press:
                # Radius error changes sign at the target, so HOLD remains
                # bounded even if vertical contact is temporarily lost.
                radial_acceleration = (self.press_radial_kp *
                                       (desired_radius - radius) -
                                       self.radial_kd * radial_velocity)
            else:
                measured_normal_force = sum(m["normal"] for m in metrics)
                force_error = self.target_normal_force - measured_normal_force
                radial_acceleration = (-self.normal_force_kp * force_error -
                                       self.radial_kd * radial_velocity)
                # If contact disappears after crossing the target plane, force
                # feedback alone would keep accelerating inward.  Restore the
                # vehicle toward the commanded face-normal radius instead.
                if radius < desired_radius - self.press_radius_safety_margin:
                    radial_acceleration = (self.press_radial_kp *
                                           (desired_radius - radius) -
                                           self.radial_kd * radial_velocity)
            acceleration_limit = self.press_accel_limit
        else:
            radial_acceleration = (self.radial_kp * (desired_radius - radius) -
                                   self.radial_kd * radial_velocity)
            acceleration_limit = self.approach_accel_limit

        tangent_acceleration = (self.tangent_kp * (desired_tangent - tangent_position) -
                                self.tangent_kd * tangent_velocity)
        radial_acceleration = self.clamp(radial_acceleration, acceleration_limit)
        tangent_acceleration = self.clamp(tangent_acceleration, self.approach_accel_limit)
        accel_x = radial_acceleration * normal[0] + tangent_acceleration * tangent[0]
        accel_y = radial_acceleration * normal[1] + tangent_acceleration * tangent[1]
        return accel_x, accel_y, yaw, radius

    def run(self):
        if not self.wait_for_data():
            rospy.logerr("grasp demo: missing robot, world-force, touch/compression, or object data")
            return False

        self.open_log()
        try:
            _, _, _, object_pose = self.snapshot()
            initial_object_z = object_pose.position.z
            if abs(initial_object_z - self.object_z) > 0.05:
                rospy.logerr("grasp demo: object is not on its initial support (z %.3f, expected %.3f)",
                             initial_object_z, self.object_z)
                return False
            inradius = self.triangle_side / (2.0 * math.sqrt(3.0))
            staging_radius = inradius + self.staging_clearance
            if not self.all_hovering():
                with self.lock:
                    takeoff_names = [name for name in self.names
                                     if self.flight_states.get(name) != 5]
                rospy.loginfo("grasp demo: arm/takeoff %s; initial object z %.3f m",
                              ",".join(takeoff_names), initial_object_z)
                self.broadcast_empty(self.start_pubs, takeoff_names)
                time.sleep(0.5)
                self.broadcast_empty(self.takeoff_pubs, takeoff_names)

                hover_deadline = time.monotonic() + 45.0
                last_state_zero_retry = time.monotonic()
                while not rospy.is_shutdown() and time.monotonic() < hover_deadline:
                    if self.all_hovering():
                        break
                    odometry, _, _, _ = self.snapshot()
                    with self.lock:
                        states = dict(self.flight_states)
                    if time.monotonic() - last_state_zero_retry >= 2.0:
                        state_zero_names = [name for name in takeoff_names
                                            if states.get(name) == 0]
                        if state_zero_names:
                            self.broadcast_empty(self.start_pubs, state_zero_names)
                            time.sleep(0.2)
                            self.broadcast_empty(self.takeoff_pubs, state_zero_names)
                        last_state_zero_retry = time.monotonic()
                    for name in self.names:
                        if states.get(name) != 5 or name not in odometry:
                            continue
                        ax, ay, yaw, _ = self.control_acceleration(
                            name, odometry[name], staging_radius, 0.0, [], False)
                        self.publish_accel_nav(name, ax, ay, yaw)
                    time.sleep(self.control_period)
            else:
                rospy.loginfo("grasp demo: Bees are already hovering; skip arm/takeoff")
            if not self.all_hovering():
                rospy.logerr("grasp demo: all robots did not reach HOVER")
                return False

            # Dampen residual takeoff motion in acceleration mode.  No root or
            # joint state is overwritten anywhere in this test.
            settle_end = time.monotonic() + 2.0
            while not rospy.is_shutdown() and time.monotonic() < settle_end:
                odometry, _, _, _ = self.snapshot()
                for name in self.names:
                    normal, tangent, yaw = self.face_frame(name)
                    vel = odometry[name].twist.twist.linear
                    ax = -self.radial_kd * (vel.x * normal[0] + vel.y * normal[1]) * normal[0]
                    ay = -self.radial_kd * (vel.x * normal[0] + vel.y * normal[1]) * normal[1]
                    tangent_vel = vel.x * tangent[0] + vel.y * tangent[1]
                    ax -= self.tangent_kd * tangent_vel * tangent[0]
                    ay -= self.tangent_kd * tangent_vel * tangent[1]
                    self.publish_accel_nav(name, self.clamp(ax, self.approach_accel_limit),
                                           self.clamp(ay, self.approach_accel_limit), yaw)
                time.sleep(self.control_period)

            odometry, forces, contacts, object_pose = self.snapshot()
            if abs(object_pose.position.z - initial_object_z) > 0.02:
                rospy.logerr("grasp demo: object moved before approach (dz=%.3f m)",
                             object_pose.position.z - initial_object_z)
                return False

            desired_radii = dict((name, staging_radius) for name in self.names)
            desired_tangents = {}
            for name in self.names:
                desired_tangents[name] = 0.0

            # Takeoff can leave appreciable horizontal drift because navigation
            # commands are intentionally ignored during TAKEOFF_STATE.  First
            # recover the face-normal staging points through the same physical
            # acceleration interface used for the approach.
            staging_start_sim = rospy.get_time()
            staging_wall_deadline = time.monotonic() + 180.0
            rospy.loginfo("grasp demo: acceleration-controlled recovery to staging radius %.3f m",
                          staging_radius)
            while not rospy.is_shutdown():
                odometry, _, _, object_pose = self.snapshot()
                settled = True
                for name in self.names:
                    ax, ay, yaw, radius = self.control_acceleration(
                        name, odometry[name], desired_radii[name], desired_tangents[name],
                        [], False)
                    self.publish_accel_nav(name, ax, ay, yaw)
                    normal, tangent, _ = self.face_frame(name)
                    pos = odometry[name].pose.pose.position
                    vel = odometry[name].twist.twist.linear
                    tangent_position = ((pos.x - self.object_x) * tangent[0] +
                                        (pos.y - self.object_y) * tangent[1])
                    planar_speed = math.hypot(vel.x, vel.y)
                    roll, pitch, measured_yaw = self.quaternion_to_rpy(
                        odometry[name].pose.pose.orientation)
                    settled = settled and abs(radius - staging_radius) < self.staging_tolerance
                    settled = settled and abs(tangent_position) < self.staging_tolerance
                    settled = settled and planar_speed < 0.08
                    settled = settled and abs(
                        self.angle_error(self.commanded_roll, roll)) < self.attitude_tolerance
                    settled = settled and abs(pitch) < self.attitude_tolerance
                    settled = settled and abs(
                        self.angle_error(yaw, measured_yaw)) < self.attitude_tolerance
                if settled:
                    break
                if rospy.get_time() - staging_start_sim > self.staging_timeout:
                    rospy.logerr("grasp demo: could not recover the staging formation")
                    return False
                if time.monotonic() > staging_wall_deadline:
                    rospy.logerr("grasp demo: staging wall-time watchdog expired")
                    return False
                time.sleep(self.control_period)

            self.ramp_attitude_at_staging(desired_radii, desired_tangents)

            baseline = self.force_baseline()
            # This is deliberately unreachable: the physical face and the
            # compliant physical feet stop the vehicle first. The position
            # error then
            # becomes a bounded inward preload through approach_accel_limit.
            minimum_fc_radius = (
                inradius + self.foot_contact_offset + self.foot_ball_radius -
                self.foot_max_compression)
            contact_counts = dict((name, 0) for name in self.names)
            last_sim_time = rospy.get_time()
            approach_start_sim = last_sim_time
            approach_wall_deadline = time.monotonic() + 180.0
            rospy.loginfo("grasp demo: acceleration-controlled face-normal approach at %.3f m/s",
                          self.approach_speed)

            while not rospy.is_shutdown():
                now_sim = rospy.get_time()
                dt = max(0.0, min(0.1, now_sim - last_sim_time))
                last_sim_time = now_sim
                odometry, forces, contacts, object_pose = self.snapshot()
                all_metrics, _ = self.log_metrics(
                    "approach", forces, contacts, baseline, object_pose)

                for name in self.names:
                    established = self.four_point_contact(all_metrics[name])
                    contact_counts[name] = contact_counts[name] + 1 if established else 0
                    if (self.force_trial_mode or
                            contact_counts[name] < self.contact_confirm_samples):
                        desired_radii[name] = max(
                            minimum_fc_radius,
                            desired_radii[name] - self.approach_speed * dt)
                    pressing = (not self.force_trial_mode and
                                contact_counts[name] >= self.contact_confirm_samples)
                    ax, ay, yaw, _ = self.control_acceleration(
                        name, odometry[name], desired_radii[name], desired_tangents[name],
                        all_metrics[name], pressing)
                    self.publish_accel_nav(name, ax, ay, yaw)

                if self.force_trial_mode and all(
                        desired_radii[name] <= minimum_fc_radius + 1.0e-6
                        for name in self.names):
                    rospy.loginfo(
                        "grasp demo: force trial reached geometric preload target")
                    break
                if (not self.force_trial_mode and
                        all(contact_counts[name] >= self.contact_confirm_samples
                            for name in self.names)):
                    break
                if now_sim - approach_start_sim > self.approach_timeout:
                    rospy.logerr("grasp demo: approach timed out in simulation time")
                    return False
                if time.monotonic() > approach_wall_deadline:
                    rospy.logerr("grasp demo: approach wall-time watchdog expired")
                    return False
                time.sleep(self.control_period)

            odometry, forces, contacts, object_pose = self.snapshot()
            all_metrics, _ = self.log_metrics("contact", forces, contacts, baseline, object_pose)
            for name in self.names:
                rospy.loginfo(
                    "%s contact: touch=%s N, normal=%s N, compression=%s mm",
                    name,
                    ",".join("{:.3f}".format(m["touch"]) for m in all_metrics[name]),
                    ",".join("{:.3f}".format(m["normal"]) for m in all_metrics[name]),
                    ",".join("{:.2f}".format(1000.0 * m["compression"])
                             for m in all_metrics[name]))
            if (not self.force_trial_mode and
                    not all(self.four_point_contact(all_metrics[name])
                            for name in self.names)):
                rospy.logerr("grasp demo: touch + normal force + compression condition was lost")
                return False

            # Do not start lifting on the first confirmed contact sample.  Hold
            # the requested inward preload long enough for all three vehicles,
            # compliant feet, and the object to reach a force-balanced state.
            preload_start = rospy.get_time()
            rospy.loginfo("grasp demo: hold inward preload for %.2f s", self.preload_duration)
            while (not rospy.is_shutdown() and
                   rospy.get_time() - preload_start < self.preload_duration):
                odometry, forces, contacts, object_pose = self.snapshot()
                all_metrics, _ = self.log_metrics(
                    "preload", forces, contacts, baseline, object_pose)
                for name in self.names:
                    ax, ay, yaw, _ = self.control_acceleration(
                        name, odometry[name], desired_radii[name], desired_tangents[name],
                        all_metrics[name], True)
                    self.publish_accel_nav(name, ax, ay, yaw)
                time.sleep(self.control_period)

            odometry, forces, contacts, object_pose = self.snapshot()
            all_metrics, _ = self.log_metrics(
                "preload_complete", forces, contacts, baseline, object_pose)
            for name in self.names:
                rospy.loginfo(
                    "%s preload: normal=%s N, compression=%s mm",
                    name,
                    ",".join("{:.3f}".format(m["normal"])
                             for m in all_metrics[name]),
                    ",".join("{:.2f}".format(1000.0 * m["compression"])
                             for m in all_metrics[name]))

            if self.preload_hold_mode:
                return self.hold_preload(
                    desired_radii, desired_tangents, baseline)

            lift_start_z = object_pose.position.z
            required_upward_force = self.object_mass * 9.80665 * self.force_margin
            friction_counts = 0
            lift_start_sim = rospy.get_time()
            lift_wall_deadline = time.monotonic() + 180.0
            rospy.loginfo("grasp demo: lift at %.3f m/s; required upward z friction %.3f N",
                          self.lift_velocity, required_upward_force)

            while not rospy.is_shutdown():
                odometry, forces, contacts, object_pose = self.snapshot()
                all_metrics, upward_friction = self.log_metrics(
                    "lift", forces, contacts, baseline, object_pose)
                all_contacts = all(self.four_point_contact(all_metrics[name])
                                   for name in self.names)
                enough_friction = upward_friction > required_upward_force
                friction_counts = friction_counts + 1 if all_contacts and enough_friction else 0

                for name in self.names:
                    ax, ay, yaw, _ = self.control_acceleration(
                        name, odometry[name], desired_radii[name], desired_tangents[name],
                        all_metrics[name], True)
                    self.publish_accel_nav(name, ax, ay, yaw,
                                           FlightNav.VEL_MODE, self.lift_velocity)

                rise = object_pose.position.z - lift_start_z
                rospy.loginfo_throttle(
                    1.0, "grasp demo: object rise %.3f m, upward z friction %.3f / %.3f N",
                    rise, upward_friction, required_upward_force)
                if (rise >= self.lift_height and
                        (self.force_trial_mode or
                         friction_counts >= self.force_confirm_samples)):
                    if self.force_trial_mode:
                        rospy.loginfo(
                            "grasp demo: FORCE TRIAL SUCCESS: z friction %.3f N; "
                            "object rose %.3f m",
                            upward_friction, rise)
                    else:
                        rospy.loginfo(
                            "grasp demo: SUCCESS: 12 touches and normal forces remain; "
                            "z friction %.3f N > object threshold %.3f N; "
                            "object rose %.3f m",
                            upward_friction, required_upward_force, rise)
                    return self.hold_lifted_object(
                        desired_radii, desired_tangents, baseline)
                if rospy.get_time() - lift_start_sim > self.lift_timeout:
                    rospy.logerr(
                        "grasp demo: lift failed: rise %.3f m, z friction %.3f / %.3f N, contacts=%s",
                        rise, upward_friction, required_upward_force, all_contacts)
                    return False
                if time.monotonic() > lift_wall_deadline:
                    rospy.logerr("grasp demo: lift wall-time watchdog expired")
                    return False
                time.sleep(self.control_period)
        finally:
            if self.odometry:
                self.publish_stop()
            if self.log_file is not None:
                self.log_file.close()
            rospy.loginfo("grasp demo: metrics written to %s", self.log_path)


def main():
    rospy.init_node("bee_mujoco_grasp_demo")
    demo = MujocoGraspDemo()
    if not demo.run():
        raise SystemExit(1)


if __name__ == "__main__":
    main()
