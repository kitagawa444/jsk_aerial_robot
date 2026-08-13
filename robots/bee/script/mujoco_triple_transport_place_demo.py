#!/usr/bin/env python

from __future__ import print_function

import math
import time

import rospy
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from std_msgs.msg import String

from mujoco_triple_press_keyboard_demo import MujocoTriplePressKeyboardDemo


class MujocoTripleTransportPlaceDemo(MujocoTriplePressKeyboardDemo):
    """Autonomously grasp, transport, place, release, and retreat."""

    def __init__(self):
        # Give the autonomous demo known-safe defaults while preserving every
        # explicit command-line override used by an experiment.
        defaults = {
            "~target_normal_force": 4.0,
            "~initial_z_velocity": 0.01,
            "~lift_hold_height": 0.05,
            "~lift_drop_abort_distance": 0.03,
            "~translation_z_range": 0.30,
            "~wrench_log_path": "/tmp/bee_transport_place.csv",
        }
        for name, value in defaults.items():
            if not rospy.has_param(name):
                rospy.set_param(name, value)

        super(MujocoTripleTransportPlaceDemo, self).__init__()

        bringup_ns = "/bee_mujoco_multi_bringup/"
        scene_destination_x = rospy.get_param(
            bringup_ns + "destination_pedestal_x", self.object_x + 1.50)
        scene_destination_y = rospy.get_param(
            bringup_ns + "destination_pedestal_y", self.object_y)
        scene_destination_height = rospy.get_param(
            bringup_ns + "destination_pedestal_height", 0.65)
        self.destination_x = rospy.get_param(
            "~destination_x", scene_destination_x)
        self.destination_y = rospy.get_param(
            "~destination_y", scene_destination_y)
        self.destination_pedestal_height = rospy.get_param(
            "~destination_pedestal_height", scene_destination_height)
        self.object_height = rospy.get_param(
            "~object_height",
            rospy.get_param(bringup_ns + "object_height", self.object_height))
        self.object_ground_clearance = rospy.get_param(
            "~object_ground_clearance",
            rospy.get_param(
                bringup_ns + "object_ground_clearance",
                self.object_ground_clearance))
        self.place_object_z = rospy.get_param(
            "~place_object_z",
            self.destination_pedestal_height +
            self.object_ground_clearance + self.object_height / 2.0)
        self.place_preload_depth = rospy.get_param(
            "~place_preload_depth", 0.010)
        self.place_control_target_z = (
            self.place_object_z - max(0.0, self.place_preload_depth))

        self.transport_velocity_limit = rospy.get_param(
            "~transport_velocity_limit", 0.02)
        self.transport_position_kp = rospy.get_param(
            "~transport_position_kp", 0.50)
        self.transport_position_tolerance = rospy.get_param(
            "~transport_position_tolerance", 0.015)
        self.transport_velocity_tolerance = rospy.get_param(
            "~transport_velocity_tolerance", 0.008)
        self.transport_arrival_duration = rospy.get_param(
            "~transport_arrival_duration", 1.0)
        self.transport_timeout = rospy.get_param(
            "~transport_timeout", 180.0)
        self.destination_settle_duration = rospy.get_param(
            "~destination_settle_duration", 2.0)

        self.place_descent_velocity = rospy.get_param(
            "~place_descent_velocity", 0.005)
        self.place_height_tolerance = rospy.get_param(
            "~place_height_tolerance", 0.003)
        self.place_velocity_tolerance = rospy.get_param(
            "~place_velocity_tolerance", 0.008)
        self.touchdown_confirm_duration = rospy.get_param(
            "~touchdown_confirm_duration", 1.5)
        self.place_timeout = rospy.get_param("~place_timeout", 40.0)

        self.unload_duration = rospy.get_param("~unload_duration", 3.0)
        self.release_normal_force = rospy.get_param(
            "~release_normal_force", 0.0)
        self.release_normal_force_ramp_rate = rospy.get_param(
            "~release_normal_force_ramp_rate", 0.80)
        self.release_force_tolerance = rospy.get_param(
            "~release_force_tolerance", 0.25)
        self.release_confirm_duration = rospy.get_param(
            "~release_confirm_duration", 1.0)
        self.release_timeout = rospy.get_param("~release_timeout", 20.0)

        self.retreat_clearance = rospy.get_param(
            "~retreat_clearance", 0.40)
        self.retreat_tolerance = rospy.get_param(
            "~retreat_tolerance", 0.05)
        self.retreat_timeout = rospy.get_param("~retreat_timeout", 30.0)
        self.attitude_unwind_duration = rospy.get_param(
            "~attitude_unwind_duration", 3.0)
        self.final_verify_duration = rospy.get_param(
            "~final_verify_duration", 2.0)

        # A lost object must never remain the moving reference for the three
        # radial controllers.  These checks use only signals that are also
        # available to the controller (object pose, odometry, and observer).
        self.safety_input_timeout = rospy.get_param(
            "~safety_input_timeout", 1.0)
        self.safety_force_loss_threshold = rospy.get_param(
            "~safety_force_loss_threshold", 0.50)
        self.safety_force_loss_count = rospy.get_param(
            "~safety_force_loss_count", 2)
        self.safety_force_loss_duration = rospy.get_param(
            "~safety_force_loss_duration", 0.25)
        self.safety_contact_z_error_limit = rospy.get_param(
            "~safety_contact_z_error_limit", 0.05)
        self.safety_object_speed_limit = rospy.get_param(
            "~safety_object_speed_limit", 0.30)
        self.safety_object_yaw_rate_limit = rospy.get_param(
            "~safety_object_yaw_rate_limit", 1.50)
        self.failsafe_hold_duration = rospy.get_param(
            "~failsafe_hold_duration", 4.0)
        self.failsafe_unwind_duration = rospy.get_param(
            "~failsafe_unwind_duration", 2.0)

        self.task_phase = "PRELOAD"
        self.phase_start_time = None
        self.phase_stable_since = None
        self.retreat_radius = None
        self.release_support_scale = self.lift_hold_force_feedforward_scale
        self.transport_success = False
        self.safety_active = False
        self.safety_reason = ""
        self.safety_low_force_since = None
        self.safety_transport_relative_contact_z = None
        self.failsafe_hold_positions = None
        self.failsafe_hold_yaws = None
        self.failsafe_start_roll = 0.0
        self.failsafe_start_wall_time = None
        self.phase_pub = rospy.Publisher(
            "/mujoco_triple_transport_place/phase",
            String, queue_size=1, latch=True)

    def run(self):
        if not rospy.get_param(
                "/bee_mujoco_multi_bringup/spawn_destination_pedestal",
                False):
            rospy.logerr(
                "transport/place: launch MuJoCo with "
                "spawn_destination_pedestal:=true")
            return False
        return super(MujocoTripleTransportPlaceDemo, self).run()

    def transition(self, phase, message):
        self.task_phase = phase
        self.phase_start_time = rospy.get_time()
        self.phase_stable_since = None
        self.phase_pub.publish(String(data=phase))
        rospy.logwarn("transport/place: %s: %s", phase, message)

    def phase_elapsed(self, now):
        if self.phase_start_time is None:
            return 0.0
        return max(0.0, now - self.phase_start_time)

    def fail_mission(self, reason):
        if self.safety_active:
            return
        rospy.logerr("transport/place: FAILED in %s: %s",
                     self.task_phase, reason)
        self.transport_success = False
        self.safety_active = True
        self.safety_reason = reason
        self.target_z_velocity = 0.0
        self.yaw_velocity_command = 0.0
        self.translation_velocity_command = [0.0, 0.0]
        self.translation_acceleration_command = [0.0, 0.0]
        self.lift_common_z_acceleration_command = 0.0
        self.lift_common_slip_acceleration = 0.0
        self.lift_yaw_tangential_acceleration = 0.0
        self.lift_tilt_moment_command = [0.0, 0.0]
        self.failsafe_start_roll = self.commanded_roll
        self.failsafe_start_wall_time = time.monotonic()
        self.transition(
            "FAILSAFE_HOLD",
            "stop object tracking and hold each Bee: {}".format(reason))
        # abort_lift() requests an immediate exit.  Keep this autonomous demo
        # alive just long enough to latch and repeat the position-hold command.
        self.stop_requested = False

    def input_safety_reason(self):
        now = time.monotonic()
        with self.lock:
            object_receipt = self.object_pose_receipt_time
            odometry_receipts = dict(self.odometry_receipt_times)
        if (object_receipt is None or
                now - object_receipt > self.safety_input_timeout):
            return "object pose input became stale"
        stale_names = [
            name for name in self.names
            if (name not in odometry_receipts or
                now - odometry_receipts[name] > self.safety_input_timeout)
        ]
        if stale_names:
            return "odometry input became stale: {}".format(
                ", ".join(stale_names))
        return None

    @staticmethod
    def finite_values(values):
        return all(
            not math.isnan(value) and not math.isinf(value)
            for value in values)

    def airborne_safety_reason(
            self, odometry, object_pose, object_twist, filtered_forces, now):
        input_reason = self.input_safety_reason()
        if input_reason is not None:
            return input_reason

        pose_values = (
            object_pose.position.x, object_pose.position.y,
            object_pose.position.z, object_pose.orientation.x,
            object_pose.orientation.y, object_pose.orientation.z,
            object_pose.orientation.w)
        if not self.finite_values(
                pose_values + tuple(object_twist) +
                tuple(self.object_angular_velocity_world)):
            return "object state contains a non-finite value"

        monitored_phases = (
            "TRANSPORT", "DESTINATION_SETTLE", "DESCEND")
        if self.task_phase not in monitored_phases:
            self.safety_low_force_since = None
            return None

        linear_speed = math.sqrt(sum(
            value * value for value in object_twist[:3]))
        if linear_speed > self.safety_object_speed_limit:
            return "object speed {:.3f} m/s exceeded {:.3f} m/s".format(
                linear_speed, self.safety_object_speed_limit)
        if abs(object_twist[3]) > self.safety_object_yaw_rate_limit:
            return "object yaw rate {:.3f} rad/s exceeded {:.3f} rad/s".format(
                abs(object_twist[3]), self.safety_object_yaw_rate_limit)

        # Close to the destination support, low force is expected during
        # touchdown.  Above it, losing two faces means the friction grasp has
        # already failed even before a large vertical drop becomes visible.
        airborne = (
            object_pose.position.z >
            self.place_object_z + 2.0 * self.place_height_tolerance)
        low_force_count = sum(
            1 for value in filtered_forces.values()
            if abs(value) < self.safety_force_loss_threshold)
        force_lost = (
            airborne and low_force_count >= self.safety_force_loss_count)
        if force_lost:
            if self.safety_low_force_since is None:
                self.safety_low_force_since = now
            elif (now - self.safety_low_force_since >=
                  self.safety_force_loss_duration):
                return "observer contact force was lost on {} Bees".format(
                    low_force_count)
        else:
            self.safety_low_force_since = None

        safety_contact_z_reference = (
            self.safety_transport_relative_contact_z
            if self.safety_transport_relative_contact_z is not None else
            self.lift_initial_relative_contact_z)
        if airborne and safety_contact_z_reference is not None:
            maximum_z_error = max(
                abs(self.contact_position_state(
                    name, odometry[name], object_pose, object_twist)[7] -
                    safety_contact_z_reference)
                for name in self.names)
            if maximum_z_error > self.safety_contact_z_error_limit:
                return "object/contact height separation reached {:.3f} m".format(
                    maximum_z_error)
        return None

    def publish_failsafe_hold(self, odometry):
        if self.failsafe_hold_positions is None:
            self.failsafe_hold_positions = dict(
                (name, (
                    odometry[name].pose.pose.position.x,
                    odometry[name].pose.pose.position.y,
                    odometry[name].pose.pose.position.z))
                for name in self.names)
            self.failsafe_hold_yaws = dict(
                (name, self.quaternion_to_rpy(
                    odometry[name].pose.pose.orientation)[2])
                for name in self.names)

        elapsed = max(
            0.0, time.monotonic() - self.failsafe_start_wall_time)
        ratio = min(
            1.0, elapsed / max(1.0e-6, self.failsafe_unwind_duration))
        blend = ratio * ratio * (3.0 - 2.0 * ratio)
        self.commanded_roll = (1.0 - blend) * self.failsafe_start_roll

        for name in self.names:
            position = self.failsafe_hold_positions[name]
            msg = FlightNav()
            msg.header.stamp = rospy.Time.now()
            msg.control_frame = FlightNav.WORLD_FRAME
            msg.target = FlightNav.COG
            msg.pos_xy_nav_mode = FlightNav.POS_MODE
            msg.target_pos_x = position[0]
            msg.target_pos_y = position[1]
            msg.pos_z_nav_mode = FlightNav.POS_MODE
            msg.target_pos_z = position[2]
            msg.yaw_nav_mode = FlightNav.POS_MODE
            msg.target_yaw = self.failsafe_hold_yaws[name]
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

            # Also clear the optional simulator-only wrench interface.
            wrench = WrenchStamped()
            wrench.header.stamp = msg.header.stamp
            wrench.header.frame_id = "world"
            self.wrench_pubs[name].publish(wrench)

        if elapsed >= self.failsafe_hold_duration:
            self.commanded_roll = 0.0
            self.transition(
                "FAILED", "safe Bee position hold latched; mission stopped")
            self.stop_requested = True

    def stable_for(self, condition, now, duration):
        if not condition:
            self.phase_stable_since = None
            return False
        if self.phase_stable_since is None:
            self.phase_stable_since = now
        return now - self.phase_stable_since >= duration

    def command_transport_velocity(self, object_pose, object_twist):
        error_x = self.destination_x - object_pose.position.x
        error_y = self.destination_y - object_pose.position.y
        distance = math.hypot(error_x, error_y)
        if distance <= self.transport_position_tolerance:
            self.translation_velocity_command = [0.0, 0.0]
        else:
            speed = min(
                self.transport_velocity_limit,
                self.transport_position_kp * distance)
            self.translation_velocity_command = [
                speed * error_x / distance,
                speed * error_y / distance,
            ]
        velocity = math.hypot(object_twist[0], object_twist[1])
        return distance, velocity

    def update_mission(self, object_pose, object_twist, filtered_forces, now):
        if self.safety_active:
            return
        if self.lift_fault:
            self.fail_mission("grasp controller reported a lift fault")
            return

        if self.task_phase == "LIFT":
            if self.lift_hold_reached:
                self.target_z_velocity = 0.0
                # The compliant contacts acquire their loaded relative height
                # during the continuous LIFT settling interval.  Rebase only
                # the transport safety monitor here; the common-slip control
                # reference and every controller state remain untouched.
                self.safety_transport_relative_contact_z = (
                    self.lift_initial_relative_contact_z +
                    self.lift_common_slip_error)
                self.transition(
                    "TRANSPORT",
                    "object lifted; move to ({:.3f}, {:.3f})".format(
                        self.destination_x, self.destination_y))

        elif self.task_phase == "TRANSPORT":
            distance, velocity = self.command_transport_velocity(
                object_pose, object_twist)
            arrived = (
                distance <= self.transport_position_tolerance and
                velocity <= self.transport_velocity_tolerance)
            if self.stable_for(
                    arrived, now, self.transport_arrival_duration):
                self.translation_velocity_command = [0.0, 0.0]
                self.transition(
                    "DESTINATION_SETTLE",
                    "destination reached with {:.3f} m error".format(
                        distance))
            elif self.phase_elapsed(now) > self.transport_timeout:
                self.fail_mission(
                    "transport timed out with {:.3f} m position error".
                    format(distance))

        elif self.task_phase == "DESTINATION_SETTLE":
            self.translation_velocity_command = [0.0, 0.0]
            self.target_z_velocity = 0.0
            if self.phase_elapsed(now) >= self.destination_settle_duration:
                # Once initial lift has been confirmed, allow the hold target
                # slightly below the support height.  This bounded virtual
                # target removes steady tracking error and establishes actual
                # pedestal load before vertical feed-forward is unloaded.
                self.lift_minimum_object_rise = (
                    self.place_control_target_z - self.lift_start_object_z)
                self.target_z_velocity = -abs(self.place_descent_velocity)
                self.transition(
                    "DESCEND",
                    "lower object onto pedestal at z={:.3f} m "
                    "(control target {:.3f} m)".format(
                        self.place_object_z, self.place_control_target_z))

        elif self.task_phase == "DESCEND":
            self.translation_velocity_command = [0.0, 0.0]
            if (self.lift_hold_target_object_z is None or
                    self.lift_hold_target_object_z >
                    self.place_control_target_z + 1.0e-5):
                self.target_z_velocity = -abs(self.place_descent_velocity)
            else:
                self.target_z_velocity = 0.0
            height_ok = (
                object_pose.position.z <=
                self.place_object_z + self.place_height_tolerance)
            vertical_speed_ok = (
                abs(object_twist[2]) <= self.place_velocity_tolerance)
            if self.stable_for(
                    height_ok and vertical_speed_ok,
                    now, self.touchdown_confirm_duration):
                self.target_z_velocity = 0.0
                self.lift_hold_target_object_z = self.place_object_z
                self.lift_common_slip_activation_rise = float("inf")
                self.lift_common_slip_feedback_active = False
                self.lift_common_slip_acceleration = 0.0
                # The pedestal now carries the object.  End the unified Z PID
                # integral here, not at LIFT/HOLD/TRANSPORT boundaries, so it
                # cannot command a residual upward acceleration during unload.
                self.lift_trajectory_integral_acceleration = 0.0
                self.lift_trajectory_integral_enabled = False
                self.release_support_scale = (
                    self.lift_hold_force_feedforward_scale)
                self.transition(
                    "UNLOAD",
                    "pedestal contact stable; transfer vertical load")
            elif self.phase_elapsed(now) > self.place_timeout:
                self.fail_mission(
                    "descent timed out at object z={:.3f} m".format(
                        object_pose.position.z))

        elif self.task_phase == "UNLOAD":
            self.translation_velocity_command = [0.0, 0.0]
            self.target_z_velocity = 0.0
            ratio = min(
                1.0, self.phase_elapsed(now) /
                max(1.0e-6, self.unload_duration))
            self.lift_hold_force_feedforward_scale = (
                (1.0 - ratio) * self.release_support_scale)
            if ratio >= 1.0:
                self.reset_local_moment_control()
                self.normal_force_ramp_rate = \
                    self.release_normal_force_ramp_rate
                self.set_common_normal_force(self.release_normal_force)
                self.transition(
                    "RELEASE",
                    "vertical load transferred; remove normal preload")

        elif self.task_phase == "RELEASE":
            force_released = (
                self.applied_common_normal_force() <=
                self.release_normal_force + 0.05 and
                max(abs(value) for value in filtered_forces.values()) <=
                self.release_force_tolerance)
            if self.stable_for(
                    force_released, now, self.release_confirm_duration):
                self.retreat_radius = (
                    self.contact_surface_radius + self.retreat_clearance)
                self.transition(
                    "RETREAT",
                    "contact released; move all Bees radially outward")
            elif self.phase_elapsed(now) > self.release_timeout:
                self.fail_mission(
                    "normal force did not release; observer max {:.3f} N".
                    format(max(abs(value)
                               for value in filtered_forces.values())))

        elif self.task_phase == "RETREAT":
            if self.phase_elapsed(now) > self.retreat_timeout:
                self.fail_mission("Bee retreat timed out")

        elif self.task_phase == "UNWIND":
            ratio = min(
                1.0, self.phase_elapsed(now) /
                max(1.0e-6, self.attitude_unwind_duration))
            self.commanded_roll = (1.0 - ratio) * self.approach_roll
            if ratio >= 1.0:
                self.commanded_roll = 0.0
                self.transition(
                    "VERIFY",
                    "Bees clear; verify object remains on destination")

        elif self.task_phase == "VERIFY":
            position_error = math.hypot(
                object_pose.position.x - self.destination_x,
                object_pose.position.y - self.destination_y)
            height_error = abs(
                object_pose.position.z - self.place_object_z)
            stable = (
                position_error <= 2.0 * self.transport_position_tolerance and
                height_error <= 2.0 * self.place_height_tolerance and
                math.sqrt(sum(value * value for value in object_twist[:3])) <=
                self.transport_velocity_tolerance)
            if self.stable_for(stable, now, self.final_verify_duration):
                self.transport_success = True
                self.transition(
                    "COMPLETE",
                    "object placed at ({:.3f}, {:.3f}, {:.3f})".format(
                        object_pose.position.x,
                        object_pose.position.y,
                        object_pose.position.z))
                self.stop_requested = True
            elif self.phase_elapsed(now) > \
                    3.0 * self.final_verify_duration:
                self.fail_mission(
                    "placed object did not settle: xy/z error {:.3f}/{:.3f} m".
                    format(position_error, height_error))

    def publish_retreat_commands(self, odometry, object_pose):
        settled = True
        for name in self.names:
            ax, ay, yaw, radius, _, radial_velocity, _ = \
                self.approach_acceleration(
                    name, odometry[name], object_pose, self.retreat_radius)
            self.publish_accel_nav(name, ax, ay, yaw)
            settled = (
                settled and
                abs(radius - self.retreat_radius) < self.retreat_tolerance and
                abs(radial_velocity) < 0.08)
        if (self.task_phase == "RETREAT" and settled and
                self.phase_elapsed(rospy.get_time()) >= 1.0):
            self.transition(
                "UNWIND", "retreat complete; restore upright attitude")

    def interactive_press(self, observer_baseline, contact_radius):
        """Run the autonomous mission using the proven grasp control loop."""
        filtered_forces = dict(
            (name, self.observer_normal_force(name, observer_baseline))
            for name in self.names)
        _, initial_odometry, _, initial_object_pose = self.dual_snapshot()
        self.relative_contact_z = sum(
            self.contact_position_state(
                name, initial_odometry[name], initial_object_pose,
                (0.0, 0.0, 0.0, 0.0))[7]
            for name in self.names) / len(self.names)
        self.reset_object_twist(initial_object_pose)
        self.reset_local_moment_control()
        self.enter_lift_mode()
        self.task_phase = "LIFT"
        self.phase_start_time = rospy.get_time()
        self.phase_pub.publish(String(data=self.task_phase))
        rospy.logwarn(
            "transport/place: LIFT: grasp at %.2f N/vehicle, destination "
            "(%.3f, %.3f, %.3f)",
            self.common_normal_force(), self.destination_x,
            self.destination_y, self.place_object_z)

        log_file, log_writer = self.open_wrench_log()
        last_log_flush = time.monotonic()
        last_control_time = rospy.get_time()
        try:
            while not rospy.is_shutdown() and not self.stop_requested:
                _, odometry, force_sites, object_pose = self.dual_snapshot()
                object_twist = self.update_object_twist(object_pose)
                if self.safety_active:
                    # Do not let the regular drop detector or force/moment
                    # controller run again after the object has been removed
                    # as the control reference.
                    self.publish_failsafe_hold(odometry)
                    time.sleep(self.control_period)
                    continue
                try:
                    (local_wrenches, object_force,
                     object_force_arm_moment, object_local_moment_sum,
                     object_moment) = self.observer_contact_wrenches(
                         odometry, object_pose)
                except RuntimeError as error:
                    self.fail_mission(
                        "observer input failed: {}".format(error))
                    self.publish_failsafe_hold(odometry)
                    time.sleep(self.control_period)
                    continue

                alpha = max(0.0, min(1.0, self.force_filter_alpha))
                for name in self.names:
                    measured = self.observer_normal_force(
                        name, observer_baseline)
                    filtered_forces[name] += alpha * (
                        measured - filtered_forces[name])

                now = rospy.get_time()
                control_dt = now - last_control_time
                last_control_time = now
                self.update_common_normal_force(control_dt)

                contact_control = self.task_phase not in (
                    "RELEASE", "RETREAT", "UNWIND", "VERIFY", "COMPLETE",
                    "FAILSAFE_HOLD", "FAILED")
                if contact_control:
                    self.update_local_moment_control(
                        local_wrenches, filtered_forces, control_dt)
                    self.update_object_moment_control(
                        local_wrenches, object_pose, object_moment,
                        filtered_forces, control_dt)
                    self.update_lift_progress(
                        object_pose, odometry, object_twist)

                safety_reason = self.airborne_safety_reason(
                    odometry, object_pose, object_twist,
                    filtered_forces, now)
                if safety_reason is not None:
                    self.fail_mission(safety_reason)
                else:
                    self.update_mission(
                        object_pose, object_twist, filtered_forces, now)
                if self.stop_requested:
                    break

                if self.safety_active:
                    self.publish_failsafe_hold(odometry)
                    time.sleep(self.control_period)
                    continue

                contact_control = self.task_phase not in (
                    "RELEASE", "RETREAT", "UNWIND", "VERIFY", "COMPLETE",
                    "FAILSAFE_HOLD", "FAILED")
                if contact_control:
                    self.update_lift_z_control(
                        odometry, object_pose, object_twist,
                        object_force[2], control_dt)
                    translation_acceleration = \
                        self.update_translation_control(
                            object_twist, control_dt)
                    self.update_lift_yaw_control(
                        object_pose, object_twist, control_dt)
                else:
                    self.translation_velocity_command = [0.0, 0.0]
                    self.translation_acceleration_command = [0.0, 0.0]
                    translation_acceleration = (0.0, 0.0)

                if self.task_phase in ("RETREAT", "UNWIND", "VERIFY"):
                    self.publish_retreat_commands(odometry, object_pose)
                else:
                    for name in self.names:
                        ax, ay, yaw = self.press_acceleration(
                            name, odometry[name], object_pose,
                            filtered_forces[name], contact_radius,
                            object_twist)
                        ax += translation_acceleration[0]
                        ay += translation_acceleration[1]
                        if self.lift_mode_active:
                            z_mode = FlightNav.ACC_MODE
                            z_command = self.contact_z_acceleration(name)
                        else:
                            z_mode = FlightNav.VEL_MODE
                            z_command = self.contact_z_velocity(
                                name, odometry[name], object_pose,
                                object_twist)
                        self.publish_accel_nav(
                            name, ax, ay, yaw, z_mode, z_command)

                self.publish_observer_wrench_metrics(
                    local_wrenches, object_force,
                    object_force_arm_moment,
                    object_local_moment_sum, object_moment)
                self.publish_local_moment_trims()
                self.write_wrench_log(
                    log_writer, object_pose, force_sites, local_wrenches,
                    object_force, object_force_arm_moment,
                    object_local_moment_sum, object_moment)
                if time.monotonic() - last_log_flush >= \
                        self.wrench_log_flush_period:
                    log_file.flush()
                    last_log_flush = time.monotonic()
                self.publish_interactive_state(filtered_forces)
                rospy.loginfo_throttle(
                    0.5,
                    "transport/place: %s object [%.3f, %.3f, %.3f], "
                    "goal error xy/z %.3f/%+.3f m, force [%.2f %.2f %.2f] N",
                    self.task_phase,
                    object_pose.position.x,
                    object_pose.position.y,
                    object_pose.position.z,
                    math.hypot(
                        object_pose.position.x - self.destination_x,
                        object_pose.position.y - self.destination_y),
                    object_pose.position.z - self.place_object_z,
                    filtered_forces["bee1"], filtered_forces["bee2"],
                    filtered_forces["bee3"])
                time.sleep(self.control_period)
        finally:
            log_file.flush()
            log_file.close()
        return self.transport_success


def main():
    rospy.init_node("bee_mujoco_triple_transport_place_demo")
    demo = MujocoTripleTransportPlaceDemo()
    try:
        if not demo.run():
            raise SystemExit(1)
    finally:
        if demo.odometry:
            demo.publish_stop()


if __name__ == "__main__":
    main()
