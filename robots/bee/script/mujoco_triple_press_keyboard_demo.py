#!/usr/bin/env python

from __future__ import print_function

import csv
import math
import os
import select
import sys
import termios
import time
import tty

import rospy
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from std_msgs.msg import Float32

from mujoco_dual_push_demo import MujocoDualPushDemo


HELP = """
Three-Bee observer force + object-relative contact control
----------------------------------------------------------
  w : increase common normal force
  s : decrease common normal force
  i/k : increase/decrease object world-X velocity after HOLD
  j/l : increase/decrease object world-Y velocity after HOLD
  q/e : increase/decrease object yaw velocity after HOLD
  m : stop object XY translation
  [ : increase lift trajectory velocity limit
  ] : decrease lift trajectory velocity limit
      (after HOLD: increase/decrease object world-Z velocity)
SPACE: stop object XYZ and yaw translation
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
        self.commanded_common_normal_force = common_force
        self.active_common_normal_force = common_force
        self.object_moment_force_offsets = dict(
            (name, 0.0) for name in self.names)
        self.object_moment_force_integrals = dict(
            (name, 0.0) for name in self.names)
        self.set_common_normal_force(common_force)

        self.force_step = rospy.get_param("~keyboard_force_step", 0.25)
        self.minimum_normal_force = rospy.get_param("~minimum_normal_force", 0.0)
        self.maximum_normal_force = rospy.get_param("~maximum_normal_force", 8.0)
        self.z_velocity_step = rospy.get_param("~keyboard_z_velocity_step", 0.02)
        self.maximum_abs_z_velocity = rospy.get_param(
            "~maximum_abs_z_velocity", 0.20)
        self.target_z_velocity = rospy.get_param("~initial_z_velocity", 0.0)
        self.translation_velocity_step = rospy.get_param(
            "~keyboard_translation_velocity_step", 0.01)
        self.translation_velocity_limit = rospy.get_param(
            "~translation_velocity_limit", 0.05)
        self.translation_z_velocity_step = rospy.get_param(
            "~keyboard_translation_z_velocity_step", 0.005)
        self.translation_z_velocity_limit = rospy.get_param(
            "~translation_z_velocity_limit", 0.02)
        self.translation_velocity_kp = rospy.get_param(
            "~translation_velocity_kp", 1.5)
        self.translation_acceleration_limit = rospy.get_param(
            "~translation_acceleration_limit", 0.10)
        self.translation_acceleration_rate_limit = rospy.get_param(
            "~translation_acceleration_rate_limit", 0.30)
        self.translation_z_range = rospy.get_param(
            "~translation_z_range", 0.30)
        self.translation_z_limit_active = 0
        self.yaw_velocity_step = rospy.get_param(
            "~keyboard_yaw_velocity_step", 0.02)
        self.yaw_velocity_limit = rospy.get_param(
            "~yaw_velocity_limit", 0.10)
        self.yaw_velocity_command = 0.0
        self.translation_velocity_command = [0.0, 0.0]
        self.translation_acceleration_command = [0.0, 0.0]
        self.normal_force_ramp_rate = rospy.get_param(
            "~normal_force_ramp_rate", 0.30)
        self.normal_force_feedforward_scale = rospy.get_param(
            "~normal_force_feedforward_scale", 1.0)
        self.relative_tangent_kp = rospy.get_param(
            "~relative_contact_tangent_kp", 2.0)
        self.relative_tangent_kd = rospy.get_param(
            "~relative_contact_tangent_kd", 3.0)
        self.relative_z_kp = rospy.get_param("~relative_contact_z_kp", 1.5)
        self.relative_z_kd = rospy.get_param("~relative_contact_z_kd", 1.0)
        self.z_correction_limit = rospy.get_param(
            "~relative_contact_z_velocity_limit", 0.08)
        self.object_twist_filter_alpha = rospy.get_param(
            "~object_twist_filter_alpha", 0.20)
        self.object_twist = [0.0, 0.0, 0.0, 0.0]
        self.previous_object_state = None
        self.relative_contact_z = None
        # Nominal centre of the four physical rubber balls, expressed from
        # the FC site in base axes.  This is rigid geometry only: no MuJoCo
        # touch value or spring compression is used by the controller.
        self.contact_plane_center_in_fc = tuple(rospy.get_param(
            "~contact_plane_center_in_fc", [0.0175, 0.0, -0.161]))
        if len(self.contact_plane_center_in_fc) != 3:
            raise ValueError("contact_plane_center_in_fc must have 3 values")
        self.contact_surface_radius = None
        # The controller uses only the six-axis momentum observer.  MuJoCo
        # touch values and spring compression remain monitoring signals and
        # are deliberately excluded from feedback.
        self.observer_torque_sign = rospy.get_param(
            "~observer_torque_sign", self.observer_force_sign)
        self.observer_torque_scale = rospy.get_param(
            "~observer_torque_scale", 1.0)
        self.observer_wrench_baseline_cog = None
        self.wrench_log_path = os.path.expanduser(rospy.get_param(
            "~wrench_log_path",
            "/tmp/bee_mujoco_triple_press_wrench.csv"))
        self.wrench_log_flush_period = rospy.get_param(
            "~wrench_log_flush_period", 0.5)
        self.local_moment_control_enabled = rospy.get_param(
            "~local_moment_control_enabled", True)
        self.local_moment_filter_alpha = rospy.get_param(
            "~local_moment_filter_alpha", 0.10)
        self.local_moment_tangent_trim_kp = rospy.get_param(
            "~local_moment_tangent_trim_kp", 1.0)
        self.local_moment_tangent_trim_ki = rospy.get_param(
            "~local_moment_tangent_trim_ki", 0.15)
        self.local_moment_vertical_trim_kp = rospy.get_param(
            "~local_moment_vertical_trim_kp", 1.0)
        self.local_moment_vertical_trim_ki = rospy.get_param(
            "~local_moment_vertical_trim_ki", 0.05)
        self.local_moment_deadband = rospy.get_param(
            "~local_moment_deadband", 0.0002)
        self.local_moment_control_force_threshold = rospy.get_param(
            "~local_moment_control_force_threshold", 0.20)
        self.local_moment_trim_rate_limit = rospy.get_param(
            "~local_moment_trim_rate_limit", 0.01)
        self.local_moment_trim_angle_limit = rospy.get_param(
            "~local_moment_trim_angle_limit", 0.02)
        self.object_moment_control_enabled = rospy.get_param(
            "~object_moment_control_enabled", True)
        self.object_moment_filter_alpha = rospy.get_param(
            "~object_moment_filter_alpha", 0.05)
        self.object_moment_force_kp = rospy.get_param(
            "~object_moment_force_kp", 0.20)
        self.object_moment_force_ki = rospy.get_param(
            "~object_moment_force_ki", 0.30)
        self.object_moment_deadband = rospy.get_param(
            "~object_moment_deadband", 0.0003)
        self.object_moment_force_offset_limit = rospy.get_param(
            "~object_moment_force_offset_limit", 0.60)
        self.object_moment_force_rate_limit = rospy.get_param(
            "~object_moment_force_rate_limit", 0.50)
        self.object_moment_yaw_trim_kp = rospy.get_param(
            "~object_moment_yaw_trim_kp", 0.35)
        self.object_moment_yaw_trim_ki = rospy.get_param(
            "~object_moment_yaw_trim_ki", 0.01)
        self.object_moment_yaw_integral_unwind_rate = rospy.get_param(
            "~object_moment_yaw_integral_unwind_rate", 0.5)
        self.object_moment_yaw_trim = 0.0
        self.object_moment_yaw_integral = 0.0
        self.lift_mode_active = False
        self.lift_fault = False
        self.lift_command_start_time = None
        self.lift_start_object_z = None
        self.lift_progress_confirmed = False
        self.lift_rise_confirmation_start_time = None
        self.lift_hold_reached = False
        self.lift_hold_target_object_z = None
        self.lift_peak_object_z = None
        self.lift_velocity_threshold = rospy.get_param(
            "~lift_velocity_threshold", 0.001)
        self.lift_moment_control_enabled = rospy.get_param(
            "~lift_moment_control_enabled", True)
        self.lift_moment_deadband = rospy.get_param(
            "~lift_moment_deadband", 0.001)
        self.lift_moment_acceleration_gain = rospy.get_param(
            "~lift_moment_acceleration_gain", 0.5)
        self.lift_moment_acceleration_limit = rospy.get_param(
            "~lift_moment_acceleration_limit", 0.20)
        self.lift_moment_acceleration_rate_limit = rospy.get_param(
            "~lift_moment_acceleration_rate_limit", 1.0)
        self.lift_yaw_control_enabled = rospy.get_param(
            "~lift_yaw_control_enabled", True)
        self.lift_yaw_angle_kp = rospy.get_param(
            "~lift_yaw_angle_kp", 0.50)
        self.lift_yaw_rate_kd = rospy.get_param(
            "~lift_yaw_rate_kd", 0.20)
        self.lift_yaw_observer_moment_gain = rospy.get_param(
            "~lift_yaw_observer_moment_gain", 0.80)
        self.lift_yaw_moment_deadband = rospy.get_param(
            "~lift_yaw_moment_deadband", 0.002)
        self.lift_yaw_tangential_acceleration_limit = rospy.get_param(
            "~lift_yaw_tangential_acceleration_limit", 0.12)
        self.lift_yaw_tangential_acceleration_rate_limit = rospy.get_param(
            "~lift_yaw_tangential_acceleration_rate_limit", 0.30)
        self.lift_yaw_error_abort_limit = rospy.get_param(
            "~lift_yaw_error_abort_limit", 0.35)
        self.lift_abort_moment_threshold = rospy.get_param(
            "~lift_abort_moment_threshold", 0.05)
        self.lift_progress_timeout = rospy.get_param(
            "~lift_progress_timeout", 5.0)
        self.lift_minimum_object_rise = rospy.get_param(
            "~lift_minimum_object_rise", 0.003)
        self.lift_rise_confirmation_duration = rospy.get_param(
            "~lift_rise_confirmation_duration", 1.0)
        self.lift_hold_height = rospy.get_param(
            "~lift_hold_height", 0.02)
        self.lift_drop_abort_distance = rospy.get_param(
            "~lift_drop_abort_distance", 0.015)
        object_mass_default = rospy.get_param(
            "/bee_mujoco_multi_bringup/object_mass", 0.20)
        self.grasp_object_mass = rospy.get_param(
            "~grasp_object_mass", object_mass_default)
        self.gravity_acceleration = rospy.get_param(
            "~gravity_acceleration", 9.81)
        self.lift_trajectory_acceleration_limit = rospy.get_param(
            "~lift_trajectory_acceleration_limit", 0.03)
        self.lift_trajectory_jerk_limit = rospy.get_param(
            "~lift_trajectory_jerk_limit", 0.10)
        self.lift_trajectory_position_kp = rospy.get_param(
            "~lift_trajectory_position_kp", 6.0)
        self.lift_trajectory_velocity_kd = rospy.get_param(
            "~lift_trajectory_velocity_kd", 3.0)
        self.lift_trajectory_tracking_acceleration_limit = rospy.get_param(
            "~lift_trajectory_tracking_acceleration_limit", 0.15)
        self.lift_force_ramp_duration = rospy.get_param(
            "~lift_force_ramp_duration", 1.5)
        self.lift_breakaway_force_margin = rospy.get_param(
            "~lift_breakaway_force_margin", 0.40)
        self.lift_hold_recovery_force_kp = rospy.get_param(
            "~lift_hold_recovery_force_kp", 80.0)
        self.lift_hold_recovery_force_kd = rospy.get_param(
            "~lift_hold_recovery_force_kd", 4.0)
        self.lift_hold_recovery_force_rate_limit = rospy.get_param(
            "~lift_hold_recovery_force_rate_limit", 2.0)
        self.lift_hold_support_floor_scale = rospy.get_param(
            "~lift_hold_support_floor_scale", 0.0)
        self.lift_hold_force_feedforward_scale = rospy.get_param(
            "~lift_hold_force_feedforward_scale", 1.15)
        self.lift_observer_force_time_constant = rospy.get_param(
            "~lift_observer_force_time_constant", 0.50)
        self.lift_observer_force_kp = rospy.get_param(
            "~lift_observer_force_kp", 0.04)
        self.lift_observer_force_ki = rospy.get_param(
            "~lift_observer_force_ki", 0.002)
        self.lift_vertical_force_deadband = rospy.get_param(
            "~lift_vertical_force_deadband", 0.03)
        self.lift_observer_force_integral_limit = rospy.get_param(
            "~lift_observer_force_integral_limit", 0.05)
        self.lift_observer_force_correction_limit = rospy.get_param(
            "~lift_observer_force_correction_limit", 0.15)
        self.lift_hold_observer_force_time_constant = rospy.get_param(
            "~lift_hold_observer_force_time_constant", 1.0)
        self.lift_hold_observer_force_kp = rospy.get_param(
            "~lift_hold_observer_force_kp", 0.02)
        self.lift_hold_observer_force_ki = rospy.get_param(
            "~lift_hold_observer_force_ki", 0.0)
        self.lift_hold_observer_force_correction_limit = rospy.get_param(
            "~lift_hold_observer_force_correction_limit", 0.08)
        self.lift_common_z_acceleration_limit = rospy.get_param(
            "~lift_common_z_acceleration_limit", 1.0)
        self.filtered_object_vertical_force = 0.0
        self.lift_vertical_force_target = 0.0
        self.lift_observer_force_integral = 0.0
        self.lift_observer_force_correction = 0.0
        self.lift_reference_z = None
        self.lift_reference_velocity = 0.0
        self.lift_reference_acceleration = 0.0
        self.lift_object_acceleration_command = 0.0
        self.lift_common_z_acceleration_command = 0.0
        self.lift_force_ramp_ratio = 0.0
        self.lift_breakaway_force_active = False
        self.lift_hold_recovery_force = 0.0
        self.lift_common_slip_error = 0.0
        self.lift_common_slip_velocity = 0.0
        self.lift_common_slip_acceleration = 0.0
        self.lift_common_slip_feedback_active = False
        self.lift_yaw_target = None
        self.lift_yaw_error = 0.0
        self.lift_yaw_rate = 0.0
        self.lift_yaw_moment_feedback = 0.0
        self.lift_yaw_tangential_acceleration = 0.0
        self.lift_common_slip_activation_rise = rospy.get_param(
            "~lift_common_slip_activation_rise", self.lift_hold_height)
        self.lift_common_slip_kp = rospy.get_param(
            "~lift_common_slip_kp", 15.0)
        self.lift_common_slip_kd = rospy.get_param(
            "~lift_common_slip_kd", 3.0)
        self.lift_common_slip_acceleration_limit = rospy.get_param(
            "~lift_common_slip_acceleration_limit", 0.08)
        self.lift_common_slip_acceleration_rate_limit = rospy.get_param(
            "~lift_common_slip_acceleration_rate_limit", 2.0)
        self.lift_relative_z_kp = rospy.get_param(
            "~lift_relative_contact_z_kp", 8.0)
        self.lift_relative_z_kd = rospy.get_param(
            "~lift_relative_contact_z_kd", 2.0)
        self.lift_relative_z_error_limit = rospy.get_param(
            "~lift_relative_contact_z_error_limit", 0.06)
        self.lift_relative_z_difference_limit = rospy.get_param(
            "~lift_relative_contact_z_difference_limit", 0.04)
        self.lift_contact_z_acceleration_limit = rospy.get_param(
            "~lift_contact_z_acceleration_limit", 0.30)
        self.lift_contact_z_acceleration_rate_limit = rospy.get_param(
            "~lift_contact_z_acceleration_rate_limit", 1.0)
        self.lift_initial_relative_contact_z = None
        self.lift_contact_z_acceleration_offsets = dict(
            (name, 0.0) for name in self.names)
        self.lift_z_acceleration_offsets = dict(
            (name, 0.0) for name in self.names)
        self.filtered_object_moment = [0.0, 0.0, 0.0]
        self.filtered_local_moment_contact = dict(
            (name, [0.0, 0.0, 0.0]) for name in self.names)
        self.local_moment_roll_trim = dict(
            (name, 0.0) for name in self.names)
        self.local_moment_yaw_trim = dict(
            (name, 0.0) for name in self.names)
        self.local_moment_roll_integral = dict(
            (name, 0.0) for name in self.names)
        self.local_moment_yaw_integral = dict(
            (name, 0.0) for name in self.names)
        self.stop_requested = False

        self.target_force_pub = rospy.Publisher(
            "/mujoco_triple_press/target_normal_force", Float32, queue_size=1)
        self.target_z_velocity_pub = rospy.Publisher(
            "/mujoco_triple_press/target_z_velocity", Float32, queue_size=1)
        self.translation_command_pub = rospy.Publisher(
            "/mujoco_triple_press/object_translation_command",
            Vector3Stamped, queue_size=1)
        self.translation_acceleration_pub = rospy.Publisher(
            "/mujoco_triple_press/object_translation_acceleration",
            Vector3Stamped, queue_size=1)
        self.observer_forces_pub = rospy.Publisher(
            "/mujoco_triple_press/observer_normal_forces",
            Vector3Stamped, queue_size=1)
        self.contact_error_pubs = dict(
            (name, rospy.Publisher(
                "/mujoco_triple_press/{}/contact_position_error".format(name),
                Vector3Stamped, queue_size=1))
            for name in self.names)
        self.local_contact_wrench_pubs = dict(
            (name, rospy.Publisher(
                "/mujoco_triple_press/{}/observer_local_contact_wrench".format(
                    name),
                WrenchStamped, queue_size=1))
            for name in self.names)
        self.object_resultant_wrench_pub = rospy.Publisher(
            "/mujoco_triple_press/observer_object_resultant_wrench",
            WrenchStamped, queue_size=1)
        self.object_force_arm_moment_pub = rospy.Publisher(
            "/mujoco_triple_press/observer_object_force_arm_moment",
            Vector3Stamped, queue_size=1)
        self.object_local_moment_sum_pub = rospy.Publisher(
            "/mujoco_triple_press/observer_object_local_moment_sum",
            Vector3Stamped, queue_size=1)
        self.local_moment_trim_pubs = dict(
            (name, rospy.Publisher(
                "/mujoco_triple_press/{}/local_moment_attitude_trim".format(
                    name),
                Vector3Stamped, queue_size=1))
            for name in self.names)
        self.allocated_force_targets_pub = rospy.Publisher(
            "/mujoco_triple_press/allocated_normal_force_targets",
            Vector3Stamped, queue_size=1)
        self.object_moment_yaw_trim_pub = rospy.Publisher(
            "/mujoco_triple_press/object_moment_yaw_trim",
            Float32, queue_size=1)
        self.lift_z_acceleration_offsets_pub = rospy.Publisher(
            "/mujoco_triple_press/lift_z_acceleration_offsets",
            Vector3Stamped, queue_size=1)
        self.lift_contact_z_acceleration_offsets_pub = rospy.Publisher(
            "/mujoco_triple_press/lift_contact_z_acceleration_offsets",
            Vector3Stamped, queue_size=1)
        self.lift_vertical_force_target_pub = rospy.Publisher(
            "/mujoco_triple_press/lift_vertical_force_target",
            Float32, queue_size=1)
        self.lift_vertical_force_filtered_pub = rospy.Publisher(
            "/mujoco_triple_press/lift_vertical_force_filtered",
            Float32, queue_size=1)
        self.lift_common_z_acceleration_pub = rospy.Publisher(
            "/mujoco_triple_press/lift_common_z_acceleration_command",
            Float32, queue_size=1)
        self.lift_reference_acceleration_pub = rospy.Publisher(
            "/mujoco_triple_press/lift_reference_acceleration",
            Float32, queue_size=1)
        self.lift_observer_force_correction_pub = rospy.Publisher(
            "/mujoco_triple_press/lift_observer_force_correction",
            Float32, queue_size=1)
        self.lift_force_ramp_ratio_pub = rospy.Publisher(
            "/mujoco_triple_press/lift_force_ramp_ratio",
            Float32, queue_size=1)
        self.lift_common_slip_acceleration_pub = rospy.Publisher(
            "/mujoco_triple_press/lift_common_slip_acceleration",
            Float32, queue_size=1)
        self.lift_yaw_state_pub = rospy.Publisher(
            "/mujoco_triple_press/lift_yaw_state",
            Vector3Stamped, queue_size=1)
        self.lift_yaw_tangential_acceleration_pub = rospy.Publisher(
            "/mujoco_triple_press/lift_yaw_tangential_acceleration",
            Float32, queue_size=1)
        self.object_yaw_velocity_command_pub = rospy.Publisher(
            "/mujoco_triple_press/object_yaw_velocity_command",
            Float32, queue_size=1)
        self.active_force_pub = rospy.Publisher(
            "/mujoco_triple_press/active_common_normal_force",
            Float32, queue_size=1)

    @staticmethod
    def clamp_value(value, lower, upper):
        return max(lower, min(upper, value))

    def set_common_normal_force(self, force):
        self.commanded_common_normal_force = force
        self.apply_normal_force_targets()

    def update_common_normal_force(self, dt):
        """Slew the applied preload so a keyboard step cannot shock contact."""
        dt = self.clamp_value(dt, 0.0, 0.05)
        maximum_step = self.normal_force_ramp_rate * dt
        self.active_common_normal_force += self.clamp_value(
            self.commanded_common_normal_force -
            self.active_common_normal_force,
            -maximum_step, maximum_step)
        self.apply_normal_force_targets()

    def apply_normal_force_targets(self):
        for name in self.names:
            target = (self.active_common_normal_force +
                      self.object_moment_force_offsets[name])
            if hasattr(self, "minimum_normal_force"):
                target = self.clamp_value(
                    target, self.minimum_normal_force,
                    self.maximum_normal_force)
            self.target_normal_forces[name] = target

    def common_normal_force(self):
        return self.commanded_common_normal_force

    def applied_common_normal_force(self):
        return self.active_common_normal_force

    def reset_lift_mode(self):
        self.lift_mode_active = False
        self.lift_fault = False
        self.lift_command_start_time = None
        self.lift_start_object_z = None
        self.lift_progress_confirmed = False
        self.lift_rise_confirmation_start_time = None
        self.lift_hold_reached = False
        self.lift_hold_target_object_z = None
        self.lift_peak_object_z = None
        self.filtered_object_vertical_force = 0.0
        self.lift_vertical_force_target = 0.0
        self.lift_observer_force_integral = 0.0
        self.lift_observer_force_correction = 0.0
        self.lift_reference_z = None
        self.lift_reference_velocity = 0.0
        self.lift_reference_acceleration = 0.0
        self.lift_object_acceleration_command = 0.0
        self.lift_common_z_acceleration_command = 0.0
        self.lift_force_ramp_ratio = 0.0
        self.lift_breakaway_force_active = False
        self.lift_hold_recovery_force = 0.0
        self.lift_common_slip_error = 0.0
        self.lift_common_slip_velocity = 0.0
        self.lift_common_slip_acceleration = 0.0
        self.lift_common_slip_feedback_active = False
        self.lift_yaw_target = None
        self.lift_yaw_error = 0.0
        self.lift_yaw_rate = 0.0
        self.lift_yaw_moment_feedback = 0.0
        self.lift_yaw_tangential_acceleration = 0.0
        self.yaw_velocity_command = 0.0
        self.translation_z_limit_active = 0
        self.lift_initial_relative_contact_z = None
        self.translation_velocity_command = [0.0, 0.0]
        self.translation_acceleration_command = [0.0, 0.0]
        for name in self.names:
            self.lift_z_acceleration_offsets[name] = 0.0
            self.lift_contact_z_acceleration_offsets[name] = 0.0

    def enter_lift_mode(self):
        if self.lift_mode_active:
            return
        self.lift_mode_active = True
        self.lift_fault = False
        self.lift_command_start_time = None
        self.lift_start_object_z = None
        self.lift_progress_confirmed = False
        self.lift_rise_confirmation_start_time = None
        self.lift_hold_reached = False
        self.lift_hold_target_object_z = None
        self.lift_peak_object_z = None
        self.filtered_object_vertical_force = 0.0
        self.lift_vertical_force_target = 0.0
        self.lift_observer_force_integral = 0.0
        self.lift_observer_force_correction = 0.0
        self.lift_reference_z = None
        self.lift_reference_velocity = 0.0
        self.lift_reference_acceleration = 0.0
        self.lift_object_acceleration_command = 0.0
        self.lift_common_z_acceleration_command = 0.0
        self.lift_force_ramp_ratio = 0.0
        self.lift_breakaway_force_active = True
        self.lift_hold_recovery_force = 0.0
        self.lift_common_slip_error = 0.0
        self.lift_common_slip_velocity = 0.0
        self.lift_common_slip_acceleration = 0.0
        self.lift_common_slip_feedback_active = False
        self.lift_yaw_target = None
        self.lift_yaw_error = 0.0
        self.lift_yaw_rate = 0.0
        self.lift_yaw_moment_feedback = 0.0
        self.lift_yaw_tangential_acceleration = 0.0
        self.yaw_velocity_command = 0.0
        self.translation_z_limit_active = 0
        # Capture the actual three-Bee mean on the first LIFT update.  The
        # geometric target can differ slightly from the settled contact pose.
        self.lift_initial_relative_contact_z = None
        self.translation_velocity_command = [0.0, 0.0]
        self.translation_acceleration_command = [0.0, 0.0]
        for name in self.names:
            self.lift_z_acceleration_offsets[name] = 0.0
            self.lift_contact_z_acceleration_offsets[name] = 0.0
        rospy.logwarn(
            "triple press: enter LIFT mode; trajectory force feed-forward "
            "controls common Z, observer supplies low-band correction")

    def lift_preload_ready(self):
        return (
            abs(self.commanded_common_normal_force -
                self.active_common_normal_force) <= 0.05 and
            max(abs(value) for value in
                self.object_moment_force_offsets.values()) <= 0.02)

    def abort_lift(self, reason):
        if self.lift_fault:
            return
        self.lift_fault = True
        self.target_z_velocity = 0.0
        self.lift_common_z_acceleration_command = 0.0
        self.lift_common_slip_acceleration = 0.0
        self.lift_yaw_tangential_acceleration = 0.0
        self.yaw_velocity_command = 0.0
        self.translation_velocity_command = [0.0, 0.0]
        self.translation_acceleration_command = [0.0, 0.0]
        for name in self.names:
            self.lift_z_acceleration_offsets[name] = 0.0
            self.lift_contact_z_acceleration_offsets[name] = 0.0
        rospy.logerr("triple press: LIFT HOLD: %s", reason)
        # Continuing radial force control after losing the object makes the
        # Bees chase a falling/rotating target and can destabilize all three.
        # End the demo so main() publishes a neutral stop command instead.
        self.stop_requested = True

    def target_baselink_rpy(self, name):
        return (self.commanded_roll + self.local_moment_roll_trim[name],
                0.0, 0.0)

    def moment_corrected_yaw(self, name, nominal_yaw):
        return self.angle_error(
            nominal_yaw + self.object_moment_yaw_trim +
            self.local_moment_yaw_trim[name], 0.0)

    def deadband_value(self, value):
        if abs(value) <= self.local_moment_deadband:
            return 0.0
        return value - math.copysign(self.local_moment_deadband, value)

    def reset_local_moment_control(self):
        for name in self.names:
            self.filtered_local_moment_contact[name] = [0.0, 0.0, 0.0]
            self.local_moment_roll_trim[name] = 0.0
            self.local_moment_yaw_trim[name] = 0.0
            self.local_moment_roll_integral[name] = 0.0
            self.local_moment_yaw_integral[name] = 0.0
            self.object_moment_force_offsets[name] = 0.0
            self.object_moment_force_integrals[name] = 0.0
        self.filtered_object_moment = [0.0, 0.0, 0.0]
        self.object_moment_yaw_trim = 0.0
        self.object_moment_yaw_integral = 0.0
        self.reset_lift_mode()
        self.apply_normal_force_targets()

    @staticmethod
    def solve_three_by_three(matrix, right_hand_side):
        """Solve a small dense system without adding a numpy dependency."""
        a, b, c = matrix[0]
        d, e, f = matrix[1]
        g, h, i = matrix[2]
        determinant = (a * (e * i - f * h) -
                       b * (d * i - f * g) +
                       c * (d * h - e * g))
        if abs(determinant) < 1.0e-10:
            return None
        x, y, z = right_hand_side
        return (
            (x * (e * i - f * h) - b * (y * i - f * z) +
             c * (y * h - e * z)) / determinant,
            (a * (y * i - f * z) - x * (d * i - f * g) +
             c * (d * z - y * g)) / determinant,
            (a * (e * z - y * h) - b * (d * z - y * g) +
            x * (d * h - e * g)) / determinant,
        )

    def unwind_normal_force_allocation(self, dt):
        """Return to equal preload before vertical friction carries load."""
        maximum_step = self.object_moment_force_rate_limit * dt
        for name in self.names:
            self.object_moment_force_integrals[name] = 0.0
            self.object_moment_force_offsets[name] += self.clamp_value(
                -self.object_moment_force_offsets[name],
                -maximum_step, maximum_step)
        offset_mean = sum(
            self.object_moment_force_offsets.values()) / len(self.names)
        for name in self.names:
            self.object_moment_force_offsets[name] -= offset_mean
        self.apply_normal_force_targets()

    def update_lift_progress(
            self, object_pose, odometry=None, object_twist=None):
        """Confirm object motion, stop at lift height, and detect a drop."""
        if (not self.lift_mode_active or self.lift_fault or
                not self.lift_preload_ready()):
            return
        now = rospy.get_time()
        if self.lift_command_start_time is None:
            if (self.target_z_velocity <= self.lift_velocity_threshold or
                    self.lift_force_ramp_ratio < 1.0 - 1.0e-6):
                return
            self.lift_command_start_time = now
            self.lift_start_object_z = object_pose.position.z
            self.lift_peak_object_z = object_pose.position.z
            return
        self.lift_peak_object_z = max(
            self.lift_peak_object_z, object_pose.position.z)
        rise = object_pose.position.z - self.lift_start_object_z
        if not self.lift_progress_confirmed:
            if rise >= self.lift_minimum_object_rise:
                if self.lift_rise_confirmation_start_time is None:
                    self.lift_rise_confirmation_start_time = now
                elif (now - self.lift_rise_confirmation_start_time >=
                      self.lift_rise_confirmation_duration):
                    self.lift_progress_confirmed = True
                    rospy.loginfo(
                        "triple press: sustained LIFT confirmed, "
                        "object rise %.4f m",
                        rise)
            else:
                self.lift_rise_confirmation_start_time = None
        if (not self.lift_progress_confirmed and
                now - self.lift_command_start_time >=
                self.lift_progress_timeout):
            self.abort_lift(
                "object slipped: rise {:.4f} m in {:.1f} s".format(
                    rise, self.lift_progress_timeout))
            return
        if (not self.lift_hold_reached and
                self.lift_hold_height > 0.0 and
                rise >= self.lift_hold_height):
            self.lift_progress_confirmed = True
            self.target_z_velocity = 0.0
            self.lift_hold_reached = True
            self.lift_breakaway_force_active = False
            self.lift_hold_recovery_force = 0.0
            # The acceleration-limited reference decelerates at the requested
            # height, so retain that geometric target instead of freezing a
            # possible measurement overshoot as the new hover altitude.
            self.lift_hold_target_object_z = (
                self.lift_start_object_z + self.lift_hold_height)
            # The reference trajectory has already decelerated toward this
            # altitude.  Clear only the slow observer correction integral;
            # gravity and trajectory acceleration remain feed-forward terms.
            self.lift_observer_force_integral = 0.0
            self.lift_observer_force_correction = 0.0
            rospy.loginfo(
                "triple press: LIFT HOLD reached at object rise %.4f m; "
                "switch total-Z target to mg; retained initial contact z %.4f m",
                rise, self.lift_initial_relative_contact_z)
        if (self.lift_hold_reached and
                self.lift_hold_target_object_z - object_pose.position.z >=
                self.lift_drop_abort_distance):
            self.abort_lift(
                "object fell {:.4f} m below hold target".format(
                    self.lift_hold_target_object_z -
                    object_pose.position.z))

    def update_lift_reference(self, object_pose, dt):
        """Advance an acceleration/jerk-limited object-height reference."""
        if self.lift_reference_z is None:
            self.lift_reference_z = object_pose.position.z
            self.lift_reference_velocity = 0.0
            self.lift_reference_acceleration = 0.0

        target_z = (
            self.lift_hold_target_object_z if self.lift_hold_reached else
            self.lift_start_object_z + self.lift_hold_height)
        error = target_z - self.lift_reference_z
        acceleration_limit = max(
            1.0e-6, self.lift_trajectory_acceleration_limit)
        velocity_limit = max(
            self.lift_velocity_threshold, abs(self.target_z_velocity))
        if self.lift_hold_reached:
            velocity_limit = max(velocity_limit, self.lift_velocity_threshold)

        stopping_velocity = math.sqrt(
            max(0.0, 2.0 * acceleration_limit * abs(error)))
        desired_velocity = math.copysign(
            min(velocity_limit, stopping_velocity), error)
        if abs(error) < 1.0e-5:
            desired_velocity = 0.0
        desired_acceleration = self.clamp_value(
            (desired_velocity - self.lift_reference_velocity) /
            max(dt, 1.0e-6),
            -acceleration_limit, acceleration_limit)
        maximum_acceleration_step = self.lift_trajectory_jerk_limit * dt
        self.lift_reference_acceleration += self.clamp_value(
            desired_acceleration - self.lift_reference_acceleration,
            -maximum_acceleration_step, maximum_acceleration_step)
        self.lift_reference_velocity += (
            self.lift_reference_acceleration * dt)
        self.lift_reference_velocity = self.clamp_value(
            self.lift_reference_velocity, -velocity_limit, velocity_limit)
        next_reference_z = (
            self.lift_reference_z + self.lift_reference_velocity * dt)
        if ((target_z - self.lift_reference_z) *
                (target_z - next_reference_z) <= 0.0):
            self.lift_reference_z = target_z
            self.lift_reference_velocity = 0.0
            self.lift_reference_acceleration = 0.0
        else:
            self.lift_reference_z = next_reference_z

    def update_lift_z_control(
            self, odometry, object_pose, object_twist,
            observed_object_vertical_force, dt):
        """Generate common feed-forward and zero-sum Z accelerations.

        The object trajectory supplies Fz=m_object*(g+a_ref) before observer
        feedback.  The momentum observer is deliberately restricted to a
        slow, bounded force correction.  Contact-height and Mx/My components
        remain zero-sum and therefore cannot alter the requested common force.
        """
        if not self.lift_mode_active:
            return

        dt = self.clamp_value(dt, 0.0, 0.05)
        filter_tau = max(
            1.0e-3,
            self.lift_hold_observer_force_time_constant
            if self.lift_hold_reached else
            self.lift_observer_force_time_constant)
        alpha = dt / (filter_tau + dt) if dt > 0.0 else 0.0
        self.filtered_object_vertical_force += alpha * (
            observed_object_vertical_force -
            self.filtered_object_vertical_force)

        states = dict(
            (name, self.contact_position_state(
                name, odometry[name], object_pose, object_twist))
            for name in self.names)
        relative_z_mean = sum(
            state[7] for state in states.values()) / len(self.names)
        relative_z_velocity_mean = sum(
            state[8] for state in states.values()) / len(self.names)
        if self.lift_initial_relative_contact_z is None:
            self.lift_initial_relative_contact_z = relative_z_mean

        self.lift_common_slip_error = (
            relative_z_mean - self.lift_initial_relative_contact_z)
        self.lift_common_slip_velocity = relative_z_velocity_mean
        if (not self.lift_common_slip_feedback_active and
                self.lift_start_object_z is not None and
                object_pose.position.z - self.lift_start_object_z >=
                self.lift_common_slip_activation_rise):
            # The compliant rubber balls need a small loading displacement
            # before static vertical friction is established.  Latch the
            # relative height only once the object itself has left the
            # pedestal; from here onward common motion is true slip.
            self.lift_common_slip_feedback_active = True
            self.lift_initial_relative_contact_z = relative_z_mean
            self.lift_common_slip_error = 0.0
            self.lift_common_slip_velocity = relative_z_velocity_mean
            self.lift_common_slip_acceleration = 0.0
            rospy.loginfo(
                "triple press: common-slip feedback latched at object rise "
                "%.4f m, relative contact z %.4f m",
                object_pose.position.z - self.lift_start_object_z,
                relative_z_mean)

        if self.lift_common_slip_feedback_active:
            if abs(self.lift_common_slip_error) > \
                    self.lift_relative_z_error_limit:
                self.abort_lift(
                    "mean contact plane slid {:.4f} m vertically".format(
                        self.lift_common_slip_error))
                return

            desired_common_slip_acceleration = self.clamp_value(
                -self.lift_common_slip_kp * self.lift_common_slip_error -
                self.lift_common_slip_kd * self.lift_common_slip_velocity,
                -self.lift_common_slip_acceleration_limit,
                self.lift_common_slip_acceleration_limit)
            maximum_common_slip_step = (
                self.lift_common_slip_acceleration_rate_limit * dt)
            self.lift_common_slip_acceleration += self.clamp_value(
                desired_common_slip_acceleration -
                self.lift_common_slip_acceleration,
                -maximum_common_slip_step, maximum_common_slip_step)
        else:
            self.lift_common_slip_acceleration = 0.0

        desired_contact_offsets = {}
        for name in self.names:
            relative_difference = states[name][7] - relative_z_mean
            if abs(relative_difference) > self.lift_relative_z_difference_limit:
                self.abort_lift(
                    "{} contact height differs from formation by {:.4f} m".
                    format(name, relative_difference))
                return
            velocity_difference = states[name][8] - relative_z_velocity_mean
            desired_contact_offsets[name] = self.clamp_value(
                -self.lift_relative_z_kp * relative_difference -
                self.lift_relative_z_kd * velocity_difference,
                -self.lift_contact_z_acceleration_limit,
                self.lift_contact_z_acceleration_limit)
        desired_mean = sum(
            desired_contact_offsets.values()) / len(self.names)
        maximum_contact_step = (
            self.lift_contact_z_acceleration_rate_limit * dt)
        for name in self.names:
            desired = desired_contact_offsets[name] - desired_mean
            current = self.lift_contact_z_acceleration_offsets[name]
            self.lift_contact_z_acceleration_offsets[name] += \
                self.clamp_value(
                    desired - current,
                    -maximum_contact_step, maximum_contact_step)
        applied_mean = sum(
            self.lift_contact_z_acceleration_offsets.values()) / len(self.names)
        for name in self.names:
            self.lift_contact_z_acceleration_offsets[name] -= applied_mean

        if (self.lift_fault or not self.lift_preload_ready() or dt <= 0.0):
            self.lift_common_z_acceleration_command = 0.0
            return
        if (not self.lift_hold_reached and
                self.target_z_velocity <= self.lift_velocity_threshold):
            self.lift_common_z_acceleration_command = 0.0
            self.lift_observer_force_integral = 0.0
            self.lift_observer_force_correction = 0.0
            return

        if (self.lift_hold_reached and
                self.lift_hold_target_object_z is not None and
                self.lift_start_object_z is not None):
            # After the initial lift, [ and ] become a velocity joystick for
            # the object altitude.  Move the reference, not the Bee roots, so
            # the same force/observer loop remains responsible for support.
            nominal_hold_z = (
                self.lift_start_object_z + self.lift_hold_height)
            minimum_target_z = max(
                self.lift_start_object_z + self.lift_minimum_object_rise,
                nominal_hold_z - self.translation_z_range)
            maximum_target_z = nominal_hold_z + self.translation_z_range
            unclamped_target_z = (
                self.lift_hold_target_object_z +
                self.target_z_velocity * dt)
            self.lift_hold_target_object_z = self.clamp_value(
                unclamped_target_z, minimum_target_z, maximum_target_z)
            self.translation_z_limit_active = 0
            if abs(self.lift_hold_target_object_z - unclamped_target_z) > \
                    1.0e-9:
                self.translation_z_limit_active = (
                    1 if unclamped_target_z > maximum_target_z else -1)
                self.target_z_velocity = 0.0
                rospy.logwarn(
                    "triple press: object Z command stopped at %s limit "
                    "%.3f m (set ~translation_z_range to extend it)",
                    "upper" if self.translation_z_limit_active > 0 else
                    "lower",
                    self.lift_hold_target_object_z)

        ramp_duration = max(0.0, self.lift_force_ramp_duration)
        if ramp_duration <= 1.0e-6:
            self.lift_force_ramp_ratio = 1.0
        else:
            self.lift_force_ramp_ratio = min(
                1.0, self.lift_force_ramp_ratio + dt / ramp_duration)

        if (self.lift_force_ramp_ratio >= 1.0 - 1.0e-6 and
                self.lift_start_object_z is not None):
            self.update_lift_reference(object_pose, dt)
            tracking_acceleration = self.clamp_value(
                self.lift_trajectory_position_kp * (
                    self.lift_reference_z - object_pose.position.z) +
                self.lift_trajectory_velocity_kd * (
                    self.lift_reference_velocity - object_twist[2]),
                -self.lift_trajectory_tracking_acceleration_limit,
                self.lift_trajectory_tracking_acceleration_limit)
            self.lift_object_acceleration_command = self.clamp_value(
                self.lift_reference_acceleration + tracking_acceleration,
                -self.lift_trajectory_acceleration_limit -
                self.lift_trajectory_tracking_acceleration_limit,
                self.lift_trajectory_acceleration_limit +
                self.lift_trajectory_tracking_acceleration_limit)
        else:
            # First unload the pedestal smoothly.  Do not advance the object
            # trajectory until the complete static mg feed-forward is active.
            self.lift_reference_z = object_pose.position.z
            self.lift_reference_velocity = 0.0
            self.lift_reference_acceleration = 0.0
            self.lift_object_acceleration_command = 0.0

        if (self.lift_hold_reached and
                self.lift_hold_target_object_z is not None):
            # Do not switch a fixed 0.4 N margin on and off.  That drove a
            # several-millimetre height limit cycle which ratcheted the rubber
            # contacts upward at every cycle.  A bounded force-domain PD term
            # supplies only the missing support and is slew-limited before it
            # reaches the acceleration feed-forward path.
            hold_drop = (
                self.lift_hold_target_object_z - object_pose.position.z)
            desired_recovery_force = self.clamp_value(
                self.lift_hold_recovery_force_kp * hold_drop -
                self.lift_hold_recovery_force_kd * object_twist[2],
                0.0, self.lift_breakaway_force_margin)
            maximum_recovery_step = (
                self.lift_hold_recovery_force_rate_limit * dt)
            self.lift_hold_recovery_force += self.clamp_value(
                desired_recovery_force - self.lift_hold_recovery_force,
                -maximum_recovery_step, maximum_recovery_step)
            self.lift_breakaway_force_active = (
                self.lift_hold_recovery_force > 1.0e-3)
            breakaway_force = self.lift_hold_recovery_force
        else:
            self.lift_hold_recovery_force = 0.0
            breakaway_force = (
                self.lift_force_ramp_ratio * self.lift_breakaway_force_margin
                if self.lift_breakaway_force_active else 0.0)
        self.lift_vertical_force_target = max(
            0.0, self.grasp_object_mass * (
                self.lift_force_ramp_ratio * self.gravity_acceleration +
                self.lift_object_acceleration_command) + breakaway_force)
        force_error = self.deadband_with_limit(
            self.lift_vertical_force_target -
            self.filtered_object_vertical_force,
            self.lift_vertical_force_deadband)
        observer_kp = (
            self.lift_hold_observer_force_kp if self.lift_hold_reached else
            self.lift_observer_force_kp)
        observer_ki = (
            self.lift_hold_observer_force_ki if self.lift_hold_reached else
            self.lift_observer_force_ki)
        observer_correction_limit = (
            self.lift_hold_observer_force_correction_limit
            if self.lift_hold_reached else
            self.lift_observer_force_correction_limit)
        self.lift_observer_force_integral = self.clamp_value(
            self.lift_observer_force_integral +
            observer_ki * force_error * dt,
            -self.lift_observer_force_integral_limit,
            self.lift_observer_force_integral_limit)
        self.lift_observer_force_correction = self.clamp_value(
            observer_kp * force_error +
            self.lift_observer_force_integral,
            -observer_correction_limit,
            observer_correction_limit)

        force_feedforward_scale = (
            self.lift_hold_force_feedforward_scale
            if self.lift_hold_reached else 1.0)
        corrected_object_force = max(
            0.0, force_feedforward_scale *
            self.lift_vertical_force_target +
            self.lift_observer_force_correction)
        force_acceleration_per_bee = (
            corrected_object_force /
            (len(self.names) * max(1.0e-6, self.robot_mass)))
        common_acceleration = (
            self.lift_object_acceleration_command +
            force_acceleration_per_bee +
            self.lift_common_slip_acceleration)
        if self.lift_common_slip_feedback_active:
            # Relative-position correction must not remove the acceleration
            # needed to carry the object's static weight.  If friction is
            # marginal, reducing below this floor makes the object fall and
            # turns the slip loop into positive feedback.
            support_floor = (
                self.lift_hold_support_floor_scale *
                self.grasp_object_mass * self.gravity_acceleration /
                (len(self.names) * max(1.0e-6, self.robot_mass)))
            common_acceleration = max(common_acceleration, support_floor)
        self.lift_common_z_acceleration_command = self.clamp_value(
            common_acceleration,
            -self.lift_common_z_acceleration_limit,
            self.lift_common_z_acceleration_limit)

    def update_translation_control(self, object_twist, dt):
        """Track a keyboard world-XY velocity with common Bee acceleration.

        The same acceleration is added to all three Bees.  Radial preload,
        relative tangential position, and zero-sum moment corrections remain
        independent, so this moves the grasped formation without changing the
        requested mean normal force.
        """
        if (not self.lift_hold_reached or self.lift_fault or dt <= 0.0):
            self.translation_acceleration_command = [0.0, 0.0]
            return tuple(self.translation_acceleration_command)

        dt = self.clamp_value(dt, 0.0, 0.05)
        desired = [
            self.translation_velocity_kp * (
                self.translation_velocity_command[0] - object_twist[0]),
            self.translation_velocity_kp * (
                self.translation_velocity_command[1] - object_twist[1]),
        ]
        desired_norm = math.hypot(desired[0], desired[1])
        if desired_norm > self.translation_acceleration_limit:
            scale = self.translation_acceleration_limit / desired_norm
            desired[0] *= scale
            desired[1] *= scale
        maximum_step = self.translation_acceleration_rate_limit * dt
        for axis in range(2):
            self.translation_acceleration_command[axis] += self.clamp_value(
                desired[axis] - self.translation_acceleration_command[axis],
                -maximum_step, maximum_step)
        return tuple(self.translation_acceleration_command)

    def update_lift_yaw_control(self, object_pose, object_twist, dt):
        """Hold object yaw with a pure circulating tangential command.

        Every Bee receives the same scalar acceleration along its own face
        tangent.  The three tangential world-force vectors sum to zero for an
        equilateral grasp, while their moments about the prism centre add.
        Object yaw and yaw rate provide the primary restoring feedback; the
        filtered observer Mz is only a low-band correction for contact bias.
        """
        current_yaw = self.quaternion_to_rpy(object_pose.orientation)[2]
        if not self.lift_mode_active or self.lift_fault:
            self.lift_yaw_target = None
            self.lift_yaw_error = 0.0
            self.lift_yaw_rate = object_twist[3]
            self.lift_yaw_moment_feedback = 0.0
            self.lift_yaw_tangential_acceleration = 0.0
            return 0.0

        if self.lift_yaw_target is None:
            self.lift_yaw_target = current_yaw
            rospy.loginfo(
                "triple press: latch LIFT object yaw target %.4f rad",
                self.lift_yaw_target)

        dt = self.clamp_value(dt, 0.0, 0.05)
        if self.lift_hold_reached and dt > 0.0:
            self.lift_yaw_target = self.angle_error(
                self.lift_yaw_target +
                self.yaw_velocity_command * dt, 0.0)
        self.lift_yaw_error = self.angle_error(
            current_yaw, self.lift_yaw_target)
        self.lift_yaw_rate = object_twist[3]
        self.lift_yaw_moment_feedback = self.deadband_with_limit(
            self.filtered_object_moment[2],
            self.lift_yaw_moment_deadband)

        if (not self.lift_yaw_control_enabled or dt <= 0.0 or
                not self.lift_preload_ready()):
            desired = 0.0
        else:
            desired = -(
                self.lift_yaw_angle_kp * self.lift_yaw_error +
                self.lift_yaw_rate_kd * self.lift_yaw_rate +
                self.lift_yaw_observer_moment_gain *
                self.lift_yaw_moment_feedback)
            desired = self.clamp_value(
                desired,
                -self.lift_yaw_tangential_acceleration_limit,
                self.lift_yaw_tangential_acceleration_limit)

        maximum_step = (
            self.lift_yaw_tangential_acceleration_rate_limit * dt)
        self.lift_yaw_tangential_acceleration += self.clamp_value(
            desired - self.lift_yaw_tangential_acceleration,
            -maximum_step, maximum_step)

        if (self.lift_hold_reached and
                abs(self.lift_yaw_error) > self.lift_yaw_error_abort_limit):
            self.abort_lift(
                "object yaw error {:.3f} rad exceeded {:.3f} rad".format(
                    self.lift_yaw_error, self.lift_yaw_error_abort_limit))
        return self.lift_yaw_tangential_acceleration

    def update_lift_moment_control(
            self, local_states, object_pose, normal_forces, dt):
        """Balance vertical friction with per-Bee Z-acceleration offsets.

        During lift, an upward force at a side contact creates object Mx/My
        through the large horizontal arm.  Correct it with vertical motion,
        not with radial force through the very small contact-height arm.
        """
        if not self.lift_mode_active:
            for name in self.names:
                self.lift_z_acceleration_offsets[name] = 0.0
            return
        horizontal_moment = math.sqrt(
            self.filtered_object_moment[0] ** 2 +
            self.filtered_object_moment[1] ** 2)
        if horizontal_moment > self.lift_abort_moment_threshold:
            self.abort_lift(
                "horizontal object moment {:.4f} Nm exceeds {:.4f} Nm".format(
                    horizontal_moment, self.lift_abort_moment_threshold))
            return
        if (self.lift_fault or not self.lift_moment_control_enabled or
                min(normal_forces.values()) <
                self.local_moment_control_force_threshold):
            return

        # Column i maps one newton of upward object force at contact i to
        # object Mx/My.  The third equation keeps acceleration corrections
        # zero-mean, so this loop cannot change total vertical force.
        columns = []
        for name in self.names:
            contact = local_states[name]["contact_center"]
            arm = (contact[0] - object_pose.position.x,
                   contact[1] - object_pose.position.y,
                   contact[2] - object_pose.position.z)
            columns.append(self.cross(arm, (0.0, 0.0, 1.0)))
        allocation_matrix = (
            tuple(column[0] for column in columns),
            tuple(column[1] for column in columns),
            (1.0, 1.0, 1.0),
        )
        moment_error = (
            self.deadband_with_limit(
                self.filtered_object_moment[0], self.lift_moment_deadband),
            self.deadband_with_limit(
                self.filtered_object_moment[1], self.lift_moment_deadband),
        )
        vertical_force_correction = self.solve_three_by_three(
            allocation_matrix,
            (-moment_error[0], -moment_error[1], 0.0))
        if vertical_force_correction is None:
            self.abort_lift("vertical-friction allocation is singular")
            return
        maximum_step = self.lift_moment_acceleration_rate_limit * dt
        for index, name in enumerate(self.names):
            desired = self.clamp_value(
                self.lift_moment_acceleration_gain *
                vertical_force_correction[index] /
                max(1.0e-6, self.robot_mass),
                -self.lift_moment_acceleration_limit,
                self.lift_moment_acceleration_limit)
            self.lift_z_acceleration_offsets[name] += self.clamp_value(
                desired - self.lift_z_acceleration_offsets[name],
                -maximum_step, maximum_step)
        offset_mean = sum(
            self.lift_z_acceleration_offsets.values()) / len(self.names)
        for name in self.names:
            self.lift_z_acceleration_offsets[name] -= offset_mean

    def update_object_moment_control(
            self, local_states, object_pose, object_moment,
            normal_forces, dt):
        """Allocate the common preload to cancel object roll/pitch moment.

        The three force offsets are constrained to sum to zero, so keyboard
        input still sets the mean preload.  Object yaw moment is handled by
        the per-Bee local vertical-moment attitude loop; radial force
        allocation has only two independent degrees of freedom.
        """
        alpha = self.clamp_value(
            self.object_moment_filter_alpha, 0.0, 1.0)
        for axis in range(3):
            self.filtered_object_moment[axis] += alpha * (
                object_moment[axis] - self.filtered_object_moment[axis])
        dt = self.clamp_value(dt, 0.0, 0.05)
        if (not self.object_moment_control_enabled or dt <= 0.0 or
                min(normal_forces.values()) <
                self.local_moment_control_force_threshold):
            self.apply_normal_force_targets()
            return

        if self.lift_mode_active:
            # Attitude trims were calibrated for side preload.  Freeze them
            # during lift, unwind radial allocation, and use the correctly
            # conditioned vertical-friction allocation instead.
            self.unwind_normal_force_allocation(dt)
            self.update_lift_moment_control(
                local_states, object_pose, normal_forces, dt)
            return

        # A common negative yaw target reduces positive resultant object Mz
        # in the final-base-link-rot convention used here.  This sign was
        # validated by a reversed-sign test, which diverged immediately.
        yaw_error = self.deadband_with_limit(
            self.filtered_object_moment[2], self.object_moment_deadband)
        # Once Mz crosses zero, the accumulated trim has the wrong sign and
        # would otherwise cause a minute-scale limit cycle.  Unwind only in
        # that condition; keep ordinary integral action for static bias.
        if self.object_moment_yaw_integral * yaw_error > 0.0:
            unwind = max(
                0.0, 1.0 -
                self.object_moment_yaw_integral_unwind_rate * dt)
            self.object_moment_yaw_integral *= unwind
        self.object_moment_yaw_integral = self.clamp_value(
            self.object_moment_yaw_integral -
            self.object_moment_yaw_trim_ki * yaw_error * dt,
            -self.local_moment_trim_angle_limit,
            self.local_moment_trim_angle_limit)
        desired_yaw_trim = self.clamp_value(
            self.object_moment_yaw_integral -
            self.object_moment_yaw_trim_kp * yaw_error,
            -self.local_moment_trim_angle_limit,
            self.local_moment_trim_angle_limit)
        maximum_yaw_step = self.local_moment_trim_rate_limit * dt
        self.object_moment_yaw_trim += self.clamp_value(
            desired_yaw_trim - self.object_moment_yaw_trim,
            -maximum_yaw_step, maximum_yaw_step)

        # Column i is d(M_object)/d(F_i) for one additional newton of
        # outward observer force on Bee i.  The object receives -normal_i.
        columns = []
        for name in self.names:
            contact = local_states[name]["contact_center"]
            normal, _, _ = self.face_frame(name, object_pose)
            arm = (contact[0] - object_pose.position.x,
                   contact[1] - object_pose.position.y,
                   contact[2] - object_pose.position.z)
            columns.append(self.cross(arm, (-normal[0], -normal[1], 0.0)))
        allocation_matrix = (
            tuple(column[0] for column in columns),
            tuple(column[1] for column in columns),
            (1.0, 1.0, 1.0),
        )
        moment_error = (
            self.deadband_with_limit(
                self.filtered_object_moment[0], self.object_moment_deadband),
            self.deadband_with_limit(
                self.filtered_object_moment[1], self.object_moment_deadband),
        )
        correction = self.solve_three_by_three(
            allocation_matrix,
            (-moment_error[0], -moment_error[1], 0.0))
        if correction is None:
            rospy.logwarn_throttle(
                1.0, "triple press: object moment allocation is singular")
            return

        limit = self.object_moment_force_offset_limit
        for index, name in enumerate(self.names):
            self.object_moment_force_integrals[name] = self.clamp_value(
                self.object_moment_force_integrals[name] +
                self.object_moment_force_ki * correction[index] * dt,
                -limit, limit)
        integral_mean = sum(
            self.object_moment_force_integrals.values()) / len(self.names)
        for name in self.names:
            self.object_moment_force_integrals[name] -= integral_mean

        maximum_step = self.object_moment_force_rate_limit * dt
        for index, name in enumerate(self.names):
            desired = self.clamp_value(
                self.object_moment_force_integrals[name] +
                self.object_moment_force_kp * correction[index],
                -limit, limit)
            self.object_moment_force_offsets[name] += self.clamp_value(
                desired - self.object_moment_force_offsets[name],
                -maximum_step, maximum_step)
        offset_mean = sum(
            self.object_moment_force_offsets.values()) / len(self.names)
        for name in self.names:
            self.object_moment_force_offsets[name] -= offset_mean
        self.apply_normal_force_targets()

    @staticmethod
    def deadband_with_limit(value, deadband):
        if abs(value) <= deadband:
            return 0.0
        return value - math.copysign(deadband, value)

    def update_local_moment_control(
            self, local_states, normal_forces, dt):
        """Trim roll and differential yaw; never use per-foot sensors."""
        alpha = self.clamp_value(
            self.local_moment_filter_alpha, 0.0, 1.0)
        dt = self.clamp_value(dt, 0.0, 0.05)
        for name in self.names:
            measured = local_states[name]["local_moment_contact"]
            filtered = self.filtered_local_moment_contact[name]
            for axis in range(3):
                filtered[axis] += alpha * (measured[axis] - filtered[axis])
        if self.lift_mode_active:
            return
        vertical_mean = sum(
            self.filtered_local_moment_contact[name][2]
            for name in self.names) / len(self.names)
        active_yaw_names = []
        for name in self.names:
            filtered = self.filtered_local_moment_contact[name]
            if (not self.local_moment_control_enabled or
                    normal_forces[name] <
                    self.local_moment_control_force_threshold or dt <= 0.0):
                continue

            # At roll=+pi/2, contact tangent is base +X and world Z is
            # approximately base +Y.  Positive roll/yaw trim drives the
            # corresponding local moment negative, so integrating with the
            # measured-moment sign supplies negative feedback.
            tangent_error = self.deadband_value(filtered[1])
            # Common vertical moment belongs to the object-resultant loop.
            # Here only equalise the three Bees, using zero-sum yaw trims.
            vertical_error = self.deadband_value(
                filtered[2] - vertical_mean)
            # Observer noise can otherwise drive a very slow attitude limit
            # cycle, so both PI terms stop inside the validated noise band.
            self.local_moment_roll_integral[name] = self.clamp_value(
                self.local_moment_roll_integral[name] +
                self.local_moment_tangent_trim_ki * tangent_error * dt,
                -self.local_moment_trim_angle_limit,
                self.local_moment_trim_angle_limit)
            self.local_moment_yaw_integral[name] = self.clamp_value(
                self.local_moment_yaw_integral[name] +
                self.local_moment_vertical_trim_ki * vertical_error * dt,
                -self.local_moment_trim_angle_limit,
                self.local_moment_trim_angle_limit)
            desired_roll_trim = self.clamp_value(
                self.local_moment_roll_integral[name] +
                self.local_moment_tangent_trim_kp * tangent_error,
                -self.local_moment_trim_angle_limit,
                self.local_moment_trim_angle_limit)
            desired_yaw_trim = self.clamp_value(
                self.local_moment_yaw_integral[name] +
                self.local_moment_vertical_trim_kp * vertical_error,
                -self.local_moment_trim_angle_limit,
                self.local_moment_trim_angle_limit)
            maximum_step = self.local_moment_trim_rate_limit * dt
            self.local_moment_roll_trim[name] += self.clamp_value(
                desired_roll_trim - self.local_moment_roll_trim[name],
                -maximum_step, maximum_step)
            self.local_moment_yaw_trim[name] += self.clamp_value(
                desired_yaw_trim - self.local_moment_yaw_trim[name],
                -maximum_step, maximum_step)
            active_yaw_names.append(name)
        if active_yaw_names:
            integral_mean = sum(
                self.local_moment_yaw_integral[name]
                for name in active_yaw_names) / len(active_yaw_names)
            trim_mean = sum(
                self.local_moment_yaw_trim[name]
                for name in active_yaw_names) / len(active_yaw_names)
            for name in active_yaw_names:
                self.local_moment_yaw_integral[name] -= integral_mean
                self.local_moment_yaw_trim[name] -= trim_mean

    def publish_local_moment_trims(self):
        stamp = rospy.Time.now()
        for name in self.names:
            message = Vector3Stamped()
            message.header.stamp = stamp
            message.header.frame_id = "world"
            message.vector.x = self.local_moment_roll_trim[name]
            message.vector.z = self.local_moment_yaw_trim[name]
            self.local_moment_trim_pubs[name].publish(message)
        self.object_moment_yaw_trim_pub.publish(
            Float32(data=self.object_moment_yaw_trim))

    @staticmethod
    def cross(first, second):
        return (
            first[1] * second[2] - first[2] * second[1],
            first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0],
        )

    @staticmethod
    def vector_norm3(vector):
        return math.sqrt(sum(value * value for value in vector))

    @staticmethod
    def raw_wrench_tuple(message):
        return (
            message.wrench.force.x,
            message.wrench.force.y,
            message.wrench.force.z,
            message.wrench.torque.x,
            message.wrench.torque.y,
            message.wrench.torque.z,
        )

    def checked_raw_observer_wrench(self, name, wrenches, receipt_times):
        if name not in wrenches:
            raise RuntimeError(
                "{} momentum observer is unavailable".format(name))
        age = time.monotonic() - receipt_times[name]
        if age > self.observer_timeout:
            raise RuntimeError(
                "{} momentum observer is stale ({:.2f} s)".format(
                    name, age))
        return self.raw_wrench_tuple(wrenches[name])

    def face_frame(self, name, object_pose=None):
        """Return each face frame using the prism's current TF orientation."""
        if object_pose is None:
            object_pose = self.object_pose
        object_yaw = self.object_yaw
        if object_pose is not None:
            object_yaw = self.quaternion_to_rpy(object_pose.orientation)[2]
        index = self.names.index(name)
        angle = object_yaw + (
            math.pi / 3.0, math.pi, 5.0 * math.pi / 3.0)[index]
        normal = (math.cos(angle), math.sin(angle))
        tangent = (-normal[1], normal[0])
        inward_yaw = math.atan2(-normal[1], -normal[0])
        approach_yaw = self.angle_error(inward_yaw - math.pi / 2.0, 0.0)
        return normal, tangent, approach_yaw

    def observer_baseline_dual(self, staging_radius):
        """Calibrate all six observer axes before contact.

        The parent demo only needs a world-force baseline.  Keep that return
        format for its normal-force controller, while retaining a separate
        six-axis CoG-frame baseline for the monitoring wrench calculation.
        """
        world_force_totals = dict(
            (name, [0.0, 0.0, 0.0]) for name in self.names)
        cog_wrench_totals = dict(
            (name, [0.0] * 6) for name in self.names)
        samples = 0
        deadline = time.monotonic() + self.observer_baseline_duration
        rospy.loginfo(
            "triple press: calibrate six-axis observer baseline for %.2f s",
            self.observer_baseline_duration)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            _, odometry, _, object_pose = self.dual_snapshot()
            wrenches, receipt_times, cog_odometry = self.observer_snapshot()
            for name in self.names:
                raw = self.checked_raw_observer_wrench(
                    name, wrenches, receipt_times)
                for axis in range(6):
                    cog_wrench_totals[name][axis] += raw[axis]
                force_world = self.rotate_cog_to_world(
                    raw[:3], cog_odometry[name].pose.pose.orientation)
                for axis in range(3):
                    world_force_totals[name][axis] += force_world[axis]
                ax, ay, yaw, _, _, _, _ = self.approach_acceleration(
                    name, odometry[name], object_pose, staging_radius)
                self.publish_accel_nav(name, ax, ay, yaw)
            samples += 1
            time.sleep(self.control_period)
        if samples == 0:
            raise RuntimeError("six-axis observer baseline has no samples")

        self.observer_wrench_baseline_cog = dict(
            (name, tuple(value / samples
                         for value in cog_wrench_totals[name]))
            for name in self.names)
        world_force_baseline = dict(
            (name, tuple(value / samples
                         for value in world_force_totals[name]))
            for name in self.names)
        for name in self.names:
            baseline = self.observer_wrench_baseline_cog[name]
            rospy.loginfo(
                "triple press: %s observer baseline CoG "
                "F[%.3f, %.3f, %.3f] N M[%.4f, %.4f, %.4f] Nm",
                name, baseline[0], baseline[1], baseline[2],
                baseline[3], baseline[4], baseline[5])
        return world_force_baseline

    def observer_contact_wrenches(self, odometry, object_pose):
        """Return observer contact wrenches without feeding them to control.

        Each local wrench is the external wrench acting on the Bee, expressed
        in world axes and transported from the Bee CoG to the nominal centre
        of its four rubber balls.  The object resultant applies the opposite
        local wrenches at those centres and transports them to the object CoG.
        """
        if self.observer_wrench_baseline_cog is None:
            raise RuntimeError("six-axis observer baseline is unavailable")
        wrenches, receipt_times, cog_odometry = self.observer_snapshot()
        local_states = {}
        object_force = [0.0, 0.0, 0.0]
        object_force_arm_moment = [0.0, 0.0, 0.0]
        object_local_moment_sum = [0.0, 0.0, 0.0]
        object_center = (
            object_pose.position.x,
            object_pose.position.y,
            object_pose.position.z,
        )

        for name in self.names:
            raw = self.checked_raw_observer_wrench(
                name, wrenches, receipt_times)
            baseline = self.observer_wrench_baseline_cog[name]
            delta_force_cog = tuple(
                raw[axis] - baseline[axis] for axis in range(3))
            delta_moment_cog = tuple(
                raw[axis + 3] - baseline[axis + 3] for axis in range(3))
            cog_pose = cog_odometry[name].pose.pose
            force_bee = tuple(
                self.observer_force_sign * self.observer_force_scale * value
                for value in self.rotate_cog_to_world(
                    delta_force_cog, cog_pose.orientation))
            moment_bee_cog = tuple(
                self.observer_torque_sign * self.observer_torque_scale * value
                for value in self.rotate_cog_to_world(
                    delta_moment_cog, cog_pose.orientation))
            contact_center, _ = self.contact_plane_world_state(odometry[name])
            cog_center = (
                cog_pose.position.x,
                cog_pose.position.y,
                cog_pose.position.z,
            )
            cog_to_contact = tuple(
                contact_center[axis] - cog_center[axis] for axis in range(3))
            transport_moment = self.cross(cog_to_contact, force_bee)
            local_moment_bee = tuple(
                moment_bee_cog[axis] - transport_moment[axis]
                for axis in range(3))

            # Newton's third law at the same contact representative point.
            force_object = tuple(-value for value in force_bee)
            local_moment_object = tuple(-value for value in local_moment_bee)
            object_to_contact = tuple(
                contact_center[axis] - object_center[axis]
                for axis in range(3))
            object_transport_moment = self.cross(
                object_to_contact, force_object)
            for axis in range(3):
                object_force[axis] += force_object[axis]
                object_force_arm_moment[axis] += \
                    object_transport_moment[axis]
                object_local_moment_sum[axis] += \
                    local_moment_object[axis]

            normal, tangent, _ = self.face_frame(name, object_pose)
            local_states[name] = {
                "contact_center": contact_center,
                "force_bee": force_bee,
                "moment_bee_cog": moment_bee_cog,
                "local_moment_bee": local_moment_bee,
                "local_moment_contact": (
                    local_moment_bee[0] * normal[0] +
                    local_moment_bee[1] * normal[1],
                    local_moment_bee[0] * tangent[0] +
                    local_moment_bee[1] * tangent[1],
                    local_moment_bee[2],
                ),
            }
        object_moment = tuple(
            object_force_arm_moment[axis] + object_local_moment_sum[axis]
            for axis in range(3))
        return (local_states, tuple(object_force),
                tuple(object_force_arm_moment),
                tuple(object_local_moment_sum), object_moment)

    @staticmethod
    def make_world_wrench(stamp, force, moment):
        message = WrenchStamped()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.wrench.force.x = force[0]
        message.wrench.force.y = force[1]
        message.wrench.force.z = force[2]
        message.wrench.torque.x = moment[0]
        message.wrench.torque.y = moment[1]
        message.wrench.torque.z = moment[2]
        return message

    def publish_observer_wrench_metrics(
            self, local_states, object_force, object_force_arm_moment,
            object_local_moment_sum, object_moment):
        stamp = rospy.Time.now()
        for name in self.names:
            state = local_states[name]
            self.local_contact_wrench_pubs[name].publish(
                self.make_world_wrench(
                    stamp, state["force_bee"], state["local_moment_bee"]))
        self.object_resultant_wrench_pub.publish(
            self.make_world_wrench(stamp, object_force, object_moment))
        for publisher, moment in (
                (self.object_force_arm_moment_pub, object_force_arm_moment),
                (self.object_local_moment_sum_pub, object_local_moment_sum)):
            message = Vector3Stamped()
            message.header.stamp = stamp
            message.header.frame_id = "world"
            message.vector.x = moment[0]
            message.vector.y = moment[1]
            message.vector.z = moment[2]
            publisher.publish(message)

    def open_wrench_log(self):
        directory = os.path.dirname(self.wrench_log_path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        log_file = open(self.wrench_log_path, "w")
        fields = [
            "sim_time", "target_normal_force", "active_normal_force",
            "target_z_velocity",
            "translation_target_velocity_x",
            "translation_target_velocity_y",
            "yaw_velocity_command",
            "translation_target_z",
            "translation_z_limit_active",
            "translation_acceleration_x",
            "translation_acceleration_y",
            "control_mode", "lift_fault",
            "lift_hold_reached", "lift_vertical_force_target",
            "filtered_object_vertical_force",
            "lift_observer_force_correction",
            "lift_force_ramp_ratio",
            "lift_force_feedforward_scale",
            "lift_breakaway_force_active",
            "lift_hold_recovery_force",
            "lift_common_slip_feedback_active",
            "lift_common_slip_error",
            "lift_common_slip_velocity",
            "lift_common_slip_acceleration",
            "lift_yaw_target", "lift_yaw_error", "object_yaw_rate",
            "lift_yaw_moment_feedback",
            "lift_yaw_tangential_acceleration",
            "lift_reference_z", "lift_reference_velocity",
            "lift_reference_acceleration",
            "lift_object_acceleration_command",
            "lift_common_z_acceleration_command",
            "object_z_velocity",
            "object_x", "object_y", "object_z", "object_yaw",
            "object_force_world_x", "object_force_world_y",
            "object_force_world_z", "object_moment_world_x",
            "object_moment_world_y", "object_moment_world_z",
            "object_force_arm_moment_world_x",
            "object_force_arm_moment_world_y",
            "object_force_arm_moment_world_z",
            "object_local_moment_sum_world_x",
            "object_local_moment_sum_world_y",
            "object_local_moment_sum_world_z",
            "filtered_object_moment_world_x",
            "filtered_object_moment_world_y",
            "filtered_object_moment_world_z",
            "object_moment_yaw_trim",
        ]
        for name in self.names:
            fields.extend([
                "{}_target_normal_force".format(name),
                "{}_object_moment_force_offset".format(name),
                "{}_lift_z_acceleration_offset".format(name),
                "{}_lift_contact_z_acceleration_offset".format(name),
                "{}_commanded_z_acceleration".format(name),
                "{}_force_site_z_sum".format(name),
                "{}_force_site_upward_sum".format(name),
                "{}_contact_x".format(name),
                "{}_contact_y".format(name),
                "{}_contact_z".format(name),
                "{}_force_bee_world_x".format(name),
                "{}_force_bee_world_y".format(name),
                "{}_force_bee_world_z".format(name),
                "{}_moment_bee_cog_world_x".format(name),
                "{}_moment_bee_cog_world_y".format(name),
                "{}_moment_bee_cog_world_z".format(name),
                "{}_local_moment_bee_world_x".format(name),
                "{}_local_moment_bee_world_y".format(name),
                "{}_local_moment_bee_world_z".format(name),
                "{}_local_moment_bee_contact_normal".format(name),
                "{}_local_moment_bee_contact_tangent".format(name),
                "{}_local_moment_bee_contact_vertical".format(name),
                "{}_filtered_local_moment_contact_normal".format(name),
                "{}_filtered_local_moment_contact_tangent".format(name),
                "{}_filtered_local_moment_contact_vertical".format(name),
                "{}_local_moment_roll_trim".format(name),
                "{}_local_moment_yaw_trim".format(name),
            ])
        writer = csv.DictWriter(log_file, fieldnames=fields)
        writer.writeheader()
        return log_file, writer

    def write_wrench_log(self, writer, object_pose, force_sites, local_states,
                         object_force, object_force_arm_moment,
                         object_local_moment_sum, object_moment):
        row = {
            "sim_time": "{:.6f}".format(rospy.get_time()),
            "target_normal_force": "{:.6f}".format(
                self.common_normal_force()),
            "active_normal_force": "{:.6f}".format(
                self.applied_common_normal_force()),
            "target_z_velocity": "{:.6f}".format(self.target_z_velocity),
            "translation_target_velocity_x": "{:.6f}".format(
                self.translation_velocity_command[0]),
            "translation_target_velocity_y": "{:.6f}".format(
                self.translation_velocity_command[1]),
            "yaw_velocity_command": "{:.6f}".format(
                self.yaw_velocity_command),
            "translation_target_z": "{:.8f}".format(
                self.lift_hold_target_object_z
                if self.lift_hold_target_object_z is not None else
                object_pose.position.z),
            "translation_z_limit_active": str(
                self.translation_z_limit_active),
            "translation_acceleration_x": "{:.6f}".format(
                self.translation_acceleration_command[0]),
            "translation_acceleration_y": "{:.6f}".format(
                self.translation_acceleration_command[1]),
            "control_mode": (
                "LIFT_HOLD" if self.lift_hold_reached else
                "LIFT" if self.lift_mode_active else "PRELOAD"),
            "lift_fault": "1" if self.lift_fault else "0",
            "lift_hold_reached": "1" if self.lift_hold_reached else "0",
            "lift_vertical_force_target": "{:.6f}".format(
                self.lift_vertical_force_target),
            "filtered_object_vertical_force": "{:.6f}".format(
                self.filtered_object_vertical_force),
            "lift_observer_force_correction": "{:.6f}".format(
                self.lift_observer_force_correction),
            "lift_force_ramp_ratio": "{:.6f}".format(
                self.lift_force_ramp_ratio),
            "lift_force_feedforward_scale": "{:.6f}".format(
                self.lift_hold_force_feedforward_scale
                if self.lift_hold_reached else 1.0),
            "lift_breakaway_force_active": (
                "1" if self.lift_breakaway_force_active else "0"),
            "lift_hold_recovery_force": "{:.6f}".format(
                self.lift_hold_recovery_force),
            "lift_common_slip_feedback_active": (
                "1" if self.lift_common_slip_feedback_active else "0"),
            "lift_common_slip_error": "{:.6f}".format(
                self.lift_common_slip_error),
            "lift_common_slip_velocity": "{:.6f}".format(
                self.lift_common_slip_velocity),
            "lift_common_slip_acceleration": "{:.6f}".format(
                self.lift_common_slip_acceleration),
            "lift_yaw_target": "{:.8f}".format(
                self.lift_yaw_target
                if self.lift_yaw_target is not None else 0.0),
            "lift_yaw_error": "{:.8f}".format(self.lift_yaw_error),
            "object_yaw_rate": "{:.8f}".format(self.lift_yaw_rate),
            "lift_yaw_moment_feedback": "{:.8f}".format(
                self.lift_yaw_moment_feedback),
            "lift_yaw_tangential_acceleration": "{:.8f}".format(
                self.lift_yaw_tangential_acceleration),
            "lift_reference_z": "{:.6f}".format(
                self.lift_reference_z if self.lift_reference_z is not None
                else object_pose.position.z),
            "lift_reference_velocity": "{:.6f}".format(
                self.lift_reference_velocity),
            "lift_reference_acceleration": "{:.6f}".format(
                self.lift_reference_acceleration),
            "lift_object_acceleration_command": "{:.6f}".format(
                self.lift_object_acceleration_command),
            "lift_common_z_acceleration_command": "{:.6f}".format(
                self.lift_common_z_acceleration_command),
            "object_z_velocity": "{:.6f}".format(self.object_twist[2]),
            "object_x": "{:.8f}".format(object_pose.position.x),
            "object_y": "{:.8f}".format(object_pose.position.y),
            "object_z": "{:.8f}".format(object_pose.position.z),
            "object_yaw": "{:.8f}".format(
                self.quaternion_to_rpy(object_pose.orientation)[2]),
            "object_force_world_x": "{:.8f}".format(object_force[0]),
            "object_force_world_y": "{:.8f}".format(object_force[1]),
            "object_force_world_z": "{:.8f}".format(object_force[2]),
            "object_moment_world_x": "{:.8f}".format(object_moment[0]),
            "object_moment_world_y": "{:.8f}".format(object_moment[1]),
            "object_moment_world_z": "{:.8f}".format(object_moment[2]),
            "object_force_arm_moment_world_x": "{:.8f}".format(
                object_force_arm_moment[0]),
            "object_force_arm_moment_world_y": "{:.8f}".format(
                object_force_arm_moment[1]),
            "object_force_arm_moment_world_z": "{:.8f}".format(
                object_force_arm_moment[2]),
            "object_local_moment_sum_world_x": "{:.8f}".format(
                object_local_moment_sum[0]),
            "object_local_moment_sum_world_y": "{:.8f}".format(
                object_local_moment_sum[1]),
            "object_local_moment_sum_world_z": "{:.8f}".format(
                object_local_moment_sum[2]),
            "filtered_object_moment_world_x": "{:.8f}".format(
                self.filtered_object_moment[0]),
            "filtered_object_moment_world_y": "{:.8f}".format(
                self.filtered_object_moment[1]),
            "filtered_object_moment_world_z": "{:.8f}".format(
                self.filtered_object_moment[2]),
            "object_moment_yaw_trim": "{:.8f}".format(
                self.object_moment_yaw_trim),
        }
        for name in self.names:
            state = local_states[name]
            row["{}_target_normal_force".format(name)] = \
                "{:.8f}".format(self.target_normal_forces[name])
            row["{}_object_moment_force_offset".format(name)] = \
                "{:.8f}".format(self.object_moment_force_offsets[name])
            row["{}_lift_z_acceleration_offset".format(name)] = \
                "{:.8f}".format(self.lift_z_acceleration_offsets[name])
            row["{}_lift_contact_z_acceleration_offset".format(name)] = \
                "{:.8f}".format(
                    self.lift_contact_z_acceleration_offsets[name])
            row["{}_commanded_z_acceleration".format(name)] = \
                "{:.8f}".format(self.contact_z_acceleration(name))
            z_forces = [force.z for force in force_sites.get(name, [])]
            row["{}_force_site_z_sum".format(name)] = \
                "{:.8f}".format(sum(z_forces))
            row["{}_force_site_upward_sum".format(name)] = \
                "{:.8f}".format(sum(max(0.0, value) for value in z_forces))
            for prefix, vector in (
                    ("contact", state["contact_center"]),
                    ("force_bee_world", state["force_bee"]),
                    ("moment_bee_cog_world", state["moment_bee_cog"]),
                    ("local_moment_bee_world", state["local_moment_bee"])):
                for axis, value in zip(("x", "y", "z"), vector):
                    row["{}_{}_{}".format(name, prefix, axis)] = \
                        "{:.8f}".format(value)
            for axis, value in zip(
                    ("normal", "tangent", "vertical"),
                    state["local_moment_contact"]):
                row["{}_local_moment_bee_contact_{}".format(name, axis)] = \
                    "{:.8f}".format(value)
            for axis, value in zip(
                    ("normal", "tangent", "vertical"),
                    self.filtered_local_moment_contact[name]):
                row["{}_filtered_local_moment_contact_{}".format(
                    name, axis)] = "{:.8f}".format(value)
            row["{}_local_moment_roll_trim".format(name)] = \
                "{:.8f}".format(self.local_moment_roll_trim[name])
            row["{}_local_moment_yaw_trim".format(name)] = \
                "{:.8f}".format(self.local_moment_yaw_trim[name])
        writer.writerow(row)

    def reset_object_twist(self, object_pose):
        yaw = self.quaternion_to_rpy(object_pose.orientation)[2]
        self.previous_object_state = (
            rospy.get_time(), object_pose.position.x,
            object_pose.position.y, object_pose.position.z, yaw)
        self.object_twist = [0.0, 0.0, 0.0, 0.0]

    def update_object_twist(self, object_pose):
        """Estimate world vx, vy, vz and yaw rate from the object pose topic."""
        now = rospy.get_time()
        yaw = self.quaternion_to_rpy(object_pose.orientation)[2]
        current = (now, object_pose.position.x, object_pose.position.y,
                   object_pose.position.z, yaw)
        if self.previous_object_state is None:
            self.previous_object_state = current
            return tuple(self.object_twist)
        dt = now - self.previous_object_state[0]
        if dt <= 1.0e-6 or dt > 0.5:
            self.previous_object_state = current
            return tuple(self.object_twist)
        raw = (
            (current[1] - self.previous_object_state[1]) / dt,
            (current[2] - self.previous_object_state[2]) / dt,
            (current[3] - self.previous_object_state[3]) / dt,
            self.angle_error(current[4], self.previous_object_state[4]) / dt,
        )
        alpha = self.clamp_value(self.object_twist_filter_alpha, 0.0, 1.0)
        for axis in range(4):
            self.object_twist[axis] += alpha * (
                raw[axis] - self.object_twist[axis])
        self.previous_object_state = current
        return tuple(self.object_twist)

    def contact_plane_world_state(self, odom):
        """Return four-ball centre position and velocity in world axes."""
        quaternion = odom.pose.pose.orientation
        offset = self.rotate_cog_to_world(
            self.contact_plane_center_in_fc, quaternion)
        fc_position = odom.pose.pose.position
        fc_velocity = odom.twist.twist.linear
        angular_body = odom.twist.twist.angular
        angular_world = self.rotate_cog_to_world(
            (angular_body.x, angular_body.y, angular_body.z), quaternion)
        rotational_velocity = (
            angular_world[1] * offset[2] - angular_world[2] * offset[1],
            angular_world[2] * offset[0] - angular_world[0] * offset[2],
            angular_world[0] * offset[1] - angular_world[1] * offset[0],
        )
        position = (
            fc_position.x + offset[0],
            fc_position.y + offset[1],
            fc_position.z + offset[2],
        )
        velocity = (
            fc_velocity.x + rotational_velocity[0],
            fc_velocity.y + rotational_velocity[1],
            fc_velocity.z + rotational_velocity[2],
        )
        return position, velocity

    def contact_position_state(self, name, odom, object_pose, object_twist):
        """Measure the Bee contact-plane centre relative to the object TF."""
        normal, tangent, yaw = self.face_frame(name, object_pose)
        position, velocity = self.contact_plane_world_state(odom)
        dx = position[0] - object_pose.position.x
        dy = position[1] - object_pose.position.y
        radius = dx * normal[0] + dy * normal[1]
        tangent_position = dx * tangent[0] + dy * tangent[1]
        relative_vx = velocity[0] - object_twist[0]
        relative_vy = velocity[1] - object_twist[1]
        # The object-frame basis rotates with the prism.  Include those basis
        # derivatives so a rigidly translating/rotating formation has zero
        # relative velocity instead of being damped against the world.
        radial_velocity = (
            relative_vx * normal[0] + relative_vy * normal[1] +
            object_twist[3] * tangent_position)
        tangent_velocity = (
            relative_vx * tangent[0] + relative_vy * tangent[1] -
            object_twist[3] * radius)
        relative_z = position[2] - object_pose.position.z
        relative_z_velocity = velocity[2] - object_twist[2]
        return (normal, tangent, yaw, radius, tangent_position,
                radial_velocity, tangent_velocity,
                relative_z, relative_z_velocity)

    def contact_z_velocity(self, name, odom, object_pose, object_twist):
        if self.lift_fault:
            return 0.0
        if self.lift_mode_active:
            return 0.0

        state = self.contact_position_state(
            name, odom, object_pose, object_twist)
        relative_z = state[7]
        relative_z_velocity = state[8]
        correction = (
            -self.relative_z_kp * (relative_z - self.relative_contact_z) -
            self.relative_z_kd * relative_z_velocity)
        correction = self.clamp_value(
            correction, -self.z_correction_limit, self.z_correction_limit)
        command = self.target_z_velocity + correction
        return self.clamp_value(
            command, -self.maximum_abs_z_velocity,
            self.maximum_abs_z_velocity)

    def contact_z_acceleration(self, name):
        """Return common plus zero-sum differential world-Z acceleration."""
        if self.lift_fault or not self.lift_preload_ready():
            if not self.lift_fault:
                rospy.loginfo_throttle(
                    1.0, "triple press: LIFT waits for equal ramped preload")
            return 0.0
        command = (
            self.lift_common_z_acceleration_command +
            self.lift_contact_z_acceleration_offsets[name] +
            self.lift_z_acceleration_offsets[name])
        limit = (
            self.lift_common_z_acceleration_limit +
            self.lift_contact_z_acceleration_limit +
            self.lift_moment_acceleration_limit)
        return self.clamp_value(command, -limit, limit)

    def publish_contact_error(self, name, radius, desired_radius,
                              tangent_error, z_error):
        message = Vector3Stamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "grasp_prism"
        message.vector.x = radius - desired_radius
        message.vector.y = tangent_error
        message.vector.z = z_error
        self.contact_error_pubs[name].publish(message)

    def publish_interactive_state(self, observer_forces):
        self.target_force_pub.publish(Float32(data=self.common_normal_force()))
        self.active_force_pub.publish(
            Float32(data=self.applied_common_normal_force()))
        self.target_z_velocity_pub.publish(Float32(data=self.target_z_velocity))
        message = Vector3Stamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "bee1_bee2_bee3"
        message.vector.x = observer_forces.get("bee1", 0.0)
        message.vector.y = observer_forces.get("bee2", 0.0)
        message.vector.z = observer_forces.get("bee3", 0.0)
        self.observer_forces_pub.publish(message)
        target_message = Vector3Stamped()
        target_message.header.stamp = message.header.stamp
        target_message.header.frame_id = "bee1_bee2_bee3"
        target_message.vector.x = self.target_normal_forces["bee1"]
        target_message.vector.y = self.target_normal_forces["bee2"]
        target_message.vector.z = self.target_normal_forces["bee3"]
        self.allocated_force_targets_pub.publish(target_message)
        lift_message = Vector3Stamped()
        lift_message.header.stamp = message.header.stamp
        lift_message.header.frame_id = "bee1_bee2_bee3"
        lift_message.vector.x = self.lift_z_acceleration_offsets["bee1"]
        lift_message.vector.y = self.lift_z_acceleration_offsets["bee2"]
        lift_message.vector.z = self.lift_z_acceleration_offsets["bee3"]
        self.lift_z_acceleration_offsets_pub.publish(lift_message)
        contact_message = Vector3Stamped()
        contact_message.header = lift_message.header
        contact_message.vector.x = \
            self.lift_contact_z_acceleration_offsets["bee1"]
        contact_message.vector.y = \
            self.lift_contact_z_acceleration_offsets["bee2"]
        contact_message.vector.z = \
            self.lift_contact_z_acceleration_offsets["bee3"]
        self.lift_contact_z_acceleration_offsets_pub.publish(contact_message)
        self.lift_vertical_force_target_pub.publish(
            Float32(data=self.lift_vertical_force_target))
        self.lift_vertical_force_filtered_pub.publish(
            Float32(data=self.filtered_object_vertical_force))
        self.lift_common_z_acceleration_pub.publish(
            Float32(data=self.lift_common_z_acceleration_command))
        self.lift_reference_acceleration_pub.publish(
            Float32(data=self.lift_reference_acceleration))
        self.lift_observer_force_correction_pub.publish(
            Float32(data=self.lift_observer_force_correction))
        self.lift_force_ramp_ratio_pub.publish(
            Float32(data=self.lift_force_ramp_ratio))
        self.lift_common_slip_acceleration_pub.publish(
            Float32(data=self.lift_common_slip_acceleration))
        yaw_state = Vector3Stamped()
        yaw_state.header.stamp = message.header.stamp
        yaw_state.header.frame_id = "world"
        yaw_state.vector.x = (
            self.lift_yaw_target
            if self.lift_yaw_target is not None else 0.0)
        yaw_state.vector.y = self.lift_yaw_error
        yaw_state.vector.z = self.lift_yaw_rate
        self.lift_yaw_state_pub.publish(yaw_state)
        self.lift_yaw_tangential_acceleration_pub.publish(
            Float32(data=self.lift_yaw_tangential_acceleration))
        self.object_yaw_velocity_command_pub.publish(
            Float32(data=self.yaw_velocity_command))
        translation = Vector3Stamped()
        translation.header.stamp = message.header.stamp
        translation.header.frame_id = "world"
        translation.vector.x = self.translation_velocity_command[0]
        translation.vector.y = self.translation_velocity_command[1]
        translation.vector.z = (
            self.target_z_velocity if self.lift_hold_reached else 0.0)
        self.translation_command_pub.publish(translation)
        translation_acceleration = Vector3Stamped()
        translation_acceleration.header = translation.header
        translation_acceleration.vector.x = \
            self.translation_acceleration_command[0]
        translation_acceleration.vector.y = \
            self.translation_acceleration_command[1]
        translation_acceleration.vector.z = \
            self.lift_object_acceleration_command
        self.translation_acceleration_pub.publish(translation_acceleration)

    def log_command(self):
        rospy.loginfo(
            "triple press command: requested/active force %.2f/%.2f N, "
            "object velocity XYZ [%+.3f, %+.3f, %+.3f] m/s, yaw rate "
            "%+.3f rad/s, target Z/yaw [%.3f, %+.3f]%s, mode %s%s",
            self.common_normal_force(), self.applied_common_normal_force(),
            self.translation_velocity_command[0],
            self.translation_velocity_command[1],
            self.target_z_velocity,
            self.yaw_velocity_command,
            self.lift_hold_target_object_z
            if self.lift_hold_target_object_z is not None else 0.0,
            self.lift_yaw_target
            if self.lift_yaw_target is not None else 0.0,
            " Z-LIMIT" if self.translation_z_limit_active else "",
            "LIFT" if self.lift_mode_active else "PRELOAD",
            " FAULT" if self.lift_fault else "")

    def adjust_translation_velocity(self, axis, increment):
        if not self.lift_hold_reached or self.lift_fault:
            rospy.logwarn_throttle(
                1.0, "triple press: object translation is enabled after HOLD")
            return False
        self.translation_velocity_command[axis] += increment
        speed = math.hypot(
            self.translation_velocity_command[0],
            self.translation_velocity_command[1])
        if speed > self.translation_velocity_limit:
            scale = self.translation_velocity_limit / speed
            self.translation_velocity_command[0] *= scale
            self.translation_velocity_command[1] *= scale
        return True

    def adjust_yaw_velocity(self, increment):
        if not self.lift_hold_reached or self.lift_fault:
            rospy.logwarn_throttle(
                1.0, "triple press: object yaw motion is enabled after HOLD")
            return False
        self.yaw_velocity_command = self.clamp_value(
            self.yaw_velocity_command + increment,
            -self.yaw_velocity_limit, self.yaw_velocity_limit)
        return True

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
        elif key in ("i", "k", "j", "l"):
            axis = 0 if key in ("i", "k") else 1
            direction = 1.0 if key in ("i", "j") else -1.0
            if not self.adjust_translation_velocity(
                    axis, direction * self.translation_velocity_step):
                return
        elif key == "m":
            self.translation_velocity_command = [0.0, 0.0]
        elif key in ("q", "e"):
            direction = 1.0 if key == "q" else -1.0
            if not self.adjust_yaw_velocity(
                    direction * self.yaw_velocity_step):
                return
        elif key == "[":
            if self.lift_hold_reached:
                self.target_z_velocity = self.clamp_value(
                    self.target_z_velocity +
                    self.translation_z_velocity_step,
                    -self.translation_z_velocity_limit,
                    self.translation_z_velocity_limit)
            else:
                self.target_z_velocity = self.clamp_value(
                    self.target_z_velocity + self.z_velocity_step,
                    -self.maximum_abs_z_velocity,
                    self.maximum_abs_z_velocity)
                if self.target_z_velocity > self.lift_velocity_threshold:
                    self.enter_lift_mode()
        elif key == "]":
            step = (self.translation_z_velocity_step
                    if self.lift_hold_reached else self.z_velocity_step)
            limit = (self.translation_z_velocity_limit
                     if self.lift_hold_reached else
                     self.maximum_abs_z_velocity)
            self.target_z_velocity = self.clamp_value(
                self.target_z_velocity - step, -limit, limit)
        elif key == " ":
            self.translation_velocity_command = [0.0, 0.0]
            self.target_z_velocity = 0.0
            self.yaw_velocity_command = 0.0
        elif key == "r":
            self.set_common_normal_force(self.initial_normal_force)
            self.translation_velocity_command = [0.0, 0.0]
            self.translation_acceleration_command = [0.0, 0.0]
            self.target_z_velocity = 0.0
            self.yaw_velocity_command = 0.0
            self.reset_lift_mode()
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
                           desired_radius, object_twist=None):
        """Track force with physical F/m feed-forward plus observer feedback."""
        if object_twist is None:
            object_twist = (0.0, 0.0, 0.0, 0.0)
        (normal, tangent, yaw, radius, tangent_position,
         radial_velocity, tangent_velocity, _, _) = \
            self.contact_position_state(
                name, odom, object_pose, object_twist)
        fc_position = odom.pose.pose.position
        fc_dx = fc_position.x - object_pose.position.x
        fc_dy = fc_position.y - object_pose.position.y
        vehicle_radius = fc_dx * normal[0] + fc_dy * normal[1]

        target_force = self.target_normal_forces[name]
        force_error = target_force - measured_force
        force_feedforward = (self.normal_force_feedforward_scale *
                             target_force / max(1.0e-6, self.robot_mass))
        radial = (-force_feedforward -
                  self.normal_force_kp * force_error -
                  self.radial_kd * radial_velocity)
        if vehicle_radius < desired_radius - self.press_radius_safety_margin:
            radial = (self.press_radial_kp *
                      (desired_radius - vehicle_radius) -
                      self.radial_kd * radial_velocity)
        tangential = (-self.relative_tangent_kp * tangent_position -
                      self.relative_tangent_kd * tangent_velocity)
        if self.lift_mode_active:
            # Equal scalar commands along the three face tangents create a
            # yaw couple without adding a common XY translation command.
            tangential += self.lift_yaw_tangential_acceleration
        radial = self.clamp(radial, self.press_accel_limit)
        tangential = self.clamp(tangential, self.approach_accel_limit)
        return (radial * normal[0] + tangential * tangent[0],
                radial * normal[1] + tangential * tangent[1],
                self.moment_corrected_yaw(name, yaw))

    def approach_acceleration(self, name, odom, object_pose, desired_radius):
        """Approach radially while centring the physical four-ball plane."""
        object_twist = (0.0, 0.0, 0.0, 0.0)
        (normal, tangent, yaw, _, tangent_position,
         _, tangent_velocity, _, _) = self.contact_position_state(
             name, odom, object_pose, object_twist)
        fc_position = odom.pose.pose.position
        fc_velocity = odom.twist.twist.linear
        dx = fc_position.x - object_pose.position.x
        dy = fc_position.y - object_pose.position.y
        radius = dx * normal[0] + dy * normal[1]
        radial_velocity = (
            fc_velocity.x * normal[0] + fc_velocity.y * normal[1])
        radial = (self.radial_kp * (desired_radius - radius) -
                  self.radial_kd * radial_velocity)
        tangential = (-self.relative_tangent_kp * tangent_position -
                      self.relative_tangent_kd * tangent_velocity)
        radial = self.clamp(radial, self.approach_accel_limit)
        tangential = self.clamp(tangential, self.approach_accel_limit)
        return (radial * normal[0] + tangential * tangent[0],
                radial * normal[1] + tangential * tangent[1],
                self.moment_corrected_yaw(name, yaw), radius,
                tangent_position, radial_velocity, tangent_velocity)

    def stage_and_approach(self):
        inradius = self.triangle_side / (2.0 * (3.0 ** 0.5))
        staging_radius = inradius + self.staging_clearance
        contact_radius = (inradius + self.foot_contact_offset +
                          self.foot_ball_radius - self.foot_max_compression)
        self.contact_surface_radius = inradius + self.foot_ball_radius

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
        _, initial_odometry, _, initial_object_pose = self.dual_snapshot()
        self.relative_contact_z = sum(
            self.contact_position_state(
                name, initial_odometry[name], initial_object_pose,
                (0.0, 0.0, 0.0, 0.0))[7]
            for name in self.names) / len(self.names)
        self.reset_object_twist(initial_object_pose)
        self.reset_local_moment_control()
        if self.target_z_velocity > self.lift_velocity_threshold:
            self.enter_lift_mode()
        print(HELP)
        self.log_command()
        rospy.loginfo(
            "triple press keyboard steps: force %.2f N (range %.2f..%.2f), "
            "lift relative-Z limit step %.3f m/s (absolute limit %0.3f)",
            self.force_step, self.minimum_normal_force,
            self.maximum_normal_force, self.z_velocity_step,
            self.maximum_abs_z_velocity)
        rospy.loginfo(
            "triple press: observer force and object-relative contact-position "
            "feedback active; target relative z %.3f m",
            self.relative_contact_z)
        log_file, log_writer = self.open_wrench_log()
        last_log_flush = time.monotonic()
        rospy.loginfo(
            "triple press: observer local/object-moment control %s/%s; "
            "CSV %s",
            "enabled" if self.local_moment_control_enabled else "disabled",
            "enabled" if self.object_moment_control_enabled else "disabled",
            self.wrench_log_path)
        last_moment_control_time = rospy.get_time()
        try:
            with TerminalKeyboard() as keyboard:
                while not rospy.is_shutdown() and not self.stop_requested:
                    for key in keyboard.read_all():
                        self.handle_key(key)

                    _, odometry, force_sites, object_pose = self.dual_snapshot()
                    object_twist = self.update_object_twist(object_pose)
                    (local_wrenches, object_force, object_force_arm_moment,
                     object_local_moment_sum, object_moment) = \
                        self.observer_contact_wrenches(odometry, object_pose)
                    alpha = max(0.0, min(1.0, self.force_filter_alpha))
                    for name in self.names:
                        measured = self.observer_normal_force(
                            name, observer_baseline)
                        filtered_forces[name] += alpha * (
                            measured - filtered_forces[name])

                    now = rospy.get_time()
                    control_dt = now - last_moment_control_time
                    self.update_common_normal_force(control_dt)
                    self.update_local_moment_control(
                        local_wrenches, filtered_forces, control_dt)
                    self.update_object_moment_control(
                        local_wrenches, object_pose, object_moment,
                        filtered_forces, control_dt)
                    self.update_lift_progress(
                        object_pose, odometry, object_twist)
                    self.update_lift_z_control(
                        odometry, object_pose, object_twist,
                        object_force[2], control_dt)
                    translation_acceleration = \
                        self.update_translation_control(
                            object_twist, control_dt)
                    self.update_lift_yaw_control(
                        object_pose, object_twist, control_dt)
                    last_moment_control_time = now
                    for name in self.names:
                        ax, ay, yaw = self.press_acceleration(
                            name, odometry[name], object_pose,
                            filtered_forces[name], contact_radius, object_twist)
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
                        state = self.contact_position_state(
                            name, odometry[name], object_pose, object_twist)
                        self.publish_contact_error(
                            name, state[3], self.contact_surface_radius, state[4],
                            state[7] - self.relative_contact_z)

                    self.publish_observer_wrench_metrics(
                        local_wrenches, object_force, object_force_arm_moment,
                        object_local_moment_sum, object_moment)
                    self.publish_local_moment_trims()
                    self.write_wrench_log(
                        log_writer, object_pose, force_sites, local_wrenches,
                        object_force, object_force_arm_moment,
                        object_local_moment_sum, object_moment)
                    if (time.monotonic() - last_log_flush >=
                            self.wrench_log_flush_period):
                        log_file.flush()
                        last_log_flush = time.monotonic()

                    self.publish_interactive_state(filtered_forces)
                    rospy.loginfo_throttle(
                        0.5,
                        "triple press: requested/active %.2f/%.2f N, observer "
                        "[%.3f, %.3f, %.3f] N, Fz %.3f/%.3f N, "
                        "ramp %.2f, a_ref %+.3f, slip %.4f m/"
                        "%+.3f m/s2, common az %+.3f m/s2, "
                        "mode %s%s, |M_object| %.4f Nm, yaw err/rate "
                        "%+.3f/%+.3f, yaw at %+.3f m/s2, object z %.3f m",
                        self.common_normal_force(),
                        self.applied_common_normal_force(),
                        filtered_forces["bee1"], filtered_forces["bee2"],
                        filtered_forces["bee3"],
                        self.filtered_object_vertical_force,
                        self.lift_vertical_force_target,
                        self.lift_force_ramp_ratio,
                        self.lift_reference_acceleration,
                        self.lift_common_slip_error,
                        self.lift_common_slip_acceleration,
                        self.lift_common_z_acceleration_command,
                        "LIFT" if self.lift_mode_active else "PRELOAD",
                        " FAULT" if self.lift_fault else "",
                        self.vector_norm3(object_moment),
                        self.lift_yaw_error,
                        self.lift_yaw_rate,
                        self.lift_yaw_tangential_acceleration,
                        object_pose.position.z)
                    time.sleep(self.control_period)
        finally:
            log_file.flush()
            log_file.close()
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
