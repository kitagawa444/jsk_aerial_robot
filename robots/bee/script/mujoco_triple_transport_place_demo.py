#!/usr/bin/env python

from __future__ import print_function

import math
import threading
import time

import numpy as np
import rospy
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from std_msgs.msg import String

from mujoco_triple_press_keyboard_demo import MujocoTriplePressKeyboardDemo


class MujocoTripleTransportPlaceDemo(MujocoTriplePressKeyboardDemo):
    """Autonomously grasp, transport, place, release, and retreat."""

    MISSION_TOPIC_ROOT = "/mujoco_triple_transport_place"

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

        # The legacy controller remains the default so existing experiments
        # are bit-for-bit unchanged.  whole_body_mpc replaces only the
        # continuous-contact LIFT/TRANSPORT/DESCEND outer loops; approach,
        # preload, release, retreat, and all safety handling stay shared.
        self.control_strategy = str(rospy.get_param(
            "~control_strategy", "legacy")).lower()
        if self.control_strategy == "science_robotics":
            self.control_strategy = "whole_body_mpc"
        if self.control_strategy not in ("legacy", "whole_body_mpc"):
            raise ValueError(
                "control_strategy must be legacy or whole_body_mpc")
        self.whole_body_planner_enabled = (
            self.control_strategy == "whole_body_mpc")
        self.whole_body_planner = None
        self.planner_result = None
        self.planner_solution_ros_time = None
        self.planner_last_attempt_ros_time = None
        self.planner_last_success_wall_time = None
        self.planner_first_attempt_wall_time = None
        self.planner_target_rpy = None
        self.planner_hold_xy = None
        self.planner_relative_contact_reference = {}
        self.planner_filtered_object_forces = dict(
            (name, np.zeros(3)) for name in self.names)
        self.planner_observer_initialized = False
        self.planner_solve_thread = None
        self.planner_result_lock = threading.Lock()
        self.planner_pending_result = None
        self.planner_disturbance_linear_acceleration = np.zeros(3)
        self.planner_disturbance_angular_acceleration = np.zeros(3)
        self.planner_measured_object_acceleration = np.zeros(3)
        self.planner_measured_object_angular_acceleration = np.zeros(3)
        self.planner_previous_object_velocity = None
        self.planner_previous_object_angular_velocity = None
        self.planner_last_model_acceleration = None
        self.planner_last_model_angular_acceleration = None

        self.planner_rate = rospy.get_param("~planner_rate", 5.0)
        self.planner_horizon = rospy.get_param("~planner_horizon", 0.6)
        self.planner_nodes = rospy.get_param("~planner_nodes", 6)
        self.planner_stale_timeout = rospy.get_param(
            "~planner_stale_timeout", 3.0)
        self.planner_bootstrap_timeout = rospy.get_param(
            "~planner_bootstrap_timeout", 3.0)
        self.planner_timestamp_at_acceptance = rospy.get_param(
            "~planner_timestamp_at_acceptance", True)
        self.planner_max_open_loop_time = rospy.get_param(
            "~planner_max_open_loop_time", -1.0)
        self.planner_solver_backend = rospy.get_param(
            "~planner_solver_backend", "acados")
        self.planner_casadi_nlp_solver = rospy.get_param(
            "~planner_casadi_nlp_solver", "ipopt")
        self.planner_casadi_qp_solver = rospy.get_param(
            "~planner_casadi_qp_solver", "qrqp")
        self.planner_solver_max_iterations = rospy.get_param(
            "~planner_solver_max_iterations", 100)
        self.planner_solver_tolerance = rospy.get_param(
            "~planner_solver_tolerance", 5.0e-4)
        self.planner_friction_coefficient = rospy.get_param(
            "~planner_friction_coefficient",
            rospy.get_param(
                "/bee_mujoco_multi_bringup/object_sliding_friction", 0.5))
        self.planner_friction_margin = rospy.get_param(
            "~planner_friction_margin", 0.90)
        self.planner_normal_force_min = rospy.get_param(
            "~planner_normal_force_min", 1.0)
        self.planner_normal_force_max = rospy.get_param(
            "~planner_normal_force_max", self.maximum_normal_force)
        self.planner_force_rate_limit = rospy.get_param(
            "~planner_force_rate_limit", 5.0)
        self.planner_normal_force_rate_limit = rospy.get_param(
            "~planner_normal_force_rate_limit", 2.0)
        self.planner_object_acceleration_limit = rospy.get_param(
            "~planner_object_acceleration_limit", 0.02)
        self.planner_angular_velocity_limit = rospy.get_param(
            "~planner_object_angular_velocity_limit", 0.20)
        self.planner_horizontal_velocity_constraint_limit = rospy.get_param(
            "~planner_horizontal_velocity_constraint_limit", 0.08)
        self.planner_vertical_velocity_constraint_limit = rospy.get_param(
            "~planner_vertical_velocity_constraint_limit", 0.08)
        self.planner_maximum_specific_thrust = rospy.get_param(
            "~planner_maximum_specific_thrust", 15.0)
        self.planner_maximum_vehicle_tilt = rospy.get_param(
            "~planner_maximum_vehicle_tilt", 0.65)
        self.planner_observer_time_constant = rospy.get_param(
            "~planner_observer_time_constant", 0.50)
        self.planner_external_force_compensation_gain = rospy.get_param(
            "~planner_external_force_compensation_gain", 1.0)
        self.planner_force_admittance_gain = rospy.get_param(
            "~planner_force_admittance_gain", 0.08)
        self.planner_normal_force_kp = rospy.get_param(
            "~planner_normal_force_kp", self.normal_force_kp)
        self.planner_radial_velocity_kd = rospy.get_param(
            "~planner_radial_velocity_kd", self.radial_kd)
        self.planner_normal_acceleration_limit = rospy.get_param(
            "~planner_normal_acceleration_limit", 2.0)
        self.planner_observer_acceleration_limit = rospy.get_param(
            "~planner_observer_acceleration_limit", 3.0)
        self.planner_relative_position_kp = rospy.get_param(
            "~planner_relative_position_kp", 1.5)
        self.planner_relative_velocity_kd = rospy.get_param(
            "~planner_relative_velocity_kd", 2.5)
        self.planner_relative_acceleration_limit = rospy.get_param(
            "~planner_relative_acceleration_limit", 0.10)
        self.planner_disturbance_time_constant = rospy.get_param(
            "~planner_disturbance_time_constant", 0.75)
        self.planner_disturbance_gain = rospy.get_param(
            "~planner_disturbance_gain", 1.0)
        self.planner_disturbance_linear_limit = rospy.get_param(
            "~planner_disturbance_linear_limit", 0.10)
        self.planner_disturbance_angular_limit = rospy.get_param(
            "~planner_disturbance_angular_limit", 0.50)
        self.planner_disturbance_linear_residual_limit = rospy.get_param(
            "~planner_disturbance_linear_residual_limit", 0.50)
        self.planner_disturbance_angular_residual_limit = rospy.get_param(
            "~planner_disturbance_angular_residual_limit", 3.0)

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
        # Keep the payload centred while it is lowered and unloaded.  A zero
        # XY velocity command only damps motion; it cannot reject the steady
        # lateral bias left by unequal compliant contacts.
        self.place_xy_hold_position_kp = rospy.get_param(
            "~place_xy_hold_position_kp", self.transport_position_kp)
        self.place_xy_hold_velocity_limit = rospy.get_param(
            "~place_xy_hold_velocity_limit", 0.01)
        self.place_xy_hold_deadband = rospy.get_param(
            "~place_xy_hold_deadband", 0.002)

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
        topic_root = self.MISSION_TOPIC_ROOT
        self.phase_pub = rospy.Publisher(
            topic_root + "/phase",
            String, queue_size=1, latch=True)
        self.planner_status_pub = rospy.Publisher(
            topic_root + "/planner/status",
            String, queue_size=1, latch=True)
        self.planner_metrics_pub = rospy.Publisher(
            topic_root + "/planner/metrics",
            Vector3Stamped, queue_size=1)
        self.planner_disturbance_linear_pub = rospy.Publisher(
            topic_root + "/planner/external_linear_acceleration",
            Vector3Stamped, queue_size=1)
        self.planner_disturbance_angular_pub = rospy.Publisher(
            topic_root + "/planner/external_angular_acceleration",
            Vector3Stamped, queue_size=1)
        self.planner_force_pubs = dict(
            (name, rospy.Publisher(
                topic_root + "/planner/{}/contact_force".format(name),
                WrenchStamped, queue_size=1))
            for name in self.names)
        self.planner_status_pub.publish(String(data=self.control_strategy))

    def run(self):
        if not rospy.get_param(
                "/bee_mujoco_multi_bringup/spawn_destination_pedestal",
                False):
            rospy.logerr(
                "transport/place: launch MuJoCo with "
                "spawn_destination_pedestal:=true")
            return False
        return super(MujocoTripleTransportPlaceDemo, self).run()

    @staticmethod
    def object_rotation_matrix(rpy):
        roll, pitch, yaw = rpy
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr,
             cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr,
             sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])

    def planner_object_inertia(self):
        override = rospy.get_param("~planner_object_inertia", [])
        if override:
            values = np.asarray(override, dtype=float)
            if values.size == 3:
                return values
            if values.size == 9:
                return values.reshape((3, 3))
            raise ValueError("planner_object_inertia must have 3 or 9 values")

        bringup_ns = "/bee_mujoco_multi_bringup/"
        outer_side = rospy.get_param(
            bringup_ns + "object_triangle_side", self.triangle_side)
        inner_side = rospy.get_param(
            bringup_ns + "object_inner_triangle_side", 0.0)
        height = rospy.get_param(
            bringup_ns + "object_height", self.object_height)
        # A concentric equilateral triangular tube has cross-section inertia
        # proportional to a_outer^2 + a_inner^2.  This preserves the physical
        # inertia/mass ratio when a launch-file mass override is used.
        cross_section_scale = outer_side ** 2 + inner_side ** 2
        i_horizontal = self.grasp_object_mass * (
            cross_section_scale / 24.0 + height ** 2 / 12.0)
        i_vertical = (
            self.grasp_object_mass * cross_section_scale / 12.0)
        return np.array([i_horizontal, i_horizontal, i_vertical])

    def planner_grasp_geometry(self, odometry, object_pose):
        """Estimate the current object-to-contact and object-to-CG arms.

        Geometry is expressed in the current object frame.  The contact arm
        uses the representative center of the four rubber balls, while the
        vehicle arm uses the wrench observer's CG odometry.
        """
        object_rpy = self.quaternion_to_rpy(object_pose.orientation)
        rotation_object_from_world = self.object_rotation_matrix(
            object_rpy).T
        object_position = np.array([
            object_pose.position.x,
            object_pose.position.y,
            object_pose.position.z,
        ])
        _, _, cog_odometry = self.observer_snapshot()
        contact_arms = []
        vehicle_arms = []
        for name in self.names:
            if name not in cog_odometry or cog_odometry[name] is None:
                raise RuntimeError(
                    "no current CG odometry for {}".format(name))
            contact_center, _ = self.contact_plane_world_state(
                odometry[name])
            contact_arms.append(np.dot(
                rotation_object_from_world,
                np.asarray(contact_center) - object_position))
            cog_position = cog_odometry[name].pose.pose.position
            vehicle_arms.append(np.dot(
                rotation_object_from_world,
                np.array([
                    cog_position.x, cog_position.y, cog_position.z]) -
                object_position))
        return np.asarray(contact_arms), np.asarray(vehicle_arms)

    def initialize_whole_body_planner(
            self, odometry, object_pose, object_twist):
        if not self.whole_body_planner_enabled:
            return
        from whole_body_kinodynamic_planner import \
            WholeBodyKinodynamicPlanner

        object_rpy = self.quaternion_to_rpy(object_pose.orientation)
        contact_arms, vehicle_arms = self.planner_grasp_geometry(
            odometry, object_pose)
        for name in self.names:
            contact_state = self.contact_position_state(
                name, odometry[name], object_pose, object_twist)
            self.planner_relative_contact_reference[name] = (
                contact_state[4], contact_state[7])

        self.whole_body_planner = WholeBodyKinodynamicPlanner(
            self.names,
            self.grasp_object_mass,
            self.planner_object_inertia(),
            self.robot_mass,
            horizon=self.planner_horizon,
            nodes=self.planner_nodes,
            gravity=self.gravity_acceleration,
            friction_coefficient=self.planner_friction_coefficient,
            friction_margin=self.planner_friction_margin,
            normal_force_min=self.planner_normal_force_min,
            normal_force_max=self.planner_normal_force_max,
            normal_force_target=self.common_normal_force(),
            force_rate_limit=self.planner_force_rate_limit,
            normal_force_rate_limit=self.planner_normal_force_rate_limit,
            object_acceleration_limit=
            self.planner_object_acceleration_limit,
            object_horizontal_velocity_limit=
            self.transport_velocity_limit,
            object_vertical_velocity_limit=max(
                self.lift_trajectory_velocity_limit,
                abs(self.place_descent_velocity),
                self.lift_velocity_threshold),
            object_horizontal_velocity_constraint_limit=
            self.planner_horizontal_velocity_constraint_limit,
            object_vertical_velocity_constraint_limit=
            self.planner_vertical_velocity_constraint_limit,
            object_angular_velocity_limit=
            self.planner_angular_velocity_limit,
            maximum_specific_thrust=
            self.planner_maximum_specific_thrust,
            maximum_vehicle_tilt=self.planner_maximum_vehicle_tilt,
            solver_backend=self.planner_solver_backend,
            casadi_nlp_solver=self.planner_casadi_nlp_solver,
            casadi_qp_solver=self.planner_casadi_qp_solver,
            solver_max_iterations=self.planner_solver_max_iterations,
            solver_tolerance=self.planner_solver_tolerance)
        initial_forces = np.zeros((3, 3))
        for index, name in enumerate(self.names):
            initial_forces[index, 0] = self.target_normal_forces[name]
        self.whole_body_planner.reset(
            contact_arms, vehicle_arms, initial_forces)
        if self.planner_max_open_loop_time < 0.0:
            # Receding-horizon MPC should normally apply only its first
            # control interval and then replan.  If optimization misses that
            # deadline (especially in faster-than-real-time headless MuJoCo),
            # hold the first interval instead of skipping to the terminal
            # zero-acceleration input.
            real_time_factor = rospy.get_param(
                "/bee_mujoco_multi_bringup/real_time_factor", 0.0)
            self.planner_max_open_loop_time = (
                0.0 if real_time_factor > 0.0 else
                self.whole_body_planner.dt)
        self.planner_target_rpy = tuple(object_rpy)
        self.planner_hold_xy = (
            object_pose.position.x, object_pose.position.y)
        self.lift_initial_relative_contact_z = sum(
            self.contact_position_state(
                name, odometry[name], object_pose, object_twist)[7]
            for name in self.names) / len(self.names)
        self.planner_result = None
        self.planner_solution_ros_time = None
        self.planner_last_attempt_ros_time = None
        self.planner_last_success_wall_time = None
        self.planner_first_attempt_wall_time = None
        self.planner_solve_thread = None
        with self.planner_result_lock:
            self.planner_pending_result = None
        self.planner_observer_initialized = False
        self.reset_planner_disturbance_estimator(object_twist)
        rospy.logwarn(
            "transport/place: whole-body MPC initialized: %.2f s/%d nodes, "
            "%s backend, inertia [%.4f %.4f %.4f] kg m^2, mu %.3f x %.2f",
            self.planner_horizon, self.planner_nodes,
            self.whole_body_planner.solver_backend,
            self.whole_body_planner.object_inertia[0, 0],
            self.whole_body_planner.object_inertia[1, 1],
            self.whole_body_planner.object_inertia[2, 2],
            self.planner_friction_coefficient,
            self.planner_friction_margin)

    def whole_body_planner_active(self):
        return (
            self.whole_body_planner_enabled and
            self.task_phase in (
                "LIFT", "TRANSPORT", "DESTINATION_SETTLE", "DESCEND"))

    def planner_state(self, object_pose, object_twist):
        position = (
            object_pose.position.x,
            object_pose.position.y,
            object_pose.position.z)
        velocity = tuple(object_twist[:3])
        rpy = self.quaternion_to_rpy(object_pose.orientation)
        return self.whole_body_planner.state(
            position, velocity, rpy, self.object_angular_velocity_world)

    def reset_planner_disturbance_estimator(self, object_twist):
        self.planner_disturbance_linear_acceleration = np.zeros(3)
        self.planner_disturbance_angular_acceleration = np.zeros(3)
        self.planner_measured_object_acceleration = np.zeros(3)
        self.planner_measured_object_angular_acceleration = np.zeros(3)
        self.planner_previous_object_velocity = np.asarray(
            object_twist[:3], dtype=float)
        self.planner_previous_object_angular_velocity = np.asarray(
            self.object_angular_velocity_world, dtype=float)
        self.planner_last_model_acceleration = None
        self.planner_last_model_angular_acceleration = None

    def update_planner_disturbance_estimator(self, object_twist, dt):
        """Estimate additive model acceleration from one-step prediction error."""
        velocity = np.asarray(object_twist[:3], dtype=float)
        angular_velocity = np.asarray(
            self.object_angular_velocity_world, dtype=float)
        if (self.planner_previous_object_velocity is None or
                self.planner_previous_object_angular_velocity is None or
                dt <= 1.0e-6 or dt > 0.10):
            self.planner_previous_object_velocity = velocity
            self.planner_previous_object_angular_velocity = angular_velocity
            return

        measured_linear = (
            velocity - self.planner_previous_object_velocity) / dt
        measured_angular = (
            angular_velocity -
            self.planner_previous_object_angular_velocity) / dt
        self.planner_previous_object_velocity = velocity
        self.planner_previous_object_angular_velocity = angular_velocity
        self.planner_measured_object_acceleration = self.clamp_vector(
            measured_linear,
            self.planner_disturbance_linear_residual_limit)
        self.planner_measured_object_angular_acceleration = \
            self.clamp_vector(
                measured_angular,
                self.planner_disturbance_angular_residual_limit)
        if (self.planner_last_model_acceleration is None or
                self.planner_last_model_angular_acceleration is None):
            return

        linear_residual = self.clamp_vector(
            self.planner_measured_object_acceleration -
            self.planner_last_model_acceleration,
            self.planner_disturbance_linear_residual_limit)
        angular_residual = self.clamp_vector(
            self.planner_measured_object_angular_acceleration -
            self.planner_last_model_angular_acceleration,
            self.planner_disturbance_angular_residual_limit)
        time_constant = max(1.0e-3, self.planner_disturbance_time_constant)
        alpha = self.clamp_value(
            self.planner_disturbance_gain * dt / (time_constant + dt),
            0.0, 1.0)
        self.planner_disturbance_linear_acceleration = self.clamp_vector(
            self.planner_disturbance_linear_acceleration +
            alpha * linear_residual,
            self.planner_disturbance_linear_limit)
        self.planner_disturbance_angular_acceleration = self.clamp_vector(
            self.planner_disturbance_angular_acceleration +
            alpha * angular_residual,
            self.planner_disturbance_angular_limit)

    def planner_goal(self, object_pose):
        if self.task_phase == "LIFT":
            goal_xy = self.planner_hold_xy
        else:
            goal_xy = (self.destination_x, self.destination_y)
        if self.task_phase == "DESCEND":
            goal_z = self.place_control_target_z
        elif self.lift_hold_target_object_z is not None:
            goal_z = self.lift_hold_target_object_z
        else:
            goal_z = object_pose.position.z
        return ((goal_xy[0], goal_xy[1], goal_z),
                self.planner_target_rpy)

    @staticmethod
    def clamp_vector(vector, limit):
        vector = np.asarray(vector, dtype=float)
        norm = np.linalg.norm(vector)
        if norm > limit > 0.0:
            vector *= limit / norm
        return vector

    def planner_ramp_commands(
            self, odometry, object_pose, object_twist,
            filtered_forces, contact_radius, dt,
            update_planner_force_state=True):
        ramp_duration = max(0.0, self.lift_force_ramp_duration)
        if ramp_duration <= 1.0e-6:
            self.lift_force_ramp_ratio = 1.0
        else:
            self.lift_force_ramp_ratio = min(
                1.0, self.lift_force_ramp_ratio + dt / ramp_duration)
        vertical_force = (
            self.lift_force_ramp_ratio * self.grasp_object_mass *
            self.gravity_acceleration / len(self.names))
        self.lift_vertical_force_target = vertical_force * len(self.names)
        self.lift_reference_z = object_pose.position.z
        self.lift_reference_velocity = 0.0
        self.lift_reference_acceleration = 0.0
        self.lift_object_acceleration_command = 0.0
        self.lift_common_z_acceleration_command = (
            vertical_force / max(1.0e-6, self.robot_mass))

        applied_forces = np.zeros((3, 3))
        commands = {}
        for index, name in enumerate(self.names):
            applied_forces[index] = (
                self.target_normal_forces[name], 0.0, vertical_force)
            ax, ay, yaw = self.press_acceleration(
                name, odometry[name], object_pose,
                filtered_forces[name], contact_radius, object_twist)
            commands[name] = (
                ax, ay, yaw, self.lift_common_z_acceleration_command)
        if update_planner_force_state:
            self.whole_body_planner.previous_force = applied_forces
            self.whole_body_planner.warm_start = np.tile(
                applied_forces.reshape((1, 3, 3)),
                (self.whole_body_planner.nodes, 1, 1))
        return commands

    def planner_solve_worker(
            self, state, goal_position, goal_rpy, contact_arms,
            vehicle_arms, external_linear_acceleration,
            external_angular_acceleration, request_ros_time):
        try:
            result = self.whole_body_planner.solve(
                state, goal_position, goal_rpy,
                contact_arms_body=contact_arms,
                vehicle_arms_body=vehicle_arms,
                external_linear_acceleration=external_linear_acceleration,
                external_angular_acceleration=external_angular_acceleration)
            pending = (request_ros_time, result, None)
        except Exception as error:  # keep the control loop alive for failsafe
            pending = (request_ros_time, None, str(error))
        with self.planner_result_lock:
            self.planner_pending_result = pending

    def consume_planner_result(self, now):
        with self.planner_result_lock:
            pending = self.planner_pending_result
            self.planner_pending_result = None
        if pending is None:
            return
        request_ros_time, result, error = pending
        if error is not None:
            rospy.logerr("whole-body planner exception: %s", error)
            self.planner_status_pub.publish(String(
                data="EXCEPTION {}".format(error)))
            return
        if result.success:
            self.planner_result = result
            # Accelerated headless MuJoCo can advance several simulated
            # tenths while an NLP consumes only a fraction of a wall second.
            # Starting the sampler at request time would skip directly to the
            # terminal braking input.  Real hardware may select request-time
            # sampling to account for communication/computation latency.
            self.planner_solution_ros_time = (
                now if self.planner_timestamp_at_acceptance else
                request_ros_time)
            self.planner_last_success_wall_time = time.monotonic()
            self.planner_status_pub.publish(String(
                data="OK {:.1f}ms {}".format(
                    1000.0 * result.solve_time, result.status)))
        else:
            rospy.logerr_throttle(
                1.0,
                "whole-body planner failed: %s (%.1f ms)",
                result.status, 1000.0 * result.solve_time)
            self.planner_status_pub.publish(String(
                data="FAILED {}".format(result.status)))

    def publish_planner_state(self, sample, result):
        stamp = rospy.Time.now()
        metrics = Vector3Stamped()
        metrics.header.stamp = stamp
        metrics.header.frame_id = "planner_seconds_ratio_cost"
        metrics.vector.x = result.solve_time
        metrics.vector.y = float(np.max(sample["friction_ratios"]))
        metrics.vector.z = result.cost
        self.planner_metrics_pub.publish(metrics)
        for publisher, frame_id, acceleration in (
                (self.planner_disturbance_linear_pub,
                 "world_additive_linear_acceleration",
                 self.planner_disturbance_linear_acceleration),
                (self.planner_disturbance_angular_pub,
                 "world_additive_angular_acceleration",
                 self.planner_disturbance_angular_acceleration)):
            disturbance = Vector3Stamped()
            disturbance.header.stamp = stamp
            disturbance.header.frame_id = frame_id
            disturbance.vector.x = acceleration[0]
            disturbance.vector.y = acceleration[1]
            disturbance.vector.z = acceleration[2]
            publisher.publish(disturbance)
        for index, name in enumerate(self.names):
            message = WrenchStamped()
            message.header.stamp = stamp
            message.header.frame_id = "grasp_prism_normal_tangent_z"
            message.wrench.force.x = sample["forces"][index, 0]
            message.wrench.force.y = sample["forces"][index, 1]
            message.wrench.force.z = sample["forces"][index, 2]
            self.planner_force_pubs[name].publish(message)

    def update_whole_body_planner(
            self, odometry, object_pose, object_twist, local_wrenches,
            filtered_forces, contact_radius, dt, now):
        if self.whole_body_planner is None:
            self.fail_mission("whole-body planner was not initialized")
            return None

        # Before the continuous-contact OCP starts, transfer the object's
        # weight from the pedestal to the contacts without a force step.
        if (self.lift_force_ramp_ratio < 1.0 - 1.0e-6 or
                self.lift_start_object_z is None):
            # Do not learn pedestal reactions as a grasp-model disturbance.
            # Synchronizing the velocity history also prevents a derivative
            # impulse when the MPC takes over after the support-force ramp.
            self.reset_planner_disturbance_estimator(object_twist)
            return self.planner_ramp_commands(
                odometry, object_pose, object_twist,
                filtered_forces, contact_radius, dt)

        self.update_planner_disturbance_estimator(object_twist, dt)
        state = self.planner_state(object_pose, object_twist)
        goal_position, goal_rpy = self.planner_goal(object_pose)
        self.consume_planner_result(now)
        planning_period = 1.0 / max(1.0e-6, self.planner_rate)
        thread_running = (
            self.planner_solve_thread is not None and
            self.planner_solve_thread.is_alive())
        due = (
            not thread_running and
            (self.planner_last_attempt_ros_time is None or
             now - self.planner_last_attempt_ros_time >= planning_period))
        if due:
            try:
                contact_arms, vehicle_arms = self.planner_grasp_geometry(
                    odometry, object_pose)
            except RuntimeError as error:
                rospy.logerr_throttle(
                    1.0, "cannot update planner grasp geometry: %s", error)
            else:
                self.planner_last_attempt_ros_time = now
                if self.planner_first_attempt_wall_time is None:
                    self.planner_first_attempt_wall_time = time.monotonic()
                self.planner_solve_thread = threading.Thread(
                    target=self.planner_solve_worker,
                    args=(np.array(state, copy=True), tuple(goal_position),
                          tuple(goal_rpy), np.array(contact_arms, copy=True),
                          np.array(vehicle_arms, copy=True),
                          np.array(
                              self.planner_disturbance_linear_acceleration,
                              copy=True),
                          np.array(
                              self.planner_disturbance_angular_acceleration,
                              copy=True), now))
                self.planner_solve_thread.daemon = True
                self.planner_solve_thread.start()

        stale = (
            self.planner_last_success_wall_time is None or
            time.monotonic() - self.planner_last_success_wall_time >
            self.planner_stale_timeout)
        bootstrap_waiting = (
            self.planner_result is None and
            self.planner_first_attempt_wall_time is not None and
            time.monotonic() - self.planner_first_attempt_wall_time <=
            self.planner_bootstrap_timeout)
        if bootstrap_waiting:
            return self.planner_ramp_commands(
                odometry, object_pose, object_twist,
                filtered_forces, contact_radius, 0.0,
                update_planner_force_state=False)
        if self.planner_result is None or stale:
            self.fail_mission("whole-body planner has no recent feasible plan")
            return None

        elapsed = max(0.0, now - self.planner_solution_ros_time)
        if self.planner_max_open_loop_time > 0.0:
            elapsed = min(elapsed, self.planner_max_open_loop_time)
        sample = self.whole_body_planner.sample(
            self.planner_result, elapsed)
        # This is the acceleration predicted with the disturbance estimate
        # used by the accepted solution.  The next measured velocity
        # derivative is compared against it to update the next OCP's constant
        # horizon disturbance.
        self.planner_last_model_acceleration = np.array(
            sample["object_acceleration"], copy=True)
        self.planner_last_model_angular_acceleration = np.array(
            sample["object_angular_acceleration"], copy=True)
        planned_world_forces = self.whole_body_planner.world_contact_forces(
            state, sample["forces"])
        if not self.planner_observer_initialized:
            for name in self.names:
                self.planner_filtered_object_forces[name] = -np.asarray(
                    local_wrenches[name]["force_bee"], dtype=float)
            self.planner_observer_initialized = True
        filter_tau = max(1.0e-3, self.planner_observer_time_constant)
        alpha = dt / (filter_tau + dt) if dt > 0.0 else 0.0

        contact_states = dict(
            (name, self.contact_position_state(
                name, odometry[name], object_pose, object_twist))
            for name in self.names)
        commands = {}
        applied_accelerations = []
        for index, name in enumerate(self.names):
            observed_force = -np.asarray(
                local_wrenches[name]["force_bee"], dtype=float)
            self.planner_filtered_object_forces[name] += alpha * (
                observed_force - self.planner_filtered_object_forces[name])
            planned_force = np.asarray(planned_world_forces[index])
            filtered_force = self.planner_filtered_object_forces[name]
            # The nominal whole-body solution contains planned_force/m as
            # contact-reaction feed-forward.  Replace that modelled reaction
            # with the low-pass observer value; otherwise a lost vertical
            # contact keeps commanding support acceleration and the Bee alone
            # slides upward.  The separate admittance term moves into the
            # contact along planned - measured to restore the desired force.
            force_error = planned_force - filtered_force
            observer_correction = (
                self.planner_external_force_compensation_gain *
                (filtered_force - planned_force) /
                max(1.0e-6, self.robot_mass) +
                self.planner_force_admittance_gain *
                force_error)
            observer_correction = self.clamp_vector(
                observer_correction,
                self.planner_observer_acceleration_limit)

            contact_state = contact_states[name]
            tangent_reference, z_reference = \
                self.planner_relative_contact_reference[name]
            tangent_correction = (
                -self.planner_relative_position_kp *
                (contact_state[4] - tangent_reference) -
                self.planner_relative_velocity_kd * contact_state[6])
            z_correction = (
                -self.planner_relative_position_kp *
                (contact_state[7] - z_reference) -
                self.planner_relative_velocity_kd * contact_state[8])
            normal, tangent, yaw = self.face_frame(name, object_pose)
            # The OCP supplies the desired resultant contact force, while the
            # real/simulated Bee and compliant feet remain the inner plant.
            # Close that model gap with the same observer-normal projection
            # used by the proven legacy controller.  This is an inner force
            # servo, not force-site feedback.
            normal_acceleration = (
                -self.planner_normal_force_kp *
                (sample["forces"][index, 0] - filtered_forces[name]) -
                self.planner_radial_velocity_kd * contact_state[5])
            normal_acceleration = self.clamp_value(
                normal_acceleration,
                -self.planner_normal_acceleration_limit,
                self.planner_normal_acceleration_limit)
            normal_correction = normal_acceleration * np.array(
                [normal[0], normal[1], 0.0])
            relative_correction = self.clamp_vector(
                (tangent_correction * np.array(
                    [tangent[0], tangent[1], 0.0]) +
                 np.array([0.0, 0.0, z_correction])),
                self.planner_relative_acceleration_limit)
            command = (
                np.asarray(sample["vehicle_accelerations"][index]) +
                observer_correction + normal_correction +
                relative_correction)
            commands[name] = (
                command[0], command[1],
                self.moment_corrected_yaw(name, yaw), command[2])
            applied_accelerations.append(command)

        reference_state = sample["reference_state"]
        self.lift_reference_z = reference_state[2]
        self.lift_reference_velocity = reference_state[5]
        self.lift_reference_acceleration = sample["object_acceleration"][2]
        self.lift_object_acceleration_command = \
            sample["object_acceleration"][2]
        self.translation_acceleration_command = list(
            sample["object_acceleration"][0:2])
        self.lift_vertical_force_target = sum(
            force[2] for force in planned_world_forces)
        self.lift_common_z_acceleration_command = float(np.mean(
            [command[2] for command in applied_accelerations]))
        for index, name in enumerate(self.names):
            self.lift_contact_z_acceleration_offsets[name] = (
                applied_accelerations[index][2] -
                self.lift_common_z_acceleration_command)
            self.lift_z_acceleration_offsets[name] = 0.0
        self.publish_planner_state(sample, self.planner_result)
        rospy.loginfo_throttle(
            0.5,
            "whole-body MPC: solve %.1f ms, max friction %.3f, "
            "Fz %.3f N, a_obj [%+.3f %+.3f %+.3f], "
            "d_ext [%+.3f %+.3f %+.3f]",
            1000.0 * self.planner_result.solve_time,
            float(np.max(sample["friction_ratios"])),
            self.lift_vertical_force_target,
            sample["object_acceleration"][0],
            sample["object_acceleration"][1],
            sample["object_acceleration"][2],
            self.planner_disturbance_linear_acceleration[0],
            self.planner_disturbance_linear_acceleration[1],
            self.planner_disturbance_linear_acceleration[2])
        return commands

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

    def command_place_xy_hold_velocity(self, object_pose):
        """Hold the payload over the destination during place and unload."""
        error_x = self.destination_x - object_pose.position.x
        error_y = self.destination_y - object_pose.position.y
        distance = math.hypot(error_x, error_y)
        if distance <= self.place_xy_hold_deadband:
            self.translation_velocity_command = [0.0, 0.0]
            return distance
        speed = min(
            self.place_xy_hold_velocity_limit,
            self.place_xy_hold_position_kp * distance)
        self.translation_velocity_command = [
            speed * error_x / distance,
            speed * error_y / distance,
        ]
        return distance

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
            self.command_place_xy_hold_velocity(object_pose)
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
            self.command_place_xy_hold_velocity(object_pose)
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
            self.command_place_xy_hold_velocity(object_pose)
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
            self.translation_velocity_command = [0.0, 0.0]
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
        self.initialize_whole_body_planner(
            initial_odometry, initial_object_pose,
            (0.0, 0.0, 0.0, 0.0))
        rospy.logwarn(
            "transport/place: LIFT (%s): grasp at %.2f N/vehicle, "
            "destination (%.3f, %.3f, %.3f)",
            self.control_strategy,
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
                    if not self.whole_body_planner_active():
                        self.update_local_moment_control(
                            local_wrenches, filtered_forces, control_dt)
                        self.update_object_moment_control(
                            local_wrenches, object_pose, object_moment,
                            filtered_forces, control_dt)
                    self.update_lift_progress(
                        object_pose, odometry, object_twist,
                        object_force[2] if
                        self.whole_body_planner_active() else None)

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
                planner_commands = None
                if contact_control and self.whole_body_planner_active():
                    planner_commands = self.update_whole_body_planner(
                        odometry, object_pose, object_twist,
                        local_wrenches, filtered_forces,
                        contact_radius, control_dt, now)
                    translation_acceleration = tuple(
                        self.translation_acceleration_command)
                elif contact_control:
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

                if self.safety_active:
                    self.publish_failsafe_hold(odometry)
                    time.sleep(self.control_period)
                    continue

                if self.task_phase in ("RETREAT", "UNWIND", "VERIFY"):
                    self.publish_retreat_commands(odometry, object_pose)
                elif planner_commands is not None:
                    for name in self.names:
                        ax, ay, yaw, az = planner_commands[name]
                        self.publish_accel_nav(
                            name, ax, ay, yaw,
                            FlightNav.ACC_MODE, az)
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
                    "goal error xy/z %.3f/%+.3f m, force [%s] N",
                    self.task_phase,
                    object_pose.position.x,
                    object_pose.position.y,
                    object_pose.position.z,
                    math.hypot(
                        object_pose.position.x - self.destination_x,
                        object_pose.position.y - self.destination_y),
                    object_pose.position.z - self.place_object_z,
                    " ".join("{:.2f}".format(filtered_forces[name])
                             for name in self.names))
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
