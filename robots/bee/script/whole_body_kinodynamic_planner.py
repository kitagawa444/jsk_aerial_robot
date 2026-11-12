#!/usr/bin/env python

"""Reduced whole-body kinodynamic planner for the three-Bee grasp.

The Science Robotics cable-suspended planner optimizes the load and all cable
states in one finite-horizon problem, then supplies smooth trajectories to the
individual vehicle controllers.  The Bee grasp has unilateral frictional
contacts instead of taut cables.  This module uses the same trajectory-based
architecture with one resultant contact per four-ball foot plane:

  u_i = [normal force, face-tangent force, object-Z force].

The load pose/twist is integrated through rigid-body dynamics.  The implied
vehicle trajectories are obtained from fixed grasp-formation arms and are used
to constrain thrust and tilt.  Friction cones and contact-force slew limits are
hard constraints.  Contact acquisition/release are intentionally outside this
continuous-contact model and remain in the demo state machine.
"""

from __future__ import print_function

import math
import os
import tempfile
import time
from collections import namedtuple

import numpy as np
from scipy.optimize import minimize

try:
    import casadi as ca
except ImportError:
    ca = None

try:
    from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
except ImportError:
    AcadosModel = None
    AcadosOcp = None
    AcadosOcpSolver = None


PlannerResult = namedtuple(
    "PlannerResult",
    ("success", "status", "solve_time", "cost", "states", "forces",
     "object_accelerations", "object_angular_accelerations",
     "vehicle_accelerations", "friction_ratios"))


def _wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _rotation_matrix(rpy):
    """Return the object-to-world ZYX rotation matrix."""
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


class WholeBodyKinodynamicPlanner(object):
    """Finite-horizon contact-force and whole-body trajectory optimizer."""

    FACE_ANGLES = (math.pi / 3.0, math.pi, 5.0 * math.pi / 3.0)
    STATE_SIZE = 12
    FORCE_SIZE = 3

    def __init__(self, names, object_mass, object_inertia, robot_mass,
                 horizon=1.2, nodes=12, gravity=9.81,
                 friction_coefficient=0.5, friction_margin=0.9,
                 normal_force_min=1.0, normal_force_max=8.0,
                 normal_force_target=4.0, force_rate_limit=5.0,
                 normal_force_rate_limit=2.0,
                 object_acceleration_limit=0.20,
                 object_horizontal_velocity_limit=0.03,
                 object_vertical_velocity_limit=0.015,
                 object_horizontal_velocity_constraint_limit=None,
                 object_vertical_velocity_constraint_limit=None,
                 object_angular_velocity_limit=0.20,
                 maximum_specific_thrust=15.0,
                 maximum_vehicle_tilt=0.65,
                 solver_backend="acados",
                 casadi_nlp_solver="ipopt",
                 casadi_qp_solver="qrqp",
                 solver_max_iterations=35, solver_tolerance=1.0e-4,
                 position_weights=(35.0, 35.0, 70.0),
                 velocity_weights=(10.0, 10.0, 16.0),
                 attitude_weights=(35.0, 35.0, 18.0),
                 angular_velocity_weights=(8.0, 8.0, 5.0),
                 normal_force_weight=0.08,
                 tangential_force_weight=0.04,
                 force_smoothness_weight=0.30,
                 terminal_weight_scale=5.0):
        self.names = tuple(names)
        if len(self.names) != 3:
            raise ValueError("planner currently requires exactly three Bees")
        self.object_mass = float(object_mass)
        self.object_inertia = np.asarray(object_inertia, dtype=float)
        if self.object_inertia.shape == (3,):
            self.object_inertia = np.diag(self.object_inertia)
        if self.object_inertia.shape != (3, 3):
            raise ValueError("object_inertia must have 3 or 9 values")
        self.robot_mass = float(robot_mass)
        self.gravity = float(gravity)
        self.nodes = int(nodes)
        self.horizon = float(horizon)
        if self.object_mass <= 0.0 or self.robot_mass <= 0.0:
            raise ValueError("object and robot masses must be positive")
        if self.nodes < 3 or self.horizon <= 0.0:
            raise ValueError("planner requires nodes >= 3 and horizon > 0")
        self.dt = self.horizon / self.nodes

        self.friction_coefficient = float(friction_coefficient)
        self.friction_margin = float(friction_margin)
        self.normal_force_min = float(normal_force_min)
        self.normal_force_max = float(normal_force_max)
        self.normal_force_target = float(normal_force_target)
        self.force_rate_limit = float(force_rate_limit)
        self.normal_force_rate_limit = float(normal_force_rate_limit)
        self.object_acceleration_limit = float(object_acceleration_limit)
        self.horizontal_velocity_limit = float(
            object_horizontal_velocity_limit)
        self.vertical_velocity_limit = float(object_vertical_velocity_limit)
        self.horizontal_velocity_constraint_limit = float(
            object_horizontal_velocity_constraint_limit
            if object_horizontal_velocity_constraint_limit is not None else
            self.horizontal_velocity_limit)
        self.vertical_velocity_constraint_limit = float(
            object_vertical_velocity_constraint_limit
            if object_vertical_velocity_constraint_limit is not None else
            self.vertical_velocity_limit)
        self.angular_velocity_limit = float(object_angular_velocity_limit)
        self.maximum_specific_thrust = float(maximum_specific_thrust)
        self.maximum_vehicle_tilt = float(maximum_vehicle_tilt)
        self.solver_backend = str(solver_backend).lower()
        self.casadi_nlp_solver = str(casadi_nlp_solver).lower()
        self.casadi_qp_solver = str(casadi_qp_solver).lower()
        self.solver_max_iterations = int(solver_max_iterations)
        self.solver_tolerance = float(solver_tolerance)

        self.position_weights = np.asarray(position_weights, dtype=float)
        self.velocity_weights = np.asarray(velocity_weights, dtype=float)
        self.attitude_weights = np.asarray(attitude_weights, dtype=float)
        self.angular_velocity_weights = np.asarray(
            angular_velocity_weights, dtype=float)
        self.normal_force_weight = float(normal_force_weight)
        self.tangential_force_weight = float(tangential_force_weight)
        self.force_smoothness_weight = float(force_smoothness_weight)
        self.terminal_weight_scale = float(terminal_weight_scale)

        if not 0.0 < self.friction_margin <= 1.0:
            raise ValueError("friction_margin must be in (0, 1]")
        if self.friction_coefficient <= 0.0:
            raise ValueError("friction_coefficient must be positive")
        if not 0.0 <= self.normal_force_min < self.normal_force_max:
            raise ValueError("invalid normal force limits")
        if self.solver_backend not in ("acados", "casadi", "scipy"):
            raise ValueError(
                "solver_backend must be acados, casadi, or scipy")
        if self.solver_backend == "acados" and AcadosOcpSolver is None:
            raise ImportError(
                "acados backend requested but acados_template is unavailable")
        if self.solver_backend == "casadi" and ca is None:
            self.solver_backend = "scipy"
        if self.casadi_nlp_solver not in ("sqpmethod", "ipopt"):
            raise ValueError("casadi_nlp_solver must be sqpmethod or ipopt")

        self.contact_arms_body = None
        self.vehicle_arms_body = None
        self.external_linear_acceleration = np.zeros(3)
        self.external_angular_acceleration = np.zeros(3)
        self.previous_force = None
        self.warm_start = None
        self.last_result = None
        self._casadi_solver = None
        self._casadi_lbg = None
        self._casadi_ubg = None
        self._acados_solver = None
        self._acados_build_directory = None

        self.face_normals_body = tuple(
            np.array([math.cos(angle), math.sin(angle), 0.0])
            for angle in self.FACE_ANGLES)
        self.face_tangents_body = tuple(
            np.array([-math.sin(angle), math.cos(angle), 0.0])
            for angle in self.FACE_ANGLES)
        self.object_z_body = np.array([0.0, 0.0, 1.0])

    @property
    def input_size(self):
        return len(self.names) * self.FORCE_SIZE

    def _set_grasp_geometry(self, contact_arms_body, vehicle_arms_body):
        contact_arms = np.asarray(contact_arms_body, dtype=float)
        vehicle_arms = np.asarray(vehicle_arms_body, dtype=float)
        if contact_arms.shape != (3, 3):
            raise ValueError("contact_arms_body must be 3x3")
        if vehicle_arms.shape != (3, 3):
            raise ValueError("vehicle_arms_body must be 3x3")
        if not (np.all(np.isfinite(contact_arms)) and
                np.all(np.isfinite(vehicle_arms))):
            raise ValueError("grasp geometry must contain finite values")
        # Replace the complete snapshots so a running controller never sees
        # a matrix that is only partly updated.
        self.contact_arms_body = np.array(contact_arms, copy=True)
        self.vehicle_arms_body = np.array(vehicle_arms, copy=True)

    def _dynamic_parameters(self):
        return np.concatenate((
            self.contact_arms_body.reshape(-1),
            self.vehicle_arms_body.reshape(-1),
            self.external_linear_acceleration,
            self.external_angular_acceleration))

    def _set_external_disturbance(
            self, linear_acceleration, angular_acceleration):
        linear = np.asarray(linear_acceleration, dtype=float)
        angular = np.asarray(angular_acceleration, dtype=float)
        if linear.shape != (3,):
            raise ValueError("external linear acceleration must have 3 values")
        if angular.shape != (3,):
            raise ValueError(
                "external angular acceleration must have 3 values")
        if not (np.all(np.isfinite(linear)) and
                np.all(np.isfinite(angular))):
            raise ValueError("external disturbance must contain finite values")
        self.external_linear_acceleration = np.array(linear, copy=True)
        self.external_angular_acceleration = np.array(angular, copy=True)

    def reset(self, contact_arms_body, vehicle_arms_body,
              initial_contact_forces=None):
        self._set_grasp_geometry(contact_arms_body, vehicle_arms_body)
        self._set_external_disturbance(np.zeros(3), np.zeros(3))
        if initial_contact_forces is None:
            initial_contact_forces = self.equilibrium_forces()
        self.previous_force = np.asarray(
            initial_contact_forces, dtype=float).reshape((3, 3))
        self.warm_start = np.tile(
            self.previous_force.reshape(1, 3, 3), (self.nodes, 1, 1))
        self.last_result = None
        if self.solver_backend == "acados" and self._acados_solver is None:
            self._build_acados_solver()
        if self.solver_backend == "casadi" and self._casadi_solver is None:
            self._build_casadi_solver()

    def equilibrium_forces(self):
        forces = np.zeros((3, 3))
        forces[:, 0] = np.clip(
            self.normal_force_target,
            self.normal_force_min, self.normal_force_max)
        forces[:, 2] = self.object_mass * self.gravity / 3.0
        return forces

    @staticmethod
    def state(position, velocity, rpy, angular_velocity):
        return np.concatenate((
            np.asarray(position, dtype=float),
            np.asarray(velocity, dtype=float),
            np.asarray(rpy, dtype=float),
            np.asarray(angular_velocity, dtype=float)))

    def _world_contact_forces(self, state, contact_forces):
        rotation = _rotation_matrix(state[6:9])
        result = []
        for index in range(3):
            normal_force, tangent_force, vertical_force = \
                contact_forces[index]
            force_body = (
                -normal_force * self.face_normals_body[index] +
                tangent_force * self.face_tangents_body[index] +
                vertical_force * self.object_z_body)
            result.append(np.dot(rotation, force_body))
        return result

    def world_contact_forces(self, state, contact_forces):
        """Convert [normal, tangent, vertical] contact forces to world axes."""
        return tuple(self._world_contact_forces(
            np.asarray(state, dtype=float),
            np.asarray(contact_forces, dtype=float).reshape((3, 3))))

    def _dynamics(self, state, contact_forces):
        rotation = _rotation_matrix(state[6:9])
        world_forces = self._world_contact_forces(state, contact_forces)
        total_force = np.sum(world_forces, axis=0)
        object_acceleration = total_force / self.object_mass
        object_acceleration[2] -= self.gravity
        object_acceleration += self.external_linear_acceleration

        total_moment = np.zeros(3)
        for index in range(3):
            arm_world = np.dot(rotation, self.contact_arms_body[index])
            total_moment += np.cross(arm_world, world_forces[index])
        inertia_world = np.dot(
            rotation, np.dot(self.object_inertia, rotation.T))
        angular_velocity = state[9:12]
        gyroscopic_moment = np.cross(
            angular_velocity, np.dot(inertia_world, angular_velocity))
        angular_acceleration = np.linalg.solve(
            inertia_world, total_moment - gyroscopic_moment)
        angular_acceleration += self.external_angular_acceleration

        vehicle_accelerations = []
        for index in range(3):
            arm_world = np.dot(rotation, self.vehicle_arms_body[index])
            formation_acceleration = (
                object_acceleration +
                np.cross(angular_acceleration, arm_world) +
                np.cross(angular_velocity,
                         np.cross(angular_velocity, arm_world)))
            # Object force is opposite to the reaction acting on the Bee.
            # Adding f_object/m_robot is the thrust feed-forward required for
            # the Bee to track the rigid-formation acceleration.
            vehicle_accelerations.append(
                formation_acceleration +
                world_forces[index] / self.robot_mass)
        return (object_acceleration, angular_acceleration,
                world_forces, vehicle_accelerations)

    def _step(self, state, contact_forces):
        (linear_acceleration, angular_acceleration,
         _, _) = self._dynamics(state, contact_forces)
        next_state = np.array(state, copy=True)
        next_state[0:3] += (
            state[3:6] * self.dt +
            0.5 * linear_acceleration * self.dt ** 2)
        next_state[3:6] += linear_acceleration * self.dt
        next_state[6:9] += (
            state[9:12] * self.dt +
            0.5 * angular_acceleration * self.dt ** 2)
        next_state[6:9] = np.array(
            [_wrap_angle(value) for value in next_state[6:9]])
        next_state[9:12] += angular_acceleration * self.dt
        return next_state

    def _rollout(self, initial_state, flat_forces):
        force_sequence = np.asarray(flat_forces).reshape(
            (self.nodes, 3, 3))
        states = [np.asarray(initial_state, dtype=float)]
        object_accelerations = []
        object_angular_accelerations = []
        vehicle_accelerations = []
        for force in force_sequence:
            dynamics = self._dynamics(states[-1], force)
            object_accelerations.append(dynamics[0])
            object_angular_accelerations.append(dynamics[1])
            vehicle_accelerations.append(dynamics[3])
            states.append(self._step(states[-1], force))
        return (
            np.asarray(states), np.asarray(object_accelerations),
            np.asarray(object_angular_accelerations),
            np.asarray(vehicle_accelerations))

    def _reference_trajectory(self, initial_state, target_position,
                              target_rpy):
        target_position = np.asarray(target_position, dtype=float)
        target_rpy = np.asarray(target_rpy, dtype=float)
        position_error = target_position - initial_state[0:3]
        attitude_error = np.array([
            _wrap_angle(target_rpy[axis] - initial_state[6 + axis])
            for axis in range(3)])
        reference = np.zeros((self.nodes + 1, self.STATE_SIZE))
        reference[0] = initial_state
        for node in range(1, self.nodes + 1):
            duration = node * self.dt
            horizontal_distance = np.linalg.norm(position_error[0:2])
            if horizontal_distance > 1.0e-9:
                fraction_xy = min(
                    1.0,
                    self.horizontal_velocity_limit * duration /
                    horizontal_distance)
                reference[node, 0:2] = (
                    initial_state[0:2] +
                    fraction_xy * position_error[0:2])
                if fraction_xy < 1.0:
                    reference[node, 3:5] = (
                        self.horizontal_velocity_limit *
                        position_error[0:2] / horizontal_distance)
            else:
                reference[node, 0:2] = target_position[0:2]

            z_distance = abs(position_error[2])
            if z_distance > 1.0e-9:
                fraction_z = min(
                    1.0,
                    self.vertical_velocity_limit * duration / z_distance)
                reference[node, 2] = (
                    initial_state[2] + fraction_z * position_error[2])
                if fraction_z < 1.0:
                    reference[node, 5] = math.copysign(
                        self.vertical_velocity_limit, position_error[2])
            else:
                reference[node, 2] = target_position[2]

            attitude_distance = np.linalg.norm(attitude_error)
            if attitude_distance > 1.0e-9:
                fraction_attitude = min(
                    1.0,
                    self.angular_velocity_limit * duration /
                    attitude_distance)
                reference[node, 6:9] = initial_state[6:9] + (
                    fraction_attitude * attitude_error)
                if fraction_attitude < 1.0:
                    reference[node, 9:12] = (
                        self.angular_velocity_limit * attitude_error /
                        attitude_distance)
            else:
                reference[node, 6:9] = target_rpy
        return reference

    def _objective(self, flat_forces, initial_state, reference):
        force_sequence = np.asarray(flat_forces).reshape(
            (self.nodes, 3, 3))
        states, _, _, _ = self._rollout(initial_state, force_sequence)
        cost = 0.0
        previous = self.previous_force
        for node in range(1, self.nodes + 1):
            state = states[node]
            desired = reference[node]
            scale = (
                self.terminal_weight_scale
                if node == self.nodes else 1.0)
            position_error = state[0:3] - desired[0:3]
            velocity_error = state[3:6] - desired[3:6]
            attitude_error = np.array([
                _wrap_angle(state[6 + axis] - desired[6 + axis])
                for axis in range(3)])
            angular_velocity_error = state[9:12] - desired[9:12]
            cost += scale * (
                np.dot(self.position_weights, position_error ** 2) +
                np.dot(self.velocity_weights, velocity_error ** 2) +
                np.dot(self.attitude_weights, attitude_error ** 2) +
                np.dot(self.angular_velocity_weights,
                       angular_velocity_error ** 2))

            force = force_sequence[node - 1]
            cost += self.normal_force_weight * np.sum(
                (force[:, 0] - self.normal_force_target) ** 2)
            support_force = self.object_mass * self.gravity / 3.0
            cost += self.tangential_force_weight * (
                np.sum(force[:, 1] ** 2) +
                np.sum((force[:, 2] - support_force) ** 2))
            cost += self.force_smoothness_weight * np.sum(
                (force - previous) ** 2)
            previous = force
        return float(cost)

    def _constraints(self, flat_forces, initial_state):
        force_sequence = np.asarray(flat_forces).reshape(
            (self.nodes, 3, 3))
        (states, object_accelerations, _,
         vehicle_accelerations) = self._rollout(
             initial_state, force_sequence)
        values = []
        previous = self.previous_force
        friction_limit_scale = (
            self.friction_margin * self.friction_coefficient)
        tangent_of_tilt = math.tan(self.maximum_vehicle_tilt)
        for node in range(self.nodes):
            force = force_sequence[node]
            # Circular friction cone for each representative contact.
            for index in range(3):
                tangential_norm = math.hypot(
                    force[index, 1], force[index, 2])
                values.append(
                    friction_limit_scale * force[index, 0] -
                    tangential_norm)

            delta = force - previous
            normal_step = self.normal_force_rate_limit * self.dt
            tangent_step = self.force_rate_limit * self.dt
            for index in range(3):
                values.extend((
                    normal_step - delta[index, 0],
                    normal_step + delta[index, 0],
                ))
                for component in (1, 2):
                    values.extend((
                        tangent_step - delta[index, component],
                        tangent_step + delta[index, component],
                    ))
            previous = force

            acceleration = object_accelerations[node]
            values.append(
                self.object_acceleration_limit -
                np.linalg.norm(acceleration))
            predicted_state = states[node + 1]
            values.append(
                self.horizontal_velocity_constraint_limit -
                np.linalg.norm(predicted_state[3:5]))
            values.extend((
                self.vertical_velocity_constraint_limit - predicted_state[5],
                self.vertical_velocity_constraint_limit + predicted_state[5],
            ))
            values.append(
                self.angular_velocity_limit -
                np.linalg.norm(predicted_state[9:12]))

            for vehicle_acceleration in vehicle_accelerations[node]:
                specific_thrust = np.array(vehicle_acceleration, copy=True)
                specific_thrust[2] += self.gravity
                values.append(
                    self.maximum_specific_thrust -
                    np.linalg.norm(specific_thrust))
                values.append(
                    tangent_of_tilt * specific_thrust[2] -
                    np.linalg.norm(specific_thrust[0:2]))
                values.append(specific_thrust[2])
        return np.asarray(values)

    def _initial_guess(self):
        if self.warm_start is None:
            return np.tile(
                self.equilibrium_forces().reshape(1, 3, 3),
                (self.nodes, 1, 1))
        shifted = np.empty_like(self.warm_start)
        shifted[:-1] = self.warm_start[1:]
        shifted[-1] = self.warm_start[-1]
        # A shifted solution can violate the first-node slew bound after a
        # disturbance.  Start exactly from the force currently being applied.
        shifted[0] = self.previous_force
        return shifted

    @staticmethod
    def _ca_rotation_matrix(rpy):
        roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
        cr, sr = ca.cos(roll), ca.sin(roll)
        cp, sp = ca.cos(pitch), ca.sin(pitch)
        cy, sy = ca.cos(yaw), ca.sin(yaw)
        return ca.vertcat(
            ca.horzcat(cy * cp, cy * sp * sr - sy * cr,
                       cy * sp * cr + sy * sr),
            ca.horzcat(sy * cp, sy * sp * sr + cy * cr,
                       sy * sp * cr - cy * sr),
            ca.horzcat(-sp, cp * sr, cp * cr))

    def _ca_dynamics(self, state, force, contact_arms_body=None,
                     vehicle_arms_body=None,
                     external_linear_acceleration=None,
                     external_angular_acceleration=None):
        if contact_arms_body is None:
            contact_arms_body = tuple(
                ca.DM(arm) for arm in self.contact_arms_body)
        if vehicle_arms_body is None:
            vehicle_arms_body = tuple(
                ca.DM(arm) for arm in self.vehicle_arms_body)
        if external_linear_acceleration is None:
            external_linear_acceleration = ca.DM(
                self.external_linear_acceleration)
        if external_angular_acceleration is None:
            external_angular_acceleration = ca.DM(
                self.external_angular_acceleration)
        rotation = self._ca_rotation_matrix(state[6:9])
        total_force = ca.SX.zeros(3, 1)
        total_moment = ca.SX.zeros(3, 1)
        world_forces = []
        for index in range(3):
            force_body = (
                -force[3 * index] * ca.DM(
                    self.face_normals_body[index]) +
                force[3 * index + 1] * ca.DM(
                    self.face_tangents_body[index]) +
                force[3 * index + 2] * ca.DM(self.object_z_body))
            force_world = ca.mtimes(rotation, force_body)
            arm_world = ca.mtimes(rotation, contact_arms_body[index])
            total_force += force_world
            total_moment += ca.cross(arm_world, force_world)
            world_forces.append(force_world)

        object_acceleration = total_force / self.object_mass + ca.DM(
            [0.0, 0.0, -self.gravity]) + external_linear_acceleration
        inertia_world = ca.mtimes(
            rotation,
            ca.mtimes(ca.DM(self.object_inertia), rotation.T))
        angular_velocity = state[9:12]
        angular_acceleration = ca.solve(
            inertia_world,
            total_moment - ca.cross(
                angular_velocity,
                ca.mtimes(inertia_world, angular_velocity))) + \
            external_angular_acceleration

        vehicle_accelerations = []
        for index in range(3):
            vehicle_arm = ca.mtimes(rotation, vehicle_arms_body[index])
            formation_acceleration = (
                object_acceleration +
                ca.cross(angular_acceleration, vehicle_arm) +
                ca.cross(angular_velocity,
                         ca.cross(angular_velocity, vehicle_arm)))
            vehicle_accelerations.append(
                formation_acceleration +
                world_forces[index] / self.robot_mass)
        return (object_acceleration, angular_acceleration,
                vehicle_accelerations)

    def _ca_step(self, state, force, contact_arms_body=None,
                 vehicle_arms_body=None,
                 external_linear_acceleration=None,
                 external_angular_acceleration=None):
        linear_acceleration, angular_acceleration, _ = \
            self._ca_dynamics(
                state, force, contact_arms_body, vehicle_arms_body,
                external_linear_acceleration,
                external_angular_acceleration)
        return ca.vertcat(
            state[0:3] + state[3:6] * self.dt +
            0.5 * linear_acceleration * self.dt ** 2,
            state[3:6] + linear_acceleration * self.dt,
            state[6:9] + state[9:12] * self.dt +
            0.5 * angular_acceleration * self.dt ** 2,
            state[9:12] + angular_acceleration * self.dt)

    def _build_acados_solver(self):
        """Generate a compact SQP-RTI OCP with dynamic grasp geometry.

        The object pose/twist and the nine contact-force components are OCP
        states.  Contact-force rates are controls.  This exposes the temporal
        structure that the condensed CasADi/IPOPT formulation hides and turns
        the force slew limits into simple control bounds.
        """
        if ca is None or AcadosOcpSolver is None:
            raise RuntimeError("acados requires CasADi and acados_template")

        object_state_size = self.STATE_SIZE
        force_state_size = self.input_size
        acados_state_size = object_state_size + force_state_size
        x = ca.SX.sym("x", acados_state_size)
        force_rate = ca.SX.sym("force_rate", force_state_size)
        dynamic_parameters = ca.SX.sym("dynamic_parameters", 24)
        contact_arms = tuple(
            dynamic_parameters[3 * index:3 * index + 3]
            for index in range(3))
        vehicle_arms = tuple(
            dynamic_parameters[9 + 3 * index:9 + 3 * index + 3]
            for index in range(3))
        external_linear_acceleration = dynamic_parameters[18:21]
        external_angular_acceleration = dynamic_parameters[21:24]
        object_state = x[0:object_state_size]
        previous_force = x[object_state_size:acados_state_size]
        next_force = previous_force + self.dt * force_rate
        next_object_state = self._ca_step(
            object_state, next_force, contact_arms, vehicle_arms,
            external_linear_acceleration,
            external_angular_acceleration)

        model = AcadosModel()
        model.name = "bee_whole_body_acados"
        model.x = x
        model.u = force_rate
        model.p = dynamic_parameters
        model.disc_dyn_expr = ca.vertcat(next_object_state, next_force)

        # The stage residual is evaluated at the end of each shooting
        # interval, matching the original single-shooting planner exactly.
        model.cost_y_expr = ca.vertcat(
            next_object_state, next_force, self.dt * force_rate)
        model.cost_y_expr_e = x[0:object_state_size]

        (object_acceleration, _, vehicle_accelerations) = \
            self._ca_dynamics(
                object_state, next_force, contact_arms, vehicle_arms,
                external_linear_acceleration,
                external_angular_acceleration)
        constraints = []
        lower_constraints = []
        upper_constraints = []

        tangential_bound = (
            self.friction_coefficient * self.normal_force_max)
        for index in range(3):
            normal = next_force[3 * index]
            tangent = next_force[3 * index + 1]
            vertical = next_force[3 * index + 2]
            for value, lower, upper in (
                    (normal, self.normal_force_min,
                     self.normal_force_max),
                    (tangent, -tangential_bound, tangential_bound),
                    (vertical, -tangential_bound, tangential_bound)):
                constraints.append(value)
                lower_constraints.append(lower)
                upper_constraints.append(upper)
            friction_scale = (
                self.friction_margin * self.friction_coefficient)
            constraints.append(
                (friction_scale * normal) ** 2 -
                tangent ** 2 - vertical ** 2)
            lower_constraints.append(0.0)
            upper_constraints.append(1.0e15)

        # L1 polytopes retain a useful SQP linearization at zero and are
        # contained inside the corresponding Euclidean balls.  Unlike
        # independent component bounds they therefore cannot produce a plan
        # that fails the exact nonlinear norm check when several axes act at
        # once.  Axis-aligned motion can still use the full configured limit.
        for sign_x in (-1.0, 1.0):
            for sign_y in (-1.0, 1.0):
                for sign_z in (-1.0, 1.0):
                    constraints.append(
                        sign_x * object_acceleration[0] +
                        sign_y * object_acceleration[1] +
                        sign_z * object_acceleration[2])
                    lower_constraints.append(-1.0e15)
                    upper_constraints.append(self.object_acceleration_limit)
        for sign_x in (-1.0, 1.0):
            for sign_y in (-1.0, 1.0):
                constraints.append(
                    sign_x * next_object_state[3] +
                    sign_y * next_object_state[4])
                lower_constraints.append(-1.0e15)
                upper_constraints.append(
                    self.horizontal_velocity_constraint_limit)
        constraints.append(next_object_state[5])
        lower_constraints.append(-self.vertical_velocity_constraint_limit)
        upper_constraints.append(self.vertical_velocity_constraint_limit)
        for sign_x in (-1.0, 1.0):
            for sign_y in (-1.0, 1.0):
                for sign_z in (-1.0, 1.0):
                    constraints.append(
                        sign_x * next_object_state[9] +
                        sign_y * next_object_state[10] +
                        sign_z * next_object_state[11])
                    lower_constraints.append(-1.0e15)
                    upper_constraints.append(self.angular_velocity_limit)

        tilt_tangent_squared = math.tan(self.maximum_vehicle_tilt) ** 2
        for vehicle_acceleration in vehicle_accelerations:
            specific_thrust = vehicle_acceleration + ca.DM(
                [0.0, 0.0, self.gravity])
            constraints.append(ca.dot(specific_thrust, specific_thrust))
            lower_constraints.append(0.0)
            upper_constraints.append(self.maximum_specific_thrust ** 2)
            constraints.append(
                ca.dot(specific_thrust[0:2], specific_thrust[0:2]) -
                tilt_tangent_squared * specific_thrust[2] ** 2)
            lower_constraints.append(-1.0e15)
            upper_constraints.append(0.0)
            constraints.append(specific_thrust[2])
            lower_constraints.append(0.0)
            upper_constraints.append(1.0e15)

        model.con_h_expr = ca.vertcat(*constraints)
        model.con_h_expr_0 = model.con_h_expr

        ocp = AcadosOcp()
        ocp.model = model
        ocp.parameter_values = self._dynamic_parameters()
        ocp.solver_options.N_horizon = self.nodes
        ocp.solver_options.tf = self.horizon
        ocp.solver_options.integrator_type = "DISCRETE"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.qp_solver_cond_N = self.nodes
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.qp_solver_warm_start = 1
        ocp.solver_options.qp_solver_iter_max = max(
            20, self.solver_max_iterations)
        ocp.solver_options.nlp_solver_tol_stat = self.solver_tolerance
        ocp.solver_options.nlp_solver_tol_eq = self.solver_tolerance
        ocp.solver_options.nlp_solver_tol_ineq = self.solver_tolerance
        ocp.solver_options.nlp_solver_tol_comp = self.solver_tolerance
        ocp.solver_options.print_level = 0

        state_weights = np.concatenate((
            self.position_weights,
            self.velocity_weights,
            self.attitude_weights,
            self.angular_velocity_weights))
        force_weights = np.tile(np.array([
            self.normal_force_weight,
            self.tangential_force_weight,
            self.tangential_force_weight]), 3)
        delta_weights = np.full(
            force_state_size, self.force_smoothness_weight)
        # acados uses 0.5 * residual' W residual.
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.W = 2.0 * np.diag(np.concatenate((
            state_weights, force_weights, delta_weights)))
        ocp.cost.yref = np.zeros(
            object_state_size + 2 * force_state_size)
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W_e = 2.0 * np.diag(
            (self.terminal_weight_scale - 1.0) * state_weights)
        ocp.cost.yref_e = np.zeros(object_state_size)

        normal_rate_bounds = []
        for _ in self.names:
            normal_rate_bounds.extend((
                self.normal_force_rate_limit,
                self.force_rate_limit,
                self.force_rate_limit))
        normal_rate_bounds = np.asarray(normal_rate_bounds)
        ocp.constraints.idxbu = np.arange(force_state_size)
        ocp.constraints.lbu = -normal_rate_bounds
        ocp.constraints.ubu = normal_rate_bounds
        ocp.constraints.x0 = np.zeros(acados_state_size)
        ocp.constraints.lh = np.asarray(lower_constraints)
        ocp.constraints.uh = np.asarray(upper_constraints)
        ocp.constraints.lh_0 = np.asarray(lower_constraints)
        ocp.constraints.uh_0 = np.asarray(upper_constraints)

        self._acados_build_directory = tempfile.mkdtemp(
            prefix="bee_whole_body_acados_")
        ocp.code_export_directory = os.path.join(
            self._acados_build_directory, "c_generated_code")
        json_file = os.path.join(
            self._acados_build_directory, "acados_ocp.json")
        self._acados_solver = AcadosOcpSolver(
            ocp, json_file=json_file, verbose=False)

    def _solve_acados(self, initial_guess, initial_state, reference):
        """Run one preparation/feedback SQP-RTI iteration."""
        solver = self._acados_solver
        support_force = self.object_mass * self.gravity / 3.0
        force_reference = np.tile(np.array([
            self.normal_force_target, 0.0, support_force]), 3)
        zero_delta = np.zeros(self.input_size)
        dynamic_parameters = self._dynamic_parameters()

        # The measured contact/vehicle arms are frozen only over this short
        # horizon.  Updating p avoids regenerating or recompiling acados when
        # the balls slide on the object between MPC iterations.
        for node in range(self.nodes + 1):
            solver.set(node, "p", dynamic_parameters)

        # Seed both the primal state trajectory and force-rate controls from
        # the shifted previous solution.  acados retains its QP memory too.
        rollout_states, _, _, _ = self._rollout(
            initial_state, initial_guess)
        previous = self.previous_force.reshape(-1)
        augmented_initial = np.concatenate((initial_state, previous))
        solver.set(0, "x", augmented_initial)
        for node in range(self.nodes):
            force = initial_guess[node].reshape(-1)
            solver.set(node, "u", (force - previous) / self.dt)
            solver.set(node + 1, "x", np.concatenate((
                rollout_states[node + 1], force)))
            solver.cost_set(node, "yref", np.concatenate((
                reference[node + 1], force_reference, zero_delta)))
            previous = force
        solver.cost_set(self.nodes, "yref", reference[-1])

        solver.set(0, "lbx", augmented_initial)
        solver.set(0, "ubx", augmented_initial)
        start_time = time.monotonic()
        preparation_status = 5
        feedback_status = 5
        solver_time = 0.0
        # A norm constraint linearized exactly at zero has zero first
        # derivative.  Three inexpensive RTI sweeps let the first sweep find
        # a useful direction and the following sweeps restore nonlinear
        # feasibility while remaining far below the control deadline.
        for _ in range(3):
            solver.options_set("rti_phase", 1)
            preparation_status = int(solver.solve())
            solver_time += float(solver.get_stats("time_tot"))
            solver.options_set("rti_phase", 2)
            feedback_status = int(solver.solve())
            solver_time += float(solver.get_stats("time_tot"))
        wall_time = time.monotonic() - start_time

        forces = np.asarray([
            solver.get(node + 1, "x")[self.STATE_SIZE:]
            for node in range(self.nodes)]).reshape((self.nodes, 3, 3))
        success = bool(
            preparation_status in (0, 5) and feedback_status == 0 and
            np.all(np.isfinite(forces)))
        status = (
            "acados SQP_RTI prep={} feedback={} "
            "solver={:.3f}ms wall={:.3f}ms".format(
                preparation_status, feedback_status,
                1000.0 * solver_time,
                1000.0 * wall_time))
        try:
            cost = float(solver.get_cost())
        except Exception:
            cost = float(self._objective(
                forces.reshape(-1), initial_state, reference))
        return success, status, cost, forces, wall_time

    def _build_casadi_solver(self):
        """Build a sparse NLP whose geometry is supplied on every solve."""
        inputs = ca.SX.sym("contact_forces", self.input_size, self.nodes)
        parameter_size = (
            self.STATE_SIZE +
            (self.nodes + 1) * self.STATE_SIZE + self.input_size + 24)
        parameters = ca.SX.sym("planner_parameters", parameter_size)
        state = parameters[0:self.STATE_SIZE]
        reference_offset = self.STATE_SIZE
        previous_force_offset = (
            reference_offset + (self.nodes + 1) * self.STATE_SIZE)
        previous_force = parameters[
            previous_force_offset:previous_force_offset + self.input_size]
        geometry_offset = previous_force_offset + self.input_size
        contact_arms = tuple(
            parameters[geometry_offset + 3 * index:
                       geometry_offset + 3 * index + 3]
            for index in range(3))
        vehicle_geometry_offset = geometry_offset + 9
        vehicle_arms = tuple(
            parameters[vehicle_geometry_offset + 3 * index:
                       vehicle_geometry_offset + 3 * index + 3]
            for index in range(3))
        external_linear_acceleration = parameters[
            geometry_offset + 18:geometry_offset + 21]
        external_angular_acceleration = parameters[
            geometry_offset + 21:geometry_offset + 24]

        objective = 0.0
        constraints = []
        lower_constraints = []
        upper_constraints = []
        position_weights = ca.DM(self.position_weights)
        velocity_weights = ca.DM(self.velocity_weights)
        attitude_weights = ca.DM(self.attitude_weights)
        angular_weights = ca.DM(self.angular_velocity_weights)
        friction_scale = self.friction_margin * self.friction_coefficient
        normal_step = self.normal_force_rate_limit * self.dt
        tangent_step = self.force_rate_limit * self.dt
        tilt_tangent_squared = math.tan(self.maximum_vehicle_tilt) ** 2

        for node in range(self.nodes):
            force = inputs[:, node]
            linear_acceleration, _, vehicle_accelerations = \
                self._ca_dynamics(
                    state, force, contact_arms, vehicle_arms,
                    external_linear_acceleration,
                    external_angular_acceleration)
            state = self._ca_step(
                state, force, contact_arms, vehicle_arms,
                external_linear_acceleration,
                external_angular_acceleration)
            reference_start = reference_offset + (node + 1) * self.STATE_SIZE
            reference = parameters[
                reference_start:reference_start + self.STATE_SIZE]
            position_error = state[0:3] - reference[0:3]
            velocity_error = state[3:6] - reference[3:6]
            attitude_error = ca.atan2(
                ca.sin(state[6:9] - reference[6:9]),
                ca.cos(state[6:9] - reference[6:9]))
            angular_error = state[9:12] - reference[9:12]
            weight_scale = (
                self.terminal_weight_scale
                if node == self.nodes - 1 else 1.0)
            objective += weight_scale * (
                ca.dot(position_weights, position_error ** 2) +
                ca.dot(velocity_weights, velocity_error ** 2) +
                ca.dot(attitude_weights, attitude_error ** 2) +
                ca.dot(angular_weights, angular_error ** 2))

            for index in range(3):
                normal = force[3 * index]
                tangent = force[3 * index + 1]
                vertical = force[3 * index + 2]
                objective += (
                    self.normal_force_weight *
                    (normal - self.normal_force_target) ** 2 +
                    self.tangential_force_weight *
                    (tangent ** 2 +
                     (vertical - self.object_mass * self.gravity / 3.0) ** 2))
                constraints.append(
                    (friction_scale * normal) ** 2 -
                    tangent ** 2 - vertical ** 2)
                lower_constraints.append(0.0)
                upper_constraints.append(ca.inf)

            force_delta = force - previous_force
            objective += self.force_smoothness_weight * ca.dot(
                force_delta, force_delta)
            for index in range(3):
                for component, step in ((0, normal_step),
                                        (1, tangent_step),
                                        (2, tangent_step)):
                    constraints.append(force_delta[3 * index + component])
                    lower_constraints.append(-step)
                    upper_constraints.append(step)
            previous_force = force

            constraints.append(ca.dot(
                linear_acceleration, linear_acceleration))
            lower_constraints.append(0.0)
            upper_constraints.append(self.object_acceleration_limit ** 2)
            constraints.append(ca.dot(state[3:5], state[3:5]))
            lower_constraints.append(0.0)
            upper_constraints.append(self.horizontal_velocity_limit ** 2)
            constraints.append(state[5])
            lower_constraints.append(-self.vertical_velocity_limit)
            upper_constraints.append(self.vertical_velocity_limit)
            constraints.append(ca.dot(state[9:12], state[9:12]))
            lower_constraints.append(0.0)
            upper_constraints.append(self.angular_velocity_limit ** 2)

            for vehicle_acceleration in vehicle_accelerations:
                specific_thrust = vehicle_acceleration + ca.DM(
                    [0.0, 0.0, self.gravity])
                constraints.append(ca.dot(specific_thrust, specific_thrust))
                lower_constraints.append(0.0)
                upper_constraints.append(self.maximum_specific_thrust ** 2)
                constraints.append(
                    ca.dot(specific_thrust[0:2], specific_thrust[0:2]) -
                    tilt_tangent_squared * specific_thrust[2] ** 2)
                lower_constraints.append(-ca.inf)
                upper_constraints.append(0.0)
                constraints.append(specific_thrust[2])
                lower_constraints.append(0.0)
                upper_constraints.append(ca.inf)

        decision = ca.reshape(inputs, self.input_size * self.nodes, 1)
        problem = {
            "x": decision,
            "p": parameters,
            "f": objective,
            "g": ca.vertcat(*constraints),
        }
        if self.casadi_nlp_solver == "sqpmethod":
            if self.casadi_qp_solver == "qpoases":
                qp_options = {
                    "printLevel": "none",
                    "error_on_fail": False,
                }
            else:
                qp_options = {
                    "print_header": False,
                    "print_iter": False,
                    "error_on_fail": False,
                }
            options = {
                "print_time": False,
                "print_header": False,
                "print_iteration": False,
                "print_status": False,
                "max_iter": self.solver_max_iterations,
                "tol_pr": self.solver_tolerance,
                "tol_du": self.solver_tolerance,
                "qpsol": self.casadi_qp_solver,
                "qpsol_options": qp_options,
            }
        else:
            options = {
                "print_time": False,
                "ipopt.print_level": 0,
                "ipopt.sb": "yes",
                "ipopt.max_iter": self.solver_max_iterations,
                "ipopt.tol": self.solver_tolerance,
                "ipopt.acceptable_tol": max(
                    self.solver_tolerance, 5.0e-4),
                "ipopt.warm_start_init_point": "yes",
            }
        self._casadi_solver = ca.nlpsol(
            "bee_whole_body_planner", self.casadi_nlp_solver,
            problem, options)
        self._casadi_lbg = np.asarray(lower_constraints, dtype=float)
        self._casadi_ubg = np.asarray(upper_constraints, dtype=float)

    def _force_bounds(self):
        tangential_bound = (
            self.friction_coefficient * self.normal_force_max)
        lower = []
        upper = []
        scipy_bounds = []
        for _ in range(self.nodes):
            for _ in self.names:
                triples = (
                    (self.normal_force_min, self.normal_force_max),
                    (-tangential_bound, tangential_bound),
                    (-tangential_bound, tangential_bound),
                )
                scipy_bounds.extend(triples)
                for low, high in triples:
                    lower.append(low)
                    upper.append(high)
        return np.asarray(lower), np.asarray(upper), scipy_bounds

    def _solve_casadi(self, initial_guess, initial_state, reference):
        lower, upper, _ = self._force_bounds()
        parameters = np.concatenate((
            initial_state,
            reference.reshape(-1),
            self.previous_force.reshape(-1),
            self._dynamic_parameters()))
        try:
            solution = self._casadi_solver(
                x0=initial_guess.reshape(-1),
                p=parameters,
                lbx=lower,
                ubx=upper,
                lbg=self._casadi_lbg,
                ubg=self._casadi_ubg)
            forces = np.asarray(solution["x"], dtype=float).reshape(
                (self.nodes, 3, 3))
            stats = self._casadi_solver.stats()
            status = str(stats.get("return_status", "unknown"))
            # Like the paper's SQP real-time-iteration scheme, a control
            # update may intentionally stop before full NLP convergence.  A
            # finite, constraint-feasible iterate is checked and accepted in
            # solve(); retain max-iteration output for that check.
            # NLP return codes describe convergence, not whether the latest
            # real-time iterate is usable.  solve() independently evaluates
            # every physical constraint with the nonlinear rollout.  Accept
            # any finite iterate here and let that stricter feasibility check
            # make the control decision.
            success = bool(np.all(np.isfinite(forces)))
            if success and not stats.get("success", False):
                status = "Feasible iterate ({})".format(status)
            cost = float(solution["f"])
        except RuntimeError as error:
            forces = initial_guess
            success = False
            status = "CasADi/IPOPT failed: {}".format(error)
            cost = float(self._objective(
                forces.reshape(-1), initial_state, reference))
        return success, status, cost, forces

    def _solve_scipy(self, initial_guess, initial_state, reference):
        _, _, scipy_bounds = self._force_bounds()
        solution = minimize(
            self._objective,
            initial_guess.reshape(-1),
            args=(initial_state, reference),
            method="SLSQP",
            bounds=scipy_bounds,
            constraints=({
                "type": "ineq",
                "fun": self._constraints,
                "args": (initial_state,),
            },),
            options={
                "maxiter": self.solver_max_iterations,
                "ftol": self.solver_tolerance,
                "disp": False,
            })
        return (bool(solution.success), str(solution.message),
                float(solution.fun),
                np.asarray(solution.x).reshape((self.nodes, 3, 3)))

    def solve(self, initial_state, target_position, target_rpy,
              contact_arms_body=None, vehicle_arms_body=None,
              external_linear_acceleration=None,
              external_angular_acceleration=None):
        if self.contact_arms_body is None or self.vehicle_arms_body is None:
            raise RuntimeError("reset() must provide grasp geometry first")
        if ((contact_arms_body is None) !=
                (vehicle_arms_body is None)):
            raise ValueError(
                "contact and vehicle arms must be updated together")
        if contact_arms_body is not None:
            self._set_grasp_geometry(
                contact_arms_body, vehicle_arms_body)
        if ((external_linear_acceleration is None) !=
                (external_angular_acceleration is None)):
            raise ValueError(
                "linear and angular disturbances must be updated together")
        if external_linear_acceleration is not None:
            self._set_external_disturbance(
                external_linear_acceleration,
                external_angular_acceleration)
        initial_state = np.asarray(initial_state, dtype=float)
        if initial_state.shape != (self.STATE_SIZE,):
            raise ValueError("initial_state must contain 12 values")
        reference = self._reference_trajectory(
            initial_state, target_position, target_rpy)
        initial_guess = self._initial_guess()
        start_time = time.monotonic()
        backend_solve_time = None
        if self.solver_backend == "acados":
            (solver_success, status, cost, forces,
             backend_solve_time) = self._solve_acados(
                 initial_guess, initial_state, reference)
        elif self.solver_backend == "casadi":
            solver_success, status, cost, forces = self._solve_casadi(
                initial_guess, initial_state, reference)
        else:
            solver_success, status, cost, forces = self._solve_scipy(
                initial_guess, initial_state, reference)
        solve_time = (
            backend_solve_time if backend_solve_time is not None else
            time.monotonic() - start_time)
        minimum_constraint = float(np.min(
            self._constraints(forces.reshape(-1), initial_state)))
        feasible = minimum_constraint >= -5.0e-4
        success = bool(solver_success and feasible)
        if not feasible:
            status += "; minimum constraint {:.3e}".format(
                minimum_constraint)

        (states, object_accelerations, object_angular_accelerations,
         vehicle_accelerations) = self._rollout(initial_state, forces)
        friction_ratios = np.zeros((self.nodes, 3))
        for node in range(self.nodes):
            for index in range(3):
                denominator = max(
                    1.0e-6,
                    self.friction_coefficient * forces[node, index, 0])
                friction_ratios[node, index] = math.hypot(
                    forces[node, index, 1],
                    forces[node, index, 2]) / denominator

        result = PlannerResult(
            success=success,
            status=status,
            solve_time=solve_time,
            cost=cost,
            states=states,
            forces=forces,
            object_accelerations=object_accelerations,
            object_angular_accelerations=object_angular_accelerations,
            vehicle_accelerations=vehicle_accelerations,
            friction_ratios=friction_ratios)
        if success:
            self.warm_start = forces
            self.previous_force = np.array(forces[0], copy=True)
            self.last_result = result
        return result

    def sample(self, result, elapsed):
        """Linearly sample a receding-horizon solution by timestamp."""
        if result is None or not result.success:
            raise ValueError("a successful planner result is required")
        phase = max(0.0, float(elapsed)) / self.dt
        first = min(self.nodes - 1, int(math.floor(phase)))
        second = min(self.nodes - 1, first + 1)
        ratio = min(1.0, max(0.0, phase - first))

        def interpolate(values):
            return ((1.0 - ratio) * values[first] +
                    ratio * values[second])

        return {
            "forces": interpolate(result.forces),
            "object_acceleration": interpolate(
                result.object_accelerations),
            "object_angular_acceleration": interpolate(
                result.object_angular_accelerations),
            "vehicle_accelerations": interpolate(
                result.vehicle_accelerations),
            "friction_ratios": interpolate(result.friction_ratios),
            "reference_state": interpolate(result.states[1:]),
        }
