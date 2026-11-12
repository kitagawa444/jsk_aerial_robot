#!/usr/bin/env python3

import os
import sys
import unittest

import numpy as np


SCRIPT_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "script"))
sys.path.insert(0, SCRIPT_DIR)

from whole_body_kinodynamic_planner import WholeBodyKinodynamicPlanner


class WholeBodyKinodynamicPlannerTest(unittest.TestCase):

    def make_planner(self):
        planner = WholeBodyKinodynamicPlanner(
            ("bee1", "bee2", "bee3"),
            object_mass=0.5,
            object_inertia=(0.03, 0.03, 0.022),
            robot_mass=1.84199,
            horizon=0.6,
            nodes=6,
            object_acceleration_limit=0.02,
            object_horizontal_velocity_limit=0.02,
            object_vertical_velocity_limit=0.01,
            solver_max_iterations=100,
            solver_tolerance=5.0e-4)
        contact_arms = [
            0.18 * normal for normal in planner.face_normals_body]
        vehicle_arms = [
            0.48 * normal for normal in planner.face_normals_body]
        planner.reset(contact_arms, vehicle_arms)
        return planner

    @staticmethod
    def initial_state(planner):
        return planner.state(
            (0.0, 1.0, 0.95),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0))

    def assert_feasible(self, planner, state, result):
        self.assertTrue(result.success, result.status)
        minimum = np.min(planner._constraints(
            result.forces.reshape(-1), state))
        self.assertGreaterEqual(minimum, -5.0e-4)
        self.assertLessEqual(
            np.max(result.friction_ratios),
            planner.friction_margin + 5.0e-4)

    def test_static_grasp_balances_weight(self):
        planner = self.make_planner()
        state = self.initial_state(planner)
        result = planner.solve(
            state, (0.0, 1.0, 0.95), (0.0, 0.0, 0.0))
        self.assert_feasible(planner, state, result)
        self.assertAlmostEqual(
            np.sum(result.forces[0, :, 2]),
            planner.object_mass * planner.gravity,
            delta=0.02)
        self.assertLess(
            np.linalg.norm(result.object_accelerations[0]), 0.03)

    def test_transport_plan_accelerates_toward_destination(self):
        planner = self.make_planner()
        state = self.initial_state(planner)
        result = planner.solve(
            state, (1.5, 1.0, 0.95), (0.0, 0.0, 0.0))
        self.assert_feasible(planner, state, result)
        self.assertGreater(result.object_accelerations[0, 0], 0.005)
        self.assertLessEqual(
            np.linalg.norm(result.object_accelerations[0]),
            planner.object_acceleration_limit + 5.0e-4)

    def test_timestamp_sampler_interpolates_plan(self):
        planner = self.make_planner()
        state = self.initial_state(planner)
        result = planner.solve(
            state, (0.0, 1.0, 1.0), (0.0, 0.0, 0.0))
        self.assert_feasible(planner, state, result)
        sample = planner.sample(result, 0.5 * planner.dt)
        expected = 0.5 * (result.forces[0] + result.forces[1])
        np.testing.assert_allclose(sample["forces"], expected)

    def test_measured_contact_geometry_updates_without_solver_rebuild(self):
        planner = self.make_planner()
        state = self.initial_state(planner)
        solver = planner._acados_solver
        forces = planner.equilibrium_forces()
        forces[0, 0] += 0.5
        angular_acceleration_before = planner._dynamics(
            state, forces)[1]

        contact_arms = np.array(planner.contact_arms_body, copy=True)
        vehicle_arms = np.array(planner.vehicle_arms_body, copy=True)
        contact_arms[0, 2] += 0.01
        vehicle_arms[0, 2] += 0.01
        result = planner.solve(
            state, (0.0, 1.0, 0.95), (0.0, 0.0, 0.0),
            contact_arms_body=contact_arms,
            vehicle_arms_body=vehicle_arms)

        self.assert_feasible(planner, state, result)
        self.assertIs(planner._acados_solver, solver)
        np.testing.assert_allclose(planner.contact_arms_body, contact_arms)
        np.testing.assert_allclose(planner.vehicle_arms_body, vehicle_arms)
        angular_acceleration_after = planner._dynamics(state, forces)[1]
        self.assertGreater(
            np.linalg.norm(
                angular_acceleration_after - angular_acceleration_before),
            1.0e-6)

        dynamics_without_disturbance = planner._dynamics(state, forces)
        linear_disturbance = np.array([-0.005, 0.003, -0.005])
        angular_disturbance = np.array([0.001, -0.001, 0.0005])
        disturbed_result = planner.solve(
            state, (0.0, 1.0, 0.95), (0.0, 0.0, 0.0),
            contact_arms_body=contact_arms,
            vehicle_arms_body=vehicle_arms,
            external_linear_acceleration=linear_disturbance,
            external_angular_acceleration=angular_disturbance)
        self.assert_feasible(planner, state, disturbed_result)
        self.assertIs(planner._acados_solver, solver)
        dynamics_with_disturbance = planner._dynamics(state, forces)
        np.testing.assert_allclose(
            dynamics_with_disturbance[0] - dynamics_without_disturbance[0],
            linear_disturbance, atol=1.0e-12)
        np.testing.assert_allclose(
            dynamics_with_disturbance[1] - dynamics_without_disturbance[1],
            angular_disturbance, atol=1.0e-12)

    def test_contact_and_vehicle_geometry_must_update_together(self):
        planner = self.make_planner()
        state = self.initial_state(planner)
        with self.assertRaises(ValueError):
            planner.solve(
                state, (0.0, 1.0, 0.95), (0.0, 0.0, 0.0),
                contact_arms_body=planner.contact_arms_body)
        with self.assertRaises(ValueError):
            planner.solve(
                state, (0.0, 1.0, 0.95), (0.0, 0.0, 0.0),
                external_linear_acceleration=np.zeros(3))


if __name__ == "__main__":
    unittest.main()
