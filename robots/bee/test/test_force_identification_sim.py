#!/usr/bin/env python3
"""Offline protocol guards; never creates ROS publishers or starts a master."""
import copy
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'script'))
from bee.msg import BeeCommand,ControllerStatus
import grasp_force_identification_sim as protocol


class ProtocolTest(unittest.TestCase):
    def test_refuses_default_or_remote_master_before_publishers(self):
        for uri in ['http://localhost:11311/','http://192.168.1.1:12311/','']:
            with patch.dict(os.environ,ROS_MASTER_URI=uri),patch.object(protocol.rospy,'Publisher') as publisher:
                with self.assertRaises(RuntimeError):protocol.Experiment()
                publisher.assert_not_called()

    def test_isolated_master_still_requires_mujoco(self):
        with patch.dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:12311/'):
            with patch.object(protocol.rospy,'Publisher') as publisher, \
                 patch.object(protocol.rospy,'is_shutdown',return_value=False), \
                 patch.object(protocol.rospy,'has_param',return_value=True), \
                 patch.object(protocol.rospy,'get_param',side_effect=lambda key,default: True if key=='/use_sim_time' else 'real_hardware'), \
                 patch.object(protocol.time,'monotonic',side_effect=[0,61]):
                with self.assertRaises(RuntimeError):protocol.Experiment()
                publisher.assert_not_called()

    def test_duplicate_clock_holds_previous_command(self):
        experiment=protocol.Experiment.__new__(protocol.Experiment)
        previous=BeeCommand();previous.header.stamp=protocol.rospy.Time.from_sec(10)
        previous.force.x=-4.5
        incoming=copy.deepcopy(previous);incoming.force.x=-4.6
        experiment.previous={'bee1':(10.,previous)};experiment.control_pub={'bee1':Mock()}
        experiment.reference(incoming,'bee1')
        sent=experiment.control_pub['bee1'].publish.call_args[0][0]
        self.assertEqual(sent.force.x,-4.5)

    def test_force_capacity_guards(self):
        config=dict(allocation_tangent_up_error=.1,allocation_tangent_side_error=.1,
            robot_mass=1.84,gravity=9.81,max_normal_force=8.,allocation_normal_error=.5,
            allocation_mu=.45,allocation_observer_normal_error=.5,
            allocation_thrust_limit=32.,allocation_thrust_reserve=3.)
        reference=BeeCommand();reference.acceleration_orientation.w=1;reference.acceleration_bounds=True;reference.force.x=-4.5
        reference.force.z=1.2
        measured=ControllerStatus();measured.external_force.x=4.5
        self.assertGreater(protocol.margins(reference,measured,config),0)
        for change in ['over_normal','over_tangent','under_observation','over_thrust']:
            r=copy.deepcopy(reference);s=copy.deepcopy(measured)
            if change=='over_normal':r.force.x=-7.6
            if change=='over_tangent':r.force.z=2.
            if change=='under_observation':s.external_force.x=3.
            if change=='over_thrust':r.navigation.target_acc_z=10.
            self.assertLess(protocol.margins(r,s,config),0,change)

    def test_requested_integral_mode_starts_before_settling(self):
        for mode,enabled in [('p',False),('pi',True)]:
            experiment=protocol.Experiment.__new__(protocol.Experiment)
            experiment.previous={};experiment.control_pub={'bee1':Mock()}
            experiment.phase='Hold';experiment.start=None;experiment.force_mode=mode
            reference=BeeCommand();reference.header.stamp=protocol.rospy.Time.from_sec(10)
            reference.force_integral_enabled=not enabled
            experiment.reference(reference,'bee1')
            self.assertEqual(experiment.control_pub['bee1'].publish.call_args[0][0].force_integral_enabled,enabled)

    def test_tilted_contact_frame_sets_excitation_direction(self):
        from math import sqrt
        generic=BeeCommand();generic.acceleration_bounds=True
        generic.acceleration_orientation.w=sqrt(.5)
        generic.acceleration_orientation.y=sqrt(.5)
        protocol.np.testing.assert_allclose(protocol.contact_normal(generic),[0.,0.,-1.],atol=1e-12)

    def test_excitation_changes_force_and_preserves_navigation(self):
        experiment = protocol.Experiment.__new__(protocol.Experiment)
        experiment.phase = 'Hold'
        experiment.start = 10.
        experiment.force_mode = 'pi'
        experiment.finished = False
        experiment.dwell = experiment.slew = 1.
        experiment.amplitude = .16
        experiment.delta = {}
        experiment.label = ''
        experiment.marker = Mock()
        experiment.control_pub = {'bee1': Mock()}
        experiment.config = {'allocation_force_rate': 5.}
        reference = BeeCommand()
        reference.enable_force_control = reference.acceleration_bounds = True
        reference.acceleration_orientation.w = 1.
        reference.navigation.target_acc_x = .12
        reference.force.x = -4.
        reference.force.z = 1.2
        reference.header.stamp = protocol.rospy.Time.from_sec(11.02)
        status = ControllerStatus()
        status.active = status.calibrated = True
        status.header = reference.header
        experiment.status = {'bee1': status}
        experiment.previous = {'bee1': (11., copy.deepcopy(reference))}
        with patch.object(protocol, 'margins', return_value=1.):
            experiment.nominal_control(reference, 'bee1')
        sent = experiment.control_pub['bee1'].publish.call_args[0][0]
        self.assertAlmostEqual(sent.force.x, -4.02)
        self.assertEqual(sent.force.z, reference.force.z)
        self.assertEqual(sent.navigation, reference.navigation)

    def test_stop_is_forwarded_without_waiting_for_task_diagnostics(self):
        experiment=protocol.Experiment.__new__(protocol.Experiment)
        experiment.control_pub={'bee1':Mock()}
        generic=BeeCommand();generic.stop=True
        experiment.nominal_control(generic,'bee1')
        experiment.control_pub['bee1'].publish.assert_called_once_with(generic)


if __name__=='__main__':unittest.main()
