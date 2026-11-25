#!/usr/bin/env python3
"""Offline bags with known signs/bias/gaps/contact verify the new force audit."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
import rosbag
import rospy
from bee.msg import BeeCommand,ControllerStatus,ObserverCalibrationStatus
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'script'))
from analyze_force_validation import analyze,causal


class ForceValidationTest(unittest.TestCase):
    def make_bag(self,path,mode,bias=0.,gap=False,contact=False,recalibrate=False,
                 repeated_phases=False,inactive_prefix=False,command_gap=False):
        with rosbag.Bag(str(path),'w') as b:
            b.write('/force_validation/segment',String(data=json.dumps(dict(name='x',start=100,duration=2,settle_s=.2))),rospy.Time.from_sec(100))
            for t in np.arange(100,104 if inactive_prefix else 102,.02):
                stamp=rospy.Time.from_sec(t)
                ref=BeeCommand();ref.header.stamp=stamp;ref.header.frame_id='world';ref.enable_force_control=True
                if inactive_prefix and t<101:ref.enable_force_control=False
                ref.force.x=.4;ref.force.y=.2;ref.force.z=-.1
                if not command_gap or not 100.5<t<101.1:b.write('/bee1/command',ref,stamp)
                if repeated_phases:
                    b.write('/bee_cooperative_grasp/phase',String(data='Approach' if t<101 else 'Hold'),stamp)
                known=WrenchStamped();known.header=ref.header;known.wrench.force=ref.force
                b.write('/bee1/mujoco/external_wrench',known,stamp)
                status=ControllerStatus();status.header=ref.header;status.ready=status.active=status.calibrated=True
                sign=1 if mode=='observer' else -1
                status.external_force.x=sign*.4+bias;status.external_force.y=sign*.2;status.external_force.z=sign*-.1
                if inactive_prefix and t<101:status.external_force.x=10.
                if not gap or not 100.5<t<101.1:b.write('/bee1/controller/tracking_status',status,stamp)
                c=JointState();c.header=ref.header;c.effort=[.1 if contact else 0.]
                b.write('/bee1/mujoco/grasp_contact_states',c,stamp)
            if recalibrate:
                m=ObserverCalibrationStatus();m.header.stamp=rospy.Time.from_sec(101);m.calibration_stamp=m.header.stamp
                b.write('/bee1/controller/observer_calibration',m,m.header.stamp)

    def audit(self,mode,**kw):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);self.make_bag(root/'input.bag',mode,**kw)
            return analyze(root/'input.bag',root/'report',mode)

    def test_opposite_force_conventions(self):
        for mode in ['observer','control']:
            r=self.audit(mode);self.assertTrue(r['passed'],r)
            self.assertLess(r['robots']['bee1']['vector_rmse_N'],1e-10)

    def test_bias_is_not_fitted_away(self):
        r=self.audit('observer',bias=.3)
        self.assertFalse(r['passed']);self.assertAlmostEqual(r['robots']['bee1']['vector_rmse_N'],.3)

    def test_missing_observations_fail(self):
        self.assertFalse(self.audit('observer',gap=True)['passed'])

    def test_contact_invalidates_known_force_reference(self):
        self.assertFalse(self.audit('observer',contact=True)['passed'])

    def test_recalibration_cannot_hide_force_error(self):
        self.assertFalse(self.audit('observer',recalibrate=True)['passed'])

    def test_causal_alignment_rejects_future_and_stale(self):
        values,valid=causal([(2.,np.ones(3))],np.array([1.9,2.,2.5]))
        self.assertEqual(valid.tolist(),[False,True,False])

    def test_repeated_phase_messages_keep_one_hold_window(self):
        r=self.audit('control',inactive_prefix=True,repeated_phases=True)
        self.assertTrue(r['passed'],r)
        windows=r['robots']['bee1']['windows']
        self.assertEqual(len(windows),1)
        self.assertEqual(windows[0]['name'],'Hold')

    def test_no_phase_bag_excludes_normal_motion(self):
        r=self.audit('control',inactive_prefix=True)
        self.assertTrue(r['passed'],r)
        self.assertGreaterEqual(r['robots']['bee1']['windows'][0]['start'],101)

    def test_command_gap_inside_force_mode_fails_coverage(self):
        self.assertFalse(self.audit('control',command_gap=True)['passed'])


if __name__=='__main__':unittest.main()
