#!/usr/bin/env python3
"""Offline tests: sign conventions, delayed preload, and observer data quality."""
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
import rosbag
import rospy
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'script'))
from bee.msg import BeeCommand,ControllerStatus
from std_msgs.msg import String,Float64MultiArray
from analyze_support_transfer import components,capacity_margin,first_sustained,analyze


class SupportTimingTest(unittest.TestCase):
    def test_force_signs_and_tangent_axes(self):
        n=np.array([1.,0.,0.]);force=np.array([-5.,-.3,1.2])
        np.testing.assert_allclose(components(force,n),[5,1.2,-.3])
        angle=.7;r=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
        np.testing.assert_allclose(components(r@force,r@n),[5,1.2,-.3])

    def test_detects_support_ahead_of_normal_response(self):
        normal=np.array([1.,2.,3.,4.,5.]);support=np.tile([1.2,.2],(5,1))
        margin=capacity_margin(normal,support,.45,.5,.2)
        self.assertTrue(np.all(margin[:4]<0));self.assertGreater(margin[4],0)
        self.assertAlmostEqual(margin[4],.425)

    def test_readiness_requires_sustained_samples(self):
        t=np.arange(0,1,.01);valid=((t>.1)&(t<.15))|(t>=.4)
        self.assertAlmostEqual(first_sustained(t,valid,.1),.4)
        self.assertIsNone(first_sustained(t,t>.95,.1))

    def test_bag_flags_define_start_and_default_zero_status_is_excluded(self):
        # Phase notification deliberately arrives after the first support
        # command. Early support is unsafe until normal observation catches up.
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);bagpath=root/'input.bag'
            with rosbag.Bag(str(bagpath),'w') as bag:
                for t in np.arange(98.,105.,.02):
                    if abs(t-98)<.001:bag.write('/bee_cooperative_grasp/phase',String('Preload'),rospy.Time.from_sec(t))
                    if abs(t-100.1)<.001:bag.write('/bee_cooperative_grasp/phase',String('SupportTransfer'),rospy.Time.from_sec(t))
                    if abs(t-104)<.001:bag.write('/bee_cooperative_grasp/phase',String('Lift'),rospy.Time.from_sec(t))
                    stamp=rospy.Time.from_sec(t);support=bool(t>=100-1e-6)
                    m=BeeCommand();m.header.stamp=stamp;m.header.frame_id='world'
                    m.enable_force_control=m.acceleration_bounds=support;m.acceleration_orientation.w=1.;m.force.x=-5.
                    m.force.z=1.2 if support else 0.
                    bag.write('/bee1/command',m,stamp)
                    metrics=Float64MultiArray();metrics.data=[0.]*18
                    metrics.data[7]=float(np.clip((t-100)/2,0,1))
                    bag.write('/bee_cooperative_grasp/metrics',metrics,stamp)
                    s=ControllerStatus();s.header=m.header;s.ready=s.active=s.calibrated=True
                    s.external_force.x=2. if t<101 else 5.
                    s.external_force.z=-1.2 if t>=100.3 else 0.
                    bag.write('/bee1/grasp/tracking_status',s,stamp)
                    zero=ControllerStatus();zero.header=m.header;zero.ready=zero.active=zero.calibrated=True
                    bag.write('/bee1/grasp/tracking_status',zero,stamp)
            cfg={'allocation_mu':.45,'allocation_observer_normal_error':.5,
                'allocation_normal_error':.5,'allocation_tangent_up_error':.1,'allocation_tangent_side_error':.1}
            result=analyze(bagpath,['bee1'],.1,cfg,root/'analysis')
            self.assertAlmostEqual(result['transfer_start_s'],100.,places=5)
            self.assertFalse(result['checks']['local_observer_capacity_nonnegative'])
            self.assertTrue(result['checks']['complete_force_data'])
            self.assertGreater(result['per_bee']['bee1']['negative_capacity_duration_s'],.9)
            self.assertGreater(result['excluded_observer_samples']['bee1'],300)
            self.assertAlmostEqual(result['observed_90percent_lag_s'],.3,delta=.03)
            self.assertAlmostEqual(result['support_progress_reached_one_s'],2.,delta=.03)
            self.assertAlmostEqual(result['wait_after_full_progress_s'],2.,delta=.03)


if __name__=='__main__':unittest.main()
