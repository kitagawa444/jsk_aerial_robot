#!/usr/bin/env python3
"""End-to-end offline bag test with independently generated analytic responses."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np
import rosbag
import rospy
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'script'))
from bee.msg import BeeCommand,ControllerStatus
from test_force_response_model import analytic_steps


class BagTest(unittest.TestCase):
    def test_sign_frames_relative_windows_and_held_out_fit(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);segments=[]
            basis=np.array([[-1,0,0],[0,0,1],[0,1,0]])
            with rosbag.Bag(str(root/'input.bag'),'w') as bag:
                for axis,name in enumerate(['normal','up','side']):
                    t=np.arange(0,33,.01);u,y=analytic_steps(t,.24,.07)
                    u-=4.;y-=4.*1.1+.15
                    segments.append(dict(name=name,axis=name,start=33.*axis,
                        duration=33.,validation_after=20.,integral_enabled=True))
                    for k,elapsed in enumerate(t):
                        stamp=rospy.Time.from_sec(100+33*axis+elapsed)
                        command=np.array([4.,1.2,0.]);observed=command.copy()
                        command[axis]+=u[k];observed[axis]+=y[k]
                        reference=BeeCommand();reference.header.stamp=stamp;reference.header.frame_id='world'
                        reference.enable_force_control=reference.acceleration_bounds=True
                        reference.force_integral_enabled=True;reference.acceleration_orientation.w=1
                        reference.force.x=-command[0]
                        reference.force.y=command[2];reference.force.z=command[1]
                        status=ControllerStatus();status.header=reference.header
                        status.active=status.ready=status.calibrated=True
                        reaction=-(observed@basis)
                        status.external_force.x,status.external_force.y,status.external_force.z=map(float,reaction)
                        bag.write('/bee1/command',reference,stamp)
                        bag.write('/bee1/grasp/tracking_status',status,stamp)
            (root/'config.yaml').write_text(yaml.safe_dump(dict(robot_names=['bee1'],
                time_origin='bag_start',segments=segments)))
            script=Path(__file__).resolve().parents[1]/'script/identify_grasp_force_response.py'
            subprocess.run([sys.executable,str(script),str(root/'input.bag'),'--output',str(root/'fit'),
                '--config',str(root/'config.yaml')],check=True,stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,env=dict(os.environ,OPENBLAS_NUM_THREADS='1'))
            results=json.loads((root/'fit/results.json').read_text())
            self.assertEqual(len(results['models']),3)
            for model in results['models']:
                self.assertTrue(model['accepted'],model)
                self.assertAlmostEqual(model['tau_s'],.24,delta=.025)
                self.assertAlmostEqual(model['delay_s'],.07,delta=.02)
                self.assertAlmostEqual(model['gain'],1.1,delta=.03)
                self.assertGreater(model['validation']['fit_percent'],95)
            self.assertTrue((root/'fit/model.yaml').is_file())
            self.assertTrue((root/'fit/bee1_response.png').is_file())


if __name__=='__main__':unittest.main()
