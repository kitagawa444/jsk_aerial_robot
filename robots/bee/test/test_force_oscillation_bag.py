#!/usr/bin/env python3
"""End-to-end passive bag audit with known signals and default-zero telemetry."""
import json,math,subprocess,sys,tempfile,unittest
from pathlib import Path
import rosbag,rospy,yaml
from bee.msg import BeeCommand,ControllerStatus
from aerial_robot_msgs.msg import ForceList
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

class BagTest(unittest.TestCase):
    def test_known_physical_and_observed_oscillation_and_zero_rejection(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);bag=root/'flight.bag'
            config=dict(robot_mass=1.84,contact_damping=3,normal_force_kp=.4,contact_force_kp=.4)
            (root/'controller.yaml').write_text(yaml.safe_dump(dict(controller=config)))
            with rosbag.Bag(str(bag),'w') as out:
                out.write('/bee_cooperative_grasp/phase',String(data='Hold'),rospy.Time.from_sec(10))
                for k in range(3001):
                    t=10+k*.01;stamp=rospy.Time.from_sec(t);wave=math.sin(2*math.pi*2*t)
                    ref=BeeCommand();ref.header.stamp=stamp;ref.header.frame_id='world'
                    ref.enable_force_control=ref.acceleration_bounds=True
                    ref.acceleration_orientation.w=1;ref.force.x=-4;ref.force.z=1.2
                    status=ControllerStatus();status.header=ref.header
                    status.ready=status.active=status.calibrated=True
                    status.external_force=Vector3(4,0,-1.2-.1*wave)
                    status.commanded_acceleration=Vector3(-4/1.84,0,1.2/1.84-.04*wave)
                    physical=ForceList();physical.header=ref.header;physical.forces=[Vector3(-4,0,1.474+.2*wave)]
                    out.write('/bee1/command',ref,stamp)
                    out.write('/bee1/controller/tracking_status',status,stamp)
                    out.write('/bee1/mujoco/grasp_forces_world',physical,stamp)
                    # Old status timer can publish default vectors at the same stamp.
                    zero=ControllerStatus();zero.header=ref.header;zero.ready=zero.active=zero.calibrated=True
                    out.write('/bee1/controller/tracking_status',zero,stamp)
            script=Path(__file__).resolve().parents[1]/'script/analyze_force_oscillation.py'
            subprocess.run([sys.executable,str(script),str(bag),'--output',str(root/'result')],check=True,stdout=subprocess.DEVNULL)
            d=json.loads((root/'result/results.json').read_text());r=d['results'][0]
            self.assertTrue(r['valid']);self.assertEqual(d['excluded_status_samples']['bee1'],3001)
            self.assertAlmostEqual(r['metrics']['observer']['up']['rms_ac'],.1/math.sqrt(2),places=3)
            self.assertAlmostEqual(r['metrics']['sim_site_force']['up']['rms_ac'],.2/math.sqrt(2),places=3)
            self.assertAlmostEqual(r['metrics']['combined_correction']['up']['rms_ac'],1.84*.04/math.sqrt(2),places=3)
            self.assertEqual(r['metrics']['observer']['up']['peak_0.5_8_Hz'],2)
            self.assertAlmostEqual(r['metrics']['sim_site_force']['normal']['mean'],4,places=6)
            # Current configs omit mass. Historical results must use the recorded
            # model mass, even when the URDF on this machine has since changed.
            config.pop('robot_mass'); config.pop('contact_damping')
            (root/'controller.yaml').write_text(yaml.safe_dump(dict(controller=config)))
            (root/'summary.json').write_text(json.dumps({'parameters': {
                '/bee_cooperative_grasp/controller': {'robot_mass': 1.84},
                '/bee1/controller/reference_tracking': {'velocity_damping': 3}}}))
            subprocess.run([sys.executable,str(script),str(bag),'--output',str(root/'resolved')],check=True,stdout=subprocess.DEVNULL)
            resolved=json.loads((root/'resolved/results.json').read_text())
            self.assertEqual(resolved['results'], d['results'])


if __name__=='__main__':unittest.main()
