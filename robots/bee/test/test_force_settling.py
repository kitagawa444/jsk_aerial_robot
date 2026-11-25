#!/usr/bin/env python3
import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'script'))
from analyze_force_settling import response_metrics,extract_steps

class SettlingTest(unittest.TestCase):
    def test_first_order_rise_settling_and_persistent_ringing(self):
        t=np.arange(0,2.4,.01);r=response_metrics(t,1-np.exp(-t/.1))
        self.assertAlmostEqual(r['t90_s'],.1*np.log(10),delta=.011)
        self.assertAlmostEqual(r['settling_10pct_s'],.1*np.log(10),delta=.011)
        ring=response_metrics(t,1-.4*np.cos(2*np.pi*t))
        self.assertIsNone(ring['settling_10pct_s'])
        self.assertIsNotNone(ring['t90_s'])

    def test_step_alignment_sign_and_sensor_dc(self):
        t=np.arange(0,32.5,.01);u=np.where((t/2.5).astype(int)%2,.16,0.)
        data=np.zeros(len(t),dtype=[('time_s',float),('command_up',float),('sim_site_force_up',float)])
        data['time_s']=t;data['command_up']=1.2+u;data['sim_site_force_up']=1.474+u
        grid,steps=extract_steps(data,'up','sim_site_force')
        self.assertEqual(len(steps),12)
        for s in steps:
            self.assertAlmostEqual(s['wave'][-1],1,places=10)
            self.assertLessEqual(response_metrics(grid,s['wave'])['settling_10pct_s'],.05)
        data['sim_site_force_up']+=10
        _,offset=extract_steps(data,'up','sim_site_force')
        for a,b in zip(steps,offset):np.testing.assert_allclose(a['wave'],b['wave'],atol=1e-12)

if __name__=='__main__':unittest.main()
