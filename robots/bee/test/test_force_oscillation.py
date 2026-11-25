#!/usr/bin/env python3
"""Numerical audit checks; no ROS publishers or master needed."""
from pathlib import Path
import sys,unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'script'))
from analyze_force_oscillation import spectrum_metrics,interpolate,windows,pulse_metrics

class OscillationTest(unittest.TestCase):
    def test_spectral_rms_and_peak(self):
        t=np.arange(0,30,.025)
        stats,_,_=spectrum_metrics(5+.01*t+.2*np.sin(2*np.pi*3*t),.025)
        self.assertAlmostEqual(stats['rms_2_8_Hz'],.2/np.sqrt(2),places=3)
        self.assertEqual(stats['peak_0.5_8_Hz'],3)
        self.assertLess(stats['rms_0.5_2_Hz'],.001)

    def test_rejects_gaps_and_does_not_anticipate_commands(self):
        rows=[(t,np.array([float(t>=1)])) for t in np.arange(0,2,.025)]
        y,_=interpolate(rows,np.arange(.91,1.1,.025),True)
        self.assertEqual(y[3,0],0)
        self.assertEqual(y[4,0],1)
        with self.assertRaises(ValueError):interpolate(rows[:15]+rows[40:],np.arange(.2,1.5,.025))

    def test_separates_hold_from_injected_steps(self):
        segments=windows([(0,'Lift'),(10,'Hold')],[dict(state='excite',axis='normal',axis_start=130,dwell=2.5)],170)
        self.assertAlmostEqual(segments[0]['start'],100)
        self.assertLess(segments[0]['end'],130)
        self.assertEqual(segments[1]['start'],130)

    def test_pulse_residual_is_invariant_to_constant_sensor_offset(self):
        t=np.arange(0,32.5,.01);block=(t/2.5).astype(int)
        u=.16*(block%2);y=u+.02*np.sin(2*np.pi*t)
        first=pulse_metrics(t,u,y,2.5);second=pulse_metrics(t,u,y+.274,2.5)
        self.assertEqual(first['blocks'],12)
        self.assertAlmostEqual(first['tail_error_std_mean'],second['tail_error_std_mean'],places=12)
        self.assertAlmostEqual(first['overshoot_relative_to_final_mean'],second['overshoot_relative_to_final_mean'],places=12)

if __name__=='__main__':unittest.main()
