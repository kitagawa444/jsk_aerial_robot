#!/usr/bin/env python3
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'script'))
from force_response_model import causal_sample,fit_response,ordered_unique_samples


def analytic_steps(t,tau,delay,gain=1.1,offset=.15):
    # Analytic continuous-time superposition, independent of the fitter's
    # sampled-data implementation. Include both upward/downward transitions.
    events=[(2,.25),(5,-.25),(8,-.2),(11,.2),(14,.12),(17,-.12),
            (21,.25),(24,-.25),(27,-.2),(30,.2)]
    u=np.ones_like(t)*4;y=np.ones_like(t)*(4*gain+offset)
    for start,delta in events:
        u+=delta*(t>=start)
        elapsed=t-start-delay
        y+=gain*delta*np.where(elapsed>0,1-np.exp(-np.maximum(0,elapsed)/tau),0)
    return u,y


class IdentificationTest(unittest.TestCase):
    def test_recovers_delayed_force_response_on_held_out_steps(self):
        t=np.arange(0,33,.01);u,y=analytic_steps(t,.24,.07)
        y+=np.random.default_rng(12).normal(0,.001,len(t))
        result,predicted=fit_response(t,u,y,20)
        self.assertTrue(result['accepted'],result)
        self.assertAlmostEqual(result['tau_s'],.24,delta=.025)
        self.assertAlmostEqual(result['delay_s'],.07,delta=.02)
        self.assertAlmostEqual(result['gain'],1.1,delta=.03)
        self.assertGreater(result['validation']['fit_percent'],95)

    def test_rejects_unexcited_axis(self):
        t=np.arange(0,20,.01)
        result,predicted=fit_response(t,np.ones_like(t),np.ones_like(t),10)
        self.assertFalse(result['accepted']);self.assertIsNone(predicted)
        self.assertIn('insufficient_input_excitation',result['reasons'])

    def test_validation_does_not_refit_changed_dynamics(self):
        t=np.arange(0,33,.01);u,y=analytic_steps(t,.24,.07)
        _,other=analytic_steps(t,1.2,.3)
        y[t>=20]=other[t>=20]
        result,_=fit_response(t,u,y,20)
        self.assertFalse(result['accepted'])
        self.assertIn('poor_held_out_prediction',result['reasons'])
        self.assertAlmostEqual(result['tau_s'],.24,delta=.03)

    def test_causal_alignment_never_uses_future_reference(self):
        value,valid=causal_sample([1,2,3],[10,20,30],np.array([.9,1.,1.9,2.,3.4]),max_age=.2)
        self.assertEqual(value.tolist(),[10,10,10,20,30])
        self.assertEqual(valid.tolist(),[False,True,False,True,False])

    def test_rejects_clock_discontinuity(self):
        with self.assertRaises(ValueError):causal_sample([1,2,1.5],[10,20,30],np.array([2.]))

    def test_duplicate_samples_do_not_hide_a_clock_reset(self):
        self.assertEqual(ordered_unique_samples([(1,10),(1,20),(2,30)]),[(1,20),(2,30)])
        with self.assertRaises(ValueError):ordered_unique_samples([(1,10),(2,20),(1,30)])


if __name__=='__main__':unittest.main()
