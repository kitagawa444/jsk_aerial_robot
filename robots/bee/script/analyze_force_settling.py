#!/usr/bin/env python3
"""Passive step-settling audit of CSVs from analyze_force_oscillation.py.

Report the median of aligned full-amplitude steps, plus the fraction of individual
steps that settle. This is not a worst-case bound or a true-contact force sensor.
MuJoCo site sensors include foot mass/inertia. A common 50 ms causal moving
average suppresses single-sample site spikes; that measurement delay is retained.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def causal_average(y, samples):
    return np.convolve(np.pad(y,(samples-1,0),mode='edge'),np.ones(samples)/samples,mode='valid')


def response_metrics(t, y, tolerance=.1, minimum_tail=.3):
    def crossing(value):
        mask=y>=value
        count=0
        for i,yes in enumerate(mask):
            count=count+1 if yes else 0
            if count>=5:return float(t[i-4])
        return None
    outside=np.flatnonzero(abs(y-1)>tolerance)
    index=int(outside[-1]+1) if len(outside) else 0
    settle=float(t[index]) if index<len(t) and t[-1]-t[index]>=minimum_tail else None
    return dict(t10_s=crossing(.1),t90_s=crossing(.9),settling_10pct_s=settle,
                overshoot_fraction=float(max(0.,np.max(y)-1)),
                final_gain=float(np.mean(y[t>=t[-1]-.3])),
                absolute_error_integral_s=float(np.trapz(abs(y-1),t)))


def extract_steps(data, axis, signal_name, dwell=2.5, minimum_step=.12):
    t=data['time_s'];u=data['command_'+axis];y=data[signal_name+'_'+axis]
    if not np.all(np.isfinite(np.r_[t,u,y])) or np.any(np.diff(t)<=0):
        raise ValueError('Expected finite values and strictly increasing time')
    dt=float(np.median(np.diff(t)));smooth=causal_average(y,max(1,round(.05/dt)))
    grid=np.arange(0,dwell-.1,.01)
    steps=[]
    for block in range(1,13):
        relative=t-block*dwell
        before=(relative>=-.4)&(relative<-.1)
        after=(relative>=dwell-.5)&(relative<dwell-.1)
        if not np.any(before) or not np.any(after):continue
        amplitude=float(np.mean(u[after])-np.mean(u[before]))
        if abs(amplitude)<minimum_step:continue
        base=float(np.mean(smooth[before]));wave=(np.interp(grid,relative,smooth)-base)/amplitude
        command=(np.interp(grid,relative,u)-np.mean(u[before]))/amplitude
        steps.append(dict(block=block,amplitude_N=amplitude,wave=wave,command=command))
    return grid,steps


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('analysis',help='Directory with results.json and per-Bee excitation CSVs')
    p.add_argument('--output',required=True);p.add_argument('--dwell',type=float,default=2.5)
    a=p.parse_args();source=Path(a.analysis);out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    source_result=json.loads((source/'results.json').read_text());results=[]
    for axis in ['normal','up','side']:
        paths=sorted(source.glob('*_excite_'+axis+'.csv'))
        for signal_name in ['observer','sim_site_force']:
            entries=[];curves=[];commands=[]
            for path in paths:
                data=np.genfromtxt(path,delimiter=',',names=True)
                if signal_name+'_'+axis not in data.dtype.names:continue
                t,steps=extract_steps(data,axis,signal_name,a.dwell)
                for s in steps:
                    metric=response_metrics(t,s['wave'])
                    entries.append(dict(bee=path.name.split('_')[0],block=s['block'],amplitude_N=s['amplitude_N'],**metric))
                    curves.append(s['wave']);commands.append(s['command'])
            if not curves:continue
            median=np.median(curves,axis=0);command=np.median(commands,axis=0)
            metric=response_metrics(t,median)
            result=dict(axis=axis,signal=signal_name,steps=len(entries),median_step=metric,
                individual_settled_fraction=float(np.mean([m['settling_10pct_s'] is not None for m in entries])),
                individual_steps=entries,median_command=response_metrics(t,command))
            results.append(result)
            np.savetxt(out/f'{axis}_{signal_name}.csv',np.column_stack([t,command,median,np.percentile(curves,10,axis=0),np.percentile(curves,90,axis=0)]),delimiter=',',header='time_s,command,median,p10,p90',comments='')
    doc=dict(source=str(source.resolve()),bag=source_result['bag'],bag_sha256=source_result['bag_sha256'],
        measurement='50 ms causal average; baseline removed per step; median of full-amplitude (>=0.12N) steps. Site includes foot mass/inertia.',
        criterion='Earliest time after which median response stays within +/-10% of commanded change until the end of the 2.4s window, at least 0.3s. Null = not demonstrated in this window.',results=results)
    (out/'results.json').write_text(json.dumps(doc,indent=2,allow_nan=False)+'\n')
    (out/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    for r in results:print(r['axis'],r['signal'],r['median_step'],'individual settled',r['individual_settled_fraction'])
    return 0 if results else 1


if __name__=='__main__':raise SystemExit(main())
