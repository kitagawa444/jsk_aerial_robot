#!/usr/bin/env python3
"""Reproducible force test suite: synthetic contract, known-force observer, contact-force control.

Only dedicated localhost ROS masters are started. For hardware use the passive recorder
and offline analyzer instead; this runner never connects to a hardware master.
"""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import subprocess
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True)
    p.add_argument('--checks',nargs='+',choices=['contract','observer','control'],default=['contract','observer','control'])
    p.add_argument('--port',type=int,default=12971)
    p.add_argument('--observer-rmse-limit',type=float,default=.2)
    p.add_argument('--control-rmse-limit',type=float,default=.5)
    a=p.parse_args()
    if not 1024<=a.port<=65534 or 11311 in (a.port,a.port+1):p.error('Use dedicated simulation ports, never 11311')
    root=Path(a.output).resolve();root.mkdir(parents=True,exist_ok=False)
    bee=Path(__file__).resolve().parents[1];workspace=bee.parents[3]
    summary={'started_utc':datetime.now(timezone.utc).isoformat(),'checks':[],'hardware':False}
    def run(name,command):
        with (root/(name+'.log')).open('w') as log:
            result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
        summary['checks'].append({'name':name,'returncode':result.returncode,'command':command})
        (root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(name,'PASS' if result.returncode==0 else 'FAIL',flush=True)
        return result.returncode==0
    for check in dict.fromkeys(a.checks):
        if check=='contract':
            run('contract',[sys.executable,str(bee/'test/test_observer_calibration_ros.py'),
                '--native',str(workspace/'devel/lib/bee/reference_tracker_harness'),'--output',str(root/'contract')])
        elif check=='observer':
            if run('observer-run',[sys.executable,str(bee/'test/run_observer_force_test.py'),
                    '--output',str(root/'observer'),'--port',str(a.port)]):
                run('observer-audit',[sys.executable,str(bee/'script/analyze_force_validation.py'),
                    str(root/'observer/flight.bag'),'--mode','observer','--output',str(root/'observer-audit'),
                    '--rmse-limit',str(a.observer_rmse_limit)])
        else:
            if run('control-run',[sys.executable,str(bee/'test/run_force_identification.py'),
                    '--output',str(root/'control'),'--port',str(a.port+1),
                    '--amplitude','.16','--dwell','2.5','--settle-time','15','--timeout','480']):
                run('control-audit',[sys.executable,str(bee/'script/analyze_force_validation.py'),
                    str(root/'control/flight.bag'),'--mode','control','--output',str(root/'control-audit'),
                    '--rmse-limit',str(a.control_rmse_limit)])
                if run('response-fit',[sys.executable,str(bee/'script/identify_grasp_force_response.py'),
                        str(root/'control/flight.bag'),'--output',str(root/'response-fit')]):
                    models=json.loads((root/'response-fit/results.json').read_text())['models']
                    rejected=[{'bee':m['bee'],'axis':m['axis'],'reasons':m['reasons']} for m in models if not m['accepted']]
                    summary['checks'].append({'name':'response-fit-quality',
                        'returncode':int(len(models)!=12 or bool(rejected)),
                        'models':len(models),'rejected':rejected})
    summary['finished_utc']=datetime.now(timezone.utc).isoformat()
    summary['passed']=all(c['returncode']==0 for c in summary['checks'])
    (root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return int(not summary['passed'])


if __name__=='__main__':raise SystemExit(main())
