#!/usr/bin/env python3
"""Aggregate all attempts; never silently drop failures from success counts."""
import argparse
from collections import Counter
import json
from pathlib import Path
import statistics


def summarize(root):
    root = Path(root)
    results = json.loads((root/'results.json').read_text())
    protocol = json.loads((root/'protocol.json').read_text())
    summary = {}
    for shape in protocol['conditions']:
        runs = [r for r in results if r['shape']==shape]
        successful = [r for r in runs if r['result']=='Complete']
        errors = [r['successful_placement_error_m']*1000 for r in successful]
        metrics = {}
        for key in ('mean_contact_error_m', 'relative_contact_deviation_m', 'object_tilt_deg',
                    'normal_vertical_N_per_bee', 'tangent_vertical_N_per_bee', 'command_tangent_normal_ratio',
                    'normal_force_N', 'normal_z'):
            available = [r['measurements'][key] for r in runs if key in r.get('measurements',{})]
            if available:
                metrics[key] = dict(mean_of_run_means=statistics.mean(v['mean'] for v in available),
                                    worst_maximum=max(v['maximum'] for v in available), runs=len(available))
        summary[shape] = dict(attempts=len(runs), complete=len(successful),
            started=sum(any(p['name']=='Staging' for p in r.get('phases',[])) for r in runs),
            preloaded=sum(any(p['name']=='Preload' for p in r.get('phases',[])) for r in runs),
            destination_reached=sum(any(p['name']=='DestinationSettle' for p in r.get('phases',[])) for r in runs),
            transported=sum(r.get('reached_transport',False) for r in runs),
            released=sum(r.get('released',False) for r in runs),
            failures=dict(Counter(r['reason'] or r['result'] for r in runs if r['result']!='Complete')),
            placement_error_mm=dict(mean=statistics.mean(errors), median=statistics.median(errors),
                                    minimum=min(errors), maximum=max(errors)) if errors else None,
            eigen_asserts=sum(r.get('eigen_assert',False) for r in runs), metrics=metrics)
    (root/'aggregate.json').write_text(json.dumps(summary, indent=2)+'\n')
    for shape, data in summary.items():
        print(shape, '%d/%d' % (data['complete'],data['attempts']),
              'error_mm=', data['placement_error_mm'], 'failures=', data['failures'])
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root')
    summarize(parser.parse_args().root)
