#!/usr/bin/env python3
"""Read-only force-response identification from simulation OR hardware rosbag.

No rospy publishers, services, arm, or motor commands. Models characterize the
complete command-to-filtered-observer path, including inner control and filtering.
"""
import argparse
import csv
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import platform

import numpy as np
import rosbag
from grasp_reference_axes import bag_force_command, reference_topic
import scipy
import yaml
from force_response_model import causal_sample,fit_response,ordered_unique_samples


def vec(v):return np.array([v.x,v.y,v.z],dtype=float)


def fixed_basis(normal):
    n=normal/np.linalg.norm(normal);up=np.array([0.,0.,1.])-n[2]*n
    if np.linalg.norm(up)<1e-6:raise ValueError('Use an explicitly configured basis for a vertical normal')
    up/=np.linalg.norm(up)
    return np.array([-n,up,np.cross(up,n)])


def unique_sorted(rows):
    # Source stamp, not bag arrival time, defines signal timing. Status messages
    # can repeat while /clock is paused. Only valid measured samples reach here.
    return ordered_unique_samples(rows)


def file_hash(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as source:
        for chunk in iter(lambda:source.read(1024*1024),b''):digest.update(chunk)
    return digest.hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag');p.add_argument('--output',required=True)
    p.add_argument('--config',help='YAML with robot_names, metadata and optional segments')
    p.add_argument('--sample-period',type=float,default=.01)
    a=p.parse_args()
    if not .002<=a.sample_period<=.05:raise ValueError('Invalid sample period')
    config=yaml.safe_load(Path(a.config).read_text()) if a.config else {}
    config=config or {};names=config.get('robot_names',['bee1','bee2','bee3','bee4'])
    out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=False)
    (out/'analysis_sources').mkdir()
    for filename in ['identify_grasp_force_response.py','force_response_model.py','grasp_reference_axes.py']:
        (out/'analysis_sources'/filename).write_bytes((Path(__file__).resolve().parent/filename).read_bytes())
    (out/'analysis_config.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    refs={n:[] for n in names};measurements={n:[] for n in names};events=[]
    rejected={n:{'references':0,'measurements':0} for n in names}
    topics=['/bee_force_identification/segment']
    with rosbag.Bag(a.bag) as bag:
        available = bag.get_type_and_topic_info().topics
        for n in names:
            current = f'/{n}/controller/tracking_status'
            topics.extend([reference_topic(n, available),
                           current if current in available else f'/{n}/grasp/tracking_status'])
        bag_start=bag.get_start_time()
        for topic,m,stamp in bag.read_messages(topics=topics):
            if topic.endswith('/segment'):
                events.append(json.loads(m.data));continue
            name=topic.split('/')[1];t=m.header.stamp.to_sec()
            if topic.endswith(('/command','/reference')):
                cmd = bag_force_command(m)
                if cmd is None or not cmd[3] or m.header.frame_id.lstrip('/') != 'world':
                    rejected[name]['references'] += 1
                    continue
                force, normal, _, _ = cmd
                refs[name].append((t,force,normal,bool(m.force_integral_enabled)))
            else:
                good=(m.ready and m.active and m.calibrated and not m.fault
                    and m.header.frame_id.lstrip('/')=='world' and np.linalg.norm(vec(m.external_force))>.1)
                if not good:rejected[name]['measurements']+=1;continue
                # Observer is force ON Bee; commands are force BY Bee on object.
                measurements[name].append((t,-vec(m.external_force)))
    segments=config.get('segments',[])
    explicit_segments=bool(segments)
    if not segments:
        seen=set()
        for event in events:
            if event.get('state')!='excite':continue
            key=(event['axis'],event['axis_start'])
            if key in seen:continue
            seen.add(key)
            segments.append({'name':event['axis'],'axis':event['axis'],
                'start':event['axis_start'],'duration':13*event['dwell'],
                'validation_after':event['validation_from_block']*event['dwell'],
                'integral_enabled':event['integral_enabled']})
    if not segments:raise ValueError('No experiment markers; supply explicit segments in --config')
    if explicit_segments and config.get('time_origin','absolute')=='bag_start':
        segments=[dict(s,start=s['start']+bag_start) for s in segments]
    results=[];plots={n:[] for n in names}
    for name in names:
        rr=unique_sorted(refs[name]);yy=unique_sorted(measurements[name])
        if not rr or not yy:raise ValueError('Missing valid data for '+name)
        rt=np.array([r[0] for r in rr]);rv=np.array([r[1] for r in rr]);rn=np.array([r[2] for r in rr]);ri=np.array([r[3] for r in rr])
        yt=np.array([r[0] for r in yy]);yv=np.array([r[1] for r in yy])
        for segment in segments:
            start=float(segment['start']);end=start+float(segment['duration'])
            times=np.arange(start,end-a.sample_period/2,a.sample_period)
            command,valid=causal_sample(rt,rv,times)
            normal,normal_valid=causal_sample(rt,rn,times)
            integral,mode_valid=causal_sample(rt,ri,times)
            _,measure_valid=causal_sample(yt,yv,times)
            valid&=normal_valid&mode_valid&measure_valid
            if not np.all(valid):raise ValueError(f'{name}/{segment["name"]}: stale/missing samples; split the window')
            if np.any(integral!=bool(segment.get('integral_enabled',True))):
                raise ValueError('Do not mix P-only and PI force modes in one fit')
            normal/=np.linalg.norm(normal,axis=1)[:,None]
            b=fixed_basis(normal[0]);command=command@b.T
            observed=np.column_stack([np.interp(times,yt,yv[:,j]) for j in range(3)])@b.T
            angle=float(np.degrees(np.arccos(np.clip(normal@normal[0],-1,1))).max())
            axis=['normal','up','side'].index(segment['axis'])
            result,predicted=fit_response(times-start,command[:,axis],observed[:,axis],
                float(segment['validation_after']),config.get('minimum_input_range_N',.08))
            periods={}
            for label,source_times in [('reference',rt),('observer',yt)]:
                intervals=np.diff(source_times[(source_times>=start)&(source_times<end)])
                periods[label]={'median':float(np.median(intervals)),'maximum':float(np.max(intervals))}
            resolution=max(a.sample_period,periods['reference']['median'],periods['observer']['median'])
            if 'delay_s' in result:
                result['delay_resolution']=('below_one_source_sample_resolution' if result['delay_s']<resolution
                    else 'resolved_within_model')
            result.update(source_sample_intervals_s=periods,effective_sampling_resolution_s=resolution)
            if result.get('tau_s',resolution)<resolution:
                result['reasons'].append('time_constant_below_source_sampling_resolution')
            others=[j for j in range(3) if j!=axis]
            ranges=np.percentile(command,95,axis=0)-np.percentile(command,5,axis=0)
            if angle>5:result['reasons'].append('contact_frame_changed_more_than_5_deg')
            if max(ranges[others])>.5*max(ranges[axis],.001):result['reasons'].append('strong_other_axis_excitation')
            if any(e.get('state')=='rejected' and start<=e['time']<end for e in events):
                result['reasons'].append('experiment_input_rejected_by_margin_check')
            result['accepted']=not result['reasons']
            result.update(bee=name,axis=segment['axis'],segment=segment['name'],
                absolute_start_s=start,duration_s=segment['duration'],
                frame_basis_world=b.tolist(),maximum_normal_change_deg=angle,
                cross_axis_input_ranges_N=ranges[others].tolist(),
                force_integral_enabled=bool(segment.get('integral_enabled',True)))
            filename=f'{name}_{segment["name"]}'
            with (out/(filename+'.csv')).open('w') as f:
                writer=csv.writer(f);writer.writerow(['time_s','command_N','observer_N','predicted_N','validation'])
                for k,t in enumerate(times-start):
                    writer.writerow([t,command[k,axis],observed[k,axis],
                        predicted[k] if predicted is not None else '',int(t>=segment['validation_after'])])
            results.append(result)
            plots[name].append((segment,times-start,command[:,axis],observed[:,axis],predicted,result))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for name,items in plots.items():
        fig,axes=plt.subplots(len(items),1,figsize=(11,2.8*len(items)),squeeze=False)
        for ax,item in zip(axes[:,0],items):
            seg,t,u,y,pred,r=item
            ax.plot(t,u,color='black',lw=1,label='command')
            ax.plot(t,y,color='#16817a',lw=1,label='filtered observer')
            if pred is not None:ax.plot(t,pred,color='#e58b2a',lw=1,label='model (free run)')
            ax.axvspan(seg['validation_after'],t[-1],color='#6688cc',alpha=.12,label='held-out validation')
            ax.set(title=f'{name}: {seg["axis"]} | '+('accepted' if r['accepted'] else ', '.join(r['reasons'])),ylabel='Force [N]',xlabel='Time from segment start [s]')
            ax.grid(alpha=.2);ax.legend(loc='best',fontsize=8,ncol=4)
        fig.tight_layout();fig.savefig(out/(name+'_response.png'),dpi=140);plt.close(fig)
    recording_metadata={}
    for filename in ['session.json','summary.json']:
        adjacent=Path(a.bag).resolve().parent/filename
        if adjacent.is_file():recording_metadata[filename]=json.loads(adjacent.read_text())
    result={'schema_version':1,'created':datetime.now(timezone.utc).isoformat(),
        'bag':str(Path(a.bag).resolve()),'bag_sha256':file_hash(a.bag),
        'recording_metadata':recording_metadata,
        'software_versions':{'python':platform.python_version(),'numpy':np.__version__,
            'scipy':scipy.__version__,'matplotlib':matplotlib.__version__,'pyyaml':yaml.__version__},
        'analysis_source_sha256':{name:file_hash(Path(__file__).resolve().parent/name)
            for name in ['identify_grasp_force_response.py','force_response_model.py']},
        'metadata':config.get('metadata',{}),'excluded_samples':rejected,
        'model_scope':'Command to filtered observer, including closed-loop force control and observer filters; not isolated true contact dynamics.',
        'not_applied_to_controller':True,'models':results}
    (out/'results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (out/'model.yaml').write_text(yaml.safe_dump(result,sort_keys=False))
    for r in results:print(r['bee'],r['axis'],'accepted',r['accepted'],'tau',r.get('tau_s'),'delay',r.get('delay_s'),'t95',r.get('model_t95_s'),'validation',r.get('validation'),r['reasons'],flush=True)
    return 0


if __name__=='__main__':raise SystemExit(main())
