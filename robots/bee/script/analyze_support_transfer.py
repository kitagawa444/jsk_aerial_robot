#!/usr/bin/env python3
"""Read-only support-transfer timing and force-capacity audit from a ROS bag.

Uses each Bee's filtered observer, not the central estimator's internal state or
ground-truth contact force. No ROS master, publishers or services are used.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag
from grasp_reference_axes import bag_force_command, reference_topic
import yaml
from force_response_model import causal_sample,ordered_unique_samples


def vec(v):return np.array([v.x,v.y,v.z],dtype=float)


def components(force,normal):
    normal=np.asarray(normal,dtype=float)
    normal=normal/np.linalg.norm(normal,axis=-1,keepdims=True)
    up=np.array([0.,0.,1.])-normal[...,2,None]*normal
    if np.any(np.linalg.norm(up,axis=-1)<1e-6):raise ValueError('Vertical normals require a configured tangent basis')
    up/=np.linalg.norm(up,axis=-1,keepdims=True)
    side=np.cross(up,normal)
    return np.stack([-np.sum(force*normal,axis=-1),np.sum(force*up,axis=-1),np.sum(force*side,axis=-1)],axis=-1)


def capacity_margin(normal_observed,tangent_command,mu,normal_error,tangent_error):
    return mu*np.maximum(0.,normal_observed-normal_error)-tangent_error-np.abs(tangent_command).sum(axis=-1)


def first_sustained(times,condition,duration):
    start=None
    for t,valid in zip(times,condition):
        if not valid:start=None
        elif start is None:start=t
        if start is not None and t-start>=duration-1e-9:return float(start)
    return None


def incremental_prediction(times,commands,initial,models,start):
    """No refit: initialize at last pre-transfer output, retain input history."""
    index=int(np.searchsorted(times,start));prediction=np.full_like(commands,np.nan)
    baseline=commands[max(0,index-1)].copy();state=np.zeros(3)
    prediction[index]=initial
    for k in range(index+1,len(times)):
        dt=times[k]-times[k-1]
        for axis,model in enumerate(models):
            delayed=float(np.interp(times[k-1]-model['delay_s'],times,commands[:,axis]))
            a=np.exp(-dt/model['tau_s'])
            state[axis]=a*state[axis]+(1-a)*model['gain']*(delayed-baseline[axis])
        prediction[k]=initial+state
    return prediction


def stat(values):
    values=np.asarray(values);values=values[np.isfinite(values)]
    if not len(values):return None
    return {'minimum':float(values.min()),'maximum':float(values.max()),'mean':float(values.mean()),'samples':len(values)}


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def analyze(bag_path,names,mass,config,output,model_path=None):
    out=Path(output);out.mkdir(parents=True,exist_ok=False)
    refs={n:[] for n in names};observations={n:[] for n in names}
    excluded={n:0 for n in names};phases=[];metrics=[];preload=[]
    topics=['/bee_cooperative_grasp/phase','/bee_cooperative_grasp/metrics','/bee_cooperative_grasp/preload_diagnostics']
    with rosbag.Bag(str(bag_path)) as bag:
        available = bag.get_type_and_topic_info().topics
        for n in names:
            current = f'/{n}/controller/tracking_status'
            topics.extend([reference_topic(n, available),
                           current if current in available else f'/{n}/grasp/tracking_status'])
        bag_end=bag.get_end_time()
        for topic,m,stamp in bag.read_messages(topics=topics):
            t=stamp.to_sec()
            if topic.endswith('/phase'):
                if not phases or phases[-1]['name']!=m.data:phases.append(dict(name=m.data,time=t))
            elif topic.endswith('/metrics'):
                if len(m.data)>=18:metrics.append((t,np.array(m.data)))
            elif topic.endswith('/preload_diagnostics'):
                if len(m.data)==6*len(names):preload.append((t,np.array(m.data).reshape(-1,6)))
            else:
                name=topic.split('/')[1];t=m.header.stamp.to_sec()
                if topic.endswith(('/command','/reference')):
                    cmd = bag_force_command(m)
                    if cmd is not None and m.header.frame_id.lstrip('/') == 'world':
                        force, n, _, active = cmd
                        refs[name].append((t,np.r_[force,n,float(active),float(m.force_integral_enabled)]))
                elif m.ready and m.active and m.calibrated and not m.fault and m.header.frame_id.lstrip('/')=='world' and np.linalg.norm(vec(m.external_force))>.1:
                    observations[name].append((t,-vec(m.external_force)))
                else:excluded[name]+=1
    for name in names:
        refs[name]=ordered_unique_samples(refs[name]);observations[name]=ordered_unique_samples(observations[name])
        if not refs[name] or not observations[name]:raise ValueError('Missing valid reference or observer samples for '+name)
    starts={n:next((r[0] for r in refs[n] if r[1][6]),None) for n in names}
    if any(v is None for v in starts.values()):raise ValueError('Not all Bees reached vector support tracking')
    start=max(starts.values())
    ends=[p for p in phases if p['time']>start and p['name'] in ('Lift','Aborted','Complete')]
    if not ends:raise ValueError('Support transfer did not end; use a complete or aborted recording')
    end=ends[0]['time'];completed=ends[0]['name']=='Lift'
    begin=max(start-4.,max(refs[n][0][0] for n in names),max(observations[n][0][0] for n in names))
    finish=min(end+(1. if completed else 0.),bag_end,min(observations[n][-1][0] for n in names))
    dt=.01;times=np.arange(begin,finish-dt/2,dt);active=(times>=start)&(times<end)
    if np.count_nonzero(active)<10:raise ValueError('Insufficient support-transfer samples')
    model_data=yaml.safe_load(Path(model_path).read_text()) if model_path else None
    per_bee={};curves={};all_valid=np.ones(len(times),dtype=bool)
    total_command=np.zeros(len(times));total_observed=np.zeros(len(times))
    mu=config['allocation_mu'];en=config['allocation_observer_normal_error']
    et=config['allocation_tangent_up_error']+config['allocation_tangent_side_error']
    for name in names:
        rt=np.array([r[0] for r in refs[name]]);rv=np.array([r[1] for r in refs[name]])
        yt=np.array([r[0] for r in observations[name]]);yv=np.array([r[1] for r in observations[name]])
        ref,valid_r=causal_sample(rt,rv,times,max_age=.1)
        observed,valid_y=causal_sample(yt,yv,times,max_age=.1)
        valid=valid_r&valid_y;all_valid&=valid
        command=components(ref[:,:3],ref[:,3:6]);measured=components(observed,ref[:,3:6])
        margin=capacity_margin(measured[:,0],command[:,1:],mu,en,et)
        commanded_margin=capacity_margin(command[:,0],command[:,1:],mu,config['allocation_normal_error'],et)
        observed_margin=capacity_margin(measured[:,0],measured[:,1:],mu,en,et)
        required_normal=en+(np.abs(command[:,1:]).sum(axis=1)+et)/mu
        measure_index=np.maximum(0,np.searchsorted(yt,times,side='right')-1)
        age=times-yt[measure_index]
        evaluate=active&valid&(ref[:,6]>.5)
        result={'scope':'Local filtered-observer diagnostic; not an exact audit of the central QP internal normal estimate.',
            'coverage':float(evaluate.sum()/active.sum()),
            'normal_observed_N':stat(measured[evaluate,0]),'normal_command_N':stat(command[evaluate,0]),
            'required_normal_for_current_support_N':stat(required_normal[evaluate]),
            'local_observer_capacity_margin_N':stat(margin[evaluate]),
            'commanded_friction_margin_N':stat(commanded_margin[evaluate]),
            'observed_friction_margin_N':stat(observed_margin[evaluate]),
            'negative_capacity_duration_s':float(np.sum(evaluate&(margin< -1e-3))*dt),
            'up_command_minus_observed_N':stat((command-measured)[evaluate,1]),
            'side_command_minus_observed_N':stat((command-measured)[evaluate,2]),
            'normal_command_minus_observed_N':stat((command-measured)[evaluate,0]),
            'observer_age_s':stat(age[evaluate])}
        prediction=np.full_like(command,np.nan)
        before=np.flatnonzero((times<start)&valid)
        first = np.flatnonzero(evaluate)
        if model_data and (len(before) or len(first)):
            initial = before[-1] if len(before) else first[0]
            prediction_start = start if len(before) else times[initial]
            # Use the median across the four identified box Bees; do not fit
            # a new gain/offset to this mission or treat another shape as box.
            models=[]
            for axis in ['normal','up','side']:
                source=[axes[axis] for axes in model_data['models']['p'].values()]
                models.append({key:float(np.median([r[key] for r in source])) for key in ['gain','tau_s','delay_s']})
            prediction=incremental_prediction(times,command,measured[initial],models,prediction_start)
            model_valid=evaluate&(ref[:,7]<.5)&np.isfinite(prediction).all(axis=1)
            residual=prediction[model_valid]-measured[model_valid]
            result['box_p_model_extrapolation']={'scope':'Incremental prediction, no refit; larger force changes and possibly different contact geometry from identification.',
                'initial_observation_s':float(times[initial]),'model_parameters':models,'samples':int(model_valid.sum()),
                'rmse_N_by_axis':np.sqrt(np.mean(residual**2,axis=0)).tolist() if model_valid.any() else None}
            prediction[ref[:,7]>.5]=np.nan  # identified P model is not a PI prediction
        per_bee[name]=result
        curves[name]=(command,measured,required_normal,margin,prediction,valid)
        total_command+=ref[:,2];total_observed+=observed[:,2]
        columns=np.column_stack([times-start,command,measured,required_normal,margin,commanded_margin,observed_margin,age,valid.astype(int),active.astype(int),prediction])
        with (out/(name+'.csv')).open('w') as f:
            writer=csv.writer(f);writer.writerow(['time_from_support_s','normal_command_N','up_command_N','side_command_N',
                'normal_observed_N','up_observed_N','side_observed_N','normal_required_N','local_capacity_margin_N',
                'commanded_friction_margin_N','observed_friction_margin_N','observer_age_s','valid','support_transfer',
                'normal_model_N','up_model_N','side_model_N']);writer.writerows(columns)
    gravity = config.get('gravity', 9.81)  # Legacy recordings.
    for filename in ('session.json', 'summary.json'):
        snapshot = Path(bag_path).parent / filename
        if snapshot.exists():
            metadata = json.loads(snapshot.read_text())
            central = metadata.get('parameters', {}).get('/bee_cooperative_grasp/controller', {})
            gravity = central.get('gravity', metadata.get('gravity', gravity))
    valid=active&all_valid;weight=mass*gravity
    command90=first_sustained(times-start,valid&(total_command>=.9*weight),.1)
    observed90=first_sustained(times-start,valid&(total_observed>=.9*weight),.1)
    full_progress=next((t-start for t,m in metrics if t>=start and m[7]>=1-1e-6),None)
    end_preload=[row for row in preload if row[0]<=start+.03]
    preload_result=None
    if end_preload:
        last=end_preload[-1];tail=last[0]
        for t,a in reversed(end_preload):
            if not np.all((a[:,1]>0)&(a[:,2]>=.9*a[:,1])&(a[:,3]>=-1e-6)&(a[:,5]>=-1e-5)):break
            tail=t
        preload_result={'last_record_time_s':last[0],'continuous_ready_tail_s':last[0]-tail,
            'columns':['planned_normal_N','ramped_normal_N','central_observed_normal_N','planned_support_margin_N','old_projection_gap_N','same_plan_gap_N'],
            'last_values':last[1].tolist(),'note':'Default task preload ratio 0.9 and duration 3 s; raw diagnostics retained.'}
    checks={'transfer_completed':completed,'complete_force_data':all(r['coverage']>=.99 for r in per_bee.values()),
        'local_observer_capacity_nonnegative':all(r['local_observer_capacity_margin_N'] and r['local_observer_capacity_margin_N']['minimum']>=-1e-3 for r in per_bee.values()),
        'commanded_friction_margin_nonnegative':all(r['commanded_friction_margin_N'] and r['commanded_friction_margin_N']['minimum']>=-1e-4 for r in per_bee.values())}
    result={'scope':'Support transfer command/filtered-observer comparison; observer bias and true-contact-force separation are not validated.',
        'bag':str(Path(bag_path).resolve()),'bag_sha256':sha(bag_path),'names':names,'object_mass_kg':mass,
        'configuration':config,'sample_period_s':dt,'phases':phases,'per_bee_support_start_s':starts,
        'transfer_start_s':start,'transfer_end_s':end,'transfer_duration_s':end-start,
        'support_progress_reached_one_s':full_progress,
        'wait_after_full_progress_s':end-start-full_progress if full_progress is not None else None,
        'checks':checks,'per_bee':per_bee,'preload':preload_result,'excluded_observer_samples':excluded,
        'total_vertical_command_minus_observed_N':stat((total_command-total_observed)[valid]),
        'time_to_90percent_weight_command_s':command90,'time_to_90percent_weight_observed_s':observed90,
        'observed_90percent_lag_s':observed90-command90 if observed90 is not None and command90 is not None else None,
        'central_gate_note':'support_force_lead bounds progress using the central filtered wrench. Local filtered-observer lead here is a different diagnostic, not an exact assertion of that internal limit.',
        'model_file':str(Path(model_path).resolve()) if model_path else None,
        'model_sha256':sha(model_path) if model_path else None,
        'analysis_sha256':sha(__file__)}
    summary=Path(bag_path).resolve().parent/'summary.json'
    if summary.exists():result['mission']=json.loads(summary.read_text())
    (out/'results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (out/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    (out/'force_response_model.py').write_bytes((Path(__file__).parent/'force_response_model.py').read_bytes())
    (out/'grasp_reference_axes.py').write_bytes((Path(__file__).parent/'grasp_reference_axes.py').read_bytes())
    with (out/'total.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['time_from_support_s','vertical_command_N','vertical_observed_N','valid','support_transfer'])
        writer.writerows(np.column_stack([times-start,total_command,total_observed,all_valid.astype(int),active.astype(int)]))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    x=times-start
    fig,axes=plt.subplots(len(names),3,figsize=(16,2.8*len(names)),squeeze=False)
    for row,name in enumerate(names):
        command,observed,needed,margin,prediction,good=curves[name]
        for j in range(3):
            ax=axes[row,j]
            ax.plot(x,np.where(good,command[:,j],np.nan),label='command',color='black')
            ax.plot(x,np.where(good,observed[:,j],np.nan),label='filtered observer',color='#16817a')
            if j==0:ax.plot(x,needed,label='normal needed for commanded support',color='#bb5232',ls='--')
            if model_data and j>0:ax.plot(x,prediction[:,j],label='box P model (extrapolated)',color='#b08729',alpha=.8,ls=':')
            ax.axvspan(0,end-start,color='#6688cc',alpha=.1);ax.axvline(0,color='grey',lw=.7)
            ax.set(title=name+' / '+['normal','tangent up','tangent side'][j],xlabel='Time from support start [s]',ylabel='Force [N]')
            ax.grid(alpha=.2)
            if row==0:ax.legend(fontsize=7,loc='best')
    fig.tight_layout();fig.savefig(out/'forces.png',dpi=140);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(11,9))
    axes[0].plot(x,np.where(all_valid,total_command,np.nan),label='sum of commanded Fz',color='black')
    axes[0].plot(x,np.where(all_valid,total_observed,np.nan),label='sum of observed Fz',color='#16817a')
    axes[0].axhline(weight,label='object weight',ls='--',color='grey');axes[0].set_ylabel('Vertical force [N]')
    for name in names:axes[1].plot(x,np.where(curves[name][-1],curves[name][3],np.nan),label=name)
    axes[1].axhline(0,ls='--',color='red');axes[1].set_ylabel('Local observer capacity margin [N]')
    if metrics:
        mt=np.array([r[0] for r in metrics]);mv=np.array([r[1] for r in metrics]);sel=(mt>=begin)&(mt<=finish)
        axes[2].plot(mt[sel]-start,mv[sel,7],label='support progress');axes[2].set_ylabel('Progress [0..1]')
    for ax in axes:
        ax.axvspan(0,end-start,color='#6688cc',alpha=.1);ax.grid(alpha=.2);ax.set_xlabel('Time from support start [s]')
        if ax.get_legend_handles_labels()[0]:ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(out/'support.png',dpi=140);plt.close(fig)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag');p.add_argument('--names',nargs='+',required=True);p.add_argument('--mass',type=float,required=True)
    p.add_argument('--controller-config',required=True);p.add_argument('--model');p.add_argument('--output',required=True)
    args=p.parse_args();config=yaml.safe_load(Path(args.controller_config).read_text())['controller']
    result=analyze(args.bag,args.names,args.mass,config,args.output,args.model)
    print(json.dumps({k:result[k] for k in ['checks','transfer_duration_s','total_vertical_command_minus_observed_N','observed_90percent_lag_s']},indent=2))
    return int(not all(result['checks'].values()))


if __name__=='__main__':raise SystemExit(main())
