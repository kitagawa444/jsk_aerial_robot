#!/usr/bin/env python3
"""Read-only oscillation audit of force, velocity and compensation in a ROS bag.

Optional MuJoCo site forces are NOT pure object contact forces: foot mass and
inertia are included. Compare their AC components, not absolute force bias.
Closed-loop spectra/coherence cannot identify causal loop stability margins.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy import signal
import rosbag
from grasp_reference_axes import bag_force_command, reference_topic
import yaml
from identify_grasp_force_response import vec, fixed_basis, file_hash, unique_sorted


def spectrum_metrics(y, dt):
    """Linear detrending; PSD integrals are variances, not sinusoid amplitudes."""
    y = np.asarray(y)
    z = signal.detrend(y)
    f, power = signal.welch(z, fs=1/dt, nperseg=min(len(y), round(8/dt)))
    df = f[1]-f[0]
    metrics = dict(mean=float(np.mean(y)), rms_ac=float(np.std(z)),
                   peak_to_peak=float(np.ptp(y)), percentile_span=float(np.percentile(y,99)-np.percentile(y,1)))
    for low, high in [(.1,.5),(.5,2),(2,8)]:
        k = (f>=low)&(f<high)
        metrics[f'rms_{low:g}_{high:g}_Hz'] = float(np.sqrt(np.sum(power[k])*df))
    k = (f>=.5)&(f<8)
    metrics['peak_0.5_8_Hz'] = float(f[k][np.argmax(power[k])]) if np.sum(power[k])*df>1e-16 else None
    return metrics, f, power


def interpolate(rows, times, causal=False):
    rows = unique_sorted(rows)
    ts = np.array([r[0] for r in rows]);values = np.array([r[1] for r in rows])
    if not np.all(np.isfinite(values)) or not np.all(np.isfinite(ts)):
        raise ValueError('Nonfinite source data')
    ids = np.searchsorted(ts,times,side='right')-1
    if np.any(ids<0) or np.max(times-ts[np.maximum(ids,0)])>.15:
        raise ValueError('Missing/stale source samples; split the window')
    intervals = np.diff(ts[(ts>=times[0])&(ts<=times[-1])])
    if len(intervals)<2 or np.max(intervals)>.15:
        raise ValueError('Insufficient samples or source gap >150 ms')
    if causal: y=values[ids]
    else: y=np.column_stack([np.interp(times,ts,values[:,j]) for j in range(values.shape[1])])
    return y,dict(median=float(np.median(intervals)),maximum=float(np.max(intervals)))


def windows(phases, events, end):
    result=[]
    for i,(t,name) in enumerate(phases):
        stop=phases[i+1][0] if i+1<len(phases) else end
        if name=='Hold':
            first=min([e['axis_start'] for e in events if e.get('state')=='excite' and e['axis_start']>=t]+[stop])
            if first-t>=15:result.append(dict(name='hold',start=max(t+5,first-30),end=first-.1))
        elif name=='Transport' and stop-t>12:
            result.append(dict(name='transport',start=t+3,end=stop-1))
    seen=set()
    for e in events:
        if e.get('state')=='excite' and e['axis'] not in seen:
            seen.add(e['axis']);result.append(dict(name='excite_'+e['axis'],start=e['axis_start'],end=e['axis_start']+13*e['dwell']-.1))
    return result


def pulse_metrics(times, command, measured, dwell):
    """Compare plateau tails after steps; remove per-block DC/bias only.

    RMS is not a settling-time or damping-ratio estimate. Input slew and all
    closed-loop mechanics still contribute. Exclude the first, unchanged block.
    """
    tails=[];overshoots=[]
    for block in range(1,13):
        t=times-block*dwell
        before=(t>=-.4)&(t<-.05)
        final=(t>=dwell-.5)&(t<dwell-.1)
        tail=(t>=1.)&(t<dwell-.1)
        transient=(t>=.15)&(t<dwell-.1)
        if not all(np.any(mask) for mask in [before,final,tail,transient]):continue
        step=np.mean(command[final])-np.mean(command[before])
        if abs(step)<.04:continue
        # Foot weight/inertial DC and observer bias are not inferred here.
        error=measured-command
        tails.append(float(np.std(error[tail])))
        target=np.mean(error[final])
        overshoots.append(float(max(0.,np.max(np.sign(step)*(error[transient]-target)))))
    return dict(blocks=len(tails),tail_error_std_mean=float(np.mean(tails)),
                tail_error_std_max=float(np.max(tails)),overshoot_relative_to_final_mean=float(np.mean(overshoots))) if tails else {}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag');p.add_argument('--output',required=True)
    p.add_argument('--controller-config');p.add_argument('--names',nargs='+')
    a=p.parse_args();root=Path(a.output);root.mkdir(parents=True,exist_ok=False)
    config_path=Path(a.controller_config) if a.controller_config else Path(a.bag).parent/'controller.yaml'
    if config_path.exists():config=yaml.safe_load(config_path.read_text())['controller']
    else:
        summary=json.loads((Path(a.bag).parent/'summary.json').read_text())
        config=summary['parameters']['/bee_cooperative_grasp/controller']
    # New configs deliberately contain no mass: use the run's resolved model
    # snapshot, never today's URDF or an arbitrary fallback for historical bags.
    if 'robot_mass' not in config or 'contact_damping' not in config:
        saved = {}
        for filename in ['session.json', 'summary.json']:
            snapshot = Path(a.bag).parent / filename
            if snapshot.exists(): saved.update(json.loads(snapshot.read_text()).get('parameters', {}))
        resolved = saved.get('/bee_cooperative_grasp/controller', {})
        lower = saved.get('/bee1/controller/reference_tracking',
                          saved.get('/bee1/controller', {}).get('reference_tracking', {}))
        if 'robot_mass' not in config:
            config['robot_mass'] = resolved.get('robot_mass', lower.get('model_mass'))
        if 'contact_damping' not in config:
            config['contact_damping'] = lower.get('velocity_damping')
        if config.get('robot_mass') is None or config.get('contact_damping') is None:
            raise ValueError('Record resolved model mass and velocity damping with the bag; '
                             'or supply the historical values via --controller-config')
    m=float(config['robot_mass']);damping=float(config['contact_damping'])
    phases=[];events=[]
    with rosbag.Bag(a.bag) as bag:
        names=a.names or sorted({t.split('/')[1] for t in bag.get_type_and_topic_info()[1] if t.endswith(('/command','/controller/reference','/grasp/reference'))})
        data={n:{key:[] for key in ['reference','status','site','diagnostic']} for n in names}
        rejected={n:0 for n in names}
        topics=['/bee_cooperative_grasp/phase','/bee_force_identification/segment']
        available = bag.get_type_and_topic_info().topics
        flag_names = {}
        for n in names:
            topics.extend([reference_topic(n, available), f'/{n}/mujoco/grasp_forces_world'])
            for suffix in ('tracking_status', 'force_feedback'):
                current = f'/{n}/controller/{suffix}'
                topics.append(current if current in available else f'/{n}/grasp/{suffix}')
            flag_names[n] = ('state_reset' if f'/{n}/controller/force_feedback' in available
                             else 'radius_override')
        end=bag.get_end_time()
        for topic,msg,stamp in bag.read_messages(topics=topics):
            if topic.endswith('/phase'):
                if not phases or phases[-1][1]!=msg.data:phases.append((stamp.to_sec(),msg.data))
                continue
            if topic.endswith('/segment'):events.append(json.loads(msg.data));continue
            n=topic.split('/')[1];t=msg.header.stamp.to_sec()
            if msg.header.frame_id.lstrip('/')!='world':
                raise ValueError('Expected world frame: '+topic)
            if topic.endswith(('/command','/reference')):
                cmd = bag_force_command(msg)
                if cmd is None or not cmd[3]: continue
                force, normal, acceleration, _ = cmd
                data[n]['reference'].append((t,np.r_[force,acceleration,normal,float(msg.force_integral_enabled)]))
            elif topic.endswith('/tracking_status'):
                if not(msg.ready and msg.active and msg.calibrated) or msg.fault or np.linalg.norm(vec(msg.external_force))<=.1:
                    rejected[n]+=1;continue
                data[n]['status'].append((t,np.r_[-vec(msg.external_force),vec(msg.commanded_acceleration),vec(msg.velocity),vec(msg.velocity_error)]))
            elif topic.endswith('/force_feedback'):
                data[n]['diagnostic'].append((t,np.r_[m*vec(msg.pi_unfiltered),m*vec(msg.pi_filtered),m*vec(msg.velocity_damping),
                    float(msg.correction_saturated),float(msg.acceleration_limited),float(getattr(msg,flag_names[n]))]))
            else:data[n]['site'].append((t,np.sum([vec(f) for f in msg.forces],axis=0)))
    segments=windows(phases,events,end);results=[]
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for seg in segments:
        # 100 Hz site measurements are not downsampled without anti-aliasing.
        # Interpolated observer samples retain their original ~40 Hz resolution.
        dt=.01;times=np.arange(seg['start'],seg['end'],dt)
        for n in names:
            try:
                refs,ref_intervals=interpolate(data[n]['reference'],times,True)
                status,status_intervals=interpolate(data[n]['status'],times)
            except ValueError as error:
                results.append(dict(bee=n,window=seg['name'],valid=False,reason=str(error)));continue
            b=fixed_basis(refs[0,6:9]);u=refs[:,:3];y=status[:,:3]
            force_error=u-y
            combined=m*(status[:,3:6]-refs[:,3:6])-u
            ns=refs[:,6:9];normal_error=ns*np.sum(ns*force_error,axis=1)[:,None]
            p_force=m*(config['normal_force_kp']*normal_error+config['contact_force_kp']*(force_error-normal_error))
            series=dict(command=u,observer=y,force_error=force_error,combined_correction=combined,
                        p_reconstructed=p_force,velocity=status[:,6:9],velocity_error=status[:,9:12],
                        damping_reconstructed=m*damping*status[:,9:12])
            flags={};periods={'reference':ref_intervals,'observer':status_intervals}
            for key in ['site','diagnostic']:
                if not data[n][key]:continue
                try:arr,periods[key]=interpolate(data[n][key],times)
                except ValueError:continue
                if key=='site':series['sim_site_force']=arr
                else:
                    series['pi_raw']=arr[:,:3];series['pi_filtered']=arr[:,3:6];series['damping']=arr[:,6:9]
                    flags={label:float(np.mean(arr[:,9+j]>.5)) for j,label in enumerate(['correction_saturated_fraction','acceleration_limited_fraction',flag_names[n]+'_fraction'])}
            metrics={};spectra={}
            for label,array in series.items():
                array=array@b.T;series[label]=array;metrics[label]={}
                for j,axis in enumerate(['normal','up','side']):
                    metrics[label][axis],freq,power=spectrum_metrics(array[:,j],dt)
                    spectra[label,axis]=(freq,power)
            coherence={}
            if 'sim_site_force' in series:
                for j,axis in enumerate(['normal','up','side']):
                    peak=metrics['sim_site_force'][axis]['peak_0.5_8_Hz']
                    if peak is None or np.std(series['observer'][:,j])<1e-8:
                        coherence[axis]={'valid':False,'reason':'insufficient AC variation'}
                        continue
                    f,c=signal.coherence(signal.detrend(series['observer'][:,j]),signal.detrend(series['sim_site_force'][:,j]),fs=1/dt,nperseg=min(len(times),round(8/dt)))
                    value=float(c[np.argmin(abs(f-peak))])
                    coherence[axis]={'valid':bool(np.isfinite(value)),'site_peak_hz':peak,
                                     'observer_site_coherence_at_site_peak':value if np.isfinite(value) else None}
            row=dict(bee=n,window=seg['name'],valid=True,start=times[0],duration=times[-1]-times[0],
                source_intervals=periods,integral_enabled_fraction=float(np.mean(refs[:,9])),
                normal_change_deg=float(np.degrees(np.arccos(np.clip(ns@ns[0],-1,1))).max()),metrics=metrics,coherence=coherence,flags=flags)
            if seg['name'].startswith('excite_'):
                axis=['normal','up','side'].index(seg['name'].removeprefix('excite_'))
                dwell=next(e['dwell'] for e in events if e.get('axis')==seg['name'][7:])
                row['pulse']={label:pulse_metrics(times-times[0],series['command'][:,axis],series[label][:,axis],dwell)
                              for label in ['observer','sim_site_force'] if label in series}
            results.append(row)
            with (root/f'{n}_{seg["name"]}.csv').open('w') as out:
                writer=csv.writer(out);writer.writerow(['time_s']+[f'{k}_{axis}' for k in series for axis in ['normal','up','side']])
                writer.writerows(np.column_stack([times-times[0]]+list(series.values())))
            if n==names[0]:
                fig,axes=plt.subplots(3,2,figsize=(12,8))
                for j,axis in enumerate(['normal','up','side']):
                    for k in ['command','observer','sim_site_force']:
                        if k in series:axes[j,0].plot(times-times[0],series[k][:,j]-np.mean(series[k][:,j]),label=k,lw=.8)
                    for k in ['observer','sim_site_force','force_error','combined_correction']:
                        if k in series:
                            f,power=spectra[k,axis];axes[j,1].semilogy(f,np.maximum(power,1e-16),label=k)
                    axes[j,0].set(ylabel=axis+' force, mean removed [N]',xlabel='time [s]')
                    axes[j,1].set(xlim=(0,8),ylabel='PSD [N²/Hz]',xlabel='frequency [Hz]')
                    for ax in axes[j]:ax.grid(alpha=.2);ax.legend(fontsize=7)
                fig.suptitle(n+' / '+seg['name']+' (site sensor includes foot inertia)');fig.tight_layout();fig.savefig(root/f'{n}_{seg["name"]}.png',dpi=130);plt.close(fig)
    result=dict(bag=str(Path(a.bag).resolve()),bag_sha256=file_hash(a.bag),config=config,phases=phases,windows=segments,results=results,
        excluded_status_samples=rejected,scope='Descriptive closed-loop spectra; no stability/causality claim. Site sensor includes foot mass/inertia. Reconstructed corrections use asynchronously sampled references.',
        source_sha256=file_hash(__file__))
    (root/'results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (root/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(root,'valid windows',sum(r['valid'] for r in results),'/',len(results),flush=True)
    return 0 if results and all(r['valid'] for r in results) else 1


if __name__=='__main__':raise SystemExit(main())
