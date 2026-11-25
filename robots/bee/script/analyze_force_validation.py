#!/usr/bin/env python3
"""Read-only vector-force audit for generic BeeController bags.

observer: compare the observer with a known external force injected in free-flight MuJoCo.
control: compare target force BY Bee with negative observed external force ON Bee.
The control score is observer-relative tracking, NOT true contact-force accuracy.
No publishers, services, live master, automatic delay compensation or bias fitting.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from grasp_reference_axes import force_control_enabled, reference_topic
import rosbag


def vector(v): return np.array([v.x, v.y, v.z], dtype=float)


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def causal(rows, times, max_age=.15):
    if not rows: return np.full((len(times), 3), np.nan), np.zeros(len(times), dtype=bool)
    # Last received value for a duplicate source timestamp; never interpolate future samples.
    values = {float(t): np.asarray(v, dtype=float) for t, v in rows if np.isfinite(t) and np.isfinite(v).all()}
    stamps = np.array(sorted(values)); data = np.array([values[t] for t in stamps])
    if not len(stamps): return np.full((len(times), 3), np.nan), np.zeros(len(times), dtype=bool)
    index = np.searchsorted(stamps, times, side='right')-1
    index_safe = np.maximum(index, 0)
    valid = (index >= 0) & (times-stamps[index_safe] <= max_age)
    return data[index_safe], valid


def metrics(error):
    norm = np.linalg.norm(error, axis=1)
    return dict(samples=len(error), bias_xyz_N=np.mean(error, axis=0).tolist(),
                rmse_xyz_N=np.sqrt(np.mean(error**2, axis=0)).tolist(),
                vector_rmse_N=float(np.sqrt(np.mean(norm**2))), p95_norm_N=float(np.percentile(norm, 95)),
                maximum_norm_N=float(np.max(norm)))


def analyze(bag_path, output, mode, names=None, limit=None):
    if mode not in ('observer', 'control'): raise ValueError('Unknown evaluation mode')
    limit = (.2 if mode == 'observer' else .5) if limit is None else limit
    if not np.isfinite(limit) or limit <= 0: raise ValueError('RMSE limit must be finite and positive')
    out = Path(output); out.mkdir(parents=True, exist_ok=False)
    with rosbag.Bag(str(bag_path)) as bag:
        available = bag.get_type_and_topic_info().topics
        if names is None:
            names = sorted({t.split('/')[1] for t in available if t.endswith('/controller/tracking_status')})
        if not names: raise ValueError('No generic controller status topics; specify names for legacy bags')
        data = {n:dict(status=[], commands=[], force_mode=[], contacts=[], faults=set(), calibration=set()) for n in names}
        mapping = {}
        for name in names:
            prefix = '/'+name
            status = prefix+('/controller/tracking_status' if prefix+'/controller/tracking_status' in available else '/grasp/tracking_status')
            command = reference_topic(name, available)
            mapping[status]=(name,'status'); mapping[command]=(name,'command')
            mapping[prefix+'/mujoco/external_wrench']=(name,'injected')
            mapping[prefix+'/mujoco/grasp_contact_states']=(name,'contact')
            mapping[prefix+'/controller/observer_calibration']=(name,'calibration')
        events=[];phases=[]
        for topic,m,stamp in bag.read_messages(topics=list(mapping)+['/force_validation/segment','/bee_cooperative_grasp/phase']):
            if topic == '/force_validation/segment':
                events.append(json.loads(m.data));continue
            if topic.endswith('/phase'):
                if not phases or phases[-1][1] != m.data:
                    phases.append((stamp.to_sec(),m.data))
                continue
            name,kind=mapping[topic];d=data[name];t=m.header.stamp.to_sec()
            if kind == 'status':
                if m.fault:d['faults'].add(m.fault)
                if m.ready and m.active and not m.fault and m.header.frame_id.lstrip('/')=='world' and np.isfinite(vector(m.external_force)).all():
                    d['status'].append((t,vector(m.external_force)))
            elif kind == 'command' and mode=='control':
                active = force_control_enabled(m) and not m.stop
                if not d['force_mode'] or d['force_mode'][-1][1] != active:
                    d['force_mode'].append((t,active))
                if active and m.header.frame_id.lstrip('/')=='world':
                    force = vector(m.force) if hasattr(m,'force') else -m.normal_force*vector(m.normal)+vector(m.support_vector)
                    d['commands'].append((t,force))
            elif kind == 'injected' and mode=='observer':
                if m.header.frame_id.lstrip('/')!='world':raise ValueError('Known-force input must be world frame')
                d['commands'].append((t,vector(m.wrench.force)))
            elif kind == 'contact':d['contacts'].append((t,[sum(abs(v) for v in m.effort)]*3))
            elif kind == 'calibration' and m.calibration_stamp.to_sec()>0:d['calibration'].add(m.calibration_stamp.to_sec())
    report=dict(mode=mode,bag=str(Path(bag_path).resolve()),bag_sha256=digest(bag_path),
                analyzer_sha256=digest(__file__),rmse_limit_N=limit,
                scope=('known injected external force in contact-free simulation' if mode=='observer' else
                       'target versus observer; does not establish true contact-force accuracy'),robots={},passed=True)
    for name,d in data.items():
        status={t:v for t,v in d['status']}; times=np.array(sorted(status)); observed=np.array([status[t] for t in times])
        if not len(times):raise ValueError('No valid status for '+name)
        desired,valid=causal(d['commands'],times)
        desired[~valid] = np.nan  # Do not draw/export an unavailable target as if it were current.
        if mode=='observer':
            if not events:raise ValueError('Observer evaluation needs explicit /force_validation/segment markers')
            unique={e['start']:e for e in events}; windows=[(e['name'],float(e['start'])+float(e.get('settle_s',2)),float(e['start'])+float(e['duration'])) for e in unique.values()]
        else:
            # Evaluate actual force-tracking windows, excluding initialization and release.
            windows=[]
            for i,(t,phase) in enumerate(phases):
                end=phases[i+1][0] if i+1<len(phases) else times[-1]+.001
                if phase in ('Hold','Transport','DestinationSettle') and end>t+1:
                    windows.append((phase,t+1,end))
            # Bags without task phases still contain ordinary motion commands.
            # Only explicit force-mode intervals count; command gaps within an
            # interval remain part of the coverage check.
            if not phases:
                for i,(start,active) in enumerate(d['force_mode']):
                    end=d['force_mode'][i+1][0] if i+1<len(d['force_mode']) else times[-1]+.001
                    if active and end>start:
                        windows.append(('force_tracking',start,end))
            observed=-observed
        covered=np.zeros(len(times),dtype=bool); groups=[]
        for label,start,end in windows:
            eligible=(times>=start)&(times<end)
            selected=eligible&valid
            coverage=float(np.count_nonzero(selected)/max(1,np.count_nonzero(eligible)))
            if np.count_nonzero(selected)<10:
                groups.append(dict(name=label,start=start,end=end,samples=int(np.count_nonzero(selected)),passed=False,reason='insufficient samples'))
                continue
            score=metrics(observed[selected]-desired[selected]);score.update(name=label,start=start,end=end,coverage=coverage)
            score['maximum_observation_gap_s']=float(np.max(np.diff(np.r_[start,times[eligible],end])))
            score['passed']=coverage>=.95 and score['maximum_observation_gap_s']<=.2 and score['vector_rmse_N']<=limit
            covered |= selected;groups.append(score)
        if not np.any(covered):raise ValueError('No evaluable force samples for '+name)
        entry=metrics(observed[covered]-desired[covered]);entry.update(windows=groups,faults=sorted(d['faults']))
        entry['passed']=bool(groups) and all(g['passed'] for g in groups) and not d['faults']
        if mode=='observer':
            contact,contact_valid=causal(d['contacts'],times)
            entry['contact_data_coverage']=float(np.mean(contact_valid[covered]))
            entry['maximum_contact_normal_sum_N']=float(np.max(contact[covered&contact_valid,0])) if np.any(covered&contact_valid) else None
            entry['calibrations_during_evaluation']=[t for t in d['calibration'] if times[covered][0]<=t<=times[covered][-1]]
            entry['passed'] &= entry['contact_data_coverage']>=.95 and entry['maximum_contact_normal_sum_N'] is not None and entry['maximum_contact_normal_sum_N']<.05 and not entry['calibrations_during_evaluation']
        report['robots'][name]=entry;report['passed'] &= entry['passed']
        with (out/(name+'.csv')).open('w') as f:
            w=csv.writer(f);w.writerow(['time_s','target_x_N','target_y_N','target_z_N','observed_x_N','observed_y_N','observed_z_N','evaluate'])
            for t,target,obs,keep in zip(times,desired,observed,covered):w.writerow([t,*target,*obs,int(keep)])
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True)
        for axis,ax in enumerate(axes):
            ax.plot(times,desired[:,axis],label='known force' if mode=='observer' else 'target',lw=1)
            ax.plot(times,observed[:,axis],label='observer',lw=.8)
            ax.set_ylabel('xyz'[axis]+' [N]');ax.grid(True,alpha=.25)
        axes[0].legend();axes[-1].set_xlabel('ROS time [s]');fig.suptitle(name+' — '+mode)
        fig.tight_layout();fig.savefig(out/(name+'.png'),dpi=140);plt.close(fig)
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag');p.add_argument('--output',required=True);p.add_argument('--mode',choices=['observer','control'],required=True)
    p.add_argument('--names',nargs='+');p.add_argument('--rmse-limit',type=float)
    a=p.parse_args();r=analyze(a.bag,a.output,a.mode,a.names,a.rmse_limit)
    for name,result in r['robots'].items():print(name,'PASS' if result['passed'] else 'FAIL','vector RMSE [N]',result['vector_rmse_N'])
    return int(not r['passed'])


if __name__=='__main__':raise SystemExit(main())
