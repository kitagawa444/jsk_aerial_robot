#!/usr/bin/env python3
"""Measure a completed simulation from its bag, without contacting a master."""
import json
import math
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))
import statistics
import rosbag
import yaml
from grasp_reference_axes import bag_force_command, reference_topic



def analyze(directory):
    root = Path(directory)
    result = dict(directory=str(root), **json.loads((root / 'summary.json').read_text()))
    manifest_path = next((root / 'scene').glob('*_control.yaml'))
    manifest = yaml.safe_load(manifest_path.read_text())
    result['manifest'] = manifest
    log = (root / 'launch.log').read_text(errors='replace')
    result['eigen_assert'] = 'invalid matrix product' in log or 'Assertion' in log
    result['process_crash'] = any(s in log for s in ('exit code -6', 'exit code -11', 'Segmentation fault'))
    terminal = next((p['time'] for p in result['phases'] if p['name'] in ('Complete', 'Aborted')), math.inf)
    phase = 'Idle'
    phase_data, measures = {}, {}
    last = None
    loaded = {'SupportTransfer', 'Lift', 'Hold', 'Transport', 'DestinationSettle', 'Descend'}
    def add(key, value):
        measures.setdefault(key, []).append(float(value))
    topics = ['/bee_cooperative_grasp/phase', '/bee_cooperative_grasp/metrics',
              '/bee_cooperative_grasp/contact_geometry_errors', '/bee1/mujoco/grasp_object_pose']
    with rosbag.Bag(str(root / 'flight.bag')) as bag:
        available = bag.get_type_and_topic_info().topics
        topics.extend(reference_topic(name, available) for name in manifest['robot_names'])
        for topic, msg, stamp in bag.read_messages(topics=topics):
            if stamp.to_sec() > terminal + .02:
                continue  # exclude free-fall/teardown after a latched abort
            if topic.endswith('/phase'):
                phase = msg.data
            elif topic.endswith('/grasp_object_pose'):
                p, q = msg.pose.position, msg.pose.orientation
                last = [p.x, p.y, p.z]
                tilt = math.acos(max(-1., min(1., 1-2*(q.x*q.x+q.y*q.y))))
                entry = phase_data.setdefault(phase, dict(first=last, last=last, z_min=p.z, z_max=p.z, tilt_max_rad=tilt))
                entry.update(last=last, z_min=min(entry['z_min'], p.z), z_max=max(entry['z_max'], p.z),
                             tilt_max_rad=max(entry['tilt_max_rad'], tilt))
                if phase in loaded:
                    add('object_tilt_deg', math.degrees(tilt))
            elif phase in loaded and topic.endswith('/metrics') and len(msg.data) >= 9:
                add('common_slip_m', abs(msg.data[8]))
            elif phase in loaded and topic.endswith('/contact_geometry_errors') and len(msg.data):
                norms = [math.sqrt(sum(v*v for v in msg.data[i:i+3])) for i in range(0, len(msg.data), 3)]
                mean = [statistics.mean(msg.data[axis::3]) for axis in range(3)]
                add('mean_contact_error_m', math.sqrt(sum(v*v for v in mean)))
                add('average_contact_error_norm_m', statistics.mean(norms))
                add('relative_contact_deviation_m', max(math.sqrt(sum((msg.data[i+axis]-mean[axis])**2
                    for axis in range(3))) for i in range(0, len(msg.data), 3)))
                add('worst_contact_error_m', max(norms))
            elif phase in {'Hold', 'Transport', 'DestinationSettle'} and topic.endswith(('/command','/reference')):
                cmd = bag_force_command(msg)
                if cmd is None or not cmd[3]: continue
                total, n, _, _ = cmd
                force = -total.dot(n)
                f = total + force*n
                tangential = math.sqrt(f.dot(f))
                add('normal_force_N', force)
                add('normal_z', n[2])
                add('normal_vertical_N_per_bee', -force*n[2])
                add('tangent_vertical_N_per_bee', f[2])
                add('tangent_force_N_per_bee', tangential)
                if force > .1:
                    add('command_tangent_normal_ratio', tangential/force)
    result['phase_positions'] = phase_data
    result['preterminal_position'] = last
    result['measurements'] = {key: dict(mean=statistics.mean(values), maximum=max(values), minimum=min(values), samples=len(values))
                              for key, values in measures.items()}
    p, target = result['final_position'], manifest['destination_position']
    result['final_xy_error_m'] = math.hypot(p[0]-target[0], p[1]-target[1]) if p else None
    result['successful_placement_error_m'] = result['final_xy_error_m'] if result['result']=='Complete' else None
    result['reached_transport'] = any(p['name']=='Transport' for p in result['phases'])
    result['released'] = any(p['name']=='Release' for p in result['phases'])
    (root / 'analysis.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    return result


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+')
    args = parser.parse_args()
    for directory in args.directories:
        result = analyze(directory)
        print(result['shape'], result['result'], result['reason'], result['final_xy_error_m'], flush=True)
