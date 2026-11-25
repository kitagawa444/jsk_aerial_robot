#!/usr/bin/env python3
"""Passive hardware/simulation recording. Does not publish or call services."""
import argparse
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True)
    p.add_argument('--names',nargs='+',required=True)
    p.add_argument('--source',choices=['hardware','simulation'],required=True)
    p.add_argument('--label',required=True,help='Airframe/firmware/contact-condition identifier')
    p.add_argument('--duration',type=float,default=120)
    a=p.parse_args()
    if a.duration<=0:raise ValueError('duration must be positive')
    if any(not n.replace('_','').isalnum() for n in a.names):raise ValueError('Use simple robot namespace names')
    root=Path(a.output).resolve();root.mkdir(parents=True,exist_ok=False)
    import rospy
    rospy.init_node('force_response_recorder',anonymous=True,disable_signals=True)
    metadata={'source':a.source,'label':a.label,'robot_names':a.names,
        'created':datetime.now(timezone.utc).isoformat(),'ROS_MASTER_URI':os.environ.get('ROS_MASTER_URI',''),
        'use_sim_time':rospy.get_param('/use_sim_time',False),'passive_only':True,'parameters':{}}
    keys=['/bee_cooperative_grasp/controller','/bee_cooperative_grasp/tracker','/bee_cooperative_grasp/task','/bee_grasp/scene']
    for n in a.names:keys.extend(['/'+n+'/controller','/'+n+'/tracker'])
    for key in keys:
        if rospy.has_param(key):metadata['parameters'][key]=rospy.get_param(key)
    topics=['/clock','/bee_cooperative_grasp/phase','/bee_cooperative_grasp/metrics',
            '/bee_force_identification/segment']
    for n in a.names:
        topics.extend('/'+n+'/'+suffix for suffix in ['command','controller/tracking_status','controller/force_feedback','controller/observer_calibration','controller/force_control_parameters',
            'estimated_external_wrench','uav/cog/odom','uav/baselink/odom','flight_state'])
        if a.source=='simulation':topics.append('/'+n+'/mujoco/grasp_forces_world')
    metadata['topics']=topics
    (root/'session.json').write_text(json.dumps(metadata,indent=2)+'\n')
    with (root/'record.log').open('w') as log:
        child=subprocess.Popen(['rosbag','record','--lz4','-O',str(root/'flight.bag')]+topics,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+a.duration
            while child.poll() is None and time.monotonic()<deadline:time.sleep(.1)
        except KeyboardInterrupt:pass
        finally:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=10)
    metadata['finished']=datetime.now(timezone.utc).isoformat()
    metadata['bag_exists']=(root/'flight.bag').is_file()
    (root/'session.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return int(not metadata['bag_exists'])


if __name__=='__main__':raise SystemExit(main())
