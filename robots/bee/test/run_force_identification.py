#!/usr/bin/env python3
"""Run a bounded, isolated MuJoCo force identification experiment (no hardware)."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client
from xml.sax.saxutils import escape
import yaml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True);p.add_argument('--port',type=int,default=12311)
    p.add_argument('--amplitude',type=float,default=.16);p.add_argument('--dwell',type=float,default=2.5)
    p.add_argument('--settle-time',type=float,default=120.0)
    p.add_argument('--force-mode',choices=['p','pi'],default='pi')
    p.add_argument('--controller-config',help='Independent controller YAML; copied into the experiment directory')
    p.add_argument('--reference-tracking-config',help='Generic BeeController YAML; overrides legacy lower-controller fields')
    p.add_argument('--timeout',type=float,default=480)
    a=p.parse_args()
    if not 1024<=a.port<=65535 or a.port==11311:raise ValueError('Use a separate simulation port')
    root=Path(a.output).resolve();root.mkdir(parents=True,exist_ok=False)
    bee=Path(__file__).resolve().parents[1]
    controller=Path(a.controller_config).resolve() if a.controller_config else bee/'config/grasp/controller.yaml'
    (root/'controller.yaml').write_bytes(controller.read_bytes())
    controller=root/'controller.yaml'
    reference_config = yaml.safe_load((Path(a.reference_tracking_config).resolve() if a.reference_tracking_config
        else bee/'config/controller/force_control.yaml').read_text().replace('$(arg force_observer_profile)', 'simulation'))
    if not a.reference_tracking_config:
        reference_config = reference_config['reference_tracking']
        legacy = yaml.safe_load(controller.read_text())
        c = legacy.get('controller', {}); t = legacy.get('tracker', {})
        for old, new in [('contact_force_ki','force_ki'), ('contact_force_i_limit','force_integral_limit'),
            ('contact_damping','velocity_damping'), ('contact_force_filter_tau','force_filter_tau'),
            ('allocation_thrust_limit','thrust_limit'), ('allocation_thrust_reserve','correction_reserve')]:
            if old in c: reference_config[new] = c[old]
        if 'contact_force_kp' in c:
            reference_config['force_kp'] = [c.get('normal_force_kp', .4), c['contact_force_kp'], c['contact_force_kp']]
        for key in ['external_filter_tau','input_timeout','reference_timeout','maximum_acceleration',
                    'maximum_velocity_error','velocity_error_duration','calibration_duration','calibration_min_samples']:
            if key in t: reference_config[key] = t[key]
    (root/'reference_tracking.yaml').write_text(yaml.safe_dump(reference_config))
    task=yaml.safe_load((bee/'config/grasp/task_multi.yaml').read_text())
    task['task']['automatic_transport']=False
    (root/'task.yaml').write_text(yaml.safe_dump(task))
    def xmlpath(path):return escape(str(path),{'"':'&quot;'})
    remaps='\n'.join(f'<remap from="/bee{i}/command" to="/bee{i}/controller/identification_nominal"/>' for i in range(1,5))
    launch=f'''<launch>
      <param name="/use_sim_time" value="true"/>
      <include file="{xmlpath(bee/'launch/mujoco/mujoco_world.launch')}">
        <arg name="robot_count" value="4"/><arg name="headless" value="true"/>
        <arg name="scene_config" value="{xmlpath(bee/'config/mujoco/scene_box.yaml')}"/>
        <arg name="output_dir" value="{xmlpath(root/'scene')}"/>
      </include>
      <include file="{xmlpath(bee/'launch/mujoco/multi_robot_bringup.launch')}">
        <arg name="robot_count" value="4"/>
        <arg name="controller_config" value="{xmlpath(controller)}"/>
        <arg name="reference_tracking_config" value="{xmlpath(root/'reference_tracking.yaml')}"/>
      </include>
      <node pkg="bee" type="bee_cooperative_grasp_node" name="bee_cooperative_grasp" output="screen">
        <rosparam command="load" file="{xmlpath(controller)}"/>
        <rosparam command="load" file="{xmlpath(root/'task.yaml')}"/>
        <rosparam command="load" file="{xmlpath(bee/'config/mujoco/interface_mujoco.yaml')}"/>
        <param name="execute" value="true"/>{remaps}
      </node>
      <node pkg="bee" type="grasp_force_identification_sim.py" name="bee_force_identification" output="screen" required="true">
        <param name="amplitude" value="{a.amplitude}"/><param name="dwell" value="{a.dwell}"/>
        <param name="settle_time" value="{a.settle_time}"/>
        <param name="force_mode" value="{a.force_mode}"/>
      </node>
      <node pkg="bee" type="grasp_sim_autostart.py" name="bee_grasp_sim_autostart" output="screen" required="true">
        <param name="robot_count" value="4"/>
      </node>
    </launch>'''
    (root/'identification.launch').write_text(launch)
    with socket.socket() as sock:sock.bind(('127.0.0.1',a.port))
    uri=f'http://127.0.0.1:{a.port}/'
    os.environ.update(ROS_MASTER_URI=uri,ROS_IP='127.0.0.1',ROS_LOG_DIR=str(root/'roslog'))
    os.environ.pop('ROS_HOSTNAME',None)
    source_files=[
        bee/'script/grasp_force_identification_sim.py',bee/'script/grasp_reference_axes.py',Path(__file__).resolve(),
        controller,root/'reference_tracking.yaml',bee/'config/mujoco/scene_box.yaml',root/'task.yaml',
        bee/'src/control/bee_controller.cpp',
        bee/'src/cooperative_grasp/grasp_controller.cpp', bee/'include/bee/cooperative_grasp/grasp_node.h',
        bee/'include/bee/cooperative_grasp/grasp_state_estimator.h',bee/'msg/ForceFeedback.msg']
    workspace=bee.parents[3]
    source_files.extend((workspace/'devel/lib').glob('libbee_*.so'))
    source_files.append(workspace/'devel/lib/bee/bee_cooperative_grasp_node')
    hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in source_files}
    result={'kind':'simulation_force_identification','started':datetime.now(timezone.utc).isoformat(),
        'master':uri,'amplitude_N':a.amplitude,'dwell_s':a.dwell,'settle_time_s':a.settle_time,
        'force_mode':a.force_mode,'phases':[],
        'result':'timeout','reason':'','rejections':0,'hashes':hashes,
        'scope':'4-Bee box Hold; balanced force excitation; no hardware activity'}
    log=(root/'launch.log').open('w');bag=None
    process=subprocess.Popen(['roslaunch','-p',str(a.port),str(root/'identification.launch')],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        deadline=time.monotonic()+40
        while time.monotonic()<deadline and process.poll() is None:
            try:
                server=xmlrpc.client.ServerProxy(uri)
                v=server.getParam('/identification_runner','/use_sim_time')
                if v[0]==1 and v[2] is True:break
            except OSError:pass
            time.sleep(.1)
        else:raise RuntimeError('Isolated simulation did not initialize')
        import rospy
        from std_msgs.msg import String,Bool
        rospy.init_node('force_identification_runner',disable_signals=True)
        def phase(m):
            if not result['phases'] or result['phases'][-1]['name']!=m.data:
                result['phases'].append({'name':m.data,'time':rospy.get_time()})
                print(result['phases'][-1],flush=True)
            if m.data=='Aborted':result['result']='Aborted'
        def marker(m):
            data=json.loads(m.data)
            if data.get('state')=='rejected':result['rejections']+=1
            if data.get('state')=='excite' and data['block']==0:print(data,flush=True)
        def done(m):
            if m.data:result['result']='IdentificationComplete'
        subs=[rospy.Subscriber('/bee_cooperative_grasp/phase',String,phase),
              rospy.Subscriber('/bee_cooperative_grasp/reason',String,lambda m:result.update(reason=m.data)),
              rospy.Subscriber('/bee_force_identification/segment',String,marker),
              rospy.Subscriber('/bee_force_identification/done',Bool,done)]
        topics=['/clock','/bee_cooperative_grasp/phase','/bee_cooperative_grasp/reason',
            '/bee_cooperative_grasp/metrics','/bee_force_identification/segment',
            '/bee_force_identification/done','/bee1/mujoco/grasp_object_pose']
        for i in range(1,5):
            topics += [f'/bee{i}/'+s for s in ['command','controller/identification_nominal',
                'controller/tracking_status','controller/force_feedback','controller/observer_calibration','controller/force_control_parameters','estimated_external_wrench','uav/cog/odom','ground_truth',
                'mujoco/grasp_forces_world','mujoco/grasp_contact_states']]
        bag=subprocess.Popen(['rosbag','record','--lz4','-O',str(root/'flight.bag')]+topics,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+a.timeout
        while time.monotonic()<deadline and process.poll() is None and result['result']=='timeout':time.sleep(.2)
        if process.poll() is not None:result['result']='launch_exit'
        try:
            result['parameters']={key:rospy.get_param(key,{}) for key in [
                '/bee_cooperative_grasp/controller','/bee_cooperative_grasp/tracker','/bee1/controller/reference_tracking','/bee_grasp/scene']}
        except OSError as error:
            result['parameter_snapshot_error']=str(error)
    finally:
        for proc in [bag,process]:
            if proc is not None and proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
        log.close()
        result['changed_inputs']=[p for p,h in hashes.items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
        with socket.socket() as sock:result['master_stopped']=sock.connect_ex(('127.0.0.1',a.port))!=0
        (root/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        print(result['result'],result['reason'],'rejections',result['rejections'],flush=True)
    return int(result['result']!='IdentificationComplete' or result['rejections']>0 or bool(result['changed_inputs']))


if __name__=='__main__':raise SystemExit(main())
