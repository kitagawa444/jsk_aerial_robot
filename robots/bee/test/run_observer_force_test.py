#!/usr/bin/env python3
"""Isolated MuJoCo free-flight force-observer test. Never attaches to an existing master.

Applies known WORLD forces to the simulated root body, with no object contact.
The generic controller holds position; the observer calibration is fixed before excitation.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client
from xml.sax.saxutils import quoteattr


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True)
    p.add_argument('--port', type=int, default=12931)
    p.add_argument('--amplitude', type=float, default=.4)
    p.add_argument('--dwell', type=float, default=5.)
    p.add_argument('--timeout', type=float, default=240.)
    a = p.parse_args()
    if not 1024 <= a.port <= 65535 or a.port == 11311:
        p.error('Use a dedicated simulation port, never 11311')
    if not all(math.isfinite(v) for v in [a.amplitude, a.dwell, a.timeout]) or not 0 < a.amplitude <= 1 or a.dwell < 3 or a.timeout <= 0:
        p.error('Require 0 < amplitude <= 1 N, dwell >= 3 s and positive timeout')
    with socket.socket() as sock: sock.bind(('127.0.0.1', a.port))
    root = Path(a.output).resolve(); root.mkdir(parents=True, exist_ok=False)
    bee = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(bee/'script'))
    import rospkg
    from mujoco_scene_generator import MujocoSceneGenerator
    generator = MujocoSceneGenerator(dict(robot_count=1, spawn_object=False,
        spawn_object_pedestal=False, spawn_destination_pedestal=False,
        grasp_staging_layout=False, spawn_x_start=0., spawn_y_start=0.), str(bee),
        rospkg.RosPack().get_path('mujoco_ros_control'), str(root/'scene'))
    _, model, _, _ = generator.generate()
    prefix = root/'start_backend.py'
    prefix.write_text('#!/usr/bin/env python3\nimport os,sys,time\ntime.sleep(3)\nos.execv(sys.argv[1],sys.argv[1:])\n')
    prefix.chmod(0o755)
    launch = root/'observer.launch'
    launch.write_text(f'''<launch>
      <param name="/use_sim_time" value="true"/>
      <include file={quoteattr(str(bee/'launch/mujoco/multi_robot_bringup.launch'))}>
        <arg name="robot_count" value="1"/>
      </include>
      <node pkg="mujoco_ros_control" type="mujoco_ros_control" name="mujoco_ros_control"
            output="screen" required="true" launch-prefix={quoteattr(str(prefix))}>
        <param name="mujoco_model_path" value={quoteattr(model)}/>
        <rosparam param="robot_namespaces">[bee1]</rosparam>
        <param name="headless" value="true"/><param name="real_time_factor" value="1.0"/>
      </node>
    </launch>''')
    uri = 'http://127.0.0.1:%d' % a.port
    os.environ.update(ROS_MASTER_URI=uri, ROS_IP='127.0.0.1', ROS_LOG_DIR=str(root/'roslog'))
    os.environ.pop('ROS_HOSTNAME', None)
    source_paths = [Path(__file__), bee/'src/control/bee_controller.cpp',
                    bee/'include/bee/control/bee_controller.h', bee/'config/controller/force_control.yaml',
                    Path(model), launch]
    hashes = {str(f): hashlib.sha256(f.read_bytes()).hexdigest() for f in source_paths}
    result = dict(kind='known_external_force_observer', master=uri, result='incomplete', reason='',
                  started_utc=datetime.now(timezone.utc).isoformat(), amplitude_N=a.amplitude,
                  dwell_s=a.dwell, hashes=hashes, segments=[], scope='single Bee, free flight, simulated world force on root')
    log = (root/'launch.log').open('w')
    proc = subprocess.Popen(['roslaunch', '-p', str(a.port), str(launch)], stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True)
    bag = None; feeder = None; stop = threading.Event(); data = dict(reference=None, force=[0.,0.,0.])
    deadline = time.monotonic()+a.timeout
    try:
        def wait(predicate, label, seconds=30):
            end = min(deadline, time.monotonic()+seconds)
            while time.monotonic() < end:
                if proc.poll() is not None: raise RuntimeError('simulator launch exited: '+label)
                if predicate(): return
                time.sleep(.02)
            raise RuntimeError('timeout: '+label)
        def master_ready():
            try:
                s = xmlrpc.client.ServerProxy(uri)
                return s.getPid('/force_test')[0] == 1 and s.getParam('/force_test','/use_sim_time')[2] is True
            except (OSError, xmlrpc.client.Error): return False
        wait(master_ready, 'private simulated master')
        import rospy
        from aerial_robot_msgs.msg import FlightNav
        from bee.msg import BeeCommand, ControllerStatus, ObserverCalibrationStatus
        from geometry_msgs.msg import WrenchStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Empty, String, UInt8
        rospy.init_node('observer_force_test', disable_signals=True)
        def receive(key, msg): data[key] = (msg, time.monotonic())
        subs = [rospy.Subscriber('/bee1/flight_state',UInt8,lambda m: receive('state',m)),
                rospy.Subscriber('/bee1/uav/cog/odom',Odometry,lambda m: receive('cog',m)),
                rospy.Subscriber('/bee1/controller/tracking_status',ControllerStatus,lambda m: receive('status',m)),
                rospy.Subscriber('/bee1/controller/observer_calibration',ObserverCalibrationStatus,lambda m: receive('calibration',m))]
        start = rospy.Publisher('/bee1/teleop_command/start',Empty,queue_size=1)
        takeoff = rospy.Publisher('/bee1/teleop_command/takeoff',Empty,queue_size=1)
        ref = rospy.Publisher('/bee1/command',BeeCommand,queue_size=1)
        force = rospy.Publisher('/bee1/mujoco/external_wrench',WrenchStamped,queue_size=1)
        calibrate = rospy.Publisher('/bee1/controller/calibrate_observer',Empty,queue_size=1)
        marker = rospy.Publisher('/force_validation/segment',String,queue_size=1,latch=True)
        def state(value):
            return 'state' in data and data['state'][0].data == value and time.monotonic()-data['state'][1] < .5
        wait(lambda: rospy.get_time()>0 and state(0) and start.get_num_connections() and takeoff.get_num_connections()
             and force.get_num_connections(), 'disarmed simulated vehicle')
        if rospy.get_param('/bee1/simulation/robot_hw_sim_plugin_name','') != 'mujoco_ros_control/AerialRobotHWSim':
            raise RuntimeError('simulation plugin guard rejected startup')
        first = rospy.get_time(); wait(lambda: rospy.get_time() > first+2, 'advancing clock and initialization')
        topics = ['/clock','/force_validation/segment','/bee1/mujoco/external_wrench',
            '/bee1/command','/bee1/controller/tracking_status','/bee1/controller/force_feedback',
            '/bee1/controller/observer_calibration','/bee1/controller/force_control_parameters',
            '/bee1/estimated_external_wrench','/bee1/uav/cog/odom','/bee1/ground_truth',
            '/bee1/mujoco/grasp_contact_states','/bee1/flight_state']
        bag = subprocess.Popen(['rosbag','record','--lz4','-O',str(root/'flight.bag')]+topics,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        start.publish(Empty()); wait(lambda: state(2), 'simulated arm')
        takeoff.publish(Empty()); wait(lambda: state(5), 'simulated HOVER', 90)
        wait(lambda: 'cog' in data and time.monotonic()-data['cog'][1] < .3, 'fresh world CoG odometry')
        cog = data['cog'][0]
        if cog.header.frame_id.lstrip('/') != 'world': raise RuntimeError('wrong odometry frame')
        reference = BeeCommand(); reference.header.frame_id='world'
        reference.navigation.control_frame=FlightNav.WORLD_FRAME; reference.navigation.target=FlightNav.COG
        reference.navigation.pos_xy_nav_mode=reference.navigation.pos_z_nav_mode=FlightNav.POS_MODE
        reference.navigation.target_pos_x=cog.pose.pose.position.x
        reference.navigation.target_pos_y=cog.pose.pose.position.y
        reference.navigation.target_pos_z=cog.pose.pose.position.z
        data['reference']=reference
        def feed():
            while not stop.wait(.02):
                if data['reference'] is not None:
                    data['reference'].header.stamp=rospy.Time.now(); ref.publish(data['reference'])
                msg=WrenchStamped();msg.header.stamp=rospy.Time.now();msg.header.frame_id='world'
                msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z=data['force'];force.publish(msg)
        feeder=threading.Thread(target=feed,daemon=True);feeder.start()
        wait(lambda: 'status' in data and data['status'][0].active, 'generic position tracking')
        def dwell(duration):
            end=rospy.get_time()+duration
            def done():
                if not state(5): raise RuntimeError('lost HOVER')
                if 'status' in data and data['status'][0].fault: raise RuntimeError(data['status'][0].fault)
                if bag.poll() is not None: raise RuntimeError('bag recorder exited')
                return rospy.get_time()>=end
            wait(done, 'force segment', max(30,5*duration))
        dwell(5)
        wait(lambda: calibrate.get_num_connections(), 'calibration subscriber')
        requested=rospy.get_time(); calibrate.publish(Empty())
        wait(lambda: 'calibration' in data and data['calibration'][0].state==ObserverCalibrationStatus.SUCCEEDED
             and data['calibration'][0].calibration_start_stamp.to_sec()>=requested, 'observer calibration')
        protocol=[('zero',[0.,0.,0.],a.dwell)]
        for axis in range(3):
            for sign in (1,-1):
                f=[0.,0.,0.];f[axis]=sign*a.amplitude
                protocol.extend([(f'{"xyz"[axis]}_{"pos" if sign>0 else "neg"}',f,a.dwell),('zero',[0.,0.,0.],3.)])
        for name, vector, duration in protocol:
            event=dict(name=name,start=rospy.get_time(),duration=duration,expected_force_world_N=vector,settle_s=2.)
            data['force']=vector; marker.publish(String(data=json.dumps(event)));result['segments'].append(event)
            print('SEGMENT',name,vector,flush=True);dwell(duration)
        data['force']=[0.,0.,0.];dwell(.3)
        result['result']='Complete'
    except Exception as error:
        result.update(result='Failed',reason=repr(error))
    finally:
        data['force']=[0.,0.,0.];time.sleep(.08);stop.set()
        if feeder: feeder.join(timeout=2)
        for child in [bag,proc]:
            if child is not None and child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=10)
        log.close()
        result['changed_inputs']=[name for name,h in hashes.items() if not Path(name).is_file() or hashlib.sha256(Path(name).read_bytes()).hexdigest()!=h]
        with socket.socket() as sock:result['master_stopped']=sock.connect_ex(('127.0.0.1',a.port))!=0
        (root/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        print(result['result'],result['reason'],flush=True)
    return int(result['result']!='Complete' or bool(result['changed_inputs']) or not result['master_stopped'])


if __name__=='__main__': raise SystemExit(main())
