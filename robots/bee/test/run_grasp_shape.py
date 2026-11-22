#!/usr/bin/env python3
"""Isolated, bounded MuJoCo mission. Never connects to the hardware ROS master."""
import argparse
import json
import os
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))
import signal
import socket
import subprocess
import time
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('shape', choices=['triangle', 'sphere', 'ellipsoid', 'cylinder', 'frustum', 'capsule', 'rock', 'sphere_support', 'box', 'pentagon'])
    parser.add_argument('--scene-config', type=Path, help='scene YAML for this shape')
    parser.add_argument('--port', type=int, default=11451)
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=420)
    parser.add_argument('--startup-only', action='store_true',
                        help='stop after all vehicles hover and C++ accepts grasp start; not a transport pass')
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535 or args.port == 11311:
        parser.error('Use a dedicated simulation port, never 11311')
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    probe = socket.socket()
    probe.bind(('127.0.0.1', args.port))
    probe.close()
    uri = 'http://127.0.0.1:%d' % args.port
    os.environ.update(ROS_MASTER_URI=uri, ROS_IP='127.0.0.1', ROS_LOG_DIR=str(out / 'roslog'))
    os.environ.pop('ROS_HOSTNAME', None)
    count = {'triangle': 3, 'pentagon': 5}.get(args.shape, 4)
    scene = args.scene_config.resolve() if args.scene_config else (
        Path(__file__).resolve().parents[1] / 'config' / 'mujoco' / ('scene_' + args.shape + '.yaml'))
    summary = dict(shape=args.shape, phases=[], reason='', final_position=None,
                   normal_z_min=1., normal_z_max=-1., result='timeout')
    log = open(out / 'launch.log', 'w')
    launch = subprocess.Popen(['roslaunch', '-p', str(args.port), 'bee', 'grasp_demo.launch',
                               'launch_rviz:=false', 'robot_count:=' + str(count),
                               'scene_config:=' + str(scene), 'output_dir:=' + str(out / 'scene')],
                              stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    bag = None
    try:
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            try:
                server = xmlrpc.client.ServerProxy(uri)
                clock_setting = server.getParam('/shape_test', '/use_sim_time')
                if server.getPid('/shape_test')[0] == 1 and clock_setting[0] == 1 and clock_setting[2] is True:
                    break
            except OSError:
                pass
            time.sleep(.1)
        else:
            raise RuntimeError('isolated simulation master did not start')
        import rospy
        from std_msgs.msg import String
        from geometry_msgs.msg import PoseStamped
        from bee.msg import BeeCommand
        from grasp_reference_axes import contact_normal
        rospy.init_node('shape_acceptance', disable_signals=True)
        def phase(m):
            if 'gravity' not in summary:
                summary['gravity'] = rospy.get_param('/bee_cooperative_grasp/controller/gravity')
            if not summary['phases'] or summary['phases'][-1]['name'] != m.data:
                entry = dict(name=m.data, time=rospy.get_time())
                summary['phases'].append(entry)
                print(entry, flush=True)
        def pose(m):
            p = m.pose.position
            summary['final_position'] = [p.x, p.y, p.z]
        def reference(m):
            if m.enable_force_control and m.acceleration_bounds:
                normal_z = contact_normal(m)[2]
                summary['normal_z_min'] = min(summary['normal_z_min'], normal_z)
                summary['normal_z_max'] = max(summary['normal_z_max'], normal_z)
        subs = [rospy.Subscriber('/bee_cooperative_grasp/phase', String, phase),
                rospy.Subscriber('/bee_cooperative_grasp/reason', String,
                                 lambda m: summary.update(reason=m.data)),
                rospy.Subscriber('/bee1/mujoco/grasp_object_pose', PoseStamped, pose)]
        topics = ['/clock', '/bee_cooperative_grasp/phase', '/bee_cooperative_grasp/reason',
                  '/bee_cooperative_grasp/metrics', '/bee_cooperative_grasp/contact_geometry_errors',
                  '/bee_cooperative_grasp/preload_diagnostics',
                  '/bee1/mujoco/grasp_object_pose']
        for i in range(1, count+1):
            root = '/bee%d' % i
            subs.append(rospy.Subscriber(root + '/command', BeeCommand, reference))
            topics += [root + s for s in ['/ground_truth', '/joint_states',
                                         '/mujoco/grasp_contact_states', '/mujoco/grasp_forces_world',
                                         '/command', '/controller/tracking_status', '/controller/force_feedback',
                                         '/controller/observer_calibration', '/controller/force_control_parameters',
                                         '/estimated_external_wrench', '/uav/cog/odom']]
        bag = subprocess.Popen(['rosbag', 'record', '--lz4', '-O', str(out / 'flight.bag')] + topics,
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and launch.poll() is None:
            if args.startup_only and summary['phases'] and summary['phases'][-1]['name'] == 'Staging':
                summary['result'] = 'StartupReady'
                break
            if summary['phases'] and summary['phases'][-1]['name'] in ('Complete', 'Aborted'):
                summary['result'] = summary['phases'][-1]['name']
                time.sleep(.5)
                break
            time.sleep(.2)
        if launch.poll() is not None:
            summary['result'] = 'launch_exit'
    finally:
        # Freeze mission measurements BEFORE shutting down flight controllers.
        # Physics can still advance during roslaunch teardown; that is not part
        # of the acceptance run (and can move a released sphere).
        snapshot = json.loads(json.dumps(summary))
        for proc in [bag, launch]:
            if proc is not None and proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=10)
        log.close()
        (out / 'summary.json').write_text(json.dumps(snapshot, indent=2) + '\n')
        print(json.dumps(snapshot), flush=True)
    return 0 if snapshot['result'] == 'Complete' or (args.startup_only and snapshot['result'] == 'StartupReady') else 1


if __name__ == '__main__':
    raise SystemExit(main())
