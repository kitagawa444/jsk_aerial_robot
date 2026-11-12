#!/usr/bin/env python3
"""Exercise the C++ node on an isolated ROS master with synthetic inputs.

No simulator or real vehicle is connected. Pass the built node via --node.
"""
import argparse
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
import xmlrpc.client


def wait_until(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError('condition timed out')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node', required=True)
    args = parser.parse_args()
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix='bee-grasp-ros-') as directory:
        os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:{}'.format(port)
        os.environ['ROS_HOSTNAME'] = '127.0.0.1'
        os.environ.pop('ROS_IP', None)
        os.environ['ROS_LOG_DIR'] = directory
        master_log = open(Path(directory) / 'master.log', 'w')
        master = subprocess.Popen(['roscore', '-p', str(port)], stdout=master_log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        processes = []
        stop = threading.Event()
        try:
            def master_ready():
                try:
                    return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/test')[0] == 1
                except OSError:
                    return False
            wait_until(master_ready, 10)
            import rospy
            from aerial_robot_msgs.msg import FlightNav
            from geometry_msgs.msg import PoseStamped, WrenchStamped, Vector3Stamped
            from nav_msgs.msg import Odometry
            from rosgraph_msgs.msg import Clock
            from std_msgs.msg import UInt8, String
            from std_srvs.srv import Trigger
            # Clock is supplied by this test, not by a simulator.
            xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).setParam('/test', '/use_sim_time', True)
            rospy.init_node('cooperative_grasp_test', disable_signals=True)
            clock = rospy.Publisher('/clock', Clock, queue_size=1)
            obj = rospy.Publisher('/test/object', PoseStamped, queue_size=1)
            names = ['bee1', 'bee2', 'bee3']
            angles = [math.pi / 3 + i * 2 * math.pi / 3 for i in range(3)]
            rospy.set_param('/test/scene', dict(schema_version=1, available=True, robot_names=names,
                face_angles=angles, face_radii=[.2] * 3, object_mass=.5, object_height=.6,
                staging_clearance=.8, object_position=[0., 1., .95], destination_position=[.4, 1., .95], has_destination=True))
            pubs = []
            for name in names:
                pubs.append((rospy.Publisher('/'+name+'/ground_truth', Odometry, queue_size=1),
                             rospy.Publisher('/'+name+'/uav/cog/odom', Odometry, queue_size=1),
                             rospy.Publisher('/'+name+'/estimated_external_wrench', WrenchStamped, queue_size=1),
                             rospy.Publisher('/'+name+'/flight_state', UInt8, queue_size=1)))
            enabled = threading.Event(); enabled.set()
            freeze_clock = threading.Event()
            stamp_seconds = [time.time()]
            def feed():
                while not stop.wait(.01):
                    if not freeze_clock.is_set():
                        stamp_seconds[0] += .01
                    stamp = rospy.Time.from_sec(stamp_seconds[0])
                    clock.publish(Clock(clock=stamp))
                    if not enabled.is_set():
                        continue
                    pose = PoseStamped(); pose.header.stamp = stamp; pose.header.frame_id = 'world'
                    pose.pose.position.y = 1; pose.pose.position.z = .95; pose.pose.orientation.w = 1
                    obj.publish(pose)
                    for name, angle, (base_pub, cog_pub, wrench_pub, state_pub) in zip(names, angles, pubs):
                        base = Odometry(); base.header.stamp = stamp
                        base.pose.pose.position.x = math.cos(angle)
                        base.pose.pose.position.y = 1 + math.sin(angle)
                        base.pose.pose.position.z = .95; base.pose.pose.orientation.w = 1
                        base_pub.publish(base)  # actual ground_truth also has an empty frame
                        base.header.frame_id = '/world'; base.child_frame_id = name + '/cog'; cog_pub.publish(base)
                        wrench = WrenchStamped(); wrench.header.stamp = stamp; wrench.header.frame_id = name + '/cog'; wrench_pub.publish(wrench)
                        state_pub.publish(UInt8(data=5))
            thread = threading.Thread(target=feed, daemon=True); thread.start()
            flight_messages = []
            subs = [rospy.Subscriber('/bee1/uav/nav', FlightNav, lambda msg: flight_messages.append(msg)),
                    rospy.Subscriber('/bee1/final_target_baselink_rpy', Vector3Stamped, lambda msg: None)]
            # The execution node checks command subscribers for every vehicle.
            for name in names[1:]:
                subs.extend([rospy.Subscriber('/'+name+'/uav/nav', FlightNav, lambda msg: None),
                             rospy.Subscriber('/'+name+'/final_target_baselink_rpy', Vector3Stamped, lambda msg: None)])
            for execute in (False, True):
                namespace = '/preview_test' if not execute else '/execution_test'
                enabled.set(); flight_messages.clear()
                rospy.set_param(namespace, dict(execute=execute, scene_namespace='/test/scene',
                    object_pose_topic='/test/object', allow_empty_base_frame=True,
                    safety={'input_timeout': .3}))
                previews, phases = [], []
                mode_subs = [rospy.Subscriber(namespace+'/preview/bee1/nav', FlightNav, lambda msg: previews.append(msg)),
                             rospy.Subscriber(namespace+'/phase', String, lambda msg: phases.append(msg.data))]
                log = open(Path(directory) / (namespace[1:] + '.log'), 'w')
                proc = subprocess.Popen([args.node, '__name:='+namespace[1:]], stdout=log, stderr=subprocess.STDOUT)
                processes.append(proc)
                rospy.wait_for_service(namespace+'/start', timeout=10)
                start = rospy.ServiceProxy(namespace+'/start', Trigger)
                last = [None]
                def start_ready():
                    last[0] = start()
                    return last[0].success
                wait_until(start_ready)
                wait_until(lambda: len(previews) > 5)
                if execute:
                    wait_until(lambda: any(msg.pos_xy_nav_mode == FlightNav.ACC_MODE for msg in flight_messages))
                    enabled.clear()  # exercise source freshness abort with clock still running
                    wait_until(lambda: phases and phases[-1] == 'Aborted')
                    wait_until(lambda: flight_messages and flight_messages[-1].pos_xy_nav_mode == FlightNav.VEL_MODE)
                    assert flight_messages[-1].target_vel_z == 0
                    enabled.set()
                else:
                    assert not flight_messages, 'preview published flight commands'
                    freeze_clock.set()
                    time.sleep(.6)
                    assert phases[-1] != 'Aborted', 'repeated simulation time falsely tripped heartbeat'
                    freeze_clock.clear()
                    rospy.ServiceProxy(namespace+'/abort', Trigger)()
                    wait_until(lambda: phases and phases[-1] == 'Aborted')
                assert not start().success, 'latched abort restarted'
                proc.send_signal(signal.SIGINT); proc.wait(timeout=5); log.close()
                for sub in mode_subs: sub.unregister()
                print(('execution' if execute else 'preview') + ': ROS interface and abort checks passed')
            stop.set(); thread.join(timeout=2)
            rospy.signal_shutdown('test complete')
        except Exception:
            for path in Path(directory).glob('*.log'):
                print(path.name, path.read_text()[-5000:])
            raise
        finally:
            stop.set()
            for proc in processes:
                if proc.poll() is None:
                    proc.terminate()
                    try: proc.wait(timeout=5)
                    except subprocess.TimeoutExpired: proc.kill(); proc.wait()
            os.killpg(master.pid, signal.SIGTERM)
            try: master.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(master.pid, signal.SIGKILL); master.wait()
            master_log.close()


if __name__ == '__main__':
    main()
