#!/usr/bin/env python3
"""Test generic BeeController references, calibration and runtime gains on a private master with synthetic data.

The test harness reports commands only on private observation topics. No simulator, flight controller or hardware master is used.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import xmlrpc.client


def wait_for(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('condition timed out')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    uri = 'http://127.0.0.1:%d' % port
    os.environ.update(ROS_MASTER_URI=uri, ROS_IP='127.0.0.1', ROS_LOG_DIR=str(out/'roslog'))
    os.environ.pop('ROS_HOSTNAME', None)
    processes, logs, results = [], [], []
    stop = threading.Event()
    feeder = None

    def launch(command, filename):
        log = open(out/filename, 'w'); logs.append(log)
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc)
        return proc

    try:
        launch(['roscore', '-p', str(port)], 'master.log')
        def master_ready():
            try:
                return xmlrpc.client.ServerProxy(uri).getPid('/calibration_test')[0] == 1
            except OSError:
                return False
        wait_for(master_ready, 10)
        import rospy
        from aerial_robot_msgs.msg import FlightNav
        from bee.msg import BeeCommand, ObserverCalibrationStatus, ControllerStatus, ForceControlParameters, ForceFeedback
        from bee.srv import SetForceControlParameters
        from geometry_msgs.msg import WrenchStamped, Vector3Stamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Empty, UInt8
        from std_srvs.srv import SetBool
        rospy.init_node('calibration_test', disable_signals=True)
        states = {}
        subscriptions = []
        for name, binary in [('native', args.native)]:
            ns = '/'+name
            rospy.set_param(ns+'/host/calibration_duration', .6)
            rospy.set_param(ns+'/host/calibration_min_samples', 10)
            data = states[name] = dict(feed=False, hover=False, force=[1., 2., 3.], stamp=None,
                                       reference=None, statuses=[], tracking=None, nav_count=0, observed_nav=None,
                                       observed_attitude=None, parameters=None, feedback=[])
            data['cog'] = rospy.Publisher(ns+'/uav/cog/odom', Odometry, queue_size=1)
            data['base'] = rospy.Publisher(ns+'/uav/baselink/odom', Odometry, queue_size=1)
            data['wrench'] = rospy.Publisher(ns+'/estimated_external_wrench', WrenchStamped, queue_size=1)
            data['flight'] = rospy.Publisher(ns+'/flight_state', UInt8, queue_size=1)
            data['ref'] = rospy.Publisher(ns+'/command', BeeCommand, queue_size=1)
            data['request'] = rospy.Publisher(ns+'/controller/calibrate_observer', Empty, queue_size=1)
            subscriptions += [
                rospy.Subscriber(ns+'/controller/observer_calibration', ObserverCalibrationStatus,
                                 lambda m, d=data: d['statuses'].append(m)),
                rospy.Subscriber(ns+'/controller/tracking_status', ControllerStatus,
                                 lambda m, d=data: d.update(tracking=m)),
                rospy.Subscriber(ns+'/controller/force_control_parameters', ForceControlParameters,
                                 lambda m, d=data: d.update(parameters=m)),
                rospy.Subscriber(ns+'/controller/force_feedback', ForceFeedback,
                                 lambda m, d=data: d['feedback'].append(m)),
                rospy.Subscriber(ns+'/uav/nav', FlightNav,
                                 lambda m, d=data: d.update(nav_count=d['nav_count']+1)),
                rospy.Subscriber(ns+'/final_target_baselink_rpy', Vector3Stamped, lambda m: None),
                rospy.Subscriber(ns+'/host/observed_nav', FlightNav, lambda m, d=data: d.update(observed_nav=m)),
                rospy.Subscriber(ns+'/host/observed_attitude', Vector3Stamped, lambda m, d=data: d.update(observed_attitude=m))]
            launch([binary, '__ns:='+ns, '__name:=host'], name+'.log')
        wait_for(lambda: all(d['request'].get_num_connections() and d['statuses'] for d in states.values()))

        def feed():
            while not stop.wait(.02):
                for name, d in states.items():
                    if not d['feed']:
                        continue
                    stamp = d['stamp'] or rospy.Time.now()
                    cog = Odometry(); cog.header.stamp = stamp; cog.header.frame_id = 'world'
                    cog.child_frame_id = name+'/cog'
                    cog.pose.pose.position.x = 1
                    cog.pose.pose.orientation.w = math.sqrt(.5)
                    cog.pose.pose.orientation.z = math.sqrt(.5)
                    d['cog'].publish(cog); d['base'].publish(cog)
                    wrench = WrenchStamped(); wrench.header.stamp = stamp; wrench.header.frame_id = name+'/cog'
                    wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z = d['force']
                    wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z = [.1, -.2, .3]
                    d['wrench'].publish(wrench)
                    if d['hover']:
                        d['flight'].publish(UInt8(data=5))
                    if d['reference'] is not None:
                        ref = BeeCommand(); ref.header.stamp = rospy.Time.now(); ref.header.frame_id = 'world'
                        ref.navigation.control_frame = FlightNav.WORLD_FRAME
                        ref.navigation.target = FlightNav.COG
                        ref.navigation.pos_xy_nav_mode = ref.navigation.pos_z_nav_mode = FlightNav.ACC_MODE
                        for key, value in d['reference'].items():
                            setattr(ref, key, value)
                        d['ref'].publish(ref)
        feeder = threading.Thread(target=feed, daemon=True); feeder.start()

        def event(d, request, expected):
            count = len(d['statuses']); request()
            wait_for(lambda: any(m.state == expected for m in d['statuses'][count:]))
            return next(m for m in reversed(d['statuses'][count:]) if m.state == expected)

        def request(d, expected):
            return event(d, lambda: d['request'].publish(Empty()), expected)

        def check_bias(m, force):
            for actual, expected in zip([m.baseline_cog.force.x, m.baseline_cog.force.y, m.baseline_cog.force.z], force):
                assert abs(actual-expected) < 1e-9, (actual, expected)
            for actual, expected in zip([m.baseline_world_force.x, m.baseline_world_force.y, m.baseline_world_force.z], [-force[1], force[0], force[2]]):
                assert abs(actual-expected) < 1e-9, (actual, expected)
            assert abs(m.baseline_cog.torque.y+.2) < 1e-9

        for name, d in states.items():
            def passed(case):
                results.append(dict(implementation=name, case=case, result='pass'))
                print(name+': '+case, flush=True)
            wait_for(lambda: d['parameters'] is not None and d['tracking'] is not None)
            initial = copy.deepcopy(d['parameters'])
            assert initial.force_kp.x == .4 and initial.force_ki == .1
            assert initial.external_filter_tau == .2 and initial.force_filter_tau == 0
            settings_service = rospy.ServiceProxy(ns+'/controller/set_force_control_parameters', SetForceControlParameters)
            rospy.wait_for_service(ns+'/controller/set_force_control_parameters', timeout=3)
            def apply(parameters, reset=False):
                response = settings_service(parameters, reset)
                assert response.success, response.message
                assert response.applied == parameters
                wait_for(lambda: d['parameters'] == parameters)
                return response
            settings = copy.deepcopy(initial); settings.force_kp.x = .2
            apply(settings)
            assert not d['tracking'].active
            apply(initial)
            passed('latched defaults and parameter updates without active tracking or calibration')
            m = request(d, ObserverCalibrationStatus.REJECTED)
            assert m.calibration_stamp.is_zero()
            passed('reject missing inputs')
            d['feed'] = True; time.sleep(.15)
            assert d['nav_count'] == 0
            m = request(d, ObserverCalibrationStatus.SUCCEEDED)
            check_bias(m, d['force']); assert m.samples >= 10
            wait_for(lambda: d['tracking'] and d['tracking'].calibrated)
            assert not d['tracking'].active and d['nav_count'] == 0
            passed('calibrate without references, HOVER or enabled reference mode')
            d['force'] = [2., -1., .5]; time.sleep(.1)
            request(d, ObserverCalibrationStatus.COLLECTING)
            request(d, ObserverCalibrationStatus.REJECTED)
            wait_for(lambda: d['statuses'][-1].state == ObserverCalibrationStatus.SUCCEEDED)
            m = d['statuses'][-1]; check_bias(m, d['force']); applied = m.calibration_stamp
            passed('repeat calibration; duplicate request does not restart collection')
            request(d, ObserverCalibrationStatus.COLLECTING)
            m = event(d, lambda: d.update(feed=False), ObserverCalibrationStatus.FAILED)
            assert m.calibration_stamp == applied; check_bias(m, d['force'])
            assert d['tracking'].calibrated
            passed('stale stream retains previous baseline')
            d['feed'] = True; time.sleep(.15)
            request(d, ObserverCalibrationStatus.COLLECTING)
            m = event(d, lambda: d.update(stamp=rospy.Time.now()), ObserverCalibrationStatus.FAILED)
            assert m.calibration_stamp == applied
            passed('repeated source stamp cannot complete calibration')
            d['stamp'] = None; d['force'] = [float('nan'), 0., 0.]; time.sleep(.1)
            m = request(d, ObserverCalibrationStatus.REJECTED); assert m.calibration_stamp == applied
            passed('nonfinite observer rejected')
            d['force'] = [2., -1., .5]; time.sleep(.1)
            if name == 'native':
                rospy.wait_for_service('/native/controller/set_reference_mode', timeout=3)
                assert rospy.ServiceProxy('/native/controller/set_reference_mode', SetBool)(True).success
                time.sleep(.05); assert d['tracking'].calibrated
                passed('calibration survives enabling reference mode')
            d['hover'] = True
            d['reference'] = dict(enable_force_control=True)
            wait_for(lambda: d['tracking'].active)
            time.sleep(.1)
            observed = d['tracking'].external_force
            assert max(abs(observed.x), abs(observed.y), abs(observed.z)) < 1e-8
            d['force'] = [3., -1., .5]
            # +1 N in cog X becomes +1 N in world Y; allow the 0.2 s LPF to settle.
            wait_for(lambda: d['tracking'].external_force.y > .9, timeout=2)
            observed = d['tracking'].external_force
            assert abs(observed.x) < 1e-8 and abs(observed.z) < 1e-8
            d['force'] = [2., -1., .5]
            passed('applied zero removes bias while preserving a new external load')
            # Calibration is an explicit operator action even while force tracking.
            d['force'] = [3., -1., .5]; time.sleep(.1)
            m = request(d, ObserverCalibrationStatus.SUCCEEDED)
            check_bias(m, d['force']); assert m.calibration_stamp > applied
            assert d['tracking'].active and not d['tracking'].fault
            applied = m.calibration_stamp
            passed('calibration accepted and applied during force tracking')
            d['reference'] = dict(enable_force_control=False); time.sleep(.1)
            request(d, ObserverCalibrationStatus.COLLECTING)
            m = event(d, lambda: d.update(reference=dict(enable_force_control=True)), ObserverCalibrationStatus.SUCCEEDED)
            assert m.calibration_stamp > applied
            passed('switching into force tracking does not cancel calibration')
            # Runtime changes must alter the production control output, not just the status topic.
            from geometry_msgs.msg import Vector3
            d['reference'] = dict(enable_force_control=True, force=Vector3(1, 0, 0))
            wait_for(lambda: d['feedback'] and abs(d['feedback'][-1].pi_unfiltered.x-.4) < 1e-7)
            wait_for(lambda: abs(d['tracking'].commanded_acceleration.x-(1/1.84199+.4)) < 1e-5)
            before_acceleration = d['tracking'].commanded_acceleration.x
            settings = copy.deepcopy(initial); settings.force_kp.x = .8
            apply(settings)
            wait_for(lambda: abs(d['tracking'].commanded_acceleration.x-before_acceleration-.4) < 1e-5)
            passed('live P gain changes actual acceleration with force tracking active')
            # A bad field rejects the entire transaction, including a reset request.
            for field, value in [('force_ki', -.1), ('external_filter_tau', float('nan')),
                                 ('force_filter_tau', float('inf')), ('force_integral_limit', -.1),
                                 ('velocity_damping', -1.)]:
                bad = copy.deepcopy(settings); bad.force_kp.x = .2; setattr(bad, field, value)
                response = settings_service(bad, True)
                assert not response.success and response.applied == settings
            bad = copy.deepcopy(settings); bad.force_kp.z = float('nan')
            assert not settings_service(bad, False).success
            assert d['parameters'] == settings and not d['tracking'].fault
            passed('invalid parameter sets are rejected atomically and retain active gains')
            settings.force_kp = Vector3(); settings.force_ki = .3
            apply(settings, True)
            d['reference'] = dict(enable_force_control=True, force_integral_enabled=True, force=Vector3(1, 0, 0))
            wait_for(lambda: d['feedback'][-1].integral.x > .06)
            saved_integral = d['feedback'][-1].integral.x
            bad = copy.deepcopy(settings); bad.force_ki = -1
            assert not settings_service(bad, True).success
            settings.force_ki = .05; apply(settings)
            stamp = d['feedback'][-1].header.stamp
            wait_for(lambda: d['feedback'][-1].header.stamp > stamp)
            assert d['feedback'][-1].integral.x >= saved_integral
            passed('gain updates preserve integral output; rejected reset does not clear it')
            settings.force_integral_limit = .02; apply(settings)
            wait_for(lambda: 0 < d['feedback'][-1].integral.x <= .0200001)
            passed('reducing integral limit clamps existing integral state')
            start = len(d['feedback']); apply(settings, True)
            wait_for(lambda: any(0 <= m.integral.x < .005 for m in d['feedback'][start:]))
            assert d['tracking'].calibrated and abs(d['tracking'].external_force.y) < 1e-7
            passed('explicit feedback reset clears PI state and preserves observer calibration')
            settings.force_ki = 0; apply(settings)
            wait_for(lambda: d['feedback'][-1].integral.x == 0 and d['feedback'][-1].pi_unfiltered.x == 0)
            passed('zero I gain clears integral even when upper reference enables integration')
            settings.force_kp.x = 1.; settings.force_filter_tau = .2
            start = len(d['feedback']); apply(settings, True)
            wait_for(lambda: any(abs(m.pi_unfiltered.x-1) < 1e-7 and 0 < m.pi_filtered.x < .9
                                 for m in d['feedback'][start:]))
            passed('PI output low-pass filter can be enabled during tracking')
            before_filter = d['feedback'][-1].pi_filtered.x
            stamp = d['feedback'][-1].header.stamp
            settings.force_filter_tau = .3; apply(settings)
            wait_for(lambda: d['feedback'][-1].header.stamp > stamp)
            assert d['feedback'][-1].pi_filtered.x >= before_filter
            passed('filter time-constant update preserves the filter output state')
            settings.external_filter_tau = 0; apply(settings)
            d['force'] = [3.2, -1., .5]
            wait_for(lambda: abs(d['tracking'].external_force.y-.2) < 1e-7)
            d['force'] = [3., -1., .5]
            wait_for(lambda: abs(d['tracking'].external_force.y) < 1e-7)
            passed('observer low-pass filter can be bypassed without recalibration')
            settings.force_kp = Vector3(); settings.force_filter_tau = 0; settings.velocity_damping = 1
            apply(settings, True)
            d['reference'] = dict(enable_force_control=True, force=Vector3(1, 0, 0), velocity=Vector3(.1, 0, 0))
            wait_for(lambda: abs(d['feedback'][-1].velocity_damping.x-.1) < 1e-7)
            wait_for(lambda: abs(d['tracking'].commanded_acceleration.x-(1/1.84199+.1)) < 1e-5)
            before_acceleration = d['tracking'].commanded_acceleration.x
            settings.velocity_damping = 3; apply(settings)
            wait_for(lambda: abs(d['tracking'].commanded_acceleration.x-before_acceleration-.2) < 1e-5)
            passed('live velocity damping changes actual acceleration')
            apply(initial, True)
            assert not d['tracking'].fault
            # The original motion/calibration checks continue with the YAML defaults restored.
            # Exercise all ordinary navigation modes and baselink attitude passthrough.
            from geometry_msgs.msg import Vector3
            for mode in [FlightNav.POS_MODE, FlightNav.VEL_MODE, FlightNav.ACC_MODE]:
                nav = FlightNav(); nav.control_frame = nav.WORLD_FRAME; nav.target = nav.COG
                nav.pos_xy_nav_mode = nav.pos_z_nav_mode = mode
                nav.target_pos_x = 1.2; nav.target_pos_y = -.4; nav.target_pos_z = .8
                nav.target_vel_x = .1; nav.target_vel_y = -.05; nav.target_vel_z = .02
                nav.target_acc_x = .2; nav.target_acc_y = -.1; nav.target_acc_z = .05
                d['reference'] = dict(enable_force_control=False, navigation=nav, baselink_rpy=Vector3(.2, .1, .3))
                wait_for(lambda: d['observed_nav'] and d['observed_nav'].pos_xy_nav_mode == mode)
                observed = d['observed_nav']
                assert observed.pos_z_nav_mode == mode
                assert abs(observed.target_pos_x - 1.2) < 1e-6
                assert abs(observed.target_vel_y + .05) < 1e-6
                expected_acc_z = 0.0 if mode == FlightNav.POS_MODE else nav.target_acc_z
                assert abs(observed.target_acc_z - expected_acc_z) < 1e-6
                wait_for(lambda: d['observed_attitude'] and abs(d['observed_attitude'].vector.x-.2) < 1e-6 and abs(d['observed_attitude'].vector.z-.3) < 1e-6)
                assert not d['tracking'].fault
                passed('position/velocity/acceleration plus attitude routing mode %d' % mode)
            nav = FlightNav(); nav.target = nav.BASELINK; nav.control_frame = nav.LOCAL_FRAME
            nav.pos_xy_nav_mode = nav.pos_z_nav_mode = nav.VEL_MODE; nav.target_vel_x = .12
            d['reference'] = dict(navigation=nav)
            wait_for(lambda: d['observed_nav'] and d['observed_nav'].target == nav.BASELINK)
            assert d['observed_nav'].control_frame == nav.LOCAL_FRAME
            assert abs(d['observed_nav'].target_vel_x - .12) < 1e-6
            passed('ordinary FlightNav local frame and baselink target retained')
            # A force loop needs ACC references, rather than fighting a position loop.
            d['reference'] = dict(enable_force_control=True, navigation=FlightNav())
            wait_for(lambda: d['tracking'].fault)
            passed('incompatible force and navigation modes rejected')
            reset = rospy.ServiceProxy('/native/controller/set_reference_mode', SetBool)
            d['reference'] = None; reset(False); reset(True); time.sleep(.1)
            d['reference'] = dict(enable_force_control=False); wait_for(lambda: d['tracking'].active)
            d['reference'] = None
            wait_for(lambda: 'timeout' in d['tracking'].fault)
            passed('missing upper reference stops tracking and latches a fault')
            settings = copy.deepcopy(initial); settings.force_filter_tau = .05
            apply(settings); reset(False); reset(True)
            bad = copy.deepcopy(settings); bad.force_ki = -1
            response = settings_service(bad, False)
            assert not response.success and response.applied == settings
            apply(initial)
            passed('runtime settings survive reference mode off and on')
            d['feed'] = False
        assert all(p.poll() is None for p in processes)
        (out/'summary.json').write_text(json.dumps(dict(result='pass', master=uri, checks=results), indent=2)+'\n')
        print('PASS: %d checks' % len(results), flush=True)
    finally:
        stop.set()
        if feeder:
            feeder.join(timeout=2)
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL); p.wait(timeout=5)
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
