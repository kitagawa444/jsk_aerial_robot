#!/usr/bin/env python3
"""Hardware entry point for the three-Bee observer/ACC keyboard grasp demo.

Default: inspect inputs without publishing commands. No automatic arming,
takeoff, landing, simulator forces or force-site feedback.
"""

import math
import sys
import threading
import time

import rospy
from aerial_robot_msgs.msg import FlightNav

from grasp_hardware_safety import packet_error, freshness_error, frame_name
from mujoco_triple_press_keyboard_demo import MujocoTriplePressKeyboardDemo


class RealGraspDemo(MujocoTriplePressKeyboardDemo):
    HARDWARE_MODE = True

    def __init__(self):
        if rospy.get_param('/use_sim_time', False):
            raise ValueError('hardware demo requires use_sim_time=false')
        self.execute = rospy.get_param('~execute', False)
        self.input_timeout = float(rospy.get_param('~input_timeout', 0.3))
        self.command_timeout = float(rospy.get_param('~command_timeout', 0.3))
        self.world_frame = rospy.get_param('~world_frame', 'world')
        if not all(math.isfinite(v) and 0 < v <= 1.0 for v in
                   (self.input_timeout, self.command_timeout)):
            raise ValueError('input/command timeouts must be in (0, 1] seconds')
        # Initialized before the parent subscribes: callbacks may run immediately.
        self.packet_lock = threading.Lock()
        self.command_lock = threading.RLock()
        self.packets = {}
        self.command_active = False
        self.abort_reason = None
        self.last_command_wall = time.monotonic()
        self.watchdog_exit = threading.Event()
        self.cog_frames = rospy.get_param('~cog_frames', {
            name: name + '/cog' for name in self.ACTIVE_NAMES})
        if any(not self.cog_frames.get(name) for name in self.ACTIVE_NAMES):
            raise ValueError('cog_frames must specify all three logical vehicles')
        required = ('object_pose_topic', 'object_mass', 'object_height',
                    'object_triangle_side', 'robot_mass', 'foot_contact_offset',
                    'foot_ball_radius', 'foot_max_compression', 'max_command_acceleration')
        for key in required:
            if not rospy.has_param('~' + key):
                raise ValueError('set measured hardware parameter ~' + key)
        for key in required[1:]:
            value = float(rospy.get_param('~' + key))
            if not math.isfinite(value) or value <= 0:
                raise ValueError('~{} must be finite and positive'.format(key))
        if rospy.get_param('~object_shape', 'triangular_prism') != 'triangular_prism':
            raise ValueError('this hardware entry point supports three Bees / triangular prism only')
        super(RealGraspDemo, self).__init__()
        self.grasp_object_mass = self.object_mass
        self.max_command_acceleration = float(rospy.get_param('~max_command_acceleration'))
        self.required_packets = ['object'] + [name + '/' + kind
            for name in self.names for kind in ('base', 'cog', 'wrench', 'state')]
        if len(set(self.robot_namespaces.get(n, n).strip('/') for n in self.names)) != 3:
            raise ValueError('three distinct robot namespaces are required')
        self.watchdog = threading.Thread(target=self.watchdog_loop, daemon=True)

    def accept_packet(self, key, msg, kind, frame):
        error = packet_error(msg, kind, frame, rospy.get_time(), self.input_timeout)
        with self.packet_lock:
            self.packets[key] = (time.monotonic(), msg.header.stamp.to_sec(), error)
        if error:
            rospy.logwarn_throttle(1.0, '%s: %s', key, error)
        return error is None

    def odometry_cb(self, msg, name):
        if self.accept_packet(name + '/base', msg, 'odom', self.world_frame):
            super(RealGraspDemo, self).odometry_cb(msg, name)

    def cog_odom_cb(self, msg, name):
        if frame_name(msg.child_frame_id) != frame_name(self.cog_frames[name]):
            with self.packet_lock:
                self.packets[name + '/cog'] = (time.monotonic(), None, 'unexpected CoG child frame')
            return
        if self.accept_packet(name + '/cog', msg, 'odom', self.world_frame):
            super(RealGraspDemo, self).cog_odom_cb(msg, name)

    def observer_cb(self, msg, name):
        if self.accept_packet(name + '/wrench', msg, 'wrench', self.cog_frames[name]):
            super(RealGraspDemo, self).observer_cb(msg, name)

    def object_pose_cb(self, msg):
        if self.accept_packet('object', msg, 'pose', self.world_frame):
            super(RealGraspDemo, self).object_pose_cb(msg)

    def flight_state_cb(self, msg, name):
        super(RealGraspDemo, self).flight_state_cb(msg, name)
        with self.packet_lock:
            self.packets[name + '/state'] = (time.monotonic(), None, None)

    def input_reason(self, require_hover=False):
        with self.packet_lock:
            reason = freshness_error(self.packets, self.required_packets,
                                     time.monotonic(), rospy.get_time(), self.input_timeout)
        if reason:
            return reason
        if require_hover and not self.all_hovering():
            return 'all three vehicles must remain in HOVER (state 5)'
        return None

    def wait_for_attitude_controllers(self):
        # Real FC has no ros_control controller_manager service.
        return True

    def arm_and_takeoff(self):
        # Never publish start/takeoff from this entry point.
        return self.all_hovering()

    def stop_commands(self):
        """Clear ACC commands, preserving the last attitude target.

        Zero velocity is a best-effort handover to onboard control, NOT a
        guaranteed safe stop when onboard estimation/comms are lost.
        """
        for name in self.names:
            # Do not override a pilot/controller landing, low-battery or stop state.
            with self.lock:
                if self.flight_states.get(name) != 5:
                    continue
            msg = FlightNav()
            msg.header.stamp = rospy.Time.now()
            msg.control_frame = FlightNav.WORLD_FRAME
            msg.target = FlightNav.COG
            msg.pos_xy_nav_mode = FlightNav.VEL_MODE
            msg.pos_z_nav_mode = FlightNav.VEL_MODE
            msg.yaw_nav_mode = FlightNav.NO_NAVIGATION
            self.nav_pubs[name].publish(msg)

    def latch_abort(self, reason):
        with self.command_lock:
            if not self.command_active:
                return
            if self.abort_reason is None:
                self.abort_reason = reason
                rospy.logerr('Hardware grasp stopped: %s. Operator takeover required.', reason)
            self.stop_requested = True
            self.stop_commands()

    def watchdog_loop(self):
        while not self.watchdog_exit.wait(0.05):
            with self.command_lock:
                if not self.command_active:
                    continue
                reason = self.input_reason(require_hover=True)
                if time.monotonic() - self.last_command_wall > self.command_timeout:
                    reason = reason or 'control loop heartbeat lost'
                if reason or self.abort_reason:
                    self.latch_abort(reason or self.abort_reason)

    def publish_accel_nav(self, name, accel_x, accel_y, yaw,
                          z_mode=FlightNav.POS_MODE, z_value=None, external_force_z=None):
        with self.command_lock:
            reason = self.abort_reason or self.input_reason(require_hover=True)
            values = (accel_x, accel_y, yaw) + (() if z_value is None else (z_value,))
            values += tuple(self.target_baselink_rpy(name))
            if not all(math.isfinite(v) for v in values):
                reason = reason or 'non-finite flight command'
            az = z_value if z_mode == FlightNav.ACC_MODE and z_value is not None else 0.0
            if math.hypot(accel_x, accel_y, az) > self.max_command_acceleration:
                reason = reason or 'flight acceleration exceeds configured limit'
            if not self.execute or not self.command_active:
                raise RuntimeError('flight commands are disabled')
            if reason:
                self.latch_abort(reason)
                raise RuntimeError(reason)
            self.last_command_wall = time.monotonic()
            super(RealGraspDemo, self).publish_accel_nav(
                name, accel_x, accel_y, yaw, z_mode, z_value, external_force_z)

    def publish_stop(self):
        if self.command_active:
            self.latch_abort('demo ended')

    def run(self):
        if self.execute:
            for name in self.names:
                if not rospy.get_param(self.robot_topic(name, 'controller/gravity_compensation'), False):
                    raise RuntimeError(name + ': enable gravity_compensation before controller startup')
        deadline = time.monotonic() + rospy.get_param('~preflight_timeout', 15.0)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            reason = self.input_reason(require_hover=self.execute)
            if reason is None:
                break
            rospy.logwarn_throttle(1.0, 'Hardware preflight: %s', reason)
            time.sleep(0.05)
        else:
            rospy.logerr('Hardware preflight failed; no commands sent')
            return False
        if not self.execute:
            rospy.loginfo('Hardware inputs valid. Check-only mode: no flight commands sent.')
            return True
        if not rospy.get_param('~configuration_verified', False):
            raise RuntimeError('verify physical geometry, frames, gains and emergency procedures; set configuration_verified=true')
        if not sys.stdin.isatty():
            raise RuntimeError('execute requires an interactive terminal (rosrun)')
        if self.target_z_velocity != 0.0:
            raise RuntimeError('initial_z_velocity must be zero; lift must be requested by keyboard')
        for name in self.names:
            if not (self.nav_pubs[name].get_num_connections() and
                    self.baselink_rpy_pubs[name].get_num_connections()):
                raise RuntimeError('missing command subscriber for ' + name)
        _, _, _, pose = self.dual_snapshot()
        # Marker pose must already represent the object's geometric centre.
        self.object_x, self.object_y, self.object_z = (
            pose.position.x, pose.position.y, pose.position.z)
        self.object_yaw = self.quaternion_to_rpy(pose.orientation)[2]
        rospy.logwarn('Starting real grasp approach; no automatic takeoff. Keep independent RC override ready.')
        self.command_active = True
        self.last_command_wall = time.monotonic()
        self.watchdog.start()
        try:
            prepared = self.stage_and_approach()
            if prepared is None:
                return False
            return self.interactive_press(*prepared)
        finally:
            self.publish_stop()
            self.watchdog_exit.set()
            self.watchdog.join(timeout=1.0)
            # Repeat for transient transport loss; never restart the grasp loop.
            for _ in range(10):
                if rospy.is_shutdown():
                    break
                self.stop_commands()
                time.sleep(0.05)
            self.command_active = False


def main():
    rospy.init_node('bee_real_grasp_demo')
    try:
        if not RealGraspDemo().run():
            return 1
    except (RuntimeError, ValueError, KeyError) as exc:
        rospy.logerr('Hardware grasp: %s', exc)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
