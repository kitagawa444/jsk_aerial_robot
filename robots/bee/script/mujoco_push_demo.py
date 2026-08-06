#!/usr/bin/env python

from __future__ import print_function

import math
import threading
import time

import rospy
from aerial_robot_msgs.msg import FlightNav, ForceList
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, UInt8


class MujocoPushDemo(object):
    """Use Bee1's four physical feet to push the prism off its pedestal."""

    def __init__(self):
        self.name = "bee1"
        self.lock = threading.Lock()
        self.flight_state = None
        self.odom = None
        self.forces = None
        self.object_pose = None

        self.object_yaw = rospy.get_param("~object_yaw", 0.0)
        self.triangle_side = rospy.get_param("~object_triangle_side", 0.80)
        self.staging_clearance = rospy.get_param("~grasp_staging_clearance", 0.80)
        self.object_z = rospy.get_param("~object_z", 0.800)
        self.pedestal_radius = rospy.get_param("~object_pedestal_radius", 0.20)

        self.period = rospy.get_param("~control_period", 0.01)
        self.radial_kp = rospy.get_param("~radial_kp", 3.0)
        self.radial_kd = rospy.get_param("~radial_kd", 3.0)
        self.tangent_kp = rospy.get_param("~tangent_kp", 1.2)
        self.tangent_kd = rospy.get_param("~tangent_kd", 3.0)
        self.approach_accel_limit = rospy.get_param("~approach_accel_limit", 0.8)
        self.push_accel_limit = rospy.get_param("~push_accel_limit", 3.0)
        self.approach_speed = rospy.get_param("~approach_speed", 0.035)
        self.target_normal_force = rospy.get_param("~target_normal_force", 5.0)
        self.normal_force_kp = rospy.get_param("~normal_force_kp", 0.4)
        self.push_timeout = rospy.get_param("~push_timeout", 30.0)
        self.drop_height = rospy.get_param("~drop_height", 0.08)
        self.attitude_ramp_duration = rospy.get_param("~attitude_ramp_duration", 3.0)
        self.commanded_roll = 0.0

        self.foot_contact_offset = rospy.get_param("~foot_contact_offset", 0.107)
        self.foot_ball_radius = rospy.get_param("~foot_ball_radius", 0.012)
        self.foot_max_compression = rospy.get_param("~foot_max_compression", 0.015)

        self.start_pub = rospy.Publisher(
            "/bee1/teleop_command/start", Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher(
            "/bee1/teleop_command/takeoff", Empty, queue_size=1)
        self.nav_pub = rospy.Publisher("/bee1/uav/nav", FlightNav, queue_size=1)
        self.rpy_pub = rospy.Publisher(
            "/bee1/final_target_baselink_rpy", Vector3Stamped, queue_size=1)

        rospy.Subscriber("/bee1/flight_state", UInt8, self.flight_state_cb, queue_size=1)
        rospy.Subscriber("/bee1/ground_truth", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(
            "/bee1/mujoco/grasp_forces_world", ForceList, self.force_cb, queue_size=1)
        rospy.Subscriber(
            "/bee1/mujoco/grasp_object_pose", PoseStamped,
            self.object_pose_cb, queue_size=1)

    def flight_state_cb(self, msg):
        with self.lock:
            self.flight_state = msg.data

    def odom_cb(self, msg):
        with self.lock:
            self.odom = msg

    def force_cb(self, msg):
        with self.lock:
            self.forces = list(msg.forces)

    def object_pose_cb(self, msg):
        with self.lock:
            self.object_pose = msg.pose

    def snapshot(self):
        with self.lock:
            return self.flight_state, self.odom, self.forces, self.object_pose

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def face_frame(self):
        # Bee1 is spawned outside the prism face whose outward normal is 60 deg.
        angle = self.object_yaw + math.pi / 3.0
        normal = (math.cos(angle), math.sin(angle))
        tangent = (-normal[1], normal[0])
        inward_yaw = math.atan2(-normal[1], -normal[0])
        yaw = math.atan2(
            math.sin(inward_yaw - math.pi / 2.0),
            math.cos(inward_yaw - math.pi / 2.0))
        return normal, tangent, yaw

    def wait_for_data(self, timeout=20.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, odom, forces, object_pose = self.snapshot()
            if state is not None and odom is not None and object_pose is not None and \
                    forces is not None and len(forces) == 4:
                return True
            time.sleep(0.05)
        return False

    def publish_nav(self, accel_x, accel_y, yaw, z_target):
        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG
        msg.pos_xy_nav_mode = FlightNav.ACC_MODE
        msg.target_acc_x = accel_x
        msg.target_acc_y = accel_y
        msg.pos_z_nav_mode = FlightNav.POS_MODE
        msg.target_pos_z = z_target
        msg.yaw_nav_mode = FlightNav.POS_MODE
        msg.target_yaw = yaw
        msg.roll_nav_mode = FlightNav.POS_MODE
        msg.target_roll = 0.0
        msg.pitch_nav_mode = FlightNav.POS_MODE
        msg.target_pitch = 0.0
        self.nav_pub.publish(msg)

        rpy = Vector3Stamped()
        rpy.header.stamp = msg.header.stamp
        rpy.header.frame_id = "world"
        rpy.vector.x = self.commanded_roll
        self.rpy_pub.publish(rpy)

    def publish_position_hold(self, odom, yaw):
        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG
        msg.pos_xy_nav_mode = FlightNav.POS_MODE
        msg.target_pos_x = odom.pose.pose.position.x
        msg.target_pos_y = odom.pose.pose.position.y
        msg.pos_z_nav_mode = FlightNav.POS_MODE
        msg.target_pos_z = odom.pose.pose.position.z
        msg.yaw_nav_mode = FlightNav.POS_MODE
        msg.target_yaw = yaw
        msg.roll_nav_mode = FlightNav.POS_MODE
        msg.pitch_nav_mode = FlightNav.POS_MODE
        self.nav_pub.publish(msg)

        rpy = Vector3Stamped()
        rpy.header.stamp = msg.header.stamp
        rpy.header.frame_id = "world"
        rpy.vector.x = self.commanded_roll
        self.rpy_pub.publish(rpy)

    def relative_state(self, odom, object_pose):
        normal, tangent, yaw = self.face_frame()
        pos = odom.pose.pose.position
        vel = odom.twist.twist.linear
        dx = pos.x - object_pose.position.x
        dy = pos.y - object_pose.position.y
        radius = dx * normal[0] + dy * normal[1]
        tangent_position = dx * tangent[0] + dy * tangent[1]
        radial_velocity = vel.x * normal[0] + vel.y * normal[1]
        tangent_velocity = vel.x * tangent[0] + vel.y * tangent[1]
        return normal, tangent, yaw, radius, tangent_position, \
            radial_velocity, tangent_velocity

    def approach_acceleration(self, odom, object_pose, desired_radius):
        normal, tangent, yaw, radius, tangent_position, radial_velocity, \
            tangent_velocity = self.relative_state(odom, object_pose)
        radial = self.radial_kp * (desired_radius - radius) - \
            self.radial_kd * radial_velocity
        tangential = -self.tangent_kp * tangent_position - \
            self.tangent_kd * tangent_velocity
        radial = self.clamp(radial, self.approach_accel_limit)
        tangential = self.clamp(tangential, self.approach_accel_limit)
        return (radial * normal[0] + tangential * tangent[0],
                radial * normal[1] + tangential * tangent[1], yaw, radius)

    def normal_force(self, forces, baseline):
        normal, _, _ = self.face_frame()
        total = 0.0
        for force, zero in zip(forces, baseline):
            fx = force.x - zero[0]
            fy = force.y - zero[1]
            total += abs(fx * normal[0] + fy * normal[1])
        return total

    def push_acceleration(self, odom, object_pose, measured_force, desired_radius):
        normal, tangent, yaw, radius, tangent_position, radial_velocity, \
            tangent_velocity = self.relative_state(odom, object_pose)
        force_error = self.target_normal_force - measured_force
        radial = -self.normal_force_kp * force_error - self.radial_kd * radial_velocity
        # Do not let force feedback drive Bee1 through the prism after contact loss.
        if radius < desired_radius - 0.05:
            radial = 20.0 * (desired_radius - radius) - self.radial_kd * radial_velocity
        tangential = -self.tangent_kp * tangent_position - \
            self.tangent_kd * tangent_velocity
        radial = self.clamp(radial, self.push_accel_limit)
        tangential = self.clamp(tangential, self.approach_accel_limit)
        return (radial * normal[0] + tangential * tangent[0],
                radial * normal[1] + tangential * tangent[1], yaw)

    def arm_and_takeoff(self):
        state, _, _, _ = self.snapshot()
        if state == 5:
            return True
        rospy.loginfo("push demo: arm/takeoff Bee1 only")
        connection_deadline = time.monotonic() + 10.0
        while (not rospy.is_shutdown() and
               time.monotonic() < connection_deadline and
               (self.start_pub.get_num_connections() == 0 or
                self.takeoff_pub.get_num_connections() == 0)):
            time.sleep(0.05)
        self.start_pub.publish(Empty())
        time.sleep(0.5)
        self.takeoff_pub.publish(Empty())
        deadline = time.monotonic() + 45.0
        retry_time = time.monotonic() + 2.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, _, _, _ = self.snapshot()
            if state == 5:
                return True
            if state == 0 and time.monotonic() >= retry_time:
                rospy.logwarn("push demo: Bee1 still in STOP; retry arm/takeoff")
                self.start_pub.publish(Empty())
                time.sleep(0.2)
                self.takeoff_pub.publish(Empty())
                retry_time = time.monotonic() + 2.0
            time.sleep(0.05)
        return False

    def run(self):
        if not self.wait_for_data():
            rospy.logerr("push demo: required Bee1 or object topics are missing")
            return False
        if not self.arm_and_takeoff():
            rospy.logerr("push demo: Bee1 did not reach HOVER")
            return False

        _, _, _, initial_object_pose = self.snapshot()
        initial_x = initial_object_pose.position.x
        initial_y = initial_object_pose.position.y
        initial_z = initial_object_pose.position.z
        inradius = self.triangle_side / (2.0 * math.sqrt(3.0))
        staging_radius = inradius + self.staging_clearance
        contact_radius = (inradius + self.foot_contact_offset +
                          self.foot_ball_radius - self.foot_max_compression)

        rospy.loginfo("push demo: recover Bee1 to staging radius %.3f m", staging_radius)
        stage_deadline = time.monotonic() + 90.0
        while not rospy.is_shutdown() and time.monotonic() < stage_deadline:
            _, odom, _, object_pose = self.snapshot()
            ax, ay, yaw, radius = self.approach_acceleration(
                odom, object_pose, staging_radius)
            self.publish_nav(ax, ay, yaw, self.object_z)
            _, _, _, _, tangent_position, radial_velocity, tangent_velocity = \
                self.relative_state(odom, object_pose)
            if (abs(radius - staging_radius) < 0.06 and
                    abs(tangent_position) < 0.06 and
                    abs(radial_velocity) < 0.08 and abs(tangent_velocity) < 0.08):
                break
            time.sleep(self.period)
        else:
            rospy.logerr("push demo: staging timed out")
            return False

        rospy.loginfo("push demo: ramp physical feet toward prism")
        ramp_start = time.monotonic()
        while not rospy.is_shutdown():
            alpha = min(1.0, (time.monotonic() - ramp_start) /
                        max(0.001, self.attitude_ramp_duration))
            blend = alpha * alpha * (3.0 - 2.0 * alpha)
            self.commanded_roll = blend * math.pi / 2.0
            _, odom, _, object_pose = self.snapshot()
            ax, ay, yaw, _ = self.approach_acceleration(
                odom, object_pose, staging_radius)
            self.publish_nav(ax, ay, yaw, self.object_z)
            if alpha >= 1.0:
                break
            time.sleep(self.period)

        _, _, forces, _ = self.snapshot()
        baseline = [(force.x, force.y, force.z) for force in forces]
        desired_radius = staging_radius
        last_sim = rospy.get_time()
        rospy.loginfo("push demo: approach prism with Bee1")
        approach_start = rospy.get_time()
        while not rospy.is_shutdown():
            now = rospy.get_time()
            dt = max(0.0, min(0.1, now - last_sim))
            last_sim = now
            desired_radius = max(
                contact_radius, desired_radius - self.approach_speed * dt)
            _, odom, forces, object_pose = self.snapshot()
            ax, ay, yaw, _ = self.approach_acceleration(
                odom, object_pose, desired_radius)
            self.publish_nav(ax, ay, yaw, self.object_z)
            force = self.normal_force(forces, baseline)
            if desired_radius <= contact_radius + 1.0e-6 and force > 0.2:
                break
            if now - approach_start > 40.0:
                rospy.logerr("push demo: approach timed out")
                return False
            time.sleep(self.period)

        rospy.loginfo(
            "push demo: push at %.2f N target until prism falls from pedestal",
            self.target_normal_force)
        push_start = rospy.get_time()
        while not rospy.is_shutdown():
            _, odom, forces, object_pose = self.snapshot()
            measured_force = self.normal_force(forces, baseline)
            ax, ay, yaw = self.push_acceleration(
                odom, object_pose, measured_force, contact_radius)
            self.publish_nav(ax, ay, yaw, self.object_z)
            displacement = math.hypot(
                object_pose.position.x - initial_x,
                object_pose.position.y - initial_y)
            drop = initial_z - object_pose.position.z
            rospy.loginfo_throttle(
                1.0, "push demo: force %.3f / %.3f N, object displacement "
                "%.3f m, drop %.3f m",
                measured_force, self.target_normal_force, displacement, drop)
            if drop > self.drop_height:
                rospy.loginfo(
                    "push demo: SUCCESS: prism fell from pedestal (drop %.3f m)", drop)
                hold_odom = odom
                while not rospy.is_shutdown():
                    self.publish_position_hold(hold_odom, yaw)
                    time.sleep(0.05)
                return True
            if rospy.get_time() - push_start > self.push_timeout:
                rospy.logerr(
                    "push demo: push timed out; displacement %.3f m, drop %.3f m",
                    displacement, drop)
                return False
            time.sleep(self.period)
        return True


def main():
    rospy.init_node("bee1_mujoco_push_demo")
    demo = MujocoPushDemo()
    success = False
    try:
        success = demo.run()
    finally:
        _, odom, _, _ = demo.snapshot()
        if odom is not None:
            demo.publish_position_hold(odom, demo.face_frame()[2])
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
