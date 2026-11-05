#!/usr/bin/env python

from __future__ import print_function

import math

import rospy
from std_msgs.msg import Float32

from mujoco_triple_transport_place_demo import \
    MujocoTripleTransportPlaceDemo


class MujocoFivePentagonTransportPlaceDemo(
        MujocoTripleTransportPlaceDemo):
    """Classical five-Bee transport/place demo for a regular pentagon."""

    ACTIVE_NAMES = ("bee1", "bee2", "bee3", "bee4", "bee5")
    MISSION_TOPIC_ROOT = "/mujoco_five_pentagon_transport_place"
    FACE_ANGLES = tuple(
        math.pi / 5.0 + 2.0 * math.pi * index / 5.0
        for index in range(5))

    def __init__(self):
        if not rospy.has_param("~target_normal_force"):
            rospy.set_param("~target_normal_force", 6.0)
        if not rospy.has_param("~observer_timeout"):
            rospy.set_param("~observer_timeout", 4.0)
        if not rospy.has_param("~safety_input_timeout"):
            rospy.set_param("~safety_input_timeout", 4.0)
        requested_strategy = str(rospy.get_param(
            "~control_strategy", "legacy")).lower()
        if requested_strategy == "science_robotics":
            requested_strategy = "whole_body_mpc"
        if requested_strategy != "legacy":
            raise ValueError(
                "the five-Bee pentagon demo currently supports "
                "control_strategy:=legacy only")
        if not rospy.has_param("~wrench_log_path"):
            rospy.set_param(
                "~wrench_log_path",
                "/tmp/bee_five_pentagon_transport_place.csv")
        if not rospy.has_param("~lift_abort_moment_threshold"):
            rospy.set_param("~lift_abort_moment_threshold", 0.40)

        super(MujocoFivePentagonTransportPlaceDemo, self).__init__()

        bringup_ns = "/bee_mujoco_multi_bringup/"
        self.object_shape = str(rospy.get_param(
            bringup_ns + "object_shape", "triangular_prism")).lower()
        self.pentagon_side = rospy.get_param(
            bringup_ns + "object_pentagon_side", 0.60)
        if self.object_shape != "pentagonal_prism":
            raise ValueError(
                "launch the five-Bee demo with "
                "object_shape:=pentagonal_prism")
        if self.pentagon_side <= 0.0:
            raise ValueError("object_pentagon_side must be positive")
        self.pentagon_face_radius = self.pentagon_side / (
            2.0 * math.tan(math.pi / 5.0))
        self.normal_force_state_pubs = dict(
            (name, rospy.Publisher(
                self.MISSION_TOPIC_ROOT +
                "/{}/observer_normal_force".format(name),
                Float32, queue_size=1)) for name in self.names)
        self.normal_force_target_pubs = dict(
            (name, rospy.Publisher(
                self.MISSION_TOPIC_ROOT +
                "/{}/target_normal_force".format(name),
                Float32, queue_size=1)) for name in self.names)
        self.lift_z_offset_pubs = dict(
            (name, rospy.Publisher(
                self.MISSION_TOPIC_ROOT +
                "/{}/lift_z_acceleration_offset".format(name),
                Float32, queue_size=1)) for name in self.names)
        rospy.loginfo(
            "five-pentagon demo: five Bees on regular side faces; payload "
            "side %.3f x height %.3f m, %.3f kg",
            self.pentagon_side, self.object_height,
            self.grasp_object_mass)

    def grasp_face_radius(self):
        return self.pentagon_face_radius

    def face_frame(self, name, object_pose=None):
        """Return the current frame of the assigned pentagon side face."""
        if object_pose is None:
            object_pose = self.object_pose
        object_yaw = self.object_yaw
        if object_pose is not None:
            object_yaw = self.quaternion_to_rpy(object_pose.orientation)[2]
        angle = object_yaw + self.FACE_ANGLES[self.names.index(name)]
        normal = (math.cos(angle), math.sin(angle))
        tangent = (-normal[1], normal[0])
        inward_yaw = math.atan2(-normal[1], -normal[0])
        approach_yaw = self.angle_error(
            inward_yaw - math.pi / 2.0, 0.0)
        return normal, tangent, approach_yaw

    def wait_for_triple_data(self, timeout=20.0):
        if not self.wait_for_dual_data(timeout):
            return False
        rospy.loginfo("five-pentagon demo: Bee1..Bee5 and object data ready")
        return True

    def publish_interactive_state(self, observer_forces):
        super(MujocoFivePentagonTransportPlaceDemo,
              self).publish_interactive_state(observer_forces)
        for name in self.names:
            self.normal_force_state_pubs[name].publish(
                Float32(data=observer_forces.get(name, 0.0)))
            self.normal_force_target_pubs[name].publish(
                Float32(data=self.target_normal_forces[name]))
            self.lift_z_offset_pubs[name].publish(Float32(data=(
                self.lift_z_acceleration_offsets[name] +
                self.lift_contact_z_acceleration_offsets[name])))


def main():
    rospy.init_node("bee_mujoco_five_pentagon_transport_place_demo")
    demo = MujocoFivePentagonTransportPlaceDemo()
    try:
        if not demo.run():
            raise SystemExit(1)
    finally:
        if demo.odometry:
            demo.publish_stop()


if __name__ == "__main__":
    main()
