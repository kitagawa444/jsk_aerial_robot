#!/usr/bin/env python

from __future__ import print_function

import math

import rospy
from std_msgs.msg import Float32

from mujoco_triple_transport_place_demo import \
    MujocoTripleTransportPlaceDemo


class MujocoFourBoxTransportPlaceDemo(MujocoTripleTransportPlaceDemo):
    """Classical four-Bee grasp/transport/place demo for a box payload."""

    ACTIVE_NAMES = ("bee1", "bee2", "bee3", "bee4")
    MISSION_TOPIC_ROOT = "/mujoco_four_box_transport_place"
    FACE_ANGLES = (0.0, math.pi / 2.0, math.pi,
                   3.0 * math.pi / 2.0)

    def __init__(self):
        if not rospy.has_param("~target_normal_force"):
            rospy.set_param("~target_normal_force", 6.0)
        if not rospy.has_param("~observer_timeout"):
            rospy.set_param("~observer_timeout", 3.0)
        if not rospy.has_param("~safety_input_timeout"):
            rospy.set_param("~safety_input_timeout", 3.0)
        requested_strategy = str(rospy.get_param(
            "~control_strategy", "legacy")).lower()
        if requested_strategy == "science_robotics":
            requested_strategy = "whole_body_mpc"
        if requested_strategy != "legacy":
            raise ValueError(
                "the four-Bee box demo currently supports "
                "control_strategy:=legacy only")
        if not rospy.has_param("~wrench_log_path"):
            rospy.set_param(
                "~wrench_log_path", "/tmp/bee_four_box_transport_place.csv")
        # The 0.30 m box contact arm is about 1.7 times the previous triangle
        # inradius.  Its observer-derived arm moment is also noticeably more
        # conservative during the support-force transfer.  Keep the direct
        # tilt-angle abort and all feedback gains, but avoid false wrench-only
        # aborts while the measured box remains level.
        if not rospy.has_param("~lift_abort_moment_threshold"):
            rospy.set_param("~lift_abort_moment_threshold", 0.25)

        super(MujocoFourBoxTransportPlaceDemo, self).__init__()

        bringup_ns = "/bee_mujoco_multi_bringup/"
        self.object_shape = str(rospy.get_param(
            bringup_ns + "object_shape", "triangular_prism")).lower()
        self.object_size_x = rospy.get_param(
            bringup_ns + "object_size_x", 0.60)
        self.object_size_y = rospy.get_param(
            bringup_ns + "object_size_y", 0.60)
        if self.object_shape not in ("box", "rectangular_prism"):
            raise ValueError(
                "launch the four-Bee demo with object_shape:=box")
        # A square horizontal section gives all four identical radial loops.
        # Height remains independent, so the payload is a rectangular box.
        if abs(self.object_size_x - self.object_size_y) > 1.0e-6:
            raise ValueError(
                "four-Bee demo requires object_size_x == object_size_y; "
                "got {:.3f} and {:.3f} m".format(
                    self.object_size_x, self.object_size_y))
        self.box_face_radius = self.object_size_x / 2.0
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
            "four-box demo: four Bees on +/-X,+/-Y faces; payload "
            "%.3f x %.3f x %.3f m, %.3f kg",
            self.object_size_x, self.object_size_y, self.object_height,
            self.grasp_object_mass)

    def grasp_face_radius(self):
        return self.box_face_radius

    def face_frame(self, name, object_pose=None):
        """Return the current frame of the assigned box side face."""
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
        rospy.loginfo(
            "four-box demo: Bee1..Bee4 and object data ready")
        return True

    def publish_interactive_state(self, observer_forces):
        # Retain the original aggregate topics for plotting compatibility and
        # add one scalar topic per vehicle so Bee4 is not hidden by Vector3.
        super(MujocoFourBoxTransportPlaceDemo,
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
    rospy.init_node("bee_mujoco_four_box_transport_place_demo")
    demo = MujocoFourBoxTransportPlaceDemo()
    try:
        if not demo.run():
            raise SystemExit(1)
    finally:
        if demo.odometry:
            demo.publish_stop()


if __name__ == "__main__":
    main()
