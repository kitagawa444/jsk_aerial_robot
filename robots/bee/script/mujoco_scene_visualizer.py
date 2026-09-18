#!/usr/bin/env python

from __future__ import division

import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


class MujocoSceneVisualizer(object):
    LANDING_FEET = (
        ("front", 0.069, 0.0),
        ("rear", -0.069, 0.0),
        ("left", 0.0, 0.069),
        ("right", 0.0, -0.069),
    )
    def __init__(self):
        self.lock = threading.Lock()
        self.robot_count = rospy.get_param("~robot_count", 3)
        self.robot_names = [
            "bee{}".format(index + 1) for index in range(self.robot_count)
        ]
        self.foot_compressions = {}
        self.spawn_object = rospy.get_param("~spawn_object", True)
        self.spawn_pedestal = rospy.get_param("~spawn_object_pedestal", True)
        self.object_x = rospy.get_param("~object_x", 0.0)
        self.object_y = rospy.get_param("~object_y", 1.0)
        self.object_yaw = rospy.get_param("~object_yaw", 0.0)
        self.object_height = rospy.get_param("~object_height", 0.30)
        self.triangle_side = rospy.get_param("~object_triangle_side", 0.80)
        self.ground_clearance = rospy.get_param("~object_ground_clearance", 0.002)
        self.pedestal_height = rospy.get_param("~object_pedestal_height", 0.65)
        self.pedestal_radius = rospy.get_param("~object_pedestal_radius", 0.20)
        support_height = self.pedestal_height if self.spawn_pedestal else 0.0

        self.object_pose = PoseStamped()
        self.object_pose.header.frame_id = "world"
        self.object_pose.pose.position.x = self.object_x
        self.object_pose.pose.position.y = self.object_y
        self.object_pose.pose.position.z = (
            support_height + self.ground_clearance + self.object_height / 2.0
        )
        self.object_pose.pose.orientation.z = math.sin(self.object_yaw / 2.0)
        self.object_pose.pose.orientation.w = math.cos(self.object_yaw / 2.0)
        self.publisher = rospy.Publisher(
            "/mujoco/grasp_scene_markers", MarkerArray, queue_size=1, latch=True
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        if self.spawn_object:
            rospy.Subscriber(
                "/bee1/mujoco/grasp_object_pose",
                PoseStamped,
                self.object_pose_callback,
                queue_size=1,
            )
        for name in self.robot_names:
            rospy.Subscriber(
                "/{}/mujoco/grasp_contact_states".format(name),
                JointState,
                self.contact_state_callback,
                callback_args=name,
                queue_size=1,
            )
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_markers)

    def object_pose_callback(self, message):
        with self.lock:
            self.object_pose = message

    def contact_state_callback(self, message, robot_name):
        compressions = dict(zip(message.name, message.position))
        with self.lock:
            self.foot_compressions[robot_name] = compressions

    @staticmethod
    def set_color(marker, red, green, blue, alpha=1.0):
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

    @staticmethod
    def point(values):
        result = Point()
        result.x, result.y, result.z = values
        return result

    def pedestal_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = stamp
        marker.ns = "grasp_scene"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.object_x
        marker.pose.position.y = self.object_y
        marker.pose.position.z = self.pedestal_height / 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0 * self.pedestal_radius
        marker.scale.y = 2.0 * self.pedestal_radius
        marker.scale.z = self.pedestal_height
        self.set_color(marker, 0.35, 0.38, 0.42)
        return marker

    def prism_marker(self, stamp, pose):
        radius = self.triangle_side / math.sqrt(3.0)
        rear_x = -radius / 2.0
        side_y = self.triangle_side / 2.0
        z_min = -self.object_height / 2.0
        z_max = self.object_height / 2.0
        vertices = (
            (radius, 0.0, z_min),
            (rear_x, side_y, z_min),
            (rear_x, -side_y, z_min),
            (radius, 0.0, z_max),
            (rear_x, side_y, z_max),
            (rear_x, -side_y, z_max),
        )
        faces = (
            (0, 2, 1),
            (3, 4, 5),
            (0, 1, 4),
            (0, 4, 3),
            (1, 2, 5),
            (1, 5, 4),
            (2, 0, 3),
            (2, 3, 5),
        )
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = stamp
        marker.ns = "grasp_scene"
        marker.id = 1
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        self.set_color(marker, 0.92, 0.45, 0.08)
        for face in faces:
            for index in face:
                marker.points.append(self.point(vertices[index]))
        return marker

    def publish_landing_foot_transforms(self, stamp, foot_compressions):
        transforms = []
        # The passive MuJoCo landing-foot joints are intentionally excluded
        # from ros_control. Publish sensed compression as TF so RViz renders
        # the actual physical feet without separate contact-point markers.
        for name in self.robot_names:
            robot_compressions = foot_compressions.get(name, {})
            for foot_name, foot_x, foot_y in self.LANDING_FEET:
                joint_name = "spring_foot_{}".format(foot_name)
                transform = TransformStamped()
                transform.header.stamp = stamp
                transform.header.frame_id = "{}/base_link".format(name)
                transform.child_frame_id = "{}/{}".format(name, joint_name)
                transform.transform.translation.x = foot_x
                transform.transform.translation.y = foot_y
                transform.transform.translation.z = (
                    -0.083 + robot_compressions.get(joint_name, 0.0)
                )
                transform.transform.rotation.w = 1.0
                transforms.append(transform)
        self.tf_broadcaster.sendTransform(transforms)

    def publish_markers(self, _event):
        if rospy.is_shutdown():
            return
        with self.lock:
            object_pose = self.object_pose.pose
            foot_compressions = dict(
                (name, dict(values))
                for name, values in self.foot_compressions.items()
            )
        stamp = rospy.Time.now()
        self.publish_landing_foot_transforms(stamp, foot_compressions)
        message = MarkerArray()
        if self.spawn_pedestal:
            message.markers.append(self.pedestal_marker(stamp))
        if self.spawn_object:
            message.markers.append(self.prism_marker(stamp, object_pose))
        if not rospy.is_shutdown():
            self.publisher.publish(message)


def main():
    rospy.init_node("bee_mujoco_scene_visualizer")
    MujocoSceneVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
