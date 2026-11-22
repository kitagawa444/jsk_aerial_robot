#!/usr/bin/env python3
"""Express simulated CoM ground truth at the mocap marker origin."""
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_matrix, quaternion_matrix, quaternion_from_matrix


def main():
    rospy.init_node('sim_object_mocap')
    scene = rospy.get_param('~scene_namespace', '/bee_grasp/scene')
    pub = rospy.Publisher('/mocap/object/pose', PoseStamped, queue_size=1)
    model_to_marker = None

    def publish(msg):
        nonlocal model_to_marker
        if model_to_marker is None:
            manifest = rospy.get_param(scene, {})
            if not manifest.get('available'):
                return
            offset = euler_matrix(*manifest.get('marker_to_model_rpy', [0]*3))
            offset[:3,3] = manifest.get('marker_to_model_xyz', [0]*3)
            model_to_marker = np.linalg.inv(offset)
        q = msg.pose.orientation
        p = msg.pose.position
        world = quaternion_matrix([q.x, q.y, q.z, q.w])
        world[:3,3] = [p.x, p.y, p.z]
        marker = world @ model_to_marker
        out = PoseStamped(header=msg.header)
        out.pose.position.x, out.pose.position.y, out.pose.position.z = marker[:3,3]
        rot = out.pose.orientation
        rot.x, rot.y, rot.z, rot.w = quaternion_from_matrix(marker)
        pub.publish(out)
    rospy.Subscriber('/bee1/mujoco/grasp_object_pose', PoseStamped, publish, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
