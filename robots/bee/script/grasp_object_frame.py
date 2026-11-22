#!/usr/bin/env python3
"""Publish the calibrated payload frame and CAD marker independently of flight control."""
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import euler_matrix, quaternion_matrix, quaternion_from_euler, quaternion_from_matrix
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


def main():
    rospy.init_node('grasp_object_frame')
    namespace = rospy.get_param('~scene_namespace', '/bee_grasp/scene')
    world = rospy.get_param('~world_frame', 'world').lstrip('/')
    frame = rospy.get_param('~object_frame', 'grasp_object')
    if not frame or frame.startswith('/') or frame == world:
        raise ValueError('Expected a distinct TF child frame name')
    while not rospy.is_shutdown():
        model = rospy.get_param(namespace, {})
        if model.get('available'):
            break
        rospy.rostime.wallsleep(.1)
    if rospy.is_shutdown():
        return
    offset = euler_matrix(*model.get('marker_to_model_rpy', [0]*3))
    offset[:3,3] = model.get('marker_to_model_xyz', [0]*3)
    broadcaster = TransformBroadcaster()
    publisher = rospy.Publisher('/bee_cooperative_grasp/object_mesh', Marker, queue_size=1, latch=True)
    resource = model.get('object_mesh')
    mesh = None
    last_mesh_stamp = None
    if resource:
        mesh = Marker()
        mesh.header.frame_id = frame
        mesh.ns = 'payload'
        mesh.type = Marker.MESH_RESOURCE
        if not resource.startswith(('/', 'package://')):
            raise ValueError('object_mesh must be an absolute path or a package URI')
        mesh.mesh_resource = 'file://' + resource if resource.startswith('/') else resource
        mesh.frame_locked = True
        mesh.scale.x, mesh.scale.y, mesh.scale.z = model.get('mesh_scale', [1]*3)
        mesh.pose.position.x, mesh.pose.position.y, mesh.pose.position.z = model.get('mesh_xyz', [0]*3)
        q = mesh.pose.orientation
        q.x,q.y,q.z,q.w = quaternion_from_euler(*model.get('mesh_rpy', [0]*3))
        mesh.color.r, mesh.color.g, mesh.color.b, mesh.color.a = .92, .45, .08, 1.

    def update(msg):
        nonlocal last_mesh_stamp
        if msg.header.frame_id.lstrip('/') != world or msg.header.stamp.is_zero():
            return
        q = msg.pose.orientation
        p = msg.pose.position
        quat = [q.x,q.y,q.z,q.w]
        if not np.isfinite([p.x,p.y,p.z]).all() or not abs(np.linalg.norm(quat)-1.) < .05:
            return
        pose = quaternion_matrix(quat)
        pose[:3,3] = [p.x,p.y,p.z]
        com = pose @ offset
        tf = TransformStamped(header=msg.header, child_frame_id=frame)
        tf.header.frame_id = world
        v = tf.transform.translation
        v.x,v.y,v.z = com[:3,3]
        r = tf.transform.rotation
        r.x,r.y,r.z,r.w = quaternion_from_matrix(com)
        broadcaster.sendTransform(tf)
        stamp = msg.header.stamp
        if mesh is not None and (last_mesh_stamp is None or stamp < last_mesh_stamp or
                                 (stamp - last_mesh_stamp).to_sec() >= 1.0):
            publisher.publish(mesh)
            last_mesh_stamp = stamp

    rospy.Subscriber(rospy.get_param('~object_pose_topic', '/mocap/object/pose'), PoseStamped, update, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
