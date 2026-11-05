"""ROS-independent input checks for the hardware grasp entry point."""

import math


def frame_name(value):
    return value.lstrip('/')


def packet_error(message, kind, expected_frame, now, timeout):
    if frame_name(message.header.frame_id) != frame_name(expected_frame):
        return 'unexpected frame: ' + message.header.frame_id
    stamp = message.header.stamp.to_sec()
    if not math.isfinite(stamp) or stamp <= 0 or not -0.05 <= now - stamp <= timeout:
        return 'missing, stale or future timestamp'
    if kind == 'wrench':
        vectors = (message.wrench.force, message.wrench.torque)
        values = [getattr(v, axis) for v in vectors for axis in 'xyz']
    else:
        pose = message.pose.pose if kind == 'odom' else message.pose
        q = pose.orientation
        values = [getattr(pose.position, axis) for axis in 'xyz']
        values += [q.x, q.y, q.z, q.w]
        norm = sum(v * v for v in (q.x, q.y, q.z, q.w))
        if not math.isfinite(norm) or abs(norm - 1.0) > 0.02:
            return 'invalid quaternion'
        if kind == 'odom':
            values += [getattr(v, axis) for v in
                       (message.twist.twist.linear, message.twist.twist.angular)
                       for axis in 'xyz']
    if not all(math.isfinite(v) for v in values):
        return 'non-finite state'
    return None


def freshness_error(packets, required, now_wall, now_ros, timeout):
    for key in required:
        if key not in packets:
            return 'missing input: ' + key
        receipt, stamp, error = packets[key]
        if error:
            return key + ': ' + error
        if now_wall - receipt > timeout:
            return 'stale input: ' + key
        if stamp is not None and not -0.05 <= now_ros - stamp <= timeout:
            return 'stale source timestamp: ' + key
    return None
