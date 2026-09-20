#!/usr/bin/env python3
import sys
import math
import csv
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3Stamped
from aerial_robot_msgs.msg import PoseControlPid
from nav_msgs.msg import Odometry
from differential_allocation_flip_test import FlipTest
from tf.transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion

rospy.init_node('bee_bodyz_wheel_test')
# Validate before sending any flight commands. The last step is clamped to
# exactly end_degrees even when the requested increment does not divide it.
step_degrees = float(rospy.get_param('~step_degrees', 10.0))
step_duration = float(rospy.get_param('~step_duration', 20.0))
final_hold = float(rospy.get_param('~final_hold', 40.0))
end_degrees = float(rospy.get_param('~end_degrees', 360.0))
if not math.isfinite(end_degrees) or not 0 < end_degrees <= 360:
    raise ValueError('end_degrees must be finite and in (0, 360]')
if not math.isfinite(step_degrees) or not 0 < step_degrees <= 45:
    raise ValueError('step_degrees must be finite and in (0, 45]')
if not all(math.isfinite(value) and value >= 3.0 for value in (step_duration, final_hold)):
    raise ValueError('step_duration and final_hold must be finite and >= 3 seconds')
step_count = int(math.ceil(end_degrees / step_degrees))
rospy.loginfo('WHEEL increment %.2f deg, interval %.2f s, final hold %.2f s',
              step_degrees, step_duration, final_hold)
t = FlipTest()
telemetry = {}


def save_telemetry(msg, key):
    telemetry[key] = (rospy.Time.now().to_sec(), msg)


# Ages below are ROS-time since reception, not synchronized sensor timestamps.
pid_sub = rospy.Subscriber('/bee1/debug/pose/pid', PoseControlPid, save_telemetry, 'pid', queue_size=1)
cog_sub = rospy.Subscriber('/bee1/uav/cog/odom', Odometry, save_telemetry, 'cog', queue_size=1)
records = []
ok = True
q0 = quaternion_from_euler(math.pi/2, 0, 0)
rospy.sleep(5)
t.start_time = rospy.Time.now().to_sec()
t.start_pub.publish(Empty())
rospy.sleep(2)
t.takeoff_pub.publish(Empty())
ok = t.wait_for_hover()
try:
    if not ok:
        raise RuntimeError('takeoff timeout')
    if rospy.get_param('~wait_hover_reference', True):
        deadline = rospy.Time.now().to_sec() + 20
        while len(t.debug) < 70 or t.debug[67] <= 0:
            if rospy.Time.now().to_sec() >= deadline:
                raise RuntimeError('hover reference capture timeout')
            rospy.sleep(.1)
        rospy.loginfo('WHEEL fixed hover reference %.6f N', t.debug[67])
    for index in range(step_count + 1):
        theta_degrees = min(index * step_degrees, end_degrees)
        theta = math.radians(theta_degrees)
        qref = quaternion_multiply(q0, quaternion_from_euler(0, 0, theta))
        rpy = euler_from_quaternion(qref)
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.vector.x, msg.vector.y, msg.vector.z = rpy
        t.rotation_pub.publish(msg)
        t.phase = 'bodyz_{:g}'.format(theta_degrees)
        rospy.loginfo('WHEEL command bodyZ=%.2f deg, RPY=%s', theta_degrees, [round(math.degrees(x),2) for x in rpy])
        start = rospy.Time.now().to_sec()
        duration = final_hold if index == step_count else step_duration
        rate = rospy.Rate(50)
        errors = []
        while rospy.Time.now().to_sec()-start < duration and not rospy.is_shutdown():
            # The original pure-roll test rejects pitch > 1.2 rad; here
            # pitch reaches +/- pi/2 intentionally. Use SO(3) error instead.
            t.sample(rpy[0])
            now = rospy.Time.now().to_sec()
            q = t.odom.pose.pose.orientation
            qa = (q.x,q.y,q.z,q.w)
            dot = abs(sum(a*b for a,b in zip(qref,qa)))
            error = math.degrees(2*math.acos(min(1.,dot)))
            p = t.odom.pose.pose.position
            extra = [float('nan')] * 14
            if 'pid' in telemetry:
                stamp, pid = telemetry['pid']
                extra[:7] = [now-stamp, pid.x.err_p, pid.y.err_p, pid.z.err_p,
                             pid.x.target_p, pid.y.target_p, pid.z.target_p]
            if 'cog' in telemetry:
                stamp, cog = telemetry['cog']
                cp, cv = cog.pose.pose.position, cog.twist.twist.linear
                extra[7:] = [now-stamp, cp.x, cp.y, cp.z, cv.x, cv.y, cv.z]
            records.append([now-t.start_time,theta_degrees,now-start,*qref,*qa,p.x,p.y,p.z,error,*extra])
            if not (0.15 < p.z < 2.0) or math.hypot(p.x,p.y)>2 or (now-start>10 and error>60):
                raise RuntimeError('safety stop theta={} z={:.3f} error={:.2f}'.format(theta_degrees,p.z,error))
            if now-start >= duration-3:
                errors.append(error)
            rate.sleep()
        rospy.loginfo('WHEEL terminal bodyZ=%.2f error=%.3f max last3s=%.3f z=%.3f',theta_degrees,error,max(errors),p.z)
        if max(errors)>5:
            raise RuntimeError('terminal attitude error exceeds 5 degrees')
except Exception as exc:
    rospy.logerr('WHEEL failed: %s',exc)
    ok = False
finally:
    t.write_csv()
    with open(rospy.get_param('~pose_output', '/tmp/bee_bodyz_wheel_pose.csv'),'w') as f:
        w=csv.writer(f)
        w.writerow(['time','theta_deg','stage_time','ref_qx','ref_qy','ref_qz','ref_qw','qx','qy','qz','qw','x','y','z','error_deg',
                    'pid_age','err_x','err_y','err_z','target_x','target_y','target_z',
                    'cog_age','cog_x','cog_y','cog_z','cog_vx','cog_vy','cog_vz'])
        w.writerows(records)
    rospy.loginfo('WHEEL result=%s',ok)
    if not ok:
        t.halt_pub.publish(Empty())
        rospy.sleep(.5)
sys.exit(0 if ok else 1)
