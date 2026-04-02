#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from aerial_robot_msgs.msg import FlightNav


class CmdVelToFlightNav:
    def __init__(self):
        self.pub = rospy.Publisher("uav/nav", FlightNav, queue_size=1)
        self.sub = rospy.Subscriber("cmd_vel", Twist, self.callback)
        rospy.loginfo("cmd_vel_to_flight_nav: started")

    def callback(self, msg):
        nav = FlightNav()
        nav.header.stamp = rospy.Time.now()

        nav.control_frame = FlightNav.WORLD_FRAME
        nav.target = FlightNav.COG

        # XY velocity
        nav.pos_xy_nav_mode = FlightNav.VEL_MODE
        nav.target_vel_x = msg.linear.x
        nav.target_vel_y = msg.linear.y

        # Z velocity
        nav.pos_z_nav_mode = FlightNav.VEL_MODE
        nav.target_vel_z = msg.linear.z

        # Yaw velocity
        nav.yaw_nav_mode = FlightNav.VEL_MODE
        nav.target_omega_z = msg.angular.z

        self.pub.publish(nav)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_to_flight_nav")
    node = CmdVelToFlightNav()
    rospy.spin()
