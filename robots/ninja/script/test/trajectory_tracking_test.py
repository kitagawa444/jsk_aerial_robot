#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from aerial_robot_msgs.msg import FlightNav


class LemniTrajFollow:
    """
    Publish lemniscate (Bernoulli) trajectory as target positions (x,y).

    Lemniscate of Bernoulli (param t):
        x = a * cos(t) / (1 + sin^2(t))
        y = a * sin(t) * cos(t) / (1 + sin^2(t))

    One lap corresponds to t: 0 -> 2*pi.
    """

    def __init__(self):
        # ---- Parameters (ROS params; with defaults) ----
        self.period = float(rospy.get_param("~period", 10.0))     # [s] per lap
        self.a = float(rospy.get_param("~radius", 1.0))           # scale
        self.init_phase = float(rospy.get_param("~init_theta", 0.0))  # [rad]
        self.center_x = float(rospy.get_param("~center_x", 0.0))
        self.center_y = float(rospy.get_param("~center_y", 0.0))
        self.loops = int(rospy.get_param("~loops", 1))            # number of laps
        self.publish_hz = float(rospy.get_param("~publish_hz", 40.0))

        if self.period <= 0.0:
            raise ValueError("~period must be > 0")
        if self.publish_hz <= 0.0:
            raise ValueError("~publish_hz must be > 0")
        if self.loops <= 0:
            raise ValueError("~loops must be >= 1")

        self.omega = 2.0 * math.pi / self.period  # [rad/s], 1 lap = 2pi

        # ---- Publisher ----
        self.nav_pub = rospy.Publisher("/ninja1/uav/nav", FlightNav, queue_size=1)

        # ---- Message template ----
        self.flight_nav = FlightNav()
        self.flight_nav.target = FlightNav.COG
        self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE

        # Initialize targets explicitly (avoid uninitialized use)
        self.flight_nav.target_pos_x = self.center_x
        self.flight_nav.target_pos_y = self.center_y

        rospy.loginfo(
            "lemni_trajectory_follow: period=%.3f[s], a=%.3f, loops=%d, hz=%.1f",
            self.period, self.a, self.loops, self.publish_hz
        )

    @staticmethod
    def bernoulli_lemniscate(a: float, t: float):
        s = math.sin(t)
        c = math.cos(t)
        denom = 1.0 + s * s  # never 0
        x = a * c / denom
        y = a * s * c / denom
        return x, y

    def run(self):
        rate = rospy.Rate(self.publish_hz)
        start = rospy.Time.now().to_sec()
        total_duration = self.loops * self.period

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            elapsed = now - start

            if elapsed >= total_duration:
                rospy.loginfo("Completed %d loops (%.3f s).", self.loops, total_duration)
                break

            t = self.init_phase + self.omega * elapsed  # phase [rad]
            x, y = self.bernoulli_lemniscate(self.a, t)

            self.flight_nav.target_pos_x = self.center_x + x
            self.flight_nav.target_pos_y = self.center_y + y

            # Optional debug
            # rospy.loginfo("t=%.3f, x=%.3f, y=%.3f", t, self.flight_nav.target_pos_x, self.flight_nav.target_pos_y)

            self.nav_pub.publish(self.flight_nav)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("lemni_trajectory_follow")

    try:
        node = LemniTrajFollow()
        node.run()
    except Exception as e:
        rospy.logerr("lemni_trajectory_follow error: %s", str(e))
        raise
