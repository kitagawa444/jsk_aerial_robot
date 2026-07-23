#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main flight only:
- roll/pitch: sinusoidal within [min,max] with period
- yaw: constant-rate rotation
- When yaw completes 2 revolutions (4π rad traveled), smoothly reset roll/pitch to 0 over reset_duration and exit.

ROS params (defaults):
  ~rate_hz: 100

  # roll bounds [rad] and period [s]
  ~roll_min:  -0.35
  ~roll_max:   0.35
  ~roll_period: 6.0
  ~roll_phase:  0.0

  # pitch bounds [rad] and period [s]
  ~pitch_min: -0.25
  ~pitch_max:  0.25
  ~pitch_period: 8.0
  ~pitch_phase:  0.0

  # yaw constant rate [rad/s]
  ~yaw_init: 0.0
  ~yaw_rate: 0.5   # rad/s (positive: CCW)

  # reset
  ~reset_duration: 3.0  # seconds to ramp roll/pitch to 0 after 2 yaw revolutions

  # if position-disable mode doesn't exist, hold position
  ~hold_x: 0.0
  ~hold_y: 0.0
  ~hold_z: 1.0
"""
import rospy
import numpy as np
from aerial_robot_msgs.msg import FlightNav


def sin_with_bounds(t: float, vmin: float, vmax: float, period: float, phase: float = 0.0) -> float:
    mid = 0.5 * (vmin + vmax)
    amp = 0.5 * (vmax - vmin)
    if period is None or period <= 0.0 or amp == 0.0:
        return float(mid)
    return float(mid + amp * np.sin(2.0 * np.pi * t / period + phase))


def wrap_to_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def pick_disable_mode():
    for name in ("NO_MODE", "NO_NAV_MODE", "DISABLE_MODE", "DISABLE"):
        if hasattr(FlightNav, name):
            return getattr(FlightNav, name)
    return None


def main():
    rospy.init_node("sin_rp_const_yaw_2rev_reset")

    rate_hz = float(rospy.get_param("~rate_hz", 100.0))

    # roll params
    roll_min    = float(rospy.get_param("~roll_min", -0.35))
    roll_max    = float(rospy.get_param("~roll_max",  0.35))
    roll_period = float(rospy.get_param("~roll_period", 6.0))
    roll_phase  = float(rospy.get_param("~roll_phase", 0.0))

    # pitch params
    pitch_min    = float(rospy.get_param("~pitch_min", -0.25))
    pitch_max    = float(rospy.get_param("~pitch_max",  0.25))
    pitch_period = float(rospy.get_param("~pitch_period", 8.0))
    pitch_phase  = float(rospy.get_param("~pitch_phase", 0.0))

    # yaw params (constant rate)
    yaw_init = float(rospy.get_param("~yaw_init", 0.0))
    yaw_rate = float(rospy.get_param("~yaw_rate", 0.5))  # rad/s

    # reset params
    reset_duration = float(rospy.get_param("~reset_duration", 3.0))

    # fallback position hold
    hold_x = float(rospy.get_param("~hold_x", 0.0))
    hold_y = float(rospy.get_param("~hold_y", 0.0))
    hold_z = float(rospy.get_param("~hold_z", 1.0))

    disable_mode = pick_disable_mode()

    nav_pub = rospy.Publisher("/ninja1/uav/nav", FlightNav, queue_size=1)
    rate = rospy.Rate(rate_hz)

    start_time = rospy.Time.now()
    target_travel = 4.0 * np.pi  # 2 revolutions
    rospy.loginfo("Started. yaw_rate=%.3f rad/s. Stop after 2 rev (travel=4π). Then reset roll/pitch over %.2f s.",
                  yaw_rate, reset_duration)

    # ---- main loop until 2 yaw revolutions ----
    last_yaw = yaw_init
    last_roll = 0.0
    last_pitch = 0.0

    while not rospy.is_shutdown():
        t = (rospy.Time.now() - start_time).to_sec()
        yaw_travel = abs(yaw_rate) * t

        # commands
        last_roll  = sin_with_bounds(t, roll_min,  roll_max,  roll_period,  roll_phase)
        last_pitch = sin_with_bounds(t, pitch_min, pitch_max, pitch_period, pitch_phase)
        last_yaw   = wrap_to_pi(yaw_init + yaw_rate * t)

        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()

        # Position: disable if possible, else hold
        if disable_mode is not None:
            msg.pos_xy_nav_mode = disable_mode
            msg.pos_z_nav_mode  = disable_mode
        else:
            msg.pos_xy_nav_mode = FlightNav.POS_MODE
            msg.pos_z_nav_mode  = FlightNav.POS_MODE
            msg.target_pos_x = hold_x
            msg.target_pos_y = hold_y
            msg.target_pos_z = hold_z

        # yaw: constant-rate rotation
        msg.yaw_nav_mode = FlightNav.POS_VEL_MODE
        msg.target_yaw = last_yaw
        msg.target_omega_z = yaw_rate

        # roll/pitch: sinusoidal
        msg.roll_nav_mode = FlightNav.POS_MODE
        msg.pitch_nav_mode = FlightNav.POS_MODE
        msg.target_roll = last_roll
        msg.target_pitch = last_pitch

        nav_pub.publish(msg)

        if yaw_travel >= target_travel:
            rospy.loginfo("2 yaw revolutions reached (travel=%.3f rad). Entering reset.", yaw_travel)
            break

        rate.sleep()

    if rospy.is_shutdown():
        return

    # ---- reset: ramp roll/pitch to 0 over reset_duration, hold yaw, stop yaw rotation ----
    reset_start = rospy.Time.now()
    while not rospy.is_shutdown():
        tr = (rospy.Time.now() - reset_start).to_sec()
        frac = min(max(tr / reset_duration, 0.0), 1.0) if reset_duration > 0.0 else 1.0

        roll  = last_roll  * (1.0 - frac)
        pitch = last_pitch * (1.0 - frac)
        yaw   = last_yaw  # hold final yaw

        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()

        if disable_mode is not None:
            msg.pos_xy_nav_mode = disable_mode
            msg.pos_z_nav_mode  = disable_mode
        else:
            msg.pos_xy_nav_mode = FlightNav.POS_MODE
            msg.pos_z_nav_mode  = FlightNav.POS_MODE
            msg.target_pos_x = hold_x
            msg.target_pos_y = hold_y
            msg.target_pos_z = hold_z

        # yaw hold, no rotation
        msg.yaw_nav_mode = FlightNav.POS_VEL_MODE
        msg.target_yaw = yaw
        msg.target_omega_z = 0.0

        # roll/pitch ramp to 0
        msg.roll_nav_mode = FlightNav.POS_MODE
        msg.pitch_nav_mode = FlightNav.POS_MODE
        msg.target_roll = roll
        msg.target_pitch = pitch

        nav_pub.publish(msg)

        if frac >= 1.0:
            rospy.loginfo("Reset complete. Exiting.")
            break

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
