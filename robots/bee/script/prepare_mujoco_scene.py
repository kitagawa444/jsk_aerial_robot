#!/usr/bin/env python3
"""Prepare a scene before execing the MuJoCo backend (roslaunch launch-prefix).

This process generates XML and waits for vehicle parameters. It does not launch
vehicles, controllers, or nested roslaunch processes.
"""

import argparse
import math
import os
import sys
import time

import rospkg
import rospy
import yaml

from mujoco_scene_generator import MujocoSceneGenerator


def load_scene(path, robot_count):
    with open(path) as source:
        parameters = yaml.safe_load(source)
    if not isinstance(parameters, dict):
        raise ValueError("scene_config must contain a mapping")
    configured_count = parameters.get("robot_count", robot_count)
    if type(configured_count) is not int or configured_count != robot_count:
        raise ValueError("scene_config robot_count must match launch robot_count={}".format(robot_count))
    parameters["robot_count"] = robot_count
    return parameters


def wait_for_vehicle_parameters(names, timeout):
    """Do not let the backend select DefaultRobotHWSim for an unready vehicle."""
    deadline = time.monotonic() + timeout
    previous = None
    while not rospy.is_shutdown():
        pending = []
        for name in names:
            root = "/" + name
            if rospy.get_param(root + "/simulation/robot_hw_sim_plugin_name", "") != "mujoco_ros_control/AerialRobotHWSim":
                pending.append(root + ": AerialRobotHWSim configuration")
            for suffix in ("robot_description", "servo_controller", "motor_info"):
                if not rospy.has_param(root + "/" + suffix):
                    pending.append(root + "/" + suffix)
        if not pending:
            return
        if time.monotonic() >= deadline:
            raise RuntimeError("vehicle parameters not ready: " + ", ".join(pending))
        if pending != previous:
            rospy.loginfo("Waiting for vehicle bringups: %s", ", ".join(pending))
            previous = pending
        time.sleep(.05)
    raise rospy.ROSInterruptException("scene preparation interrupted")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-count", type=int, choices=(3, 4, 5), required=True)
    parser.add_argument("--scene-config", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--manifest-namespace", default="/bee_grasp/scene")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--startup-delay", type=float, default=3.0)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("a backend command is required after --")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("timeout must be positive and finite")
    if not math.isfinite(args.startup_delay) or args.startup_delay < 0:
        parser.error("startup-delay must be nonnegative and finite")

    # Ignore the backend's __name/__log remappings while preparing its scene.
    rospy.init_node("prepare_mujoco_scene", argv=[sys.argv[0]], anonymous=True)
    rospy.set_param(args.manifest_namespace, {"schema_version": 1, "available": False})
    try:
        parameters = load_scene(args.scene_config, args.robot_count)
        packages = rospkg.RosPack()
        generator = MujocoSceneGenerator(parameters, packages.get_path("bee"),
                                        packages.get_path("mujoco_ros_control"),
                                        args.output_dir, log_info=rospy.loginfo)
        robots, model, manifest, _ = generator.generate()
        wait_for_vehicle_parameters([robot["name"] for robot in robots], args.timeout)
        # Preserve the initialization grace period used by the existing demo.
        # Use wall time because /clock is not published until MuJoCo starts.
        deadline = time.monotonic() + args.startup_delay
        while time.monotonic() < deadline and not rospy.is_shutdown():
            time.sleep(.05)
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException("scene preparation interrupted")
        rospy.set_param(args.manifest_namespace, manifest)
        rospy.loginfo("Scene ready: %s; starting shared MuJoCo", model)
    finally:
        rospy.signal_shutdown("scene preparation finished")

    # Keep the same PID so roslaunch owns shutdown and observes backend failure.
    os.execv(command[0], command)


if __name__ == "__main__":
    main()
