#!/usr/bin/env python

import os
import math
import signal
import subprocess
import sys
import time

import roslaunch
import rospkg
import rospy
import yaml


class MujocoMultiBringup(object):
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.launches = []
        self.bee_path = self.rospack.get_path("bee")
        self.mujoco_ros_control_path = self.rospack.get_path("mujoco_ros_control")

    def get_robot_namespaces(self, robot_count):
        return ["bee{}".format(index) for index in range(1, robot_count + 1)]

    def build_robot_layout(self, robot_count):
        start_x = rospy.get_param("~spawn_x_start", -2.0)
        start_y = rospy.get_param("~spawn_y_start", 0.0)
        spacing_x = rospy.get_param("~spacing_x", 1.0)
        spacing_y = rospy.get_param("~spacing_y", 1.0)
        spawn_z = rospy.get_param("~spawn_z", 0.0)
        spawn_yaw = rospy.get_param("~spawn_yaw", 0.0)
        grid_cols = rospy.get_param("~grid_cols", 0)

        if grid_cols <= 0:
            grid_cols = robot_count

        robots = []
        for index in range(robot_count):
            row = index // grid_cols
            col = index % grid_cols
            robots.append({
                "name": "bee{}".format(index + 1),
                "pos": [start_x + spacing_x * col, start_y + spacing_y * row, spawn_z],
                "yaw": spawn_yaw,
            })
        return robots

    def ensure_single_robot_model(self):
        force_regenerate = rospy.get_param("~regenerate_single_model", False)
        model_path = os.path.join(self.bee_path, "mujoco", "bee", "robot.xml")
        if os.path.isfile(model_path) and not force_regenerate:
            return model_path

        generator_path = os.path.join(self.mujoco_ros_control_path, "scripts", "mujoco_model_generator.py")
        config_path = os.path.join(self.bee_path, "config", "mujoco_model.yaml")
        subprocess.check_call([sys.executable, generator_path, config_path])
        return model_path

    def build_grasp_objects(self):
        if not rospy.get_param("~spawn_object", True):
            return []

        height = rospy.get_param("~object_height", 0.30)
        triangle_side = rospy.get_param("~object_triangle_side", 0.36)
        ground_clearance = rospy.get_param("~object_ground_clearance", 0.002)
        yaw = rospy.get_param("~object_yaw", 0.0)
        object_x = rospy.get_param("~object_x", 0.0)
        object_y = rospy.get_param("~object_y", 1.0)
        pedestal_enabled = rospy.get_param("~spawn_object_pedestal", True)
        pedestal_height = rospy.get_param("~object_pedestal_height", 0.65) if pedestal_enabled else 0.0
        pedestal_radius = rospy.get_param("~object_pedestal_radius", 0.09)

        objects = []
        if pedestal_enabled:
            objects.append({
                "name": "grasp_pedestal",
                "type": "cylinder",
                "pos": [object_x, object_y, pedestal_height / 2.0],
                "size": [pedestal_radius, pedestal_height],
                "friction": [1.2, 0.02, 0.001],
                "rgba": [0.35, 0.38, 0.42, 1.0],
            })

        objects.append({
            "name": "grasp_prism",
            "type": "triangular_prism",
            "pos": [
                object_x,
                object_y,
                pedestal_height + ground_clearance + height / 2.0,
            ],
            "euler": [0.0, 0.0, yaw],
            "size": [triangle_side, height],
            "mass": rospy.get_param("~object_mass", 0.50),
            "friction": [
                rospy.get_param("~object_sliding_friction", 1.2),
                0.02,
                0.001,
            ],
            "rgba": [0.92, 0.45, 0.08, 1.0],
        })
        return objects

    def compose_scene(self, robot_count, robots, objects):
        generated_dir = os.path.join(self.bee_path, "mujoco", "generated")
        if not os.path.isdir(generated_dir):
            os.makedirs(generated_dir)

        scene_config_path = os.path.join(generated_dir, "bee_scene_{}.yaml".format(robot_count))
        scene_model_path = os.path.join(generated_dir, "bee_scene_{}.xml".format(robot_count))

        scene_config = {
            "scene_name": "bee_scene_{}".format(robot_count),
            "source_model": os.path.join(self.bee_path, "mujoco", "bee", "robot.xml"),
            "output_model": scene_model_path,
            "robots": [],
            "objects": objects,
        }
        if objects:
            scene_config["jacobian"] = "dense"

        for robot in robots:
            entry = {
                "name": robot["name"],
                "pos": robot["pos"],
            }
            scene_config["robots"].append(entry)

        with open(scene_config_path, "w") as file_handle:
            yaml.safe_dump(scene_config, file_handle, default_flow_style=False)

        composer_path = os.path.join(self.mujoco_ros_control_path, "scripts", "mujoco_scene_composer.py")
        subprocess.check_call([sys.executable, composer_path, scene_config_path])
        return scene_model_path

    def start_launch(self, launch_file, args):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, args)])
        launch.start()
        self.launches.append(launch)
        return launch

    def start(self):
        robot_count = rospy.get_param("~robot_count", 4)
        if robot_count <= 0:
            raise RuntimeError("robot_count must be positive")

        headless = rospy.get_param("~headless", False)
        estimate_mode = rospy.get_param("~estimate_mode", 1)
        launch_rviz = rospy.get_param("~launch_rviz", not headless)
        first_robot_rviz_only = rospy.get_param("~first_robot_rviz_only", True)

        robot_namespaces = self.get_robot_namespaces(robot_count)
        robots = self.build_robot_layout(robot_count)
        objects = self.build_grasp_objects()
        self.ensure_single_robot_model()
        scene_model_path = self.compose_scene(robot_count, robots, objects)

        bringup_launch = roslaunch.rlutil.resolve_launch_arguments(["bee", "bringup.launch"])[0]
        for index, robot in enumerate(robots):
            robot_id = robot["name"].replace("bee", "")
            robot_launch_rviz = launch_rviz and (not first_robot_rviz_only or index == 0)
            bringup_args = [
                "robot_id:={}".format(robot_id),
                "real_machine:=false",
                "simulation:=true",
                "headless:=true",
                "estimate_mode:={}".format(estimate_mode),
                "mujoco:=true",
                "launch_mujoco_backend:=false",
                "launch_mujoco_controller:=false",
                "launch_rviz:={}".format("true" if robot_launch_rviz else "false"),
            ]
            self.start_launch(bringup_launch, bringup_args)

        # The top-level launch enables /use_sim_time before this node starts,
        # but the MuJoCo backend is not publishing /clock yet.
        time.sleep(rospy.get_param("~param_load_delay", 1.0))

        mujoco_launch = roslaunch.rlutil.resolve_launch_arguments(["mujoco_ros_control", "mujoco_multi.launch"])[0]
        mujoco_args = [
            "headless:={}".format("true" if headless else "false"),
            "mujoco_model:={}".format(scene_model_path),
            "robot_namespaces:={}".format(str(robot_namespaces)),
        ]
        self.start_launch(mujoco_launch, mujoco_args)

        time.sleep(rospy.get_param("~backend_start_delay", 1.0))

        controller_launch = roslaunch.rlutil.resolve_launch_arguments(["aerial_robot_simulation", "mujoco.launch"])[0]
        for robot in robots:
            controller_args = [
                "robot_ns:={}".format(robot["name"]),
                "headless:={}".format("true" if headless else "false"),
                "mujoco_model:={}".format(scene_model_path),
                "launch_backend:=false",
                "launch_controller:=true",
            ]
            self.start_launch(controller_launch, controller_args)

    def shutdown(self):
        while self.launches:
            launch = self.launches.pop()
            launch.shutdown()


def main():
    rospy.init_node("bee_mujoco_multi_bringup")
    bringup = MujocoMultiBringup()

    def signal_handler(sig, frame):
        bringup.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        bringup.start()
        rospy.spin()
    finally:
        bringup.shutdown()


if __name__ == "__main__":
    main()
