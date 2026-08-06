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
        spawn_z = rospy.get_param("~spawn_z", -0.08)
        spawn_yaw = rospy.get_param("~spawn_yaw", 0.0)
        grid_cols = rospy.get_param("~grid_cols", 0)

        if (rospy.get_param("~grasp_staging_layout", True) and
                rospy.get_param("~spawn_object", True) and robot_count >= 3):
            object_x = rospy.get_param("~object_x", 0.0)
            object_y = rospy.get_param("~object_y", 1.0)
            object_yaw = rospy.get_param("~object_yaw", 0.0)
            triangle_side = rospy.get_param("~object_triangle_side", 0.80)
            staging_clearance = rospy.get_param("~grasp_staging_clearance", 0.80)
            if triangle_side <= 0.0:
                raise RuntimeError("object_triangle_side must be positive")
            if staging_clearance <= 0.0:
                raise RuntimeError("grasp_staging_clearance must be positive")

            # For the mesh orientation used by the scene composer, the three
            # outward side-face normals are 60, 180 and 300 degrees.  Spawn
            # the first three Bees on those normals. After base_link rolls
            # +pi/2, the physical feet point along body +Y, so body +Y is
            # aligned with the inward face normal.
            inradius = triangle_side / (2.0 * math.sqrt(3.0))
            staging_radius = inradius + staging_clearance
            robots = []
            for index, base_angle in enumerate((math.pi / 3.0, math.pi, 5.0 * math.pi / 3.0)):
                normal_angle = object_yaw + base_angle
                robots.append({
                    "name": "bee{}".format(index + 1),
                    "pos": [
                        object_x + staging_radius * math.cos(normal_angle),
                        object_y + staging_radius * math.sin(normal_angle),
                        spawn_z,
                    ],
                    "yaw": normal_angle + math.pi / 2.0,
                })

            # Additional robots remain outside the three-Bee grasp formation.
            for index in range(3, robot_count):
                robots.append({
                    "name": "bee{}".format(index + 1),
                    "pos": [object_x + 1.0 + spacing_x * (index - 3),
                            object_y - 1.0, spawn_z],
                    "yaw": object_yaw + math.pi / 2.0,
                })
            return robots

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
        config_path = os.path.join(self.bee_path, "config", "mujoco_model.yaml")
        model_is_current = (
            os.path.isfile(model_path) and
            os.path.getmtime(model_path) >= os.path.getmtime(config_path)
        )
        if model_is_current and not force_regenerate:
            return model_path

        generator_path = os.path.join(self.mujoco_ros_control_path, "scripts", "mujoco_model_generator.py")
        subprocess.check_call([sys.executable, generator_path, config_path])
        return model_path

    def build_grasp_objects(self):
        if not rospy.get_param("~spawn_object", True):
            return []

        height = rospy.get_param("~object_height", 0.30)
        triangle_side = rospy.get_param("~object_triangle_side", 0.80)
        ground_clearance = rospy.get_param("~object_ground_clearance", 0.0)
        yaw = rospy.get_param("~object_yaw", 0.0)
        object_x = rospy.get_param("~object_x", 0.0)
        object_y = rospy.get_param("~object_y", 1.0)
        pedestal_enabled = rospy.get_param("~spawn_object_pedestal", True)
        pedestal_height = rospy.get_param("~object_pedestal_height", 0.65) if pedestal_enabled else 0.0
        pedestal_radius = rospy.get_param("~object_pedestal_radius", 0.20)
        sliding_friction = rospy.get_param("~object_sliding_friction", 0.5)
        torsional_friction = rospy.get_param("~object_torsional_friction", 0.5)
        pedestal_torsional_friction = rospy.get_param(
            "~pedestal_torsional_friction", 2.0)
        if (sliding_friction < 0.0 or torsional_friction < 0.0 or
                pedestal_torsional_friction < 0.0):
            raise RuntimeError("object friction coefficients must be non-negative")

        objects = []
        if pedestal_enabled:
            objects.append({
                "name": "grasp_pedestal",
                "type": "cylinder",
                "pos": [object_x, object_y, pedestal_height / 2.0],
                "size": [pedestal_radius, pedestal_height],
                "friction": [sliding_friction, pedestal_torsional_friction, 0.001],
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
            "mass": rospy.get_param("~object_mass", 0.20),
            "freejoint_damping": rospy.get_param("~object_freejoint_damping", 0.5),
            "friction": [
                sliding_friction,
                torsional_friction,
                0.001,
            ],
            "solref": [0.02, 1.0],
            "solimp": [0.9, 0.95, 0.01],
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
            yaw = robot.get("yaw")
            if yaw is not None:
                entry["quat"] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
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
        render_fps = rospy.get_param("~render_fps", 10.0)
        vsync = rospy.get_param("~vsync", False)
        render_shadows = rospy.get_param("~render_shadows", False)
        render_reflections = rospy.get_param("~render_reflections", False)
        window_width = rospy.get_param("~window_width", 640)
        window_height = rospy.get_param("~window_height", 480)
        estimate_mode = rospy.get_param("~estimate_mode", 1)
        robot_namespaces = self.get_robot_namespaces(robot_count)
        robots = self.build_robot_layout(robot_count)
        objects = self.build_grasp_objects()
        self.ensure_single_robot_model()
        scene_model_path = self.compose_scene(robot_count, robots, objects)

        bringup_launch = roslaunch.rlutil.resolve_launch_arguments(["bee", "bringup.launch"])[0]
        for robot in robots:
            robot_id = robot["name"].replace("bee", "")
            bringup_args = [
                "robot_id:={}".format(robot_id),
                "real_machine:=false",
                "simulation:=true",
                # MuJoCo GUI and the one shared multi-robot RViz instance are
                # launched by the parent.  Suppress per-robot RViz windows.
                "headless:=true",
                "estimate_mode:={}".format(estimate_mode),
                "mujoco:=true",
                "launch_mujoco_backend:=false",
                "launch_mujoco_controller:=false",
                "launch_rviz:=false",
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
            "render_fps:={}".format(render_fps),
            "vsync:={}".format("true" if vsync else "false"),
            "render_shadows:={}".format("true" if render_shadows else "false"),
            "render_reflections:={}".format("true" if render_reflections else "false"),
            "window_width:={}".format(window_width),
            "window_height:={}".format(window_height),
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
