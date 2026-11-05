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

        object_shape = str(rospy.get_param(
            "~object_shape", "triangular_prism")).lower()
        box_shape = object_shape in ("box", "rectangular_prism")
        pentagon_shape = object_shape == "pentagonal_prism"
        grasp_robot_count = 5 if pentagon_shape else (4 if box_shape else 3)
        if (rospy.get_param("~grasp_staging_layout", True) and
                rospy.get_param("~spawn_object", True) and
                robot_count >= grasp_robot_count):
            object_x = rospy.get_param("~object_x", 0.0)
            object_y = rospy.get_param("~object_y", 1.0)
            object_yaw = rospy.get_param("~object_yaw", 0.0)
            staging_clearance = rospy.get_param("~grasp_staging_clearance", 0.80)
            if staging_clearance <= 0.0:
                raise RuntimeError("grasp_staging_clearance must be positive")

            if box_shape:
                size_x = rospy.get_param("~object_size_x", 0.60)
                size_y = rospy.get_param("~object_size_y", 0.60)
                if size_x <= 0.0 or size_y <= 0.0:
                    raise RuntimeError("object box dimensions must be positive")
                base_angles = (0.0, math.pi / 2.0, math.pi,
                               3.0 * math.pi / 2.0)
                face_radii = (size_x / 2.0, size_y / 2.0,
                              size_x / 2.0, size_y / 2.0)
            elif pentagon_shape:
                pentagon_side = rospy.get_param(
                    "~object_pentagon_side", 0.60)
                if pentagon_side <= 0.0:
                    raise RuntimeError("object_pentagon_side must be positive")
                base_angles = tuple(
                    math.pi / 5.0 + 2.0 * math.pi * index / 5.0
                    for index in range(5))
                face_radius = pentagon_side / (
                    2.0 * math.tan(math.pi / 5.0))
                face_radii = (face_radius,) * 5
            else:
                triangle_side = rospy.get_param(
                    "~object_triangle_side", 0.60)
                if triangle_side <= 0.0:
                    raise RuntimeError("object_triangle_side must be positive")
                # Mesh outward side-face normals are 60, 180 and 300 deg.
                base_angles = (math.pi / 3.0, math.pi,
                               5.0 * math.pi / 3.0)
                face_radius = triangle_side / (2.0 * math.sqrt(3.0))
                face_radii = (face_radius,) * 3

            # After base_link rolls +pi/2, the physical feet point along body
            # +Y, so body +Y is aligned with the inward face normal.
            robots = []
            for index, (base_angle, face_radius) in enumerate(
                    zip(base_angles, face_radii)):
                normal_angle = object_yaw + base_angle
                staging_radius = face_radius + staging_clearance
                robots.append({
                    "name": "bee{}".format(index + 1),
                    "pos": [
                        object_x + staging_radius * math.cos(normal_angle),
                        object_y + staging_radius * math.sin(normal_angle),
                        spawn_z,
                    ],
                    "yaw": normal_angle + math.pi / 2.0,
                })

            # Additional robots remain outside the grasp formation.
            for index in range(grasp_robot_count, robot_count):
                robots.append({
                    "name": "bee{}".format(index + 1),
                    "pos": [object_x + 1.0 +
                            spacing_x * (index - grasp_robot_count),
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

        object_shape = str(rospy.get_param(
            "~object_shape", "triangular_prism")).lower()
        if object_shape == "rectangular_prism":
            object_shape = "box"
        if object_shape not in (
                "triangular_prism", "pentagonal_prism", "box"):
            raise RuntimeError("unsupported object_shape '{}'".format(
                object_shape))
        height = rospy.get_param("~object_height", 0.60)
        density = rospy.get_param(
            "~object_density", 1000.0 / 96.0)
        mass_override = rospy.get_param("~object_mass", -1.0)
        if height <= 0.0:
            raise RuntimeError("object dimensions must be positive")
        if density <= 0.0:
            raise RuntimeError("object_density must be positive")
        if object_shape == "box":
            size_x = rospy.get_param("~object_size_x", 0.60)
            size_y = rospy.get_param("~object_size_y", 0.60)
            if size_x <= 0.0 or size_y <= 0.0:
                raise RuntimeError("object box dimensions must be positive")
            volume = size_x * size_y * height
        elif object_shape == "pentagonal_prism":
            outer_side = rospy.get_param("~object_pentagon_side", 0.60)
            inner_side = 0.0
            if outer_side <= 0.0:
                raise RuntimeError("object dimensions must be positive")
            volume = (
                5.0 * outer_side ** 2 /
                (4.0 * math.tan(math.pi / 5.0)) * height)
        else:
            outer_side = rospy.get_param("~object_triangle_side", 0.60)
            inner_side = rospy.get_param(
                "~object_inner_triangle_side", 0.4186262221)
            if outer_side <= 0.0:
                raise RuntimeError("object dimensions must be positive")
            if inner_side < 0.0 or inner_side >= outer_side:
                raise RuntimeError(
                    "object inner side must be non-negative and smaller than "
                    "the outer side")
            volume = (math.sqrt(3.0) / 4.0 *
                      (outer_side ** 2 - inner_side ** 2) * height)
        density_mass = density * volume
        object_mass = mass_override if mass_override > 0.0 else density_mass
        # Controllers use this resolved value for gravity feed-forward.  Keep
        # the public parameter numeric even when launch selected density mode.
        rospy.set_param("~object_mass", object_mass)
        if object_shape == "box":
            rospy.loginfo(
                "grasp box: %.3f x %.3f x %.3f m, density %.3f kg/m^3, "
                "volume %.6f m^3, mass %.4f kg%s",
                size_x, size_y, height, density, volume, object_mass,
                " (override)" if mass_override > 0.0 else "")
        elif object_shape == "pentagonal_prism":
            rospy.loginfo(
                "grasp pentagonal_prism: side %.3f m, height %.3f m, "
                "density %.3f kg/m^3, volume %.6f m^3, mass %.4f kg%s",
                outer_side, height, density, volume, object_mass,
                " (override)" if mass_override > 0.0 else "")
        else:
            rospy.loginfo(
                "grasp triangular_prism: outer %.3f m, inner %.3f m, "
                "height %.3f m, density %.3f kg/m^3, volume %.6f m^3, "
                "mass %.4f kg%s", outer_side, inner_side, height, density,
                volume, object_mass,
                " (override)" if mass_override > 0.0 else "")
        ground_clearance = rospy.get_param("~object_ground_clearance", 0.0)
        yaw = rospy.get_param("~object_yaw", 0.0)
        object_x = rospy.get_param("~object_x", 0.0)
        object_y = rospy.get_param("~object_y", 1.0)
        pedestal_enabled = rospy.get_param("~spawn_object_pedestal", True)
        pedestal_height = rospy.get_param("~object_pedestal_height", 0.65) if pedestal_enabled else 0.0
        pedestal_radius = rospy.get_param("~object_pedestal_radius", 0.20)
        destination_pedestal_enabled = rospy.get_param(
            "~spawn_destination_pedestal", False)
        destination_pedestal_x = rospy.get_param(
            "~destination_pedestal_x", object_x + 1.50)
        destination_pedestal_y = rospy.get_param(
            "~destination_pedestal_y", object_y)
        destination_pedestal_height = rospy.get_param(
            "~destination_pedestal_height", pedestal_height)
        destination_pedestal_radius = rospy.get_param(
            "~destination_pedestal_radius", pedestal_radius)
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
        if destination_pedestal_enabled:
            objects.append({
                "name": "place_pedestal",
                "type": "cylinder",
                "pos": [
                    destination_pedestal_x,
                    destination_pedestal_y,
                    destination_pedestal_height / 2.0,
                ],
                "size": [
                    destination_pedestal_radius,
                    destination_pedestal_height,
                ],
                "friction": [
                    sliding_friction,
                    pedestal_torsional_friction,
                    0.001,
                ],
                "rgba": [0.20, 0.48, 0.32, 1.0],
            })

        grasp_object = {
            "name": "grasp_prism",
            "type": object_shape,
            "pos": [
                object_x,
                object_y,
                pedestal_height + ground_clearance + height / 2.0,
            ],
            "euler": [0.0, 0.0, yaw],
            "size": ([size_x, size_y, height] if object_shape == "box" else
                     [outer_side, height]),
            "freejoint_damping": rospy.get_param("~object_freejoint_damping", 0.5),
            "friction": [
                sliding_friction,
                torsional_friction,
                0.001,
            ],
            "solref": [0.02, 1.0],
            "solimp": [0.9, 0.95, 0.01],
            "rgba": [0.92, 0.45, 0.08, 1.0],
        }
        if object_shape == "triangular_prism":
            grasp_object["inner_triangle_side"] = inner_side
        objects.append(grasp_object)
        if mass_override > 0.0:
            objects[-1]["mass"] = object_mass
        else:
            objects[-1]["density"] = density
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
            "option": {
                "impratio": rospy.get_param("~contact_impratio", 50.0),
                "cone": rospy.get_param("~friction_cone", "elliptic"),
                "solver": rospy.get_param("~constraint_solver", "Newton"),
            },
        }
        if objects:
            scene_config["jacobian"] = "dense"
            focus_object = next(
                (entry for entry in objects
                 if entry.get("name") == "grasp_prism"), None)
            if focus_object is not None:
                scene_config["statistic"] = {
                    "center": focus_object["pos"],
                    "extent": rospy.get_param("~scene_camera_extent", 1.2),
                }

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

    def wait_for_robot_parameters(self, robot_namespaces):
        """Wait until every nested Bee bringup has populated its namespace.

        ROSLaunchParent.start() returns before all of the included launch
        files have finished loading their parameters.  Starting the shared
        backend before that point makes it permanently select
        DefaultRobotHWSim for whichever namespace is late.
        """
        expected_plugin = "mujoco_ros_control/AerialRobotHWSim"
        required_suffixes = (
            "robot_description",
            "servo_controller",
            "motor_info",
        )
        timeout = rospy.get_param("~param_load_timeout", 30.0)
        deadline = time.monotonic() + timeout
        last_pending = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            pending = []
            for robot_ns in robot_namespaces:
                root = "/{}".format(robot_ns)
                plugin_param = root + "/simulation/robot_hw_sim_plugin_name"
                if not rospy.has_param(plugin_param):
                    pending.append("{}:plugin missing".format(robot_ns))
                else:
                    plugin = rospy.get_param(plugin_param)
                    if plugin != expected_plugin:
                        pending.append(
                            "{}:plugin={}".format(robot_ns, plugin))
                for suffix in required_suffixes:
                    if not rospy.has_param(root + "/" + suffix):
                        pending.append(
                            "{}:{} missing".format(robot_ns, suffix))
            if not pending:
                rospy.loginfo(
                    "all MuJoCo robot parameters are ready: %s",
                    ",".join(robot_namespaces))
                return
            pending_state = tuple(pending)
            if pending_state != last_pending:
                rospy.loginfo(
                    "waiting for MuJoCo robot parameters: %s",
                    "; ".join(pending))
                last_pending = pending_state
            time.sleep(0.05)
        raise RuntimeError(
            "timed out waiting for MuJoCo robot parameters after {:.1f} s: "
            "{}".format(timeout, "; ".join(last_pending or ("unknown",))))

    def wait_for_controller_managers(self, robot_namespaces):
        """Wait until the shared backend has created every manager service."""
        timeout = rospy.get_param("~backend_ready_timeout", 30.0)
        deadline = time.monotonic() + timeout
        pending = set(robot_namespaces)
        while (not rospy.is_shutdown() and pending and
               time.monotonic() < deadline):
            for robot_ns in list(pending):
                service = "/{}/controller_manager/list_controllers".format(
                    robot_ns)
                try:
                    rospy.wait_for_service(service, timeout=0.05)
                    pending.remove(robot_ns)
                except rospy.ROSException:
                    pass
            if pending:
                time.sleep(0.05)
        if pending:
            raise RuntimeError(
                "timed out waiting for MuJoCo controller managers after "
                "{:.1f} s: {}".format(timeout, ",".join(sorted(pending))))
        rospy.loginfo(
            "all MuJoCo controller managers are ready: %s",
            ",".join(robot_namespaces))

    def start(self):
        robot_count = rospy.get_param("~robot_count", 4)
        if robot_count <= 0:
            raise RuntimeError("robot_count must be positive")

        headless = rospy.get_param("~headless", False)
        render_fps = rospy.get_param("~render_fps", 10.0)
        real_time_factor = rospy.get_param("~real_time_factor", 0.0)
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

        # ROSLaunchParent.start() is asynchronous.  A fixed sleep races with
        # parameter loading, especially with four robots under WSL load.
        minimum_param_delay = rospy.get_param("~param_load_delay", 0.0)
        if minimum_param_delay > 0.0:
            time.sleep(minimum_param_delay)
        self.wait_for_robot_parameters(robot_namespaces)

        mujoco_launch = roslaunch.rlutil.resolve_launch_arguments(["mujoco_ros_control", "mujoco_multi.launch"])[0]
        mujoco_args = [
            "headless:={}".format("true" if headless else "false"),
            "mujoco_model:={}".format(scene_model_path),
            "robot_namespaces:={}".format(str(robot_namespaces)),
            "render_fps:={}".format(render_fps),
            "real_time_factor:={}".format(real_time_factor),
            "vsync:={}".format("true" if vsync else "false"),
            "render_shadows:={}".format("true" if render_shadows else "false"),
            "render_reflections:={}".format("true" if render_reflections else "false"),
            "window_width:={}".format(window_width),
            "window_height:={}".format(window_height),
        ]
        self.start_launch(mujoco_launch, mujoco_args)

        minimum_backend_delay = rospy.get_param("~backend_start_delay", 0.0)
        if minimum_backend_delay > 0.0:
            time.sleep(minimum_backend_delay)
        self.wait_for_controller_managers(robot_namespaces)

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
