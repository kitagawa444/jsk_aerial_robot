#!/usr/bin/env python

"""Legacy combined launcher; new grasp_sim.launch declares world/vehicles separately."""

import signal
import sys
import time

import roslaunch
import rospkg
import rospy
from mujoco_scene_generator import MujocoSceneGenerator

class MujocoMultiBringup(object):
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.launches = []
        self.bee_path = self.rospack.get_path("bee")
        self.mujoco_ros_control_path = self.rospack.get_path("mujoco_ros_control")

    def get_robot_namespaces(self, robot_count):
        return ["bee{}".format(index) for index in range(1, robot_count + 1)]

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
        manifest_ns = rospy.get_param("~control_scene_namespace", "/bee_grasp/scene")
        rospy.set_param(manifest_ns, {"schema_version": 1, "available": False})
        scene_parameters = rospy.get_param("~", {})
        scene_parameters["robot_count"] = robot_count
        generator = MujocoSceneGenerator(
            scene_parameters, self.bee_path, self.mujoco_ros_control_path,
            log_info=rospy.loginfo)
        robots, scene_model_path, manifest, _ = generator.generate()
        # Keep the legacy demos' mass parameter while publishing a separate
        # physical contract for the new central controller.
        if generator.resolved_mass is not None:
            rospy.set_param("~object_mass", generator.resolved_mass)
        rospy.set_param(manifest_ns, manifest)

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
