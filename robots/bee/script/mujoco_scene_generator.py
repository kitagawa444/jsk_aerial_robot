#!/usr/bin/env python3
"""Generate Bee MuJoCo scenes without a ROS master or flight commands.

The input uses the same flat scene keys as mujoco_multi_module.launch.
The generated manifest is the explicit interface to cooperative grasp control.
"""

import argparse
import logging
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml
from grasp_geometry import EXTRA_SHAPES, extra_shape_geometry
from bee_mujoco_model import robot_model, set_vehicle_colors, vehicle_colors


class MujocoSceneGenerator:
    def __init__(self, parameters, bee_path, mujoco_ros_control_path,
                 output_dir=None, log_info=None):
        self.parameters = dict(parameters)
        self.bee_path = bee_path
        self.mujoco_ros_control_path = mujoco_ros_control_path
        self.output_dir = output_dir or os.path.join(bee_path, "mujoco", "generated")
        self.log_info = log_info or logging.getLogger(__name__).info
        self.resolved_mass = None

    def param(self, name, default):
        return self.parameters.get(name, default)

    def build_robot_layout(self, robot_count):
        start_x = self.param("spawn_x_start", -2.0)
        start_y = self.param("spawn_y_start", 0.0)
        spacing_x = self.param("spacing_x", 1.0)
        spacing_y = self.param("spacing_y", 1.0)
        spawn_z = self.param("spawn_z", -0.08)
        spawn_yaw = self.param("spawn_yaw", 0.0)
        grid_cols = self.param("grid_cols", 0)

        object_shape = str(self.param(
            "object_shape", "triangular_prism")).lower()
        extra = extra_shape_geometry(self.parameters)
        box_shape = object_shape in ("box", "rectangular_prism", "sphere", "ellipsoid") or extra is not None
        pentagon_shape = object_shape == "pentagonal_prism"
        grasp_robot_count = 5 if pentagon_shape else (4 if box_shape else 3)
        if (self.param("grasp_staging_layout", True) and
                self.param("spawn_object", True) and
                robot_count >= grasp_robot_count):
            object_x = self.param("object_x", 0.0)
            object_y = self.param("object_y", 1.0)
            object_yaw = self.param("object_yaw", 0.0)
            staging_clearance = self.param("grasp_staging_clearance", 0.80)
            if staging_clearance <= 0.0:
                raise RuntimeError("grasp_staging_clearance must be positive")

            if box_shape:
                size_x = self.param("object_size_x", 0.60)
                size_y = self.param("object_size_y", 0.60)
                if size_x <= 0.0 or size_y <= 0.0:
                    raise RuntimeError("object box dimensions must be positive")
                base_angles = (0.0, math.pi / 2.0, math.pi,
                               3.0 * math.pi / 2.0)
                face_radii = (size_x / 2.0, size_y / 2.0,
                              size_x / 2.0, size_y / 2.0)
                if extra:
                    face_radii = extra['radii']
            elif pentagon_shape:
                pentagon_side = self.param(
                    "object_pentagon_side", 0.60)
                if pentagon_side <= 0.0:
                    raise RuntimeError("object_pentagon_side must be positive")
                base_angles = tuple(
                    math.pi / 5.0 + 2.0 * math.pi * index / 5.0
                    for index in range(5))
                face_radius = pentagon_side / (
                    2.0 * math.tan(math.pi / 5.0))
                face_radii = (face_radius,) * 5
            else:
                triangle_side = self.param(
                    "object_triangle_side", 0.60)
                if triangle_side <= 0.0:
                    raise RuntimeError("object_triangle_side must be positive")
                # Mesh outward side-face normals are 60, 180 and 300 deg.
                base_angles = (math.pi / 3.0, math.pi,
                               5.0 * math.pi / 3.0)
                face_radius = triangle_side / (2.0 * math.sqrt(3.0))
                face_radii = (face_radius,) * 3

            # After base_link rolls +pi/2, the physical feet point along body
            # +Y, so body +Y is aligned with the inward face normal.
            base_angles = self.param("face_angles", base_angles)
            face_radii = self.param("face_radii", face_radii)
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
        return robot_model(self.bee_path, self.mujoco_ros_control_path, livox=self.param("livox", False),
                           regenerate=self.param("regenerate_single_model", False))

    def build_grasp_objects(self):
        if not self.param("spawn_object", True):
            return []

        object_shape = str(self.param(
            "object_shape", "triangular_prism")).lower()
        if object_shape == "rectangular_prism":
            object_shape = "box"
        if object_shape not in (
                "triangular_prism", "pentagonal_prism", "box", "sphere", "ellipsoid") + EXTRA_SHAPES:
            raise RuntimeError("unsupported object_shape '{}'".format(
                object_shape))
        height = self.param("object_height", 0.60)
        density = self.param(
            "object_density", 1000.0 / 96.0)
        mass_override = self.param("object_mass", -1.0)
        if height <= 0.0:
            raise RuntimeError("object dimensions must be positive")
        if density <= 0.0:
            raise RuntimeError("object_density must be positive")
        extra = extra_shape_geometry(self.parameters)
        if extra:
            volume = extra['volume']
        elif object_shape in ("box", "sphere", "ellipsoid"):
            size_x = self.param("object_size_x", 0.60)
            size_y = self.param("object_size_y", 0.60)
            if size_x <= 0.0 or size_y <= 0.0:
                raise RuntimeError("object box dimensions must be positive")
            if object_shape == "sphere" and (abs(size_x-height) > 1e-9 or abs(size_y-height) > 1e-9):
                raise RuntimeError("sphere dimensions must be equal")
            volume = size_x * size_y * height * (math.pi / 6.0 if object_shape != "box" else 1.0)
        elif object_shape == "pentagonal_prism":
            outer_side = self.param("object_pentagon_side", 0.60)
            inner_side = 0.0
            if outer_side <= 0.0:
                raise RuntimeError("object dimensions must be positive")
            volume = (
                5.0 * outer_side ** 2 /
                (4.0 * math.tan(math.pi / 5.0)) * height)
        else:
            outer_side = self.param("object_triangle_side", 0.60)
            inner_side = self.param(
                "object_inner_triangle_side", 0.4186262221)
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
        self.resolved_mass = object_mass
        if extra:
            self.log_info("grasp %s: size %s, volume %.6f m^3, mass %.4f kg",
                          object_shape, extra['size'], volume, object_mass)
        elif object_shape in ("box", "sphere", "ellipsoid"):
            self.log_info(
                "grasp %s: %.3f x %.3f x %.3f m, density %.3f kg/m^3, "
                "volume %.6f m^3, mass %.4f kg%s",
                object_shape, size_x, size_y, height, density, volume, object_mass,
                " (override)" if mass_override > 0.0 else "")
        elif object_shape == "pentagonal_prism":
            self.log_info(
                "grasp pentagonal_prism: side %.3f m, height %.3f m, "
                "density %.3f kg/m^3, volume %.6f m^3, mass %.4f kg%s",
                outer_side, height, density, volume, object_mass,
                " (override)" if mass_override > 0.0 else "")
        else:
            self.log_info(
                "grasp triangular_prism: outer %.3f m, inner %.3f m, "
                "height %.3f m, density %.3f kg/m^3, volume %.6f m^3, "
                "mass %.4f kg%s", outer_side, inner_side, height, density,
                volume, object_mass,
                " (override)" if mass_override > 0.0 else "")
        ground_clearance = self.param("object_ground_clearance", 0.0)
        yaw = self.param("object_yaw", 0.0)
        object_x = self.param("object_x", 0.0)
        object_y = self.param("object_y", 1.0)
        pedestal_enabled = self.param("spawn_object_pedestal", True)
        pedestal_height = self.param("object_pedestal_height", 0.65) if pedestal_enabled else 0.0
        pedestal_radius = self.param("object_pedestal_radius", 0.20)
        destination_pedestal_enabled = self.param(
            "spawn_destination_pedestal", False)
        destination_pedestal_x = self.param(
            "destination_pedestal_x", object_x + 1.50)
        destination_pedestal_y = self.param(
            "destination_pedestal_y", object_y)
        destination_pedestal_height = self.param(
            "destination_pedestal_height", pedestal_height)
        destination_pedestal_radius = self.param(
            "destination_pedestal_radius", pedestal_radius)
        sliding_friction = self.param("object_sliding_friction", 1.0)
        torsional_friction = self.param("object_torsional_friction", 0.5)
        pedestal_torsional_friction = self.param(
            "pedestal_torsional_friction", 2.0)
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
                pedestal_height + ground_clearance + self.param('object_bottom_offset', extra['bottom'] if extra else height / 2.0),
            ],
            "euler": [0.0, 0.0, yaw],
            "size": (extra['size'] if extra else [size_x, size_y, height] if object_shape in ("box", "sphere", "ellipsoid") else
                     [outer_side, height]),
            "freejoint_damping": self.param("object_freejoint_damping", 0.5),
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
        if extra:
            grasp_object["free"] = True  # Cylinders otherwise denote fixed pedestals.
            if object_shape == 'rock':
                for key in ('vertices', 'faces', 'unit_inertia', 'volume'):
                    grasp_object[key] = extra[key]
                grasp_object['rgba'] = [.48, .46, .43, 1.]
        objects.append(grasp_object)
        if mass_override > 0.0:
            objects[-1]["mass"] = object_mass
        else:
            objects[-1]["density"] = density
        return objects

    def compose_scene(self, robot_count, robots, objects, source_model):
        generated_dir = self.output_dir
        if not os.path.isdir(generated_dir):
            os.makedirs(generated_dir)

        scene_config_path = os.path.join(generated_dir, "bee_scene_{}.yaml".format(robot_count))
        scene_model_path = os.path.join(generated_dir, "bee_scene_{}.xml".format(robot_count))

        scene_config = {
            "scene_name": "bee_scene_{}".format(robot_count),
            "source_model": source_model,
            "output_model": scene_model_path,
            "robots": [],
            "objects": objects,
            "option": {
                "impratio": self.param("contact_impratio", 500.0),
                "cone": self.param("friction_cone", "elliptic"),
                "solver": self.param("constraint_solver", "Newton"),
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
                    "extent": self.param("scene_camera_extent", 1.2),
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

    def control_manifest(self, robots, objects):
        """Physical quantities only; no gain, ROS node name or rendering options."""
        payload = next((item for item in objects if item["name"] == "grasp_prism"), None)
        if payload is None:
            return {"schema_version": 1, "available": False}
        shape = payload["type"]
        extra = extra_shape_geometry(self.parameters)
        count = 4 if extra else {"triangular_prism": 3, "box": 4, "sphere": 4, "ellipsoid": 4, "pentagonal_prism": 5}[shape]
        if len(robots) != count:
            # The simulator also supports arbitrary robot layouts; the new
            # cooperative controller requires one vehicle per side face.
            return {"schema_version": 1, "available": False}
        if extra:
            angles = [index * math.pi / 2.0 for index in range(4)]
            radii = extra['radii']
        elif shape in ("box", "sphere", "ellipsoid"):
            angles = [index * math.pi / 2.0 for index in range(4)]
            radii = [payload["size"][index % 2] / 2.0 for index in range(4)]
        else:
            angles = [math.pi / count + 2.0 * math.pi * i / count for i in range(count)]
            radii = [payload["size"][0] / (2.0 * math.tan(math.pi / count))] * count
        destination = next((item for item in objects if item["name"] == "place_pedestal"), None)
        height = extra['height'] if extra else payload["size"][-1]
        bottom = self.param('object_bottom_offset', extra['bottom'] if extra else height / 2.0)
        return {
            # Old controllers must reject new analytic shapes instead of
            # silently interpreting them as planar faces.
            "schema_version": 3 if shape == 'rock' else 2 if extra else 1, "available": True,
            "robot_names": [robot["name"] for robot in robots],
            "object_shape": shape, "object_mass": self.resolved_mass,
            "object_height": height,
            "object_bottom_offset": bottom,
            "surface_radius": extra['radius'] if extra else 0.0,
            "surface_top_radius": extra['top_radius'] if extra else 0.0,
            "surface_length": extra['length'] if extra else 0.0,
            "surface_vertices": ([v for vertex in extra['vertices'] for v in vertex] if shape == 'rock' else []),
            "contact_normals": ([v for normal in extra['contact_normals'] for v in normal] if shape == 'rock' else []),
            "surface_radii": ([v / 2.0 for v in payload["size"]] if shape in ("sphere", "ellipsoid") else [0., 0., 0.]),
            "contact_height": self.param("contact_height", 0.0),
            "object_position": payload["pos"], "object_yaw": payload["euler"][2],
            "face_angles": self.param("face_angles", angles), "face_radii": self.param("face_radii", radii),
            "staging_clearance": self.param("grasp_staging_clearance", 0.80),
            "has_destination": destination is not None,
            "destination_position": (
                [destination["pos"][0], destination["pos"][1],
                 destination["size"][1] + self.param("object_ground_clearance", 0.0) + bottom]
                if destination else payload["pos"]),
        }

    def generate(self):
        count = self.param("robot_count", 3)
        if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
            raise ValueError("robot_count must be a positive integer")
        robots = self.build_robot_layout(count)
        objects = self.build_grasp_objects()
        source_model = self.ensure_single_robot_model()
        model = self.compose_scene(count, robots, objects, source_model)
        manifest = self.control_manifest(robots, objects)
        # Export the actual composer's inertia, including hollow payloads.
        tree = ET.parse(model)
        if self.param("world", "default") == "0066":
            from mujoco_room_0066 import add_room
            add_room(tree, self.output_dir)
        if self.param('object_mesh', None):
            from grasp_cad import apply_mesh
            apply_mesh(tree, self.parameters)
            for key in ('object_mesh', 'mesh_scale', 'mesh_xyz', 'mesh_rpy', 'marker_to_model_xyz', 'marker_to_model_rpy'):
                if key in self.parameters:
                    manifest[key] = self.parameters[key]
        for index, robot in enumerate(robots, start=1):
            set_vehicle_colors(tree, robot["name"] + "_", vehicle_colors(self.bee_path, index))
        tree.write(model, encoding="utf-8", xml_declaration=True)
        inertial = tree.find(".//body[@name='grasp_prism']/inertial")
        if inertial is not None:
            if 'quat' in inertial.attrib:
                raise ValueError('rotated inertial frame is not supported by grasp manifest')
            if 'diaginertia' in inertial.attrib:
                x, y, z = map(float, inertial.get('diaginertia').split())
                manifest['object_inertia'] = [x, 0., 0., 0., y, 0., 0., 0., z]
            elif 'fullinertia' in inertial.attrib:
                x, y, z, xy, xz, yz = map(float, inertial.get('fullinertia').split())
                manifest['object_inertia'] = [x, xy, xz, xy, y, yz, xz, yz, z]
        manifest_path = os.path.join(self.output_dir, "bee_scene_{}_control.yaml".format(count))
        with open(manifest_path, "w") as output:
            yaml.safe_dump(manifest, output, default_flow_style=False)
        return robots, model, manifest, manifest_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", help="flat scene-parameter YAML (not a composer YAML)")
    parser.add_argument("--object-model")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--bee-path")
    parser.add_argument("--mujoco-ros-control-path")
    args = parser.parse_args()
    if not args.bee_path or not args.mujoco_ros_control_path:
        import rospkg
        packages = rospkg.RosPack()
        args.bee_path = args.bee_path or packages.get_path("bee")
        args.mujoco_ros_control_path = (args.mujoco_ros_control_path or
                                      packages.get_path("mujoco_ros_control"))
    with open(args.config) as source:
        parameters = yaml.safe_load(source)
    if not isinstance(parameters, dict):
        parser.error("config must contain a mapping of scene parameters")
    if args.object_model:
        from grasp_cad import load_model
        load_model(args.object_model, parameters)
    generator = MujocoSceneGenerator(parameters, args.bee_path,
                                    args.mujoco_ros_control_path, args.output_dir)
    _, model, _, manifest = generator.generate()
    print("Model: " + model)
    print("Control manifest: " + manifest)


if __name__ == "__main__":
    main()
