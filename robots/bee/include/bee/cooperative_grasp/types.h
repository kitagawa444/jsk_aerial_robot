#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace bee::grasp
{
using Vec3 = Eigen::Vector3d;
constexpr double kPi = 3.14159265358979323846;
inline double clamp(double x, double limit)
{
  return std::clamp(x, -limit, limit);
}
inline double approach(double current, double target, double step)
{
  return current + clamp(target - current, step);
}
inline double angleError(double x)
{
  return std::atan2(std::sin(x), std::cos(x));
}
inline double deadband(double x, double band)
{
  return std::abs(x) <= band ? 0.0 : x - std::copysign(band, x);
}
inline double yaw(const Eigen::Quaterniond& q)
{
  const auto r = q.toRotationMatrix();
  return std::atan2(r(1, 0), r(0, 0));
}

// SI units throughout. Positions, linear velocities and output accelerations
// use world axes. Observer inputs use CoG axes; base angular velocity is local.
struct Pose
{
  Vec3 position = Vec3::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};
struct Wrench
{
  Vec3 force = Vec3::Zero(), moment = Vec3::Zero();
};
struct VehicleState
{
  Pose base, cog;
  Vec3 velocity_world = Vec3::Zero(), angular_velocity_base = Vec3::Zero();
  Wrench observer_cog;
  int flight_state = 0;
};
struct InputState
{
  double object_stamp = 0.0;
  Pose object;
  std::vector<VehicleState> vehicles;
};
struct ContactState
{
  Vec3 position = Vec3::Zero(), velocity = Vec3::Zero();
  Vec3 normal = Vec3::Zero(), tangent = Vec3::Zero();
  Wrench bee_world;
  Vec3 local_moment = Vec3::Zero();  // normal / tangent / world Z components
  double normal_force = 0.0;
  double radius = 0.0, tangent_position = 0.0, radial_velocity = 0.0, tangent_velocity = 0.0;
  double relative_z = 0.0, relative_vz = 0.0;
};
struct GraspState
{
  InputState input;
  Vec3 object_velocity = Vec3::Zero(), object_angular_velocity = Vec3::Zero();
  Wrench object_wrench;
  std::vector<ContactState> contacts;
};
struct SceneConfig
{
  std::vector<std::string> robot_names;
  std::vector<double> face_angles, face_radii;
  double object_mass = 0.5, object_height = 0.6, staging_clearance = 0.8;
  Vec3 object_position = Vec3(0, 1, .95), destination_position = Vec3(1.5, 1, .95);
  bool has_destination = false;
  void validate() const;
};
struct ControlConfig
{
  // Legacy path remains available for regression comparisons.
  bool distributed_tracking = false;
  double reference_z_kp = .5, reference_z_ki = .05, reference_z_i_limit = .015;
  double reference_velocity_limit = .08, reference_acceleration_limit = .1;
  double cooperative_admittance = .15;
  double robot_mass = 1.84199, gravity = 9.81;
  Vec3 contact_offset = Vec3(.0175, 0, -.161);
  double foot_contact_offset = .107, ball_radius = .012, max_compression = .015;
  double observer_force_sign = 1.0, observer_force_scale = 1.0, observer_torque_scale = 1.0;
  double force_filter_alpha = .1, twist_filter_alpha = .2;
  double radial_kp = 3, radial_kd = 3, tangent_kp = 2, tangent_kd = 3;
  bool contact_tangent_control_enabled = true;
  double approach_limit = .8, press_limit = 15, normal_force_kp = .4;
  double press_radial_kp = 400, radius_margin = .05, normal_force_rate = .3, max_normal_force = 8;
  double z_velocity_kp = 1.5, z_velocity_kd = 1, z_velocity_limit = .08;
  double trajectory_velocity = .01, trajectory_acceleration = .03, trajectory_jerk = .1;
  double z_kp = 1, z_kd = 1.5, z_ki = .2, z_pd_limit = .04, z_i_limit = .08;
  double support_ramp_duration = 1.5, support_filter_tau = .2, support_force_lead = .5;
  double support_ratio = .9, support_settle = .4, support_velocity_tolerance = .003;
  double observer_filter_tau = 1, observer_kp = .02, observer_ki = 0;
  double observer_i_limit = .05, observer_correction_limit = .08, observer_deadband = .03;
  double slip_kp = 5, slip_kd = 2, slip_limit = .03, slip_rate = 2;
  double relative_z_kp = 8, relative_z_kd = 2, relative_z_limit = .3, relative_z_rate = 1;
  double common_z_limit = 1, support_floor_scale = 0;
  double translation_kp = 1.5, translation_limit = .1, translation_rate = .3;
  double yaw_kp = .5, yaw_kd = .2, yaw_moment_gain = .8, yaw_limit = .12, yaw_rate = .3;
  double tilt_kp = .015, tilt_kd = .030, tilt_moment_gain = .5, tilt_limit = .2, tilt_rate = 1;
  double local_moment_alpha = .1, object_moment_alpha = .05, moment_force_threshold = .2;
  double local_roll_kp = 1, local_roll_ki = .15, local_yaw_kp = 1, local_yaw_ki = .05;
  double trim_limit = .02, trim_rate = .01, local_deadband = .0002;
  double radial_moment_kp = .2, radial_moment_ki = .3, radial_moment_limit = .6, radial_moment_rate = .5;
  double object_yaw_kp = .35, object_yaw_ki = .01;
  void validate() const;
};
struct TaskConfig
{
  bool automatic_transport = true;
  double friction_coefficient = .5, normal_force_margin = 1.2, design_vertical_acceleration = 0;
  double preload_force_ratio = .9;
  double approach_speed = .035, staging_tolerance = .08;
  double attitude_duration = 3, attitude_settle = 1, baseline_duration = 2, preload_duration = 3;
  double lift_height = .05, hold_tolerance = .003, hold_velocity = .003, hold_duration = 1;
  double transport_kp = .5, transport_speed = .02, position_tolerance = .015, velocity_tolerance = .008;
  double arrival_duration = 1, destination_settle = 2, descent_speed = .005;
  double touchdown_duration = 1.5, unload_duration = 3, release_rate = .8;
  double release_tolerance = .25, release_duration = 1, retreat_clearance = .4, verify_duration = 2;
  double staging_timeout = 150, approach_timeout = 35, lift_timeout = 150;
  double transport_timeout = 180, descent_timeout = 40, release_timeout = 20, retreat_timeout = 30;
  void validate() const;
};
struct SafetyConfig
{
  double input_timeout = 1, max_acceleration = 20, max_tilt = .30;
  double max_slip = .06, max_height_difference = .04, max_drop = .03;
  double max_contact_deviation = .04;
  double max_object_speed = .30, max_yaw_rate = 1.50;
  double force_loss_threshold = .5, force_loss_duration = .25;
};
enum class Phase
{
  Idle,
  Staging,
  Orient,
  Calibrate,
  Approach,
  Preload,
  SupportTransfer,
  Lift,
  Hold,
  Transport,
  DestinationSettle,
  Descend,
  Unload,
  Release,
  Retreat,
  Unwind,
  Verify,
  Complete,
  Aborted
};
const char* phaseName(Phase phase);
inline bool airborne(Phase p)
{
  return p >= Phase::Lift && p <= Phase::Descend;
}
struct Reference
{
  Phase phase = Phase::Idle;
  double roll = 0, radius_clearance = 0, normal_force = 0, normal_force_rate = .3;
  double object_z = 0, object_yaw = 0, support_scale = 1, z_velocity_limit = .01;
  Vec3 object_velocity = Vec3::Zero();
  bool begin_support = false, touchdown = false, calibrate = false;
};
enum class ZMode
{
  Position,
  Velocity,
  Acceleration
};
struct VehicleCommand
{
  bool contact_tracking = false, support_tracking = false, calibrate = false;
  Vec3 velocity = Vec3::Zero(), normal = Vec3::UnitX(), object_position = Vec3::Zero();
  double normal_force = 0, minimum_radius = 0;
  double support_force = 0, observer_blend = 0, support_scale = 1;
  Vec3 acceleration = Vec3::Zero(), baselink_rpy = Vec3::Zero();
  double yaw = 0, z = 0;
  ZMode z_mode = ZMode::Position;
};
struct ControlStatus
{
  // Object-frame offsets from the configuration captured at support transfer.
  // Empty for the legacy acceleration controller.
  std::vector<Vec3> contact_geometry_errors;
  bool support_ready = false;
  double normal_force = 0, reference_z = 0, reference_vz = 0, reference_az = 0;
  double z_integral = 0, vertical_force_target = 0, common_z_acceleration = 0;
  double support_ratio = 0, common_slip = 0;
  double reference_velocity_integral = 0;
  std::string fault;
};
struct ControlOutput
{
  std::vector<VehicleCommand> commands;
  ControlStatus status;
};
}  // namespace bee::grasp
