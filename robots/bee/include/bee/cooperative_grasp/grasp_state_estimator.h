#pragma once
#include <aerial_robot_model/model/gravity.h>

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
  // Zero radii select planar faces. Positive radii describe an analytic ellipsoid.
  Vec3 surface_radii = Vec3::Zero();
  std::vector<Vec3> surface_vertices, contact_normals;  // Convex mesh, CoM frame; one planned face per Bee.
  std::string object_shape;
  double surface_radius = 0, surface_top_radius = 0, surface_length = 0;
  // Distance from CoM to bottom. Negative retains the legacy symmetric shape.
  double object_bottom_offset = -1;
  Eigen::Matrix3d object_inertia = Eigen::Matrix3d::Identity() * .02;
  double contact_height = 0;
  double object_mass = 0.5, object_height = 0.6, staging_clearance = 0.8;
  Vec3 object_position = Vec3(0, 1, .95), destination_position = Vec3(1.5, 1, .95);
  bool has_destination = false;
  void validate() const;
};
inline void requireNonnegativeParameter(double x, const char* name)
{
  if (!std::isfinite(x) || x < 0)
    throw std::invalid_argument(std::string("invalid parameter: ") + name);
}
inline void requirePositiveParameter(double x, const char* name)
{
  requireNonnegativeParameter(x, name);
  if (x == 0)
    throw std::invalid_argument(std::string("parameter must be positive: ") + name);
}

struct ControlConfig
{
  // Legacy path remains available for regression comparisons.
  bool distributed_tracking = false, wrench_control = false;
  double object_kp = .5, object_kd = 1.4, object_ki = .04, object_i_limit = .08;
  double object_rotation_kp = 1.0, object_rotation_kd = 1.8;
  double allocation_mu = 1.0, allocation_force_rate = 20, allocation_thrust_limit = 32;
  // Contact-frame force uncertainty [N]; zero retains nominal force constraints.
  double allocation_normal_error = 0, allocation_tangent_up_error = 0, allocation_tangent_side_error = 0;
  double allocation_observer_normal_error = .1;
  // World L1 force budget [N], shared by central allocation and local feedback.
  double allocation_thrust_reserve = 3;
  double contact_force_kp = .4, contact_force_ki = .1, contact_force_i_limit = .4;
  double contact_damping = .5;
  double contact_force_filter_tau = 0;  // [s], zero bypasses force-PI output LPF.

  double reference_z_kp = .5, reference_z_ki = .05, reference_z_i_limit = .015;
  double reference_velocity_limit = .08, reference_acceleration_limit = .1;
  double cooperative_admittance = .15;
  double robot_mass = 1.84199, gravity = aerial_robot_model::kGravity;
  Vec3 contact_offset = Vec3(.0175, 0, -.143);
  double foot_contact_offset = .089, ball_radius = .010, max_compression = .015;
  double observer_force_sign = 1.0, observer_force_scale = 1.0, observer_torque_scale = 1.0;
  double force_filter_alpha = .1, twist_filter_alpha = .2;
  double radial_kp = 3, radial_kd = 3, tangent_kp = 2, tangent_kd = 3;
  bool contact_tangent_control_enabled = true;
  double approach_limit = .8, press_limit = 15, normal_force_kp = .4;
  double press_radial_kp = 400, radius_margin = .05, normal_force_rate = .3, max_normal_force = 10;
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

Vec3 tangentProjection(const Vec3& n, const Vec3& v);
bool additionalSurface(const SceneConfig& s);
double bottomOffset(const SceneConfig& s);
Vec3 plannedContactPoint(const SceneConfig& scene, std::size_t i);
Vec3 footCenterTarget(const SceneConfig& scene, std::size_t i, const Pose& object,
                      double ball_radius, double gap);
Vec3 plannedContactNormal(const SceneConfig& scene, std::size_t i);
Vec3 contactAttitude(const Vec3& world_normal, double fraction, double fallback_heading);
Vec3 surfaceNormal(const SceneConfig& s, const Vec3& p, const Vec3& planar);
double surfaceSupport(const SceneConfig& s, const Vec3& n, double planar_radius);

class GraspStateEstimator
{
public:
  GraspStateEstimator(SceneConfig scene, ControlConfig config);
  void reset();
  void beginCalibration();
  void addCalibrationSample(const InputState& input);
  bool finishCalibration();
  bool applyCalibration(std::size_t robot, const Wrench& baseline, const Vec3& world_force);
  GraspState update(const InputState& input);

private:
  SceneConfig scene_;
  ControlConfig config_;
  std::vector<Wrench> baseline_, sums_;
  std::vector<Vec3> world_baseline_, world_sums_;
  std::vector<double> normal_forces_;
  std::size_t samples_ = 0;
  double previous_stamp_ = 0;
  Pose previous_object_;
  Vec3 velocity_ = Vec3::Zero(), omega_ = Vec3::Zero();
};
}  // namespace bee::grasp
