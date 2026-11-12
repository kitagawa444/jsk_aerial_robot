#include <bee/cooperative_grasp/controller.h>
#include <numeric>

namespace bee::grasp
{
CooperativeGraspController::CooperativeGraspController(SceneConfig scene, ControlConfig config)
  : scene_(std::move(scene)), config_(std::move(config))
{
  scene_.validate();
  config_.validate();
  reset();
}
void CooperativeGraspController::reset()
{
  contact_geometry_.reset();
  tracking_initialized_ = false;
  observer_blend_ = 0;
  tracking_velocity_.assign(scene_.robot_names.size(), Vec3::Zero());
  status_ = {};
  support_active_ = false;
  slip_active_ = false;
  initial_z_ = relative_z_ = slip_reference_ = support_stable_ = 0;
  filtered_force_ = transfer_force_ = force_integral_ = slip_acceleration_ = 0;
  yaw_acceleration_ = object_yaw_trim_ = object_yaw_integral_ = 0;
  translation_.setZero();
  filtered_moment_.setZero();
  tilt_target_ = Vec3::UnitZ();
  for (auto* values :
       { &contact_z_, &moment_z_, &roll_trim_, &yaw_trim_, &roll_i_, &yaw_i_, &force_offsets_, &force_i_ })
    values->assign(scene_.robot_names.size(), 0);
  local_moments_.assign(scene_.robot_names.size(), Vec3::Zero());
}
ControlOutput CooperativeGraspController::step(const GraspState& s, const Reference& r, double dt)
{
  if (s.contacts.size() != scene_.robot_names.size() || s.input.vehicles.size() != scene_.robot_names.size())
    throw std::invalid_argument("controller vehicle count mismatch");
  if (!std::isfinite(dt) || dt < 0)
    throw std::invalid_argument("invalid control time step");
  dt = std::min(dt, .05);
  if (r.begin_support && !support_active_)
  {
    support_active_ = true;
    if (config_.distributed_tracking)
      contact_geometry_.capture(s);
    initial_z_ = s.input.object.position.z();
    status_.reference_z = initial_z_;
    relative_z_ = 0;
    for (const auto& c : s.contacts)
      relative_z_ += c.relative_z / s.contacts.size();
    slip_reference_ = relative_z_;
    tilt_target_ = s.input.object.orientation * Vec3::UnitZ();
  }
  if (!support_active_ && r.phase <= Phase::Approach)
  {
    relative_z_ = 0;
    for (const auto& c : s.contacts)
      relative_z_ += c.relative_z / s.contacts.size();
  }
  if (r.touchdown)
  {
    status_.z_integral = 0;
    slip_active_ = false;
    slip_acceleration_ = 0;
  }
  updateNormalForce(r, dt);
  if (r.phase >= Phase::Preload && r.phase <= Phase::Unload)
  {
    updateAttitude(s, r, dt);
    if (support_active_)
      updateVertical(s, r, dt);
  }
  if (!config_.distributed_tracking && support_active_ && r.phase <= Phase::Unload)
  {
    Vec3 desired = config_.translation_kp * (r.object_velocity - s.object_velocity);
    desired.z() = 0;
    if (desired.norm() > config_.translation_limit)
      desired *= config_.translation_limit / desired.norm();
    for (int axis = 0; axis < 2; ++axis)
      translation_[axis] = approach(translation_[axis], desired[axis], config_.translation_rate * dt);
  }
  else
    translation_.setZero();
  ControlOutput output;
  for (std::size_t i = 0; i < s.contacts.size(); ++i)
  {
    auto command =
        (r.phase >= Phase::Preload && r.phase <= Phase::Release) ? contactCommand(i, s, r) : approachCommand(i, s, r);
    output.commands.push_back(command);
  }
  output.status = status_;
  if (config_.distributed_tracking)
    makeTrackingReferences(s, r, dt, output);
  return output;
}
VehicleCommand CooperativeGraspController::approachCommand(std::size_t i, const GraspState& s, const Reference& r) const
{
  const auto& c = s.contacts[i];
  const auto& v = s.input.vehicles[i];
  const double radius = (v.base.position - s.input.object.position).dot(c.normal);
  const double desired_radius = scene_.face_radii[i] + r.radius_clearance;
  const double radial =
      clamp(config_.radial_kp * (desired_radius - radius) - config_.radial_kd * v.velocity_world.dot(c.normal),
            config_.approach_limit);
  const double tangent =
      clamp(-config_.tangent_kp * c.tangent_position - config_.tangent_kd * c.tangent_velocity, config_.approach_limit);
  VehicleCommand cmd;
  cmd.acceleration = radial * c.normal + tangent * c.tangent;
  cmd.yaw = std::atan2(c.normal.y(), c.normal.x()) + kPi / 2;
  cmd.baselink_rpy = Vec3(r.roll, 0, 0);
  cmd.z_mode = ZMode::Position;
  cmd.z = r.object_z;
  return cmd;
}
VehicleCommand CooperativeGraspController::contactCommand(std::size_t i, const GraspState& s, const Reference& r) const
{
  const auto& c = s.contacts[i];
  const double target = std::clamp(status_.normal_force + force_offsets_[i], 0.0, config_.max_normal_force);
  if (config_.distributed_tracking)
  {
    VehicleCommand cmd;
    cmd.contact_tracking = true;
    cmd.support_tracking = support_active_;
    cmd.normal = c.normal;
    cmd.normal_force = target;
    cmd.object_position = s.input.object.position;
    cmd.minimum_radius = scene_.face_radii[i] + config_.foot_contact_offset + config_.ball_radius -
                         config_.max_compression - config_.radius_margin;
    cmd.yaw = std::atan2(c.normal.y(), c.normal.x()) + kPi / 2 + yaw_trim_[i] + object_yaw_trim_;
    cmd.baselink_rpy = Vec3(r.roll + roll_trim_[i], 0, 0);
    cmd.z_mode = support_active_ ? ZMode::Acceleration : ZMode::Velocity;
    if (!support_active_)
      cmd.z = clamp(-config_.z_velocity_kp * (c.relative_z - relative_z_) - config_.z_velocity_kd * c.relative_vz,
                    config_.z_velocity_limit);
    return cmd;
  }
  double radial = -target / config_.robot_mass - config_.normal_force_kp * (target - c.normal_force) -
                  config_.radial_kd * c.radial_velocity;
  const double vehicle_radius = (s.input.vehicles[i].base.position - s.input.object.position).dot(c.normal);
  const double desired_radius =
      scene_.face_radii[i] + config_.foot_contact_offset + config_.ball_radius - config_.max_compression;
  if (vehicle_radius < desired_radius - config_.radius_margin)
    radial = config_.press_radial_kp * (desired_radius - vehicle_radius) - config_.radial_kd * c.radial_velocity;
  // Allow contact-centering ablation without changing approach or object yaw control.
  double tangent = config_.contact_tangent_control_enabled ?
                       -config_.tangent_kp * c.tangent_position - config_.tangent_kd * c.tangent_velocity :
                       0.0;
  if (support_active_ && r.phase <= Phase::Descend)
    tangent += yaw_acceleration_;
  VehicleCommand cmd;
  cmd.acceleration =
      clamp(radial, config_.press_limit) * c.normal + clamp(tangent, config_.approach_limit) * c.tangent + translation_;
  cmd.yaw = std::atan2(c.normal.y(), c.normal.x()) + kPi / 2 + yaw_trim_[i] + object_yaw_trim_;
  cmd.baselink_rpy = Vec3(r.roll + roll_trim_[i], 0, 0);
  if (support_active_)
  {
    cmd.z_mode = ZMode::Acceleration;
    cmd.z = clamp(status_.common_z_acceleration + contact_z_[i] + moment_z_[i],
                  config_.common_z_limit + config_.relative_z_limit + config_.tilt_limit);
    cmd.acceleration.z() = cmd.z;
  }
  else
  {
    cmd.z_mode = ZMode::Velocity;
    cmd.z = clamp(-config_.z_velocity_kp * (c.relative_z - relative_z_) - config_.z_velocity_kd * c.relative_vz,
                  config_.z_velocity_limit);
  }
  return cmd;
}
}  // namespace bee::grasp
