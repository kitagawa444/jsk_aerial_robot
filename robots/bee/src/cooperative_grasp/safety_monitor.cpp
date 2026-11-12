#include <bee/cooperative_grasp/safety_monitor.h>

namespace bee::grasp
{
std::string GraspSafetyMonitor::check(const GraspState& s, const Reference& r, const ControlOutput& output, double dt)
{
  if (!output.status.fault.empty())
    return output.status.fault;
  if (!s.input.object.position.allFinite() || !s.object_velocity.allFinite() ||
      !s.object_angular_velocity.allFinite() || !s.object_wrench.force.allFinite() ||
      !s.object_wrench.moment.allFinite())
    return "non-finite object state";
  for (const auto& v : s.input.vehicles)
    if (v.flight_state != 5)
      return "vehicle left HOVER";
  for (const auto& c : output.commands)
    if (!c.velocity.allFinite() || !c.normal.allFinite() || !c.object_position.allFinite() ||
        !std::isfinite(c.normal_force) || !std::isfinite(c.support_force) ||
        !c.acceleration.allFinite() || !c.baselink_rpy.allFinite() || !std::isfinite(c.yaw) || !std::isfinite(c.z) ||
        c.acceleration.norm() > config_.max_acceleration)
      return "invalid or excessive command";
  if (!airborne(r.phase))
  {
    force_loss_time_ = 0;
    return {};
  }
  const Vec3 vertical = s.input.object.orientation * Vec3::UnitZ();
  if (std::acos(std::clamp(vertical.z(), -1.0, 1.0)) > config_.max_tilt)
    return "object tilt limit";
  if (s.object_velocity.norm() > config_.max_object_speed ||
      std::abs(s.object_angular_velocity.z()) > config_.max_yaw_rate)
    return "object speed limit";
  if (std::abs(angleError(yaw(s.input.object.orientation) - r.object_yaw)) > .35)
    return "object yaw limit";
  const auto& geometry_errors = output.status.contact_geometry_errors;
  Vec3 mean_geometry_error = Vec3::Zero();
  if (!geometry_errors.empty())
  {
    if (geometry_errors.size() != s.contacts.size())
      return "contact geometry count mismatch";
    for (const auto& error : geometry_errors)
    {
      if (!error.allFinite()) return "non-finite contact geometry error";
      mean_geometry_error += error / geometry_errors.size();
    }
  }
  if ((!geometry_errors.empty() && mean_geometry_error.norm() > config_.max_slip) ||
      (geometry_errors.empty() && std::abs(output.status.common_slip) > config_.max_slip))
    return "mean contact slip limit";
  if (r.phase != Phase::Lift && r.object_z - s.input.object.position.z() > config_.max_drop)
    return "object dropped below target";
  double mean = 0;
  int lost = 0;
  for (const auto& c : s.contacts)
  {
    mean += c.relative_z / s.contacts.size();
    if (c.normal_force < config_.force_loss_threshold)
      ++lost;
  }
  if (!geometry_errors.empty())
  {
    for (const auto& error : geometry_errors)
      if ((error - mean_geometry_error).norm() > config_.max_contact_deviation)
        return "contact configuration deviation limit";
  }
  else
    for (const auto& c : s.contacts)
      if (std::abs(c.relative_z - mean) > config_.max_height_difference)
        return "contact height difference limit";
  force_loss_time_ = lost >= 2 ? force_loss_time_ + dt : 0;
  if (force_loss_time_ > config_.force_loss_duration)
    return "contact force lost";
  return {};
}
}  // namespace bee::grasp
