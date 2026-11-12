#include <bee/cooperative_grasp/controller.h>

namespace bee::grasp
{
void CooperativeGraspController::makeTrackingReferences(const GraspState& s, const Reference& r,
                                                       double dt, ControlOutput& output)
{
  for (auto& cmd : output.commands)
    cmd.calibrate = r.calibrate;
  if (r.phase < Phase::Preload || r.phase > Phase::Release)
  {
    tracking_initialized_ = false;
    return;
  }
  const auto& p = config_;
  if (support_active_)
  {
    // Monitor attachment drift without restoring the captured contact positions.
    status_.contact_geometry_errors = contact_geometry_.errors(s);
  }
  if (!tracking_initialized_)
  {
    // Initialize from the object motion, not from individual drift.
    for (auto& v : tracking_velocity_)
      v = s.object_velocity;
    tracking_initialized_ = true;
  }
  if (r.touchdown)
    status_.reference_velocity_integral = 0;
  double common_vz = 0;
  if (support_active_ && status_.support_ready && r.phase >= Phase::Lift && r.phase < Phase::Unload)
  {
    const double error = status_.reference_z - s.input.object.position.z();
    status_.reference_velocity_integral = clamp(status_.reference_velocity_integral + p.reference_z_ki * error * dt,
                                               p.reference_z_i_limit);
    common_vz = status_.reference_vz + p.reference_z_kp * error + status_.reference_velocity_integral;
  }
  if (status_.support_ready && r.phase < Phase::Unload)
    observer_blend_ = approach(observer_blend_, 1, dt / p.support_ramp_duration);
  for (std::size_t i = 0; i < output.commands.size(); ++i)
  {
    auto& cmd = output.commands[i];
    const auto& c = s.contacts[i];
    Vec3 desired = r.phase <= Phase::Unload ? r.object_velocity : Vec3::Zero();
    // This task holds object yaw constant. Do not feed actual angular velocity
    // into the reference: doing so follows unwanted rotation instead of damping it.
    if (p.contact_tangent_control_enabled && !support_active_)
      desired -= (p.tangent_kp / std::max(p.tangent_kd, 1e-6)) * c.tangent_position * c.tangent;
    if (support_active_ && r.phase <= Phase::Descend)
      desired += p.cooperative_admittance * yaw_acceleration_ * c.tangent;
    if (support_active_)
    {
      desired.z() = common_vz + p.cooperative_admittance * moment_z_[i];
      // Preserve an explicitly enabled common-slip correction as a velocity
      // adjustment. The shipped profile disables it.
      desired.z() += p.cooperative_admittance * slip_acceleration_;
    }
    else
      desired.z() = cmd.z;
    if (desired.norm() > p.reference_velocity_limit)
      desired *= p.reference_velocity_limit / desired.norm();
    Vec3 delta = desired - tracking_velocity_[i];
    const double step = p.reference_acceleration_limit * dt;
    if (delta.norm() > step && delta.norm() > 0)
      delta *= step / delta.norm();
    if (dt > 0)
    {
      tracking_velocity_[i] += delta;
      cmd.acceleration = delta / dt;
    }
    cmd.velocity = tracking_velocity_[i];
    cmd.support_force = status_.vertical_force_target / scene_.robot_names.size();
    cmd.observer_blend = observer_blend_;
    cmd.support_scale = r.phase == Phase::Release ? 0 : r.support_scale;
    cmd.z = support_active_ ? cmd.acceleration.z() : cmd.velocity.z();
  }
  output.status = status_;
}
}  // namespace bee::grasp
