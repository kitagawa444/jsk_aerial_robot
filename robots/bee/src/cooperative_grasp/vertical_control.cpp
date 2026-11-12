#include <bee/cooperative_grasp/controller.h>
#include <numeric>

namespace bee::grasp
{
void CooperativeGraspController::advanceZReference(double target, double velocity_limit, double dt)
{
  if (dt <= 0)
    return;
  const double error = target - status_.reference_z;
  const double limit = std::max(config_.trajectory_velocity, velocity_limit);
  double desired_v =
      std::copysign(std::min(limit, std::sqrt(2 * config_.trajectory_acceleration * std::abs(error))), error);
  if (std::abs(error) < 1e-5)
    desired_v = 0;
  const double desired_a = clamp((desired_v - status_.reference_vz) / dt, config_.trajectory_acceleration);
  status_.reference_az = approach(status_.reference_az, desired_a, config_.trajectory_jerk * dt);
  status_.reference_vz = clamp(status_.reference_vz + status_.reference_az * dt, limit);
  const double next = status_.reference_z + status_.reference_vz * dt;
  if (error * (target - next) <= 0)
  {
    status_.reference_z = target;
    status_.reference_vz = 0;
    status_.reference_az = 0;
  }
  else
    status_.reference_z = next;
}
void CooperativeGraspController::updateVertical(const GraspState& s, const Reference& r, double dt)
{
  if (dt <= 0)
    return;  // paused simulation must retain the last support command
  const auto& p = config_;
  filtered_force_ += dt / (p.observer_filter_tau + dt) * (s.object_wrench.force.z() - filtered_force_);
  transfer_force_ += dt / (p.support_filter_tau + dt) * (s.object_wrench.force.z() - transfer_force_);
  double relative_z = 0, relative_v = 0;
  for (const auto& c : s.contacts)
  {
    relative_z += c.relative_z / s.contacts.size();
    relative_v += c.relative_vz / s.contacts.size();
  }
  if (!slip_active_ && airborne(r.phase) &&
      s.input.object.position.z() - initial_z_ >= std::max(0.0, r.object_z - initial_z_ - .003))
  {
    slip_active_ = true;
    slip_reference_ = relative_z;
  }
  status_.common_slip = relative_z - slip_reference_;
  if (slip_active_)
    slip_acceleration_ =
        approach(slip_acceleration_, clamp(-p.slip_kp * status_.common_slip - p.slip_kd * relative_v, p.slip_limit),
                 p.slip_rate * dt);
  for (std::size_t i = 0; i < s.contacts.size(); ++i)
  {
    const auto& c = s.contacts[i];
    const double desired =
        clamp(-p.relative_z_kp * (c.relative_z - relative_z) - p.relative_z_kd * (c.relative_vz - relative_v),
              p.relative_z_limit);
    contact_z_[i] = approach(contact_z_[i], desired, p.relative_z_rate * dt);
  }
  const double mean = std::accumulate(contact_z_.begin(), contact_z_.end(), 0.0) / contact_z_.size();
  for (auto& value : contact_z_)
    value -= mean;
  const double weight = scene_.object_mass * p.gravity;
  const double allowed = std::clamp((std::max(0.0, transfer_force_) + p.support_force_lead) / weight, 0.0, 1.0);
  status_.support_ratio = std::min(std::min(1.0, status_.support_ratio + dt / p.support_ramp_duration),
                                   std::max(status_.support_ratio, allowed));
  if (!status_.support_ready)
  {
    const bool ready = status_.support_ratio >= 1 - 1e-6 && transfer_force_ >= p.support_ratio * weight &&
                       std::abs(s.object_velocity.z()) <= p.support_velocity_tolerance;
    support_stable_ = ready ? support_stable_ + dt : 0;
    status_.support_ready = support_stable_ >= p.support_settle;
  }
  if (p.distributed_tracking)
  {
    // Actual motion references only. Load compensation is owned by each Bee.
    if (status_.support_ready && r.phase >= Phase::Lift)
      advanceZReference(r.object_z, r.z_velocity_limit, dt);
    else
    {
      status_.reference_z = s.input.object.position.z();
      status_.reference_vz = status_.reference_az = 0;
    }
    status_.vertical_force_target = std::max(0.0, scene_.object_mass *
        (status_.support_ratio * p.gravity + status_.reference_az));
    status_.common_z_acceleration = 0;
    return;
  }
  double object_acceleration;
  if (status_.support_ready && r.phase >= Phase::Lift)
  {
    advanceZReference(r.object_z, r.z_velocity_limit, dt);
    const double error = status_.reference_z - s.input.object.position.z();
    const double step = p.z_ki * error * dt;
    if (r.phase < Phase::Unload && !((status_.z_integral >= p.z_i_limit - 1e-12 && step > 0) ||
                                     (status_.z_integral <= -p.z_i_limit + 1e-12 && step < 0)))
      status_.z_integral = clamp(status_.z_integral + step, p.z_i_limit);
    object_acceleration =
        status_.reference_az +
        clamp(p.z_kp * error + p.z_kd * (status_.reference_vz - s.object_velocity.z()), p.z_pd_limit) +
        status_.z_integral;
  }
  else
  {
    status_.reference_z = s.input.object.position.z();
    status_.reference_vz = status_.reference_az = status_.z_integral = 0;
    object_acceleration = clamp(-2.5 * s.object_velocity.z(), .03);
  }
  status_.vertical_force_target =
      std::max(0.0, scene_.object_mass * (status_.support_ratio * p.gravity + object_acceleration));
  const double error = deadband(status_.vertical_force_target - filtered_force_, p.observer_deadband);
  force_integral_ = clamp(force_integral_ + p.observer_ki * error * dt, p.observer_i_limit);
  const double correction = clamp(p.observer_kp * error + force_integral_, p.observer_correction_limit);
  const double corrected_force = std::max(0.0, r.support_scale * status_.vertical_force_target + correction);
  double command =
      object_acceleration + corrected_force / (scene_.robot_names.size() * p.robot_mass) + slip_acceleration_;
  if (slip_active_)
    command = std::max(command, p.support_floor_scale * weight / (scene_.robot_names.size() * p.robot_mass));
  status_.common_z_acceleration = clamp(command, p.common_z_limit);
}
}  // namespace bee::grasp
