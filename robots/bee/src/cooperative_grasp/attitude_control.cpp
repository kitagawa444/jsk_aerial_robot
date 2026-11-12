#include <bee/cooperative_grasp/controller.h>
#include <Eigen/QR>
#include <numeric>

namespace bee::grasp
{
namespace
{
void removeMean(std::vector<double>& values)
{
  const double mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
  for (auto& value : values)
    value -= mean;
}
}  // namespace
void CooperativeGraspController::updateAttitude(const GraspState& s, const Reference& r, double dt)
{
  const auto& p = config_;
  filtered_moment_ += p.object_moment_alpha * (s.object_wrench.moment - filtered_moment_);
  double vertical_mean = 0;
  bool loaded = true;
  for (std::size_t i = 0; i < s.contacts.size(); ++i)
  {
    local_moments_[i] += p.local_moment_alpha * (s.contacts[i].local_moment - local_moments_[i]);
    vertical_mean += local_moments_[i].z() / s.contacts.size();
    loaded = loaded && s.contacts[i].normal_force >= p.moment_force_threshold;
  }
  if (!support_active_)
  {
    std::vector<std::size_t> active;
    for (std::size_t i = 0; i < s.contacts.size(); ++i)
    {
      if (s.contacts[i].normal_force < p.moment_force_threshold)
        continue;
      const double roll_error = deadband(local_moments_[i].y(), p.local_deadband);
      const double yaw_error = deadband(local_moments_[i].z() - vertical_mean, p.local_deadband);
      roll_i_[i] = clamp(roll_i_[i] + p.local_roll_ki * roll_error * dt, p.trim_limit);
      yaw_i_[i] = clamp(yaw_i_[i] + p.local_yaw_ki * yaw_error * dt, p.trim_limit);
      roll_trim_[i] =
          approach(roll_trim_[i], clamp(roll_i_[i] + p.local_roll_kp * roll_error, p.trim_limit), p.trim_rate * dt);
      yaw_trim_[i] =
          approach(yaw_trim_[i], clamp(yaw_i_[i] + p.local_yaw_kp * yaw_error, p.trim_limit), p.trim_rate * dt);
      active.push_back(i);
    }
    if (!active.empty())
    {
      double mean_i = 0, mean_trim = 0;
      for (auto i : active)
      {
        mean_i += yaw_i_[i] / active.size();
        mean_trim += yaw_trim_[i] / active.size();
      }
      for (auto i : active)
      {
        yaw_i_[i] -= mean_i;
        yaw_trim_[i] -= mean_trim;
      }
    }
  }
  if (!loaded || dt <= 0)
    return;
  Eigen::MatrixXd allocation(3, s.contacts.size());
  for (std::size_t i = 0; i < s.contacts.size(); ++i)
  {
    const Vec3 arm = s.contacts[i].position - s.input.object.position;
    const Vec3 direction = support_active_ ? Vec3(Vec3::UnitZ()) : Vec3(-s.contacts[i].normal);
    const Vec3 moment = arm.cross(direction);
    allocation.col(i) = Vec3(moment.x(), moment.y(), 1);
  }
  Eigen::Vector3d desired = Vec3::Zero();
  if (support_active_)
  {
    const Vec3 vertical = s.input.object.orientation * Vec3::UnitZ();
    const Vec3 tilt_error = tilt_target_.cross(vertical);
    for (int axis = 0; axis < 2; ++axis)
      desired[axis] = -p.tilt_moment_gain * deadband(filtered_moment_[axis], .001) - p.tilt_kp * tilt_error[axis] -
                      p.tilt_kd * s.object_angular_velocity[axis];
    for (std::size_t i = 0; i < force_offsets_.size(); ++i)
    {
      force_offsets_[i] = approach(force_offsets_[i], 0, p.radial_moment_rate * dt);
      force_i_[i] = approach(force_i_[i], 0, p.radial_moment_rate * dt);
    }
    removeMean(force_offsets_);
    removeMean(force_i_);
    const double yaw_error = angleError(yaw(s.input.object.orientation) - r.object_yaw);
    const double target = clamp(-p.yaw_kp * yaw_error - p.yaw_kd * s.object_angular_velocity.z() -
                                    p.yaw_moment_gain * deadband(filtered_moment_.z(), .002),
                                p.yaw_limit);
    yaw_acceleration_ = approach(yaw_acceleration_, target, p.yaw_rate * dt);
  }
  else
  {
    desired.x() = -deadband(filtered_moment_.x(), .0003);
    desired.y() = -deadband(filtered_moment_.y(), .0003);
    const double error = deadband(filtered_moment_.z(), .0003);
    if (object_yaw_integral_ * error > 0)
      object_yaw_integral_ *= std::max(0.0, 1 - .5 * dt);
    object_yaw_integral_ = clamp(object_yaw_integral_ - p.object_yaw_ki * error * dt, p.trim_limit);
    object_yaw_trim_ = approach(object_yaw_trim_, clamp(object_yaw_integral_ - p.object_yaw_kp * error, p.trim_limit),
                                p.trim_rate * dt);
  }
  const Eigen::Matrix3d gram = allocation * allocation.transpose();
  Eigen::ColPivHouseholderQR<Eigen::Matrix3d> solver(gram);
  solver.setThreshold(1e-9);
  if (solver.rank() < 3)
  {
    if (support_active_)
      status_.fault = "vertical-friction allocation is singular";
    return;  // radial preload allocation can be singular at zero contact height
  }
  const Eigen::VectorXd correction = allocation.transpose() * solver.solve(desired);
  for (std::size_t i = 0; i < s.contacts.size(); ++i)
  {
    if (support_active_)
      moment_z_[i] = approach(moment_z_[i], clamp(correction[i] / p.robot_mass, p.tilt_limit), p.tilt_rate * dt);
    else
      force_i_[i] = clamp(force_i_[i] + p.radial_moment_ki * correction[i] * dt, p.radial_moment_limit);
  }
  if (support_active_)
    removeMean(moment_z_);
  else
  {
    removeMean(force_i_);
    for (std::size_t i = 0; i < s.contacts.size(); ++i)
      force_offsets_[i] =
          approach(force_offsets_[i], clamp(force_i_[i] + p.radial_moment_kp * correction[i], p.radial_moment_limit),
                   p.radial_moment_rate * dt);
    removeMean(force_offsets_);
  }
}
}  // namespace bee::grasp
