#include <bee/cooperative_grasp/types.h>
namespace bee::grasp
{
namespace
{
void nonnegative(double x, const char* name)
{
  if (!std::isfinite(x) || x < 0)
    throw std::invalid_argument(std::string("invalid parameter: ") + name);
}
void positive(double x, const char* name)
{
  nonnegative(x, name);
  if (x == 0)
    throw std::invalid_argument(std::string("parameter must be positive: ") + name);
}
}  // namespace
void ControlConfig::validate() const
{
  positive(reference_z_kp, "reference_z_kp");
  nonnegative(reference_z_ki, "reference_z_ki");
  positive(reference_z_i_limit, "reference_z_i_limit");
  positive(reference_velocity_limit, "reference_velocity_limit");
  positive(reference_acceleration_limit, "reference_acceleration_limit");
  nonnegative(cooperative_admittance, "cooperative_admittance");
  nonnegative(robot_mass, "robot_mass");
  nonnegative(gravity, "gravity");
  nonnegative(foot_contact_offset, "foot_contact_offset");
  nonnegative(ball_radius, "ball_radius");
  nonnegative(max_compression, "max_compression");
  if (observer_force_sign != 1 && observer_force_sign != -1)
    throw std::invalid_argument("observer_force_sign must be +/-1");
  nonnegative(observer_force_scale, "observer_force_scale");
  nonnegative(observer_torque_scale, "observer_torque_scale");
  nonnegative(force_filter_alpha, "force_filter_alpha");
  nonnegative(twist_filter_alpha, "twist_filter_alpha");
  nonnegative(radial_kp, "radial_kp");
  nonnegative(radial_kd, "radial_kd");
  nonnegative(tangent_kp, "tangent_kp");
  nonnegative(tangent_kd, "tangent_kd");
  nonnegative(approach_limit, "approach_limit");
  nonnegative(press_limit, "press_limit");
  nonnegative(normal_force_kp, "normal_force_kp");
  nonnegative(press_radial_kp, "press_radial_kp");
  nonnegative(radius_margin, "radius_margin");
  nonnegative(normal_force_rate, "normal_force_rate");
  nonnegative(max_normal_force, "max_normal_force");
  nonnegative(z_velocity_kp, "z_velocity_kp");
  nonnegative(z_velocity_kd, "z_velocity_kd");
  nonnegative(z_velocity_limit, "z_velocity_limit");
  nonnegative(trajectory_velocity, "trajectory_velocity");
  nonnegative(trajectory_acceleration, "trajectory_acceleration");
  nonnegative(trajectory_jerk, "trajectory_jerk");
  nonnegative(z_kp, "z_kp");
  nonnegative(z_kd, "z_kd");
  nonnegative(z_ki, "z_ki");
  nonnegative(z_pd_limit, "z_pd_limit");
  nonnegative(z_i_limit, "z_i_limit");
  nonnegative(support_ramp_duration, "support_ramp_duration");
  nonnegative(support_filter_tau, "support_filter_tau");
  nonnegative(support_force_lead, "support_force_lead");
  nonnegative(support_ratio, "support_ratio");
  nonnegative(support_settle, "support_settle");
  nonnegative(support_velocity_tolerance, "support_velocity_tolerance");
  nonnegative(observer_filter_tau, "observer_filter_tau");
  nonnegative(observer_kp, "observer_kp");
  nonnegative(observer_ki, "observer_ki");
  nonnegative(observer_i_limit, "observer_i_limit");
  nonnegative(observer_correction_limit, "observer_correction_limit");
  nonnegative(observer_deadband, "observer_deadband");
  nonnegative(slip_kp, "slip_kp");
  nonnegative(slip_kd, "slip_kd");
  nonnegative(slip_limit, "slip_limit");
  nonnegative(slip_rate, "slip_rate");
  nonnegative(relative_z_kp, "relative_z_kp");
  nonnegative(relative_z_kd, "relative_z_kd");
  nonnegative(relative_z_limit, "relative_z_limit");
  nonnegative(relative_z_rate, "relative_z_rate");
  nonnegative(common_z_limit, "common_z_limit");
  nonnegative(support_floor_scale, "support_floor_scale");
  nonnegative(translation_kp, "translation_kp");
  nonnegative(translation_limit, "translation_limit");
  nonnegative(translation_rate, "translation_rate");
  nonnegative(yaw_kp, "yaw_kp");
  nonnegative(yaw_kd, "yaw_kd");
  nonnegative(yaw_moment_gain, "yaw_moment_gain");
  nonnegative(yaw_limit, "yaw_limit");
  nonnegative(yaw_rate, "yaw_rate");
  nonnegative(tilt_kp, "tilt_kp");
  nonnegative(tilt_kd, "tilt_kd");
  nonnegative(tilt_moment_gain, "tilt_moment_gain");
  nonnegative(tilt_limit, "tilt_limit");
  nonnegative(tilt_rate, "tilt_rate");
  nonnegative(local_moment_alpha, "local_moment_alpha");
  nonnegative(object_moment_alpha, "object_moment_alpha");
  nonnegative(moment_force_threshold, "moment_force_threshold");
  nonnegative(local_roll_kp, "local_roll_kp");
  nonnegative(local_roll_ki, "local_roll_ki");
  nonnegative(local_yaw_kp, "local_yaw_kp");
  nonnegative(local_yaw_ki, "local_yaw_ki");
  nonnegative(trim_limit, "trim_limit");
  nonnegative(trim_rate, "trim_rate");
  nonnegative(local_deadband, "local_deadband");
  nonnegative(radial_moment_kp, "radial_moment_kp");
  nonnegative(radial_moment_ki, "radial_moment_ki");
  nonnegative(radial_moment_limit, "radial_moment_limit");
  nonnegative(radial_moment_rate, "radial_moment_rate");
  nonnegative(object_yaw_kp, "object_yaw_kp");
  nonnegative(object_yaw_ki, "object_yaw_ki");
  positive(robot_mass, "robot_mass");
  positive(gravity, "gravity");
  positive(trajectory_acceleration, "trajectory_acceleration");
  positive(trajectory_velocity, "trajectory_velocity");
  positive(support_ramp_duration, "support_ramp_duration");
  positive(support_filter_tau, "support_filter_tau");
  positive(observer_filter_tau, "observer_filter_tau");
  positive(max_normal_force, "max_normal_force");
  positive(approach_limit, "approach_limit");
  positive(press_limit, "press_limit");
  if (!contact_offset.allFinite())
    throw std::invalid_argument("invalid contact_offset");
  if (force_filter_alpha > 1 || twist_filter_alpha > 1 || local_moment_alpha > 1 || object_moment_alpha > 1)
    throw std::invalid_argument("filter alpha must be in [0, 1]");
}
void TaskConfig::validate() const
{
  positive(friction_coefficient, "friction_coefficient");
  positive(normal_force_margin, "normal_force_margin");
  nonnegative(design_vertical_acceleration, "design_vertical_acceleration");
  positive(preload_force_ratio, "preload_force_ratio");
  if (normal_force_margin < 1 || preload_force_ratio > 1)
    throw std::invalid_argument("normal_force_margin must be >= 1 and preload_force_ratio in (0, 1]");
  positive(approach_speed, "approach_speed");
  positive(staging_tolerance, "staging_tolerance");
  positive(attitude_duration, "attitude_duration");
  positive(attitude_settle, "attitude_settle");
  positive(baseline_duration, "baseline_duration");
  positive(preload_duration, "preload_duration");
  positive(lift_height, "lift_height");
  positive(hold_tolerance, "hold_tolerance");
  positive(hold_velocity, "hold_velocity");
  positive(hold_duration, "hold_duration");
  positive(transport_kp, "transport_kp");
  positive(transport_speed, "transport_speed");
  positive(position_tolerance, "position_tolerance");
  positive(velocity_tolerance, "velocity_tolerance");
  positive(arrival_duration, "arrival_duration");
  positive(destination_settle, "destination_settle");
  positive(descent_speed, "descent_speed");
  positive(touchdown_duration, "touchdown_duration");
  positive(unload_duration, "unload_duration");
  positive(release_rate, "release_rate");
  positive(release_tolerance, "release_tolerance");
  positive(release_duration, "release_duration");
  positive(retreat_clearance, "retreat_clearance");
  positive(verify_duration, "verify_duration");
  positive(staging_timeout, "staging_timeout");
  positive(approach_timeout, "approach_timeout");
  positive(lift_timeout, "lift_timeout");
  positive(transport_timeout, "transport_timeout");
  positive(descent_timeout, "descent_timeout");
  positive(release_timeout, "release_timeout");
  positive(retreat_timeout, "retreat_timeout");
}
}  // namespace bee::grasp
