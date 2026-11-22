#pragma once
#include <bee/cooperative_grasp/grasp_controller.h>
#include <bee/BeeCommand.h>

namespace bee::grasp
{
// Task-specific translation lives with the central controller. The Bee only
// receives motion/force goals and a generic acceleration box, never geometry.
inline bee::BeeCommand makeBeeCommand(const VehicleCommand& c,
    const VehicleState& vehicle, const ContactState& contact, const ControlConfig& p,
    const aerial_robot_msgs::FlightNav& navigation, const std_msgs::Header& header,
    const Vec3& filtered_external, double velocity_gain_xy = 3.0, double velocity_gain_z = 3.0)
{
  bee::BeeCommand out;
  out.header = header; out.navigation = navigation;
  auto copy = [](const Vec3& v, geometry_msgs::Vector3& m) { m.x=v.x(); m.y=v.y(); m.z=v.z(); };
  const auto outward = c.normal.head<2>();
  if ((c.contact_tracking || c.normal_force > 0) && outward.norm() > 1e-3)
  {
    const Eigen::Vector2d offset = 0.10 * outward.normalized();
    out.cancel_offset.x = offset.x();
    out.cancel_offset.y = offset.y();
  }
  copy(c.baselink_rpy, out.baselink_rpy);
  if (p.wrench_control)
  {
    out.enable_force_control = true;
    out.force_integral_enabled = true;
    if (!c.contact_tracking)
    {
      copy(-c.normal_force * c.normal, out.force);
      if (c.z_mode == ZMode::Position)
      {
        out.navigation.pos_xy_nav_mode = navigation.POS_VEL_MODE;
        out.navigation.target_pos_x = c.position_target.x();
        out.navigation.target_pos_y = c.position_target.y();
        out.navigation.target_acc_x = out.navigation.target_acc_y = 0;
      }
      // The motion controller already damps foot motion.
      copy(contact.velocity, out.velocity);
      copy(p.contact_offset, out.velocity_point);
      return out;
    }
  }
  else if (!c.contact_tracking) return out;
  const Vec3 n = c.normal;
  const Vec3 error = c.velocity - contact.velocity;
  const double radius = (vehicle.base.position - c.object_position).dot(n);
  const bool protective = radius < c.minimum_radius;
  double radial = 0;
  if (protective)
    radial = p.press_radial_kp * (c.minimum_radius + p.radius_margin - radius) + p.radial_kd * error.dot(n);
  out.navigation.pos_xy_nav_mode = out.navigation.pos_z_nav_mode = navigation.ACC_MODE;
  if (!p.wrench_control && !c.wrench_tracking)
  {
    if (!protective)
      radial = -c.normal_force / p.robot_mass - p.normal_force_kp * (c.normal_force - contact.normal_force) +
               p.radial_kd * error.dot(n);
    const Vec3 acceleration = contactAcceleration(n, c.acceleration, error, radial,
        c.support_vector, filtered_external, c.support_tracking, c.observer_blend,
        c.support_scale, p.robot_mass, velocity_gain_xy, velocity_gain_z,
        p.press_limit, p.approach_limit, p.common_z_limit + p.relative_z_limit + p.tilt_limit);
    out.navigation.target_acc_x = acceleration.x(); out.navigation.target_acc_y = acceleration.y();
    out.navigation.target_acc_z = acceleration.z();
    return out;
  }
  out.enable_force_control = true; out.force_integral_enabled = true;
  copy(-(protective ? 0.0 : c.normal_force) * n + c.support_vector, out.force);
  copy(c.velocity, out.velocity); copy(p.contact_offset, out.velocity_point);
  out.navigation.target_acc_x = c.acceleration.x(); out.navigation.target_acc_y = c.acceleration.y();
  out.navigation.target_acc_z = c.acceleration.z();
  Vec3 up = tangentProjection(n, Vec3::UnitZ());
  if (up.norm() < 1e-6) up = n.unitOrthogonal(); else up.normalize();
  Eigen::Matrix3d axes;
  axes.col(0) = n; axes.col(1) = up.cross(n); axes.col(2) = up;
  const Eigen::Quaterniond q(axes);
  out.acceleration_bounds = true;
  out.acceleration_orientation.x=q.x(); out.acceleration_orientation.y=q.y();
  out.acceleration_orientation.z=q.z(); out.acceleration_orientation.w=q.w();
  Vec3 upper(p.press_limit, p.approach_limit, p.common_z_limit + p.relative_z_limit + p.tilt_limit);
  Vec3 lower = -upper;
  if (c.wrench_tracking)
  {
    const Vec3 support_acceleration = axes.transpose() * c.support_vector / p.robot_mass;
    upper += support_acceleration;
    lower += support_acceleration;
  }
  if (protective)
  {
    lower.x() = upper.x() = radial;
  }
  copy(lower, out.acceleration_min); copy(upper, out.acceleration_max);
  return out;
}
}  // namespace bee::grasp
