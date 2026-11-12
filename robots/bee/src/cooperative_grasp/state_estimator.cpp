#include <bee/cooperative_grasp/state_estimator.h>
#include <set>

namespace bee::grasp
{
void SceneConfig::validate() const
{
  const auto n = robot_names.size();
  if (n < 3 || n > 5 || face_angles.size() != n || face_radii.size() != n ||
      std::set<std::string>(robot_names.begin(), robot_names.end()).size() != n)
    throw std::invalid_argument("scene requires 3..5 distinct vehicles and matching contact geometry");
  for (std::size_t i = 0; i < n; ++i)
    if (robot_names[i].empty() || !std::isfinite(face_angles[i]) || !std::isfinite(face_radii[i]) || face_radii[i] <= 0)
      throw std::invalid_argument("invalid contact geometry");
  if (!std::isfinite(object_mass) || object_mass <= 0 || !std::isfinite(object_height) || object_height <= 0 ||
      !std::isfinite(staging_clearance) || staging_clearance <= 0 || !object_position.allFinite() ||
      !destination_position.allFinite())
    throw std::invalid_argument("invalid scene dimensions, mass or position");
}

GraspStateEstimator::GraspStateEstimator(SceneConfig scene, ControlConfig config)
  : scene_(std::move(scene)), config_(std::move(config))
{
  scene_.validate();
  config_.validate();
  reset();
}
void GraspStateEstimator::reset()
{
  baseline_.assign(scene_.robot_names.size(), Wrench{});
  world_baseline_.assign(scene_.robot_names.size(), Vec3::Zero());
  normal_forces_.assign(scene_.robot_names.size(), 0);
  previous_stamp_ = 0;
  velocity_.setZero();
  omega_.setZero();
  beginCalibration();
}
void GraspStateEstimator::beginCalibration()
{
  samples_ = 0;
  sums_.assign(scene_.robot_names.size(), Wrench{});
  world_sums_.assign(scene_.robot_names.size(), Vec3::Zero());
}
void GraspStateEstimator::addCalibrationSample(const InputState& input)
{
  if (input.vehicles.size() != sums_.size())
    throw std::invalid_argument("vehicle count mismatch");
  for (std::size_t i = 0; i < sums_.size(); ++i)
  {
    sums_[i].force += input.vehicles[i].observer_cog.force;
    sums_[i].moment += input.vehicles[i].observer_cog.moment;
    world_sums_[i] += input.vehicles[i].cog.orientation * input.vehicles[i].observer_cog.force;
  }
  ++samples_;
}
bool GraspStateEstimator::finishCalibration()
{
  if (!samples_)
    return false;
  for (std::size_t i = 0; i < sums_.size(); ++i)
  {
    baseline_[i].force = sums_[i].force / samples_;
    baseline_[i].moment = sums_[i].moment / samples_;
    world_baseline_[i] = world_sums_[i] / samples_;
    normal_forces_[i] = 0;
  }
  return true;
}
GraspState GraspStateEstimator::update(const InputState& input)
{
  if (input.vehicles.size() != scene_.robot_names.size())
    throw std::invalid_argument("vehicle count mismatch");
  GraspState state;
  state.input = input;
  const double dt = input.object_stamp - previous_stamp_;
  if (previous_stamp_ > 0 && dt > 0)
  {
    const double alpha = config_.twist_filter_alpha;
    velocity_ += alpha * ((input.object.position - previous_object_.position) / dt - velocity_);
    Eigen::Quaterniond delta = input.object.orientation * previous_object_.orientation.conjugate();
    if (delta.w() < 0)
      delta.coeffs() *= -1;
    const Eigen::AngleAxisd rotation(delta.normalized());
    omega_ += alpha * (rotation.axis() * rotation.angle() / dt - omega_);
  }
  if (previous_stamp_ == 0 || dt > 0)
  {
    previous_stamp_ = input.object_stamp;
    previous_object_ = input.object;
  }
  state.object_velocity = velocity_;
  state.object_angular_velocity = omega_;
  for (std::size_t i = 0; i < input.vehicles.size(); ++i)
  {
    const auto& v = input.vehicles[i];
    ContactState c;
    const double angle = yaw(input.object.orientation) + scene_.face_angles[i];
    c.normal = Vec3(std::cos(angle), std::sin(angle), 0);
    c.tangent = Vec3(-std::sin(angle), std::cos(angle), 0);
    const Vec3 offset = v.base.orientation * config_.contact_offset;
    c.position = v.base.position + offset;
    c.velocity = v.velocity_world + (v.base.orientation * v.angular_velocity_base).cross(offset);
    c.bee_world.force = config_.observer_force_sign * config_.observer_force_scale *
                        (v.cog.orientation * (v.observer_cog.force - baseline_[i].force));
    c.bee_world.moment = config_.observer_force_sign * config_.observer_torque_scale *
                             (v.cog.orientation * (v.observer_cog.moment - baseline_[i].moment)) -
                         (c.position - v.cog.position).cross(c.bee_world.force);
    c.local_moment = Vec3(c.bee_world.moment.dot(c.normal), c.bee_world.moment.dot(c.tangent), c.bee_world.moment.z());
    const Vec3 arm = c.position - input.object.position;
    const Vec3 relative_velocity = c.velocity - velocity_;
    c.radius = arm.dot(c.normal);
    c.tangent_position = arm.dot(c.tangent);
    c.radial_velocity = relative_velocity.dot(c.normal) + omega_.z() * c.tangent_position;
    c.tangent_velocity = relative_velocity.dot(c.tangent) - omega_.z() * c.radius;
    c.relative_z = arm.z();
    c.relative_vz = relative_velocity.z();
    // Preserve legacy normal-force calibration in world axes; the six-axis
    // wrench baseline above is transported in CoG axes.
    const Vec3 force_delta = v.cog.orientation * v.observer_cog.force - world_baseline_[i];
    const double measured =
        std::max(0.0, config_.observer_force_sign * config_.observer_force_scale * force_delta.dot(c.normal));
    normal_forces_[i] += config_.force_filter_alpha * (measured - normal_forces_[i]);
    c.normal_force = normal_forces_[i];
    state.object_wrench.force -= c.bee_world.force;
    state.object_wrench.moment -= c.bee_world.moment + arm.cross(c.bee_world.force);
    state.contacts.push_back(c);
  }
  return state;
}
}  // namespace bee::grasp
