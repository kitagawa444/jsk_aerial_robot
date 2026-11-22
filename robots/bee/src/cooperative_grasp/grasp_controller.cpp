#include <Eigen/QR>
#include <bee/cooperative_grasp/grasp_controller.h>
#include <bee/cooperative_grasp/wrench_allocator.h>
#include <numeric>

namespace bee::grasp
{
// Outward normal. Never overwrite world Z after composing this vector: normal
// force and load compensation are orthogonal even for tilted contact surfaces.
Vec3 contactAcceleration(const Vec3& n, const Vec3& feedforward, const Vec3& error,
                                double radial, const Vec3& support, const Vec3& external,
                                bool loaded, double blend, double scale, double mass,
                                double gain_xy, double gain_z, double normal_limit,
                                double tangent_limit, double vertical_limit)
{
  Vec3 up = tangentProjection(n, Vec3::UnitZ());
  if (up.norm() < 1e-6) up = n.unitOrthogonal();
  else up.normalize();
  const Vec3 side = up.cross(n).normalized();
  Vec3 correction = Vec3::Zero();
  if (loaded)
    correction = scale * ((1 - blend) * tangentProjection(n, support) -
                           blend * tangentProjection(n, external)) / mass;
  const Vec3 tangential = tangentProjection(n, feedforward) + correction;
  return clamp(radial + feedforward.dot(n), normal_limit) * n +
         clamp(tangential.dot(side) + gain_xy * error.dot(side), tangent_limit) * side +
         clamp(tangential.dot(up) + gain_z * error.dot(up), vertical_limit) * up;
}

namespace
{
void removeMean(std::vector<double>& values)
{
  const double mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
  for (auto& value : values)
    value -= mean;
}
}  // namespace

GraspController::GraspController(SceneConfig scene, ControlConfig config)
  : scene_(std::move(scene)), config_(std::move(config))
{
  scene_.validate();
  config_.validate();
  reset();
}

void GraspController::reset()
{
  contact_geometry_.reset();
  tracking_initialized_ = false;
  wrench_initialized_ = false;
  allocated_forces_.resize(0);
  preload_plan_.resize(0);
  preload_targets_.assign(scene_.robot_names.size(), 0);
  preload_commands_.assign(scene_.robot_names.size(), 0);
  object_integral_.setZero();
  observer_blend_ = 0;
  tracking_velocity_.assign(scene_.robot_names.size(), Vec3::Zero());
  status_ = {};
  support_active_ = false;
  support_start_wrench_ = {};
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

ControlOutput GraspController::step(const GraspState& s, const Reference& r, double dt)
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
    if (config_.wrench_control)
    {
      support_start_wrench_ = {};
      const double upper=config_.max_normal_force-config_.allocation_normal_error;
      for (std::size_t i=0;i<s.contacts.size();++i)
      {
        const Vec3 force=-std::clamp(preload_commands_[i],0.0,upper)*s.contacts[i].normal;
        support_start_wrench_.force+=force;
        support_start_wrench_.moment+=(s.contacts[i].position-s.input.object.position).cross(force);
      }
      // This filter was inactive during preload. Starting from zero would
      // falsely report that a downward preload had already been unloaded.
      transfer_force_=filtered_force_=s.object_wrench.force.z();
    }
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
    if (!config_.wrench_control || !support_active_) updateAttitude(s, r, dt);
    if (config_.wrench_control && !support_active_) updatePreloadPlan(s,r,dt);
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
  status_.normal_force_targets.clear();
  for (std::size_t i = 0; i < s.contacts.size(); ++i)
  {
    auto command =
        (r.phase >= Phase::Preload && r.phase <= Phase::Release) ? contactCommand(i, s, r) : approachCommand(i, s, r);
    if (config_.wrench_control && r.phase == Phase::Approach) preload_commands_[i] = command.normal_force;
    output.commands.push_back(command);
    status_.normal_force_targets.push_back(command.normal_force);
  }
  output.status = status_;
  if (config_.distributed_tracking)
  {
    if (config_.wrench_control && support_active_ && r.phase <= Phase::Release)
      makeWrenchReferences(s, r, dt, output);
    else
      makeTrackingReferences(s, r, dt, output);
  }
  return output;
}

void GraspController::updateNormalForce(const Reference& r, double dt)
{
  status_.normal_force = approach(status_.normal_force, std::clamp(r.normal_force, 0.0, config_.max_normal_force),
                                  r.normal_force_rate * dt);
}

void GraspController::updateAttitude(const GraspState& s, const Reference& r, double dt)
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
  // Wrench mode keeps local foot alignment above. Object moment allocation is
  // owned by the preload planner / wrench QP, including before load transfer.
  if (p.wrench_control || !loaded || dt <= 0)
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

void GraspController::updatePreloadPlan(const GraspState& s, const Reference& r, double dt)
{
  const int count=s.contacts.size();
  ControlConfig planning=config_;
  planning.allocation_mu=std::min(config_.allocation_mu,r.preload_mu);
  planning.allocation_normal_error=std::max(config_.allocation_normal_error,
                                           config_.allocation_observer_normal_error);
  const double load=r.preload_vertical_force>0 ? r.preload_vertical_force :
      scene_.object_mass*(config_.gravity+config_.reference_acceleration_limit);
  AllocationOptions options; options.planning=true;
  options.planning_force_ratio=r.preload_force_ratio;
  options.normal_minimum.resize(count);
  try
  {
    for (int i = 0; i < count; ++i)
      options.normal_minimum[i] = requiredPreload(s.contacts[i].normal, r.preload_margin * load / count,
          planning.allocation_mu, planning.allocation_normal_error,
          planning.allocation_tangent_up_error + planning.allocation_tangent_side_error);
  }
  catch (const std::invalid_argument& e)
  {
    status_.preload_plan_ready = false;
    status_.preload_warning = e.what();
    return;
  }
  if(preload_plan_.size()!=3*count)
  {
    preload_plan_.resize(3*count);
    for(int i=0;i<count;++i)
      preload_plan_.segment<3>(3*i)=-options.normal_minimum[i]*s.contacts[i].normal;
  }
  Wrench target; target.force=Vec3(0,0,load);
  const std::vector<Vec3> own(count,Vec3(0,0,config_.robot_mass*config_.gravity));
  const auto plan=allocateWrench(s.contacts,s.input.object.position,target,own,preload_plan_,0,dt,planning,options);
  status_.preload_plan_ready=plan.converged &&
      (plan.achieved.force-target.force).norm()<.01 && plan.achieved.moment.norm()<.001;
  if(!status_.preload_plan_ready)
  {
    status_.preload_warning="preload holding plan unavailable: status="+std::to_string(plan.solver_status)+
        " force_error="+std::to_string((plan.achieved.force-target.force).norm())+
        " moment_error="+std::to_string(plan.achieved.moment.norm());
    return;
  }
  status_.preload_warning.clear();
  preload_plan_=plan.force;
  status_.preload_forces.resize(count);
  status_.preload_normal_targets.resize(count);
  for(int i=0;i<count;++i)
  {
    status_.preload_forces[i]=plan.force.segment<3>(3*i);
    preload_targets_[i]=-s.contacts[i].normal.dot(status_.preload_forces[i]);
    status_.preload_normal_targets[i]=preload_targets_[i];
    preload_commands_[i]=approach(preload_commands_[i],preload_targets_[i],r.normal_force_rate*dt);
  }
}

void GraspController::updateVertical(const GraspState& s, const Reference& r, double dt)
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
  // A sloped normal can make preload's resultant point downward. Progress is
  // measured from that commanded wrench, not from an assumed zero force.
  // Retain the old gate for the legacy controller and nonpositive load spans.
  const double start = p.wrench_control ? support_start_wrench_.force.z() : 0;
  const double span = weight-start;
  const double allowed = p.wrench_control && span>1e-6 ?
      std::clamp((transfer_force_-start+p.support_force_lead)/span,0.0,1.0) :
      std::clamp((std::max(0.0,transfer_force_)+p.support_force_lead)/weight,0.0,1.0);
  status_.support_ratio = std::min(std::min(1.0, status_.support_ratio + dt / p.support_ramp_duration),
                                   std::max(status_.support_ratio, allowed));
  if (!status_.support_ready)
  {
    const bool lifting = p.wrench_control && s.input.object.position.z() > initial_z_ && s.object_velocity.z() > 0;
    const bool ready = status_.support_ratio >= 1 - 1e-6 && transfer_force_ >= p.support_ratio * weight &&
                       (std::abs(s.object_velocity.z()) <= p.support_velocity_tolerance || lifting);
    support_stable_ = ready ? support_stable_ + dt : 0;
    status_.support_ready = support_stable_ >= p.support_settle;
  }
  if (p.distributed_tracking)
  {
    // Actual motion references only. Load compensation is owned by each Bee.
    if (status_.support_ready && r.phase >= Phase::Lift)
    {
      if (p.wrench_control)
      {
        // Continuous velocity and a derivative-consistent acceleration even
        // when the phase manager supplies a moving descent position target.
        const double desired_v = clamp(2*(r.object_z-status_.reference_z), r.z_velocity_limit);
        const double old_v = status_.reference_vz;
        status_.reference_vz = approach(old_v, desired_v, p.trajectory_acceleration*dt);
        status_.reference_az = (status_.reference_vz-old_v)/dt;
        status_.reference_z += .5*(old_v+status_.reference_vz)*dt;
      }
      else advanceZReference(r.object_z, r.z_velocity_limit, dt);
    }
    else
    {
      // Hold the height captured on entering SupportTransfer. Following the
      // measured height here hides drift caused by imperfect load compensation.
      status_.reference_z = initial_z_;
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

void GraspController::advanceZReference(double target, double velocity_limit, double dt)
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

VehicleCommand GraspController::approachCommand(std::size_t i, const GraspState& s, const Reference& r) const
{
  const auto& c = s.contacts[i];
  const auto& v = s.input.vehicles[i];
  const Vec3 ray = s.input.object.orientation * Vec3(std::cos(scene_.face_angles[i]), std::sin(scene_.face_angles[i]), 0);
  const Vec3 normal = s.input.object.orientation * plannedContactNormal(scene_, i);
  VehicleCommand cmd;
  cmd.baselink_rpy = contactAttitude(normal, r.attitude_fraction,
                                     yaw(s.input.object.orientation) + scene_.face_angles[i] + kPi / 2);
  if (r.phase >= Phase::Orient && r.phase <= Phase::Approach)
  {
    Vec3 target = footCenterTarget(scene_, i, s.input.object, config_.ball_radius, r.radius_clearance);
    if (r.phase == Phase::Realign)
    {
      const Vec3 delta = target - c.position;
      if (delta.norm() > .04) target = c.position + .04 * delta.normalized();
    }
    const Vec3 velocity = s.object_velocity + s.object_angular_velocity.cross(target - s.input.object.position);
    const Vec3 damping = velocity - c.velocity;
    const Vec3 error = target - c.position;
    const Vec3 tangent = config_.tangent_kp * tangentProjection(normal, error) +
                         config_.tangent_kd * tangentProjection(normal, damping);
    double radial = config_.radial_kp * normal.dot(error) + config_.radial_kd * normal.dot(damping);
    const bool touched = i < r.touching.size() && r.touching[i];
    const double gap = r.radius_clearance - error.dot(normal);
    const bool probing = r.phase == Phase::Approach && r.radius_clearance < 0 &&
                         gap >= -config_.max_compression && config_.max_compression > 0;
    if (config_.wrench_control && (probing || touched))
    {
      cmd.normal = normal;
      cmd.normal_force = touched ? r.touch_force :
          std::min(1.0, -r.radius_clearance / config_.max_compression) * r.touch_force;
      if (touched) radial = config_.radial_kd * normal.dot(damping);
    }
    else if (probing)
      radial -= std::min(1.0, -r.radius_clearance / config_.max_compression) *
                r.touch_force * (1.0 / config_.robot_mass + config_.normal_force_kp);
    if (touched && !config_.wrench_control)
      radial = -r.touch_force / config_.robot_mass - config_.normal_force_kp * (r.touch_force - c.normal_force) +
               config_.radial_kd * normal.dot(damping);
    cmd.acceleration = tangent + radial * normal;
    if (cmd.acceleration.norm() > config_.approach_limit)
      cmd.acceleration *= config_.approach_limit / cmd.acceleration.norm();
    cmd.position_target = v.cog.position + error;
    cmd.z_mode = touched || probing ? ZMode::Acceleration : ZMode::Position;
    cmd.z = touched || probing ? cmd.acceleration.z() : target.z() - (v.base.orientation * config_.contact_offset).z();
    return cmd;
  }
  const Vec3 tangent = s.input.object.orientation * Vec3(-std::sin(scene_.face_angles[i]), std::cos(scene_.face_angles[i]), 0);
  const double radius = (v.base.position - s.input.object.position).dot(ray);
  const double radial = clamp(config_.radial_kp * (scene_.face_radii[i] + r.radius_clearance - radius) -
                              config_.radial_kd * v.velocity_world.dot(ray), config_.approach_limit);
  const double lateral = clamp(-config_.tangent_kp * (c.position - s.input.object.position).dot(tangent) -
                               config_.tangent_kd * (c.velocity - s.object_velocity).dot(tangent), config_.approach_limit);
  cmd.acceleration = radial * ray + lateral * tangent;
  cmd.z_mode = ZMode::Position;
  cmd.z = r.object_z + scene_.contact_height;
  cmd.position_target = v.cog.position + (scene_.face_radii[i] + r.radius_clearance - radius) * ray -
                        (c.position - s.input.object.position).dot(tangent) * tangent;
  return cmd;
}

VehicleCommand GraspController::contactCommand(std::size_t i, const GraspState& s, const Reference& r) const
{
  const auto& c = s.contacts[i];
  const double upper = config_.max_normal_force-
      (config_.wrench_control ? config_.allocation_normal_error : 0);
  const double target = std::clamp(config_.wrench_control ? preload_commands_[i] :
                                   status_.normal_force + force_offsets_[i], 0.0, upper);
  if (config_.distributed_tracking)
  {
    VehicleCommand cmd;
    cmd.contact_tracking = true;
    cmd.support_tracking = support_active_;
    cmd.normal = c.normal;
    cmd.normal_force = target;
    cmd.object_position = s.input.object.position;
    const Vec3 n_object = s.input.object.orientation.conjugate() * c.normal;
    cmd.minimum_radius = surfaceSupport(scene_, n_object, scene_.face_radii[i]) + config_.foot_contact_offset + config_.ball_radius -
                         config_.max_compression - config_.radius_margin;
    cmd.baselink_rpy = contactAttitude(c.normal, r.attitude_fraction, yaw(s.input.object.orientation) + scene_.face_angles[i] + kPi / 2);
    cmd.baselink_rpy.x() += roll_trim_[i];
    cmd.baselink_rpy.z() += yaw_trim_[i] + object_yaw_trim_;
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
      surfaceSupport(scene_, s.input.object.orientation.conjugate() * c.normal, scene_.face_radii[i]) +
      config_.foot_contact_offset + config_.ball_radius - config_.max_compression;
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
  cmd.baselink_rpy = contactAttitude(c.normal, r.attitude_fraction, yaw(s.input.object.orientation) + scene_.face_angles[i] + kPi / 2);
  cmd.baselink_rpy.x() += roll_trim_[i];
  cmd.baselink_rpy.z() += yaw_trim_[i] + object_yaw_trim_;
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

void GraspController::makeWrenchReferences(const GraspState& s, const Reference& r,
                                                       double dt, ControlOutput& out)
{
  if (dt <= 0) return;
  const auto& p = config_;
  const int count = s.contacts.size();
  if (!status_.fault.empty()) { out.status=status_; return; }
  if (!wrench_initialized_)
  {
    object_reference_ = s.input.object.position;
    object_rotation_reference_ = s.input.object.orientation.toRotationMatrix();
    object_integral_.setZero(); previous_motion_.setZero();
    allocated_forces_.resize(3*count);
    last_omega_.resize(count); filtered_alpha_.assign(count, Vec3::Zero());
    for (int i=0;i<count;++i)
    {
      allocated_forces_.segment<3>(3*i) = -out.commands[i].normal_force*s.contacts[i].normal;
      last_omega_[i] = s.input.vehicles[i].base.orientation*s.input.vehicles[i].angular_velocity_base;
    }
    wrench_initialized_ = true;
  }
  // One object trajectory; there are no independent vehicle velocity targets.
  Vec3 motion = r.object_velocity;
  motion.z() = status_.reference_vz;
  const Vec3 old_reference=object_reference_;
  object_reference_ += dt*motion;
  object_reference_.z() = status_.reference_z;
  const Vec3 error = object_reference_-s.input.object.position;
  const Vec3 old_integral = object_integral_;
  if (r.phase < Phase::Unload)
    for (int j=0;j<3;++j) object_integral_[j] = clamp(object_integral_[j]+p.object_ki*error[j]*dt,p.object_i_limit);
  else object_integral_.setZero();
  Vec3 acc = p.object_kp*error+p.object_kd*(motion-s.object_velocity)+object_integral_;
  acc.z() += status_.reference_az;
  if (r.phase >= Phase::Unload) acc.setZero();
  const Eigen::Matrix3d R = s.input.object.orientation.toRotationMatrix();
  const Eigen::AngleAxisd rotation_error(object_rotation_reference_*R.transpose());
  Vec3 alpha = p.object_rotation_kp*rotation_error.angle()*rotation_error.axis()-
               p.object_rotation_kd*s.object_angular_velocity;
  if (alpha.norm() > .3) alpha *= .3/alpha.norm();
  if (r.phase >= Phase::Unload) alpha.setZero();
  const Eigen::Matrix3d inertia = R*scene_.object_inertia*R.transpose();
  const Vec3 gravity(0,0,-p.gravity), omega = s.object_angular_velocity;
  // While grounded the pedestal supplies the remaining weight. Never infer
  // free-fall acceleration from the partially transferred contact wrench.
  const double load = status_.support_ratio*r.support_scale;
  std::vector<Vec3> own_acc(count), inertial_force(count), arms(count), offsets(count);
  for (int i=0;i<count;++i)
  {
    const auto& vehicle = s.input.vehicles[i];
    const Vec3 arm = s.contacts[i].position-s.input.object.position;
    const Vec3 offset = s.contacts[i].position-vehicle.cog.position;
    const Vec3 bee_omega = vehicle.base.orientation*vehicle.angular_velocity_base;
    filtered_alpha_[i] += dt/(.15+dt)*((bee_omega-last_omega_[i])/dt-filtered_alpha_[i]);
    last_omega_[i] = bee_omega;
    arms[i]=arm;
    offsets[i]=omega.cross(omega.cross(arm))-filtered_alpha_[i].cross(offset)-
               bee_omega.cross(bee_omega.cross(offset));
  }
  const Vec3 requested_acc=acc, requested_alpha=alpha;
  if(!limitObjectMotion(arms,offsets,p,acc,alpha))
  {
    status_.fault="no common object motion within Bee acceleration limits";
    out.status=status_; return;
  }
  const bool motion_limited=(acc-requested_acc).norm()+(alpha-requested_alpha).norm()>1e-6;
  if(motion_limited) { object_integral_=old_integral; object_reference_=old_reference; }
  Wrench target;
  target.force=scene_.object_mass*(acc-load*gravity);
  // Reduce upward contact demand if the supported assembly rises ahead of its trajectory.
  const double vertical_tracking = p.object_kp * error.z() + p.object_kd * (motion.z() - s.object_velocity.z());
  if (r.phase < Phase::Unload)
    target.force.z() += load * count * p.robot_mass * std::min(0.0, vertical_tracking);
  target.moment=inertia*alpha+omega.cross(inertia*omega);
  // Begin at the actual preload command (including its moment), then ramp
  // towards gravity support using the same observer-limited progress as the
  // vertical controller. A zero-force start is discontinuous on sloped faces.
  if (r.phase < Phase::Unload)
  {
    target.force+=(1-status_.support_ratio)*support_start_wrench_.force;
    target.moment+=(1-status_.support_ratio)*support_start_wrench_.moment;
  }
  if(r.phase>=Phase::Unload) target.moment*=r.support_scale;
  for(int i=0;i<count;++i)
  {
    own_acc[i]=acc+alpha.cross(arms[i])+offsets[i];
    inertial_force[i] = p.robot_mass*(own_acc[i]-gravity);
  }
  AllocationOptions allocation_options;
  allocation_options.constrain_acceleration=true;
  allocation_options.releasing=r.phase==Phase::Release;
  if(preload_plan_.size()==3*count)
  {
    allocation_options.normal_minimum.resize(count);
    for(int i=0;i<count;++i)
    {
      if(r.phase>=Phase::Release)
        preload_targets_[i]=approach(preload_targets_[i],0,r.normal_force_rate*dt);
      allocation_options.normal_minimum[i]=preload_targets_[i];
    }
  }
  // After touchdown the pedestal takes the load back. Fixed nonzero force
  // uncertainty is not a contact-maintenance requirement after release.
  if (r.phase >= Phase::Unload)
    allocation_options.contact_margin_scale = std::clamp(r.support_scale,0.0,1.0);
  // Phase manager provides the already margin-inclusive command. The allocator
  // takes the guaranteed lower force and adds uncertainty exactly once.
  const double preload = std::max(0.0, std::min(status_.normal_force,
      p.max_normal_force-p.allocation_normal_error)-
      allocation_options.contact_margin_scale*p.allocation_normal_error);
  if (airborne(r.phase) && status_.support_ready)
  {
    allocation_options.moving_object_mass = scene_.object_mass;
    allocation_options.object_inverse_inertia = inertia.inverse();
  }
  if (r.phase == Phase::Release)
  {
    double previous_normal = 0;
    for (int i=0;i<count;++i)
      previous_normal = std::max(previous_normal, -s.contacts[i].normal.dot(allocated_forces_.segment<3>(3*i)));
    // Internal squeeze is invisible in the object wrench. Remove it through
    // an explicit shrinking constraint, not a small regularization gradient.
    const double floor=allocation_options.normal_minimum.size()==count ?
        allocation_options.normal_minimum.maxCoeff() : preload;
    allocation_options.normal_command_limit = std::max(floor, previous_normal-r.normal_force_rate*dt);
  }
  const auto allocation = allocateWrench(s.contacts,s.input.object.position,target,inertial_force,
                                         allocated_forces_,preload,dt,p,allocation_options);
  status_.allocation_violation = allocation.violation;
  status_.allocation_iterations = allocation.iterations;
  status_.allocation_restarts = allocation.restarts;
  status_.allocation_seconds = allocation.seconds;
  status_.allocation_primal = allocation.primal;
  status_.allocation_stationarity = allocation.stationarity;
  status_.allocation_complementarity = allocation.complementarity;
  if (!allocation.converged || !allocation.force.allFinite() || allocation.violation > 1e-4)
  {
    status_.fault = "contact wrench QP failed: violation=" + std::to_string(allocation.violation) +
        " primal=" + std::to_string(allocation.primal) + " dual=" + std::to_string(allocation.stationarity) +
        " status=" + std::to_string(allocation.solver_status) + " iterations=" + std::to_string(allocation.iterations) +
        " seconds=" + std::to_string(allocation.seconds);
    out.status = status_; return;
  }
  allocated_forces_ = allocation.force;
  status_.wrench_force_error = (allocation.achieved.force-target.force).norm();
  status_.wrench_moment_error = (allocation.achieved.moment-target.moment).norm();
  // Freeze integration on acceleration saturation or an unrealizable wrench.
  if (status_.wrench_force_error > .03 || acc.cwiseAbs().maxCoeff() >= p.reference_acceleration_limit-1e-6)
    object_integral_ = old_integral;
  Vec3 delta_acc = Vec3::Zero(), delta_alpha = Vec3::Zero();
  // On the pedestal an unbalanced preload is reacted by the ground. It is
  // NOT a free-body acceleration to send to the aircraft during mode handover.
  if (airborne(r.phase) && status_.support_ready)
  {
    delta_acc = (allocation.achieved.force-target.force)/scene_.object_mass;
    delta_alpha = inertia.ldlt().solve(allocation.achieved.moment-target.moment);
  }
  status_.vertical_force_target = target.force.z();
  status_.normal_force_targets.clear();
  status_.contact_geometry_errors = contact_geometry_.errors(s);
  for (int i=0;i<count;++i)
  {
    auto& cmd = out.commands[i]; const auto& c = s.contacts[i];
    const Vec3 force = allocated_forces_.segment<3>(3*i);
    const Vec3 arm = c.position-s.input.object.position;
    cmd.wrench_tracking = true; cmd.calibrate = r.calibrate;
    const double normal_upper = std::min(p.max_normal_force-p.allocation_normal_error,
                                         allocation_options.normal_command_limit);
    cmd.normal_force = std::clamp(-force.dot(c.normal), 0.0, normal_upper);
    cmd.support_vector = tangentProjection(c.normal,force);
    cmd.support_scale = 1; cmd.observer_blend = 0;
    // Velocity is diagnostic / relative-contact damping only, not a motion servo.
    cmd.velocity = s.object_velocity+omega.cross(arm);
    cmd.acceleration = own_acc[i]+delta_acc+delta_alpha.cross(arm);
    if(!referenceMotionValid(cmd.velocity,cmd.acceleration,p))
    {
      status_.fault="allocated motion violates Bee reference limits";
      out.status=status_; return;
    }
    const Vec3 basic_thrust = p.robot_mass*(cmd.acceleration-gravity)-cmd.normal_force*c.normal+
                              cmd.support_vector;
    if (basic_thrust.lpNorm<1>() > p.allocation_thrust_limit-p.allocation_thrust_reserve+1e-3)
    {
      status_.fault = "allocated motion exceeds reserved thrust capacity";
      out.status = status_; return;
    }
    cmd.z_mode = ZMode::Acceleration; cmd.z = cmd.acceleration.z();
    status_.normal_force_targets.push_back(cmd.normal_force);
  }
  out.status = status_;
}

void GraspController::makeTrackingReferences(const GraspState& s, const Reference& r,
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
  if (support_active_ && r.phase >= Phase::SupportTransfer && r.phase < Phase::Unload)
  {
    const double error = status_.reference_z - s.input.object.position.z();
    status_.reference_velocity_integral = clamp(status_.reference_velocity_integral + p.reference_z_ki * error * dt,
                                               p.reference_z_i_limit);
    common_vz = status_.reference_vz + p.reference_z_kp * error + status_.reference_velocity_integral;
  }
  if (status_.support_ready && r.phase < Phase::Unload)
    observer_blend_ = approach(observer_blend_, 1, dt / p.support_ramp_duration);
  double normal_vertical = 0, tangent_vertical = 0;
  for (std::size_t i = 0; i < s.contacts.size(); ++i)
  {
    normal_vertical -= output.commands[i].normal_force * s.contacts[i].normal.z();
    tangent_vertical += 1 - s.contacts[i].normal.z() * s.contacts[i].normal.z();
  }
  // Account for the vertical component of the inward preload before allocating
  // the remaining load to tangent planes. This reduces to equal Z sharing on
  // vertical faces. Near-pole configurations need a different allocation.
  if (support_active_ && tangent_vertical < .1)
  {
    status_.fault = "insufficient tangent support authority";
    output.status = status_;
    return;
  }
  const double tangent_load = support_active_ ?
      (status_.vertical_force_target - normal_vertical) / tangent_vertical : 0;
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
    cmd.support_vector = tangent_load * tangentProjection(c.normal, Vec3::UnitZ());
    cmd.observer_blend = observer_blend_;
    cmd.support_scale = r.phase == Phase::Release ? 0 : r.support_scale;
    cmd.z = support_active_ ? cmd.acceleration.z() : cmd.velocity.z();
  }
  output.status = status_;
}

void ControlConfig::validate() const
{
  requireNonnegativeParameter(object_kp, "object_kp");
  requireNonnegativeParameter(object_kd, "object_kd");
  requireNonnegativeParameter(object_ki, "object_ki");
  requireNonnegativeParameter(object_i_limit, "object_i_limit");
  requireNonnegativeParameter(object_rotation_kp, "object_rotation_kp");
  requireNonnegativeParameter(object_rotation_kd, "object_rotation_kd");
  requireNonnegativeParameter(allocation_mu, "allocation_mu");
  requireNonnegativeParameter(allocation_force_rate, "allocation_force_rate");
  requireNonnegativeParameter(allocation_thrust_limit, "allocation_thrust_limit");
  requireNonnegativeParameter(allocation_normal_error, "allocation_normal_error");
  requireNonnegativeParameter(allocation_tangent_up_error, "allocation_tangent_up_error");
  requireNonnegativeParameter(allocation_tangent_side_error, "allocation_tangent_side_error");
  requireNonnegativeParameter(allocation_observer_normal_error, "allocation_observer_normal_error");
  requireNonnegativeParameter(allocation_thrust_reserve, "allocation_thrust_reserve");
  requireNonnegativeParameter(contact_force_kp, "contact_force_kp");
  requireNonnegativeParameter(contact_force_ki, "contact_force_ki");
  requireNonnegativeParameter(contact_force_i_limit, "contact_force_i_limit");
  requireNonnegativeParameter(contact_damping, "contact_damping");
  requireNonnegativeParameter(contact_force_filter_tau, "contact_force_filter_tau");
  if (wrench_control && (!distributed_tracking || allocation_mu <= 0 || robot_mass <= 0))
    throw std::invalid_argument("wrench control requires distributed tracking, positive friction and mass");
  if (wrench_control &&
      (allocation_normal_error >= max_normal_force ||
       allocation_mu*(max_normal_force-2*allocation_normal_error) <=
           allocation_tangent_up_error+allocation_tangent_side_error ||
       allocation_thrust_limit-allocation_thrust_reserve <= robot_mass*gravity))
    throw std::invalid_argument("force uncertainty/reserve leaves no contact or hover capacity");
  requirePositiveParameter(reference_z_kp, "reference_z_kp");
  requireNonnegativeParameter(reference_z_ki, "reference_z_ki");
  requirePositiveParameter(reference_z_i_limit, "reference_z_i_limit");
  requirePositiveParameter(reference_velocity_limit, "reference_velocity_limit");
  requirePositiveParameter(reference_acceleration_limit, "reference_acceleration_limit");
  requireNonnegativeParameter(cooperative_admittance, "cooperative_admittance");
  requireNonnegativeParameter(robot_mass, "robot_mass");
  requireNonnegativeParameter(gravity, "gravity");
  requireNonnegativeParameter(foot_contact_offset, "foot_contact_offset");
  requireNonnegativeParameter(ball_radius, "ball_radius");
  requireNonnegativeParameter(max_compression, "max_compression");
  if (observer_force_sign != 1 && observer_force_sign != -1)
    throw std::invalid_argument("observer_force_sign must be +/-1");
  requireNonnegativeParameter(observer_force_scale, "observer_force_scale");
  requireNonnegativeParameter(observer_torque_scale, "observer_torque_scale");
  requireNonnegativeParameter(force_filter_alpha, "force_filter_alpha");
  requireNonnegativeParameter(twist_filter_alpha, "twist_filter_alpha");
  requireNonnegativeParameter(radial_kp, "radial_kp");
  requireNonnegativeParameter(radial_kd, "radial_kd");
  requireNonnegativeParameter(tangent_kp, "tangent_kp");
  requireNonnegativeParameter(tangent_kd, "tangent_kd");
  requireNonnegativeParameter(approach_limit, "approach_limit");
  requireNonnegativeParameter(press_limit, "press_limit");
  requireNonnegativeParameter(normal_force_kp, "normal_force_kp");
  requireNonnegativeParameter(press_radial_kp, "press_radial_kp");
  requireNonnegativeParameter(radius_margin, "radius_margin");
  requireNonnegativeParameter(normal_force_rate, "normal_force_rate");
  requireNonnegativeParameter(max_normal_force, "max_normal_force");
  requireNonnegativeParameter(z_velocity_kp, "z_velocity_kp");
  requireNonnegativeParameter(z_velocity_kd, "z_velocity_kd");
  requireNonnegativeParameter(z_velocity_limit, "z_velocity_limit");
  requireNonnegativeParameter(trajectory_velocity, "trajectory_velocity");
  requireNonnegativeParameter(trajectory_acceleration, "trajectory_acceleration");
  requireNonnegativeParameter(trajectory_jerk, "trajectory_jerk");
  requireNonnegativeParameter(z_kp, "z_kp");
  requireNonnegativeParameter(z_kd, "z_kd");
  requireNonnegativeParameter(z_ki, "z_ki");
  requireNonnegativeParameter(z_pd_limit, "z_pd_limit");
  requireNonnegativeParameter(z_i_limit, "z_i_limit");
  requireNonnegativeParameter(support_ramp_duration, "support_ramp_duration");
  requireNonnegativeParameter(support_filter_tau, "support_filter_tau");
  requireNonnegativeParameter(support_force_lead, "support_force_lead");
  requireNonnegativeParameter(support_ratio, "support_ratio");
  requireNonnegativeParameter(support_settle, "support_settle");
  requireNonnegativeParameter(support_velocity_tolerance, "support_velocity_tolerance");
  requireNonnegativeParameter(observer_filter_tau, "observer_filter_tau");
  requireNonnegativeParameter(observer_kp, "observer_kp");
  requireNonnegativeParameter(observer_ki, "observer_ki");
  requireNonnegativeParameter(observer_i_limit, "observer_i_limit");
  requireNonnegativeParameter(observer_correction_limit, "observer_correction_limit");
  requireNonnegativeParameter(observer_deadband, "observer_deadband");
  requireNonnegativeParameter(slip_kp, "slip_kp");
  requireNonnegativeParameter(slip_kd, "slip_kd");
  requireNonnegativeParameter(slip_limit, "slip_limit");
  requireNonnegativeParameter(slip_rate, "slip_rate");
  requireNonnegativeParameter(relative_z_kp, "relative_z_kp");
  requireNonnegativeParameter(relative_z_kd, "relative_z_kd");
  requireNonnegativeParameter(relative_z_limit, "relative_z_limit");
  requireNonnegativeParameter(relative_z_rate, "relative_z_rate");
  requireNonnegativeParameter(common_z_limit, "common_z_limit");
  requireNonnegativeParameter(support_floor_scale, "support_floor_scale");
  requireNonnegativeParameter(translation_kp, "translation_kp");
  requireNonnegativeParameter(translation_limit, "translation_limit");
  requireNonnegativeParameter(translation_rate, "translation_rate");
  requireNonnegativeParameter(yaw_kp, "yaw_kp");
  requireNonnegativeParameter(yaw_kd, "yaw_kd");
  requireNonnegativeParameter(yaw_moment_gain, "yaw_moment_gain");
  requireNonnegativeParameter(yaw_limit, "yaw_limit");
  requireNonnegativeParameter(yaw_rate, "yaw_rate");
  requireNonnegativeParameter(tilt_kp, "tilt_kp");
  requireNonnegativeParameter(tilt_kd, "tilt_kd");
  requireNonnegativeParameter(tilt_moment_gain, "tilt_moment_gain");
  requireNonnegativeParameter(tilt_limit, "tilt_limit");
  requireNonnegativeParameter(tilt_rate, "tilt_rate");
  requireNonnegativeParameter(local_moment_alpha, "local_moment_alpha");
  requireNonnegativeParameter(object_moment_alpha, "object_moment_alpha");
  requireNonnegativeParameter(moment_force_threshold, "moment_force_threshold");
  requireNonnegativeParameter(local_roll_kp, "local_roll_kp");
  requireNonnegativeParameter(local_roll_ki, "local_roll_ki");
  requireNonnegativeParameter(local_yaw_kp, "local_yaw_kp");
  requireNonnegativeParameter(local_yaw_ki, "local_yaw_ki");
  requireNonnegativeParameter(trim_limit, "trim_limit");
  requireNonnegativeParameter(trim_rate, "trim_rate");
  requireNonnegativeParameter(local_deadband, "local_deadband");
  requireNonnegativeParameter(radial_moment_kp, "radial_moment_kp");
  requireNonnegativeParameter(radial_moment_ki, "radial_moment_ki");
  requireNonnegativeParameter(radial_moment_limit, "radial_moment_limit");
  requireNonnegativeParameter(radial_moment_rate, "radial_moment_rate");
  requireNonnegativeParameter(object_yaw_kp, "object_yaw_kp");
  requireNonnegativeParameter(object_yaw_ki, "object_yaw_ki");
  requirePositiveParameter(robot_mass, "robot_mass");
  requirePositiveParameter(gravity, "gravity");
  requirePositiveParameter(trajectory_acceleration, "trajectory_acceleration");
  requirePositiveParameter(trajectory_velocity, "trajectory_velocity");
  requirePositiveParameter(support_ramp_duration, "support_ramp_duration");
  requirePositiveParameter(support_filter_tau, "support_filter_tau");
  requirePositiveParameter(observer_filter_tau, "observer_filter_tau");
  requirePositiveParameter(max_normal_force, "max_normal_force");
  requirePositiveParameter(approach_limit, "approach_limit");
  requirePositiveParameter(press_limit, "press_limit");
  if (!contact_offset.allFinite())
    throw std::invalid_argument("invalid contact_offset");
  if (force_filter_alpha > 1 || twist_filter_alpha > 1 || local_moment_alpha > 1 || object_moment_alpha > 1)
    throw std::invalid_argument("filter alpha must be in [0, 1]");
}
}  // namespace bee::grasp
