#include <bee/cooperative_grasp/grasp_controller.h>
#include <bee/cooperative_grasp/wrench_allocator.h>
#include <bee/cooperative_grasp/phase_manager.h>

namespace bee::grasp
{
const char* phaseName(Phase phase)
{
  switch (phase)
  {
#define PHASE_CASE(value)                                                                                              \
  case Phase::value:                                                                                                   \
    return #value;
    PHASE_CASE(Idle)
    PHASE_CASE(Staging) PHASE_CASE(Orient) PHASE_CASE(Calibrate) PHASE_CASE(Realign) PHASE_CASE(Approach) PHASE_CASE(Preload)
        PHASE_CASE(SupportTransfer) PHASE_CASE(Lift) PHASE_CASE(Hold) PHASE_CASE(Transport)
            PHASE_CASE(DestinationSettle) PHASE_CASE(Descend) PHASE_CASE(Unload) PHASE_CASE(Release) PHASE_CASE(Retreat)
                PHASE_CASE(Unwind) PHASE_CASE(Verify) PHASE_CASE(Complete) PHASE_CASE(Aborted)
#undef PHASE_CASE
  }
  return "Unknown";
}
GraspPhaseManager::GraspPhaseManager(SceneConfig scene, ControlConfig control, TaskConfig task)
  : scene_(std::move(scene)), control_(std::move(control)), task_(std::move(task))
{
  scene_.validate();
  control_.validate();
  task_.validate();
  // Equal vertical load sharing with a conservative design friction coefficient.
  normal_force_target_ = task_.normal_force_margin * scene_.object_mass *
                         (control_.gravity + task_.design_vertical_acceleration) /
                         (scene_.robot_names.size() * task_.friction_coefficient);
  if (control_.wrench_control)
  {
    preload_mu_ = std::min(task_.friction_coefficient, control_.allocation_mu);
    // Cover the configured object acceleration envelope before transferring
    // weight, even when the legacy task's design acceleration is zero.
    preload_design_load_ = scene_.object_mass * (control_.gravity+
        std::max(task_.design_vertical_acceleration,control_.reference_acceleration_limit)) /
        scene_.robot_names.size();
    normal_force_target_ = 0;
    for (std::size_t i=0; i<scene_.robot_names.size(); ++i)
    {
      const Vec3 normal = plannedContactNormal(scene_, i);
      normal_force_target_ = std::max(normal_force_target_,requiredPreload(normal,
          task_.normal_force_margin*preload_design_load_,preload_mu_,
          std::max(control_.allocation_normal_error,control_.allocation_observer_normal_error),
          control_.allocation_tangent_up_error+control_.allocation_tangent_side_error));
    }
  }
  const double normal_budget = control_.max_normal_force-
      (control_.wrench_control ? control_.allocation_normal_error : 0);
  if (!std::isfinite(normal_force_target_) || normal_force_target_ > normal_budget)
    throw std::invalid_argument("calculated normal force exceeds controller limit");
}
void GraspPhaseManager::transition(Phase next, double now)
{
  if (next == Phase::Approach) touching_.assign(scene_.robot_names.size(), false);
  advisory_.clear();
  phase_ = next;
  entered_ = now;
  stable_since_ = -1;
}
void GraspPhaseManager::start(const GraspState& s, double now)
{
  if (phase_ != Phase::Idle)
    throw std::logic_error("start requires Idle; restart the node after completion or abort");
  initial_z_ = s.input.object.position.z();
  lift_z_ = initial_z_ + task_.lift_height;
  yaw_ = yaw(s.input.object.orientation);
  transition(Phase::Staging, now);
}
bool GraspPhaseManager::requestRealign(double now)
{
  if (phase_ != Phase::Approach && phase_ != Phase::Preload) return false;
  realign_ready_ = resume_requested_ = false;
  transition(Phase::Realign, now);
  return true;
}
bool GraspPhaseManager::requestResume()
{
  if (phase_ != Phase::Realign || !realign_ready_) return false;
  resume_requested_ = true;
  return true;
}
void GraspPhaseManager::abort(const std::string& reason)
{
  if (phase_ != Phase::Aborted)
    fault_ = reason;
  phase_ = Phase::Aborted;
}
bool GraspPhaseManager::stable(bool condition, double now, double duration)
{
  if (!condition)
  {
    stable_since_ = -1;
    return false;
  }
  if (stable_since_ < 0)
    stable_since_ = now;
  return now - stable_since_ >= duration;
}
Reference GraspPhaseManager::step(const GraspState& s, const ControlStatus& status, double now)
{
  if (!status.fault.empty())
    abort(status.fault);
  const double elapsed = now - entered_;
  std::vector<double> preload_diagnostics;
  const auto& t = task_;

  double max_force = 0;
  for (const auto& c : s.contacts)
  {
    max_force = std::max(max_force, c.normal_force);
  }
  auto atRadius = [&](double clearance, double tolerance) {
    if (s.contacts.size() != scene_.robot_names.size())
      return false;
    for (std::size_t i = 0; i < s.contacts.size(); ++i)
    {
      const auto& c = s.contacts[i];
      const Vec3 normal = s.input.object.orientation *
                          Vec3(std::cos(scene_.face_angles[i]), std::sin(scene_.face_angles[i]), 0);
      const Vec3 tangent = s.input.object.orientation *
                           Vec3(-std::sin(scene_.face_angles[i]), std::cos(scene_.face_angles[i]), 0);
      const double radius = (s.input.vehicles[i].base.position - s.input.object.position).dot(normal);
      if (std::abs(radius - scene_.face_radii[i] - clearance) > tolerance ||
          std::abs((c.position - s.input.object.position).dot(tangent)) > tolerance ||
          std::abs((c.velocity - s.object_velocity).dot(normal)) > .08 ||
          std::abs((c.velocity - s.object_velocity).dot(tangent)) > .08)
        return false;
    }
    return true;
  };
  const double xy_error = (scene_.destination_position - s.input.object.position).head<2>().norm();
  const bool arrived = xy_error <= t.position_tolerance && s.object_velocity.head<2>().norm() <= t.velocity_tolerance;
  advisory_.clear();
  double timeout = INFINITY;
  switch (phase_)
  {
    case Phase::Staging:
      timeout = t.staging_timeout;
      {
        bool height_ready = true;
        for (const auto& v : s.input.vehicles)
          height_ready = height_ready && std::abs(v.base.position.z() - initial_z_ - scene_.contact_height) <= t.staging_tolerance &&
                         std::abs(v.velocity_world.z()) <= .08;
        if (stable(height_ready && atRadius(scene_.staging_clearance, t.staging_tolerance), now, .5))
          transition(Phase::Orient, now);
      }
      break;
    case Phase::Realign:
    case Phase::Orient:
      if (phase_ == Phase::Orient) timeout = t.staging_timeout;
      {
        bool attitude_ready = true;
        for (std::size_t i = 0; i < s.input.vehicles.size(); ++i)
        {
          const double heading = yaw(s.input.object.orientation) + scene_.face_angles[i] + kPi / 2;
          const Vec3 rpy = contactAttitude(s.input.object.orientation * plannedContactNormal(scene_, i), 1, heading);
          const Eigen::Quaterniond target =
              Eigen::AngleAxisd(rpy.z(), Vec3::UnitZ()) * Eigen::AngleAxisd(rpy.x(), Vec3::UnitX());
          const Vec3 center = footCenterTarget(scene_, i, s.input.object, control_.ball_radius, t.precontact_distance);
          const Vec3 velocity = s.object_velocity + s.object_angular_velocity.cross(center - s.input.object.position);
          attitude_ready = attitude_ready && (s.contacts[i].position - center).norm() <= t.position_tolerance &&
                           (s.contacts[i].velocity - velocity).norm() <= t.velocity_tolerance &&
                           target.angularDistance(s.input.vehicles[i].base.orientation) <= .10;
        }
        if (phase_ == Phase::Realign)
        {
          realign_ready_ = stable(attitude_ready, now, t.attitude_settle);
          advisory_ = realign_ready_ ? "Realigned: press L1+R1 to approach" : "Returning to precontact positions";
          if (resume_requested_ && realign_ready_)
          {
            yaw_ = yaw(s.input.object.orientation);
            transition(Phase::Approach, now);
          }
          resume_requested_ = false;
        }
        else if (stable(elapsed >= t.attitude_duration && attitude_ready, now, t.attitude_settle))
          transition(Phase::Calibrate, now);
      }
      break;
    case Phase::Calibrate:
      if (elapsed >= t.baseline_duration && status.calibration_ready)
        transition(Phase::Approach, now);
      break;
    case Phase::Approach:
      if (elapsed > t.approach_timeout) advisory_ = "Waiting for all contacts; operator may realign";
      {
        bool all_loaded = true;
        for (std::size_t i = 0; i < touching_.size(); ++i)
        {
          const auto& c = s.contacts.at(i);
          const Vec3 center = footCenterTarget(scene_, i, s.input.object, control_.ball_radius, 0);
          const bool near = (c.position - center).norm() <= t.touch_tolerance;
          const bool loaded = near && c.normal_force >= t.touch_force;
          touching_[i] = near && (touching_[i] || loaded);
          all_loaded = all_loaded && loaded;
        }
        if (stable(all_loaded, now, .1)) transition(Phase::Preload, now);
      }
      break;
    case Phase::Preload:
      advisory_ = status.preload_warning;
      if (advisory_.empty() && elapsed > t.lift_timeout)
        advisory_ = "Waiting for preload force and friction margin; operator may realign";
      {
        bool loaded = s.contacts.size() == scene_.robot_names.size() &&
            (control_.wrench_control ? status.preload_plan_ready &&
             status.preload_forces.size() == s.contacts.size() &&
             status.preload_normal_targets.size() == s.contacts.size() :
             status.normal_force >= normal_force_target_ - 1e-6);
        for (std::size_t i = 0; i < s.contacts.size(); ++i)
        {
          const double target = status.normal_force_targets.size() == s.contacts.size() ?
                                status.normal_force_targets[i] : normal_force_target_;
          loaded = loaded && std::isfinite(s.contacts[i].normal_force) &&
                   target > 0 && s.contacts[i].normal_force >= t.preload_force_ratio * target;
          if (elapsed >= t.preload_duration && advisory_.empty() && target > 0 &&
              s.contacts[i].normal_force < t.preload_force_ratio * target)
            advisory_ = scene_.robot_names[i] + ": observed " + std::to_string(s.contacts[i].normal_force) +
                " N, waiting for " + std::to_string(t.preload_force_ratio * target) + " N";
          if (control_.wrench_control)
          {
            ControlConfig capacity_config = control_;
            capacity_config.allocation_mu = preload_mu_;
            if(status.preload_forces.size()==s.contacts.size() &&
               status.preload_normal_targets.size()==s.contacts.size())
            {
              const double planned=status.preload_normal_targets[i];
              const auto& force=status.preload_forces[i];
              loaded = loaded && target>=planned-1e-5 &&
                  plannedContactCapacityReady(s.contacts[i],force,capacity_config);
              // Target, ramped command, observed normal, capacity slack, former
              // reprojected gap, and same-generation target gap (N per Bee).
              preload_diagnostics.insert(preload_diagnostics.end(),{planned,target,
                  s.contacts[i].normal_force,plannedContactCapacitySlack(s.contacts[i],force,capacity_config),
                  target+s.contacts[i].normal.dot(force),target-planned});
            }
            else loaded=false;
          }
        }
        if (stable(loaded, now, t.preload_duration))
          transition(Phase::SupportTransfer, now);
      }
      break;
    case Phase::SupportTransfer:
      timeout = t.lift_timeout;
      if (status.support_ready)
      {
        initial_z_ = control_.wrench_control ? status.reference_z : s.input.object.position.z();
        lift_z_ = initial_z_ + t.lift_height;
        transition(Phase::Lift, now);
      }
      break;
    case Phase::Lift:
      timeout = t.lift_timeout;
      if (stable(std::abs(s.input.object.position.z() - lift_z_) <= t.hold_tolerance &&
                     std::abs(s.object_velocity.z()) <= t.hold_velocity &&
                     std::abs(status.reference_z - lift_z_) <= .0005 && std::abs(status.reference_vz) <= .001,
                 now, t.hold_duration))
        transition(Phase::Hold, now);
      break;
    case Phase::Hold:
      if ((t.automatic_transport || transport_requested_) && scene_.has_destination)
        transition(Phase::Transport, now);
      break;
    case Phase::Transport:
      timeout = t.transport_timeout;
      if (stable(arrived, now, t.arrival_duration))
        transition(Phase::DestinationSettle, now);
      break;
    case Phase::DestinationSettle:
      if (elapsed >= t.destination_settle)
        transition(Phase::Descend, now);
      break;
    case Phase::Descend:
      timeout = t.descent_timeout;
      if (stable(s.input.object.position.z() <= scene_.destination_position.z() + t.hold_tolerance &&
                     s.input.object.position.z() >= scene_.destination_position.z() - .02 &&
                     std::abs(s.object_velocity.z()) <= t.velocity_tolerance && xy_error <= 2 * t.position_tolerance,
                 now, t.touchdown_duration))
        transition(Phase::Unload, now);
      break;
    case Phase::Unload:
      if (elapsed >= t.unload_duration)
        transition(Phase::Release, now);
      break;
    case Phase::Release:
      timeout = t.release_timeout;
      if (stable(status.normal_force <= .05 && max_force <= t.release_tolerance, now, t.release_duration))
        transition(Phase::Retreat, now);
      break;
    case Phase::Retreat:
      timeout = t.retreat_timeout;
      if (stable(atRadius(control_.ball_radius + t.retreat_clearance, .05), now, .5))
        transition(Phase::Unwind, now);
      break;
    case Phase::Unwind:
      if (elapsed >= t.attitude_duration)
        transition(Phase::Verify, now);
      break;
    case Phase::Verify:
      timeout = 3 * t.verify_duration;
      if (stable(xy_error <= 2 * t.position_tolerance &&
                     std::abs(s.input.object.position.z() - scene_.destination_position.z()) <= 2 * t.hold_tolerance &&
                     s.object_velocity.norm() <= t.velocity_tolerance,
                 now, t.verify_duration))
        transition(Phase::Complete, now);
      break;
    default:
      break;
  }
  if (phase_ != Phase::Aborted && now - entered_ > timeout)
    abort(std::string(phaseName(phase_)) + " timed out");
  Reference r;
  r.preload_diagnostics=preload_diagnostics;
  r.preload_vertical_force = scene_.robot_names.size()*preload_design_load_;
  r.preload_margin = t.normal_force_margin;
  r.preload_mu = preload_mu_;
  r.preload_force_ratio = t.preload_force_ratio;
  r.phase = phase_;
  r.object_yaw = yaw_;
  r.object_z = initial_z_;
  r.attitude_fraction = 1;
  r.radius_clearance = scene_.staging_clearance;
  r.normal_force_rate = control_.normal_force_rate;
  r.z_velocity_limit = control_.trajectory_velocity;
  const double age = now - entered_;
  if (phase_ == Phase::Staging || phase_ == Phase::Idle)
    r.attitude_fraction = 0;
  if (phase_ == Phase::Orient)
  {
    const double x = std::min(1.0, age / t.attitude_duration);
    r.attitude_fraction *= x * x * (3 - 2 * x);
  }
  r.calibrate = phase_ == Phase::Calibrate;
  if (phase_ == Phase::Orient || phase_ == Phase::Calibrate || phase_ == Phase::Realign)
    r.radius_clearance = t.precontact_distance;
  if (phase_ == Phase::Approach)
  {
    r.radius_clearance = std::max(-control_.max_compression, t.precontact_distance - t.approach_speed * age);
    r.touching = touching_;
    r.touch_force = t.touch_force;
  }
  if (phase_ >= Phase::Preload && phase_ <= Phase::Unload)
    r.normal_force = normal_force_target_;
  r.begin_support = phase_ == Phase::SupportTransfer;
  if (phase_ >= Phase::Lift && phase_ <= Phase::DestinationSettle)
    r.object_z = lift_z_;
  if (phase_ >= Phase::Transport && phase_ <= Phase::Unload)
  {
    r.object_velocity = t.transport_kp * (scene_.destination_position - s.input.object.position);
    r.object_velocity.z() = 0;
    const double limit = phase_ == Phase::Transport ? t.transport_speed : .01;
    if (r.object_velocity.norm() > limit)
      r.object_velocity *= limit / r.object_velocity.norm();
  }
  if (phase_ == Phase::Descend)
  {
    r.object_z = std::max(scene_.destination_position.z() - .01, lift_z_ - t.descent_speed * age);
    r.z_velocity_limit = t.descent_speed;
  }
  if (phase_ >= Phase::Unload && phase_ <= Phase::Complete)
  {
    r.object_z = scene_.destination_position.z();
    r.normal_force_rate = t.release_rate;
  }
  if (phase_ == Phase::Unload)
  {
    r.touchdown = true;
    r.support_scale = std::max(0.0, 1 - age / t.unload_duration);
  }
  if (phase_ >= Phase::Release)
    r.support_scale = 0;
  if (phase_ >= Phase::Retreat)
    r.radius_clearance = control_.ball_radius + t.retreat_clearance;
  if (phase_ == Phase::Unwind)
    r.attitude_fraction *= std::max(0.0, 1 - age / t.attitude_duration);
  if (phase_ == Phase::Verify || phase_ == Phase::Complete)
    r.attitude_fraction = 0;
  return r;
}

void TaskConfig::validate() const
{
  requirePositiveParameter(friction_coefficient, "friction_coefficient");
  requirePositiveParameter(normal_force_margin, "normal_force_margin");
  requireNonnegativeParameter(design_vertical_acceleration, "design_vertical_acceleration");
  requirePositiveParameter(preload_force_ratio, "preload_force_ratio");
  if (normal_force_margin < 1 || preload_force_ratio > 1)
    throw std::invalid_argument("normal_force_margin must be >= 1 and preload_force_ratio in (0, 1]");
  requirePositiveParameter(touch_force, "touch_force");
  requirePositiveParameter(touch_tolerance, "touch_tolerance");
  if (!std::isfinite(precontact_distance) || precontact_distance <= touch_tolerance)
    throw std::invalid_argument("precontact_distance must exceed touch_tolerance");
  requirePositiveParameter(approach_speed, "approach_speed");
  requirePositiveParameter(staging_tolerance, "staging_tolerance");
  requirePositiveParameter(attitude_duration, "attitude_duration");
  requirePositiveParameter(attitude_settle, "attitude_settle");
  requirePositiveParameter(baseline_duration, "baseline_duration");
  requirePositiveParameter(preload_duration, "preload_duration");
  requirePositiveParameter(lift_height, "lift_height");
  requirePositiveParameter(hold_tolerance, "hold_tolerance");
  requirePositiveParameter(hold_velocity, "hold_velocity");
  requirePositiveParameter(hold_duration, "hold_duration");
  requirePositiveParameter(transport_kp, "transport_kp");
  requirePositiveParameter(transport_speed, "transport_speed");
  requirePositiveParameter(position_tolerance, "position_tolerance");
  requirePositiveParameter(velocity_tolerance, "velocity_tolerance");
  requirePositiveParameter(arrival_duration, "arrival_duration");
  requirePositiveParameter(destination_settle, "destination_settle");
  requirePositiveParameter(descent_speed, "descent_speed");
  requirePositiveParameter(touchdown_duration, "touchdown_duration");
  requirePositiveParameter(unload_duration, "unload_duration");
  requirePositiveParameter(release_rate, "release_rate");
  requirePositiveParameter(release_tolerance, "release_tolerance");
  requirePositiveParameter(release_duration, "release_duration");
  requirePositiveParameter(retreat_clearance, "retreat_clearance");
  requirePositiveParameter(verify_duration, "verify_duration");
  requirePositiveParameter(staging_timeout, "staging_timeout");
  requirePositiveParameter(approach_timeout, "approach_timeout");
  requirePositiveParameter(lift_timeout, "lift_timeout");
  requirePositiveParameter(transport_timeout, "transport_timeout");
  requirePositiveParameter(descent_timeout, "descent_timeout");
  requirePositiveParameter(release_timeout, "release_timeout");
  requirePositiveParameter(retreat_timeout, "retreat_timeout");
}

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
        !std::isfinite(c.normal_force) || !std::isfinite(c.support_force) || !c.support_vector.allFinite() ||
        !c.acceleration.allFinite() || !c.baselink_rpy.allFinite() || !std::isfinite(c.z) ||
        c.acceleration.norm() > config_.max_acceleration)
      return "invalid or excessive command";
  if (!airborne(r.phase) && r.phase != Phase::SupportTransfer)
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
  if (r.phase == Phase::SupportTransfer &&
      std::abs(s.input.object.position.z() - output.status.reference_z) > config_.max_drop)
    return "support transfer height drift limit";
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
