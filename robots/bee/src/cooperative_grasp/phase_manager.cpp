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
    PHASE_CASE(Staging) PHASE_CASE(Orient) PHASE_CASE(Calibrate) PHASE_CASE(Approach) PHASE_CASE(Preload)
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
  if (!std::isfinite(normal_force_target_) || normal_force_target_ > control_.max_normal_force)
    throw std::invalid_argument("calculated normal force exceeds controller limit");
}
void GraspPhaseManager::transition(Phase next, double now)
{
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
  const auto& t = task_;
  const double contact_clearance = control_.foot_contact_offset + control_.ball_radius - control_.max_compression;
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
      const double radius = (s.input.vehicles[i].base.position - s.input.object.position).dot(c.normal);
      if (std::abs(radius - scene_.face_radii[i] - clearance) > tolerance || std::abs(c.tangent_position) > tolerance ||
          std::abs(c.radial_velocity) > .08 || std::abs(c.tangent_velocity) > .08)
        return false;
    }
    return true;
  };
  const double xy_error = (scene_.destination_position - s.input.object.position).head<2>().norm();
  const bool arrived = xy_error <= t.position_tolerance && s.object_velocity.head<2>().norm() <= t.velocity_tolerance;
  double timeout = INFINITY;
  switch (phase_)
  {
    case Phase::Staging:
      timeout = t.staging_timeout;
      {
        bool height_ready = true;
        for (const auto& v : s.input.vehicles)
          height_ready = height_ready && std::abs(v.base.position.z() - initial_z_) <= t.staging_tolerance &&
                         std::abs(v.velocity_world.z()) <= .08;
        if (stable(height_ready && atRadius(scene_.staging_clearance, t.staging_tolerance), now, .5))
          transition(Phase::Orient, now);
      }
      break;
    case Phase::Orient:
      timeout = t.staging_timeout;
      {
        bool attitude_ready = true;
        for (std::size_t i = 0; i < s.input.vehicles.size(); ++i)
        {
          const double heading = yaw(s.input.object.orientation) + scene_.face_angles[i] + kPi / 2;
          const Eigen::Quaterniond target =
              Eigen::AngleAxisd(heading, Vec3::UnitZ()) * Eigen::AngleAxisd(kPi / 2, Vec3::UnitX());
          attitude_ready = attitude_ready && target.angularDistance(s.input.vehicles[i].base.orientation) <= .10;
        }
        if (stable(elapsed >= t.attitude_duration && attitude_ready, now, t.attitude_settle))
          transition(Phase::Calibrate, now);
      }
      break;
    case Phase::Calibrate:
      if (elapsed >= t.baseline_duration)
        transition(Phase::Approach, now);
      break;
    case Phase::Approach:
      timeout = t.approach_timeout;
      if (elapsed * t.approach_speed >= scene_.staging_clearance - contact_clearance &&
          stable(atRadius(contact_clearance, .08), now, .1))
        transition(Phase::Preload, now);
      break;
    case Phase::Preload:
      timeout = t.lift_timeout;
      if (stable(s.contacts.size() == scene_.robot_names.size() &&
                     std::all_of(s.contacts.begin(), s.contacts.end(), [&](const ContactState& contact) {
                       return std::isfinite(contact.normal_force) &&
                              contact.normal_force >= t.preload_force_ratio * normal_force_target_;
                     }),
                 now, t.preload_duration))
        transition(Phase::SupportTransfer, now);
      break;
    case Phase::SupportTransfer:
      timeout = t.lift_timeout;
      if (status.support_ready)
      {
        initial_z_ = s.input.object.position.z();
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
  r.phase = phase_;
  r.object_yaw = yaw_;
  r.object_z = initial_z_;
  r.roll = kPi / 2;
  r.radius_clearance = scene_.staging_clearance;
  r.normal_force_rate = control_.normal_force_rate;
  r.z_velocity_limit = control_.trajectory_velocity;
  const double age = now - entered_;
  if (phase_ == Phase::Staging || phase_ == Phase::Idle)
    r.roll = 0;
  if (phase_ == Phase::Orient)
  {
    const double x = std::min(1.0, age / t.attitude_duration);
    r.roll *= x * x * (3 - 2 * x);
  }
  r.calibrate = phase_ == Phase::Calibrate;
  if (phase_ == Phase::Approach)
    r.radius_clearance = std::max(contact_clearance, scene_.staging_clearance - t.approach_speed * age);
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
    r.roll *= std::max(0.0, 1 - age / t.attitude_duration);
  if (phase_ == Phase::Verify || phase_ == Phase::Complete)
    r.roll = 0;
  return r;
}
}  // namespace bee::grasp
