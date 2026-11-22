#pragma once
#include <bee/cooperative_grasp/grasp_state_estimator.h>

namespace bee::grasp
{
struct TaskConfig
{
  bool automatic_transport = true;
  double friction_coefficient = 1.0, normal_force_margin = 1.2, design_vertical_acceleration = 0;
  double preload_force_ratio = .9;
  double precontact_distance = .20, touch_force = .35, touch_tolerance = .035;
  double approach_speed = .035, staging_tolerance = .08;
  double attitude_duration = 3, attitude_settle = 1, baseline_duration = 2, preload_duration = 3;
  double lift_height = .05, hold_tolerance = .003, hold_velocity = .003, hold_duration = 1;
  double transport_kp = .5, transport_speed = .02, position_tolerance = .015, velocity_tolerance = .008;
  double arrival_duration = 1, destination_settle = 2, descent_speed = .005;
  double touchdown_duration = 1.5, unload_duration = 3, release_rate = .8;
  double release_tolerance = .25, release_duration = 1, retreat_clearance = .4, verify_duration = 2;
  double staging_timeout = 150, approach_timeout = 35, lift_timeout = 150;
  double transport_timeout = 180, descent_timeout = 40, release_timeout = 20, retreat_timeout = 30;
  void validate() const;
};
struct SafetyConfig
{
  double input_timeout = 1, max_acceleration = 20, max_tilt = .30;
  double max_slip = .06, max_height_difference = .04, max_drop = .03;
  double max_contact_deviation = .04;
  double max_object_speed = .30, max_yaw_rate = 1.50;
  double force_loss_threshold = .5, force_loss_duration = .25;
};
enum class Phase
{
  Idle,
  Staging,
  Orient,
  Calibrate,
  Realign,
  Approach,
  Preload,
  SupportTransfer,
  Lift,
  Hold,
  Transport,
  DestinationSettle,
  Descend,
  Unload,
  Release,
  Retreat,
  Unwind,
  Verify,
  Complete,
  Aborted
};
const char* phaseName(Phase phase);
inline bool airborne(Phase p)
{
  return p >= Phase::Lift && p <= Phase::Descend;
}
struct Reference
{
  Phase phase = Phase::Idle;
  double attitude_fraction = 0, radius_clearance = 0, normal_force = 0, normal_force_rate = .3;
  double object_z = 0, object_yaw = 0, support_scale = 1, z_velocity_limit = .01;
  double preload_vertical_force = 0, preload_margin = 1.2, preload_mu = 1.0, preload_force_ratio = .9;
  Vec3 object_velocity = Vec3::Zero();
  std::vector<double> preload_diagnostics;
  std::vector<bool> touching;
  double touch_force = 0;
  bool begin_support = false, touchdown = false, calibrate = false;
};

struct ControlStatus;
struct ControlOutput;

class GraspSafetyMonitor
{
public:
  explicit GraspSafetyMonitor(SafetyConfig config) : config_(config)
  {
  }
  std::string check(const GraspState&, const Reference&, const ControlOutput&, double dt);

private:
  SafetyConfig config_;
  double force_loss_time_ = 0;
};

class GraspPhaseManager
{
public:
  GraspPhaseManager(SceneConfig scene, ControlConfig control, TaskConfig task);
  void start(const GraspState& state, double now);
  void abort(const std::string& reason);
  bool requestRealign(double now);
  bool requestResume();
  const std::string& advisory() const { return advisory_; }
  void requestTransport()
  {
    transport_requested_ = true;
  }
  Reference step(const GraspState&, const ControlStatus&, double now);
  Phase phase() const
  {
    return phase_;
  }
  double normalForceTarget() const
  {
    return normal_force_target_;
  }
  const std::string& fault() const
  {
    return fault_;
  }

private:
  void transition(Phase next, double now);
  bool stable(bool condition, double now, double duration);
  SceneConfig scene_;
  ControlConfig control_;
  TaskConfig task_;
  double normal_force_target_ = 0;
  double preload_design_load_ = 0, preload_mu_ = 0;
  Phase phase_ = Phase::Idle;
  double entered_ = 0, stable_since_ = -1, initial_z_ = 0, lift_z_ = 0, yaw_ = 0;
  std::vector<bool> touching_;
  bool transport_requested_ = false;
  bool realign_ready_ = false, resume_requested_ = false;
  std::string fault_, advisory_;
};
}  // namespace bee::grasp
