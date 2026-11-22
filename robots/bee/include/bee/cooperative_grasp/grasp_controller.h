#pragma once
#include <bee/cooperative_grasp/phase_manager.h>

namespace bee::grasp
{
enum class ZMode
{
  Position,
  Velocity,
  Acceleration
};
struct VehicleCommand
{
  bool contact_tracking = false, support_tracking = false, calibrate = false, wrench_tracking = false;
  Vec3 velocity = Vec3::Zero(), normal = Vec3::UnitX(), object_position = Vec3::Zero();
  double normal_force = 0, minimum_radius = 0;
  double support_force = 0, observer_blend = 0, support_scale = 1;
  Vec3 acceleration = Vec3::Zero(), baselink_rpy = Vec3::Zero();
  Vec3 support_vector = Vec3::Zero();
  Vec3 position_target = Vec3::Zero();
  double z = 0;
  ZMode z_mode = ZMode::Position;
};
struct ControlStatus
{
  bool calibration_ready = true;  // Set by the central node from calibration results.
  std::vector<double> normal_force_targets;
  std::vector<Vec3> preload_forces;
  std::vector<double> preload_normal_targets;
  bool preload_plan_ready = false;
  std::string preload_warning;
  // Object-frame offsets from the configuration captured at support transfer.
  // Empty for the legacy acceleration controller.
  std::vector<Vec3> contact_geometry_errors;
  double wrench_force_error = 0, wrench_moment_error = 0, allocation_violation = 0;
  double allocation_seconds = 0, allocation_primal = 0, allocation_stationarity = 0;
  double allocation_complementarity = 0;
  int allocation_iterations = 0, allocation_restarts = 0;
  bool support_ready = false;
  double normal_force = 0, reference_z = 0, reference_vz = 0, reference_az = 0;
  double z_integral = 0, vertical_force_target = 0, common_z_acceleration = 0;
  double support_ratio = 0, common_slip = 0;
  double reference_velocity_integral = 0;
  std::string fault;
};
struct ControlOutput
{
  std::vector<VehicleCommand> commands;
  ControlStatus status;
};

Vec3 contactAcceleration(const Vec3& n, const Vec3& feedforward, const Vec3& error,
                                double radial, const Vec3& support, const Vec3& external,
                                bool loaded, double blend, double scale, double mass,
                                double gain_xy, double gain_z, double normal_limit,
                                double tangent_limit, double vertical_limit);

// Contact representatives in object coordinates. Independent of world height,
// robot count, and the initial differences between attachment positions.
class ContactGeometry
{
public:
  void reset() { reference_.clear(); }
  bool initialized() const { return !reference_.empty(); }
  void capture(const GraspState& state)
  {
    reference_.clear();
    for (const auto& contact : state.contacts)
      reference_.push_back(state.input.object.orientation.conjugate() *
                           (contact.position - state.input.object.position));
  }
  std::vector<Vec3> errors(const GraspState& state) const
  {
    if (!initialized() || reference_.size() != state.contacts.size())
      throw std::logic_error("contact geometry reference unavailable");
    std::vector<Vec3> errors;
    for (std::size_t i = 0; i < reference_.size(); ++i)
      errors.push_back(state.input.object.orientation.conjugate() *
                       (state.contacts[i].position - state.input.object.position) - reference_[i]);
    return errors;
  }
private:
  std::vector<Vec3> reference_;
};

// No ROS calls or sleeping. One instance owns one grasp's memory. Solver failure
// dumps are opt-in simulation diagnostics, disabled in normal operation.
class GraspController
{
public:
  GraspController(SceneConfig scene, ControlConfig config);
  void reset();
  ControlOutput step(const GraspState& state, const Reference& reference, double dt);

private:
  void updateVertical(const GraspState&, const Reference&, double);
  void updateAttitude(const GraspState&, const Reference&, double);
  void updateNormalForce(const Reference&, double);
  VehicleCommand contactCommand(std::size_t, const GraspState&, const Reference&) const;
  VehicleCommand approachCommand(std::size_t, const GraspState&, const Reference&) const;
  void advanceZReference(double target, double velocity_limit, double dt);
  void makeTrackingReferences(const GraspState&, const Reference&, double, ControlOutput&);
  void makeWrenchReferences(const GraspState&, const Reference&, double, ControlOutput&);
  void updatePreloadPlan(const GraspState&, const Reference&, double);
  Vec3 object_reference_ = Vec3::Zero(), object_integral_ = Vec3::Zero(), previous_motion_ = Vec3::Zero();
  Eigen::Matrix3d object_rotation_reference_ = Eigen::Matrix3d::Identity();
  Eigen::VectorXd allocated_forces_;
  Eigen::VectorXd preload_plan_;
  std::vector<double> preload_targets_, preload_commands_;
  std::vector<Vec3> last_omega_, filtered_alpha_;
  bool wrench_initialized_ = false;
  SceneConfig scene_;
  ControlConfig config_;
  ControlStatus status_;
  bool support_active_ = false, slip_active_ = false;
  // Wrench commanded at the end of preload, before adding tangent support.
  // Support progress and its observer gate use this same nonzero baseline.
  Wrench support_start_wrench_;
  double initial_z_ = 0, relative_z_ = 0, slip_reference_ = 0, support_stable_ = 0;
  double filtered_force_ = 0, transfer_force_ = 0, force_integral_ = 0, slip_acceleration_ = 0;
  double yaw_acceleration_ = 0, object_yaw_trim_ = 0, object_yaw_integral_ = 0;
  Vec3 tilt_target_ = Vec3::UnitZ(), filtered_moment_ = Vec3::Zero(), translation_ = Vec3::Zero();
  std::vector<double> contact_z_, moment_z_, roll_trim_, yaw_trim_, roll_i_, yaw_i_, force_offsets_, force_i_;
  std::vector<Vec3> local_moments_;
  std::vector<Vec3> tracking_velocity_;
  bool tracking_initialized_ = false;
  double observer_blend_ = 0;
  ContactGeometry contact_geometry_;
};
}  // namespace bee::grasp
