#pragma once
#include <bee/cooperative_grasp/types.h>
#include <bee/cooperative_grasp/contact_geometry.h>

namespace bee::grasp
{
// No ROS calls, sleeping or file access. One instance owns one grasp's memory.
class CooperativeGraspController
{
public:
  CooperativeGraspController(SceneConfig scene, ControlConfig config);
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
  SceneConfig scene_;
  ControlConfig config_;
  ControlStatus status_;
  bool support_active_ = false, slip_active_ = false;
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
