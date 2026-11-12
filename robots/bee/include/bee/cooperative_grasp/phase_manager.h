#pragma once
#include <bee/cooperative_grasp/types.h>

namespace bee::grasp
{
class GraspPhaseManager
{
public:
  GraspPhaseManager(SceneConfig scene, ControlConfig control, TaskConfig task);
  void start(const GraspState& state, double now);
  void abort(const std::string& reason);
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
  Phase phase_ = Phase::Idle;
  double entered_ = 0, stable_since_ = -1, initial_z_ = 0, lift_z_ = 0, yaw_ = 0;
  bool transport_requested_ = false;
  std::string fault_;
};
}  // namespace bee::grasp
