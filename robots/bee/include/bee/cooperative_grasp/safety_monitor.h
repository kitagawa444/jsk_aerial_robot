#pragma once
#include <bee/cooperative_grasp/types.h>

namespace bee::grasp
{
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
}  // namespace bee::grasp
