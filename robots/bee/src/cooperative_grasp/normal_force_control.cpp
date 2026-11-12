#include <bee/cooperative_grasp/controller.h>
namespace bee::grasp
{
void CooperativeGraspController::updateNormalForce(const Reference& r, double dt)
{
  status_.normal_force = approach(status_.normal_force, std::clamp(r.normal_force, 0.0, config_.max_normal_force),
                                  r.normal_force_rate * dt);
}
}  // namespace bee::grasp
