#pragma once
#include <bee/cooperative_grasp/types.h>

namespace bee::grasp
{
class GraspStateEstimator
{
public:
  GraspStateEstimator(SceneConfig scene, ControlConfig config);
  void reset();
  void beginCalibration();
  void addCalibrationSample(const InputState& input);
  bool finishCalibration();
  GraspState update(const InputState& input);

private:
  SceneConfig scene_;
  ControlConfig config_;
  std::vector<Wrench> baseline_, sums_;
  std::vector<Vec3> world_baseline_, world_sums_;
  std::vector<double> normal_forces_;
  std::size_t samples_ = 0;
  double previous_stamp_ = 0;
  Pose previous_object_;
  Vec3 velocity_ = Vec3::Zero(), omega_ = Vec3::Zero();
};
}  // namespace bee::grasp
