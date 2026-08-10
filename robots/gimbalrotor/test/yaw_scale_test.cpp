#include <gtest/gtest.h>

#include <limits>

#include <gimbalrotor/control/gimbalrotor_controller.h>

namespace aerial_robot_control
{
class TestableGimbalrotorController : public GimbalrotorController
{
public:
  using GimbalrotorController::calculateMaxYawScale;
};

TEST(GimbalrotorYawScaleTest, UsesAllVirtualRotorRows)
{
  Eigen::MatrixXd integrated_map_inv_rot = Eigen::MatrixXd::Zero(8, 3);
  integrated_map_inv_rot.col(2) << 0.0028, -0.0230, 0.0028, -0.0402, 0.0028, 0.0232, 0.0028, 0.0400;

  // -0.0402 and +0.0400 both become magnitude 0.040 after transport;
  // PC and FC must keep the first row in that tie.
  EXPECT_NEAR(TestableGimbalrotorController::calculateMaxYawScale(integrated_map_inv_rot), -0.0400, 1e-12);
}

TEST(GimbalrotorYawScaleTest, KeepsSignWhenLargestMagnitudeGainIsNegative)
{
  Eigen::MatrixXd integrated_map_inv_rot = Eigen::MatrixXd::Zero(8, 3);
  integrated_map_inv_rot.col(2) << -0.028, 0.002, -0.028, -0.002, -0.028, -0.002, -0.028, 0.002;

  EXPECT_DOUBLE_EQ(TestableGimbalrotorController::calculateMaxYawScale(integrated_map_inv_rot), -0.028);
}

TEST(GimbalrotorYawScaleTest, IgnoresNonFiniteGain)
{
  Eigen::MatrixXd integrated_map_inv_rot = Eigen::MatrixXd::Zero(4, 3);
  integrated_map_inv_rot.col(2) << std::numeric_limits<double>::quiet_NaN(), 0.02,
      std::numeric_limits<double>::infinity(), -0.04;

  EXPECT_DOUBLE_EQ(TestableGimbalrotorController::calculateMaxYawScale(integrated_map_inv_rot), -0.04);
}

TEST(GimbalrotorYawScaleTest, UsesRosserialQuantizedScale)
{
  Eigen::MatrixXd integrated_map_inv_rot = Eigen::MatrixXd::Zero(4, 3);
  integrated_map_inv_rot.col(2) << 0.0028, -0.02818, 0.0024, -0.02799;

  EXPECT_DOUBLE_EQ(TestableGimbalrotorController::calculateMaxYawScale(integrated_map_inv_rot), -0.028);
}

TEST(GimbalrotorYawScaleTest, KeepsZeroWhenAllGainsAreZero)
{
  const Eigen::MatrixXd integrated_map_inv_rot = Eigen::MatrixXd::Zero(4, 3);

  EXPECT_DOUBLE_EQ(TestableGimbalrotorController::calculateMaxYawScale(integrated_map_inv_rot), 0.0);
}
}  // namespace aerial_robot_control

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
