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

  EXPECT_NEAR(TestableGimbalrotorController::calculateMaxYawScale(integrated_map_inv_rot), 0.0400, 1e-12);
}

TEST(GimbalrotorYawScaleTest, KeepsZeroWhenNoPositiveGainExists)
{
  Eigen::MatrixXd integrated_map_inv_rot = Eigen::MatrixXd::Zero(4, 3);
  integrated_map_inv_rot.col(2) << -0.01, -0.02, -0.03, -0.04;

  EXPECT_DOUBLE_EQ(TestableGimbalrotorController::calculateMaxYawScale(integrated_map_inv_rot), 0.0);
}

TEST(GimbalrotorYawScaleTest, IgnoresNonFiniteGain)
{
  Eigen::MatrixXd integrated_map_inv_rot = Eigen::MatrixXd::Zero(4, 3);
  integrated_map_inv_rot.col(2) << std::numeric_limits<double>::quiet_NaN(), 0.02,
      std::numeric_limits<double>::infinity(), -0.04;

  EXPECT_DOUBLE_EQ(TestableGimbalrotorController::calculateMaxYawScale(integrated_map_inv_rot), 0.02);
}
}  // namespace aerial_robot_control

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
