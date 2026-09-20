// -*- mode: c++ -*-

#pragma once

#include <algorithm>
#include <numeric>
#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <aerial_robot_control/control/fully_actuated_controller.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Empty.h>
#include <mutex>
#include <gimbalrotor/model/gimbalrotor_robot_model.h>
#include <spinal/WrenchAllocationMatrix.h>
#include <spinal/DifferentialAllocationConfig.h>

namespace aerial_robot_control
{
class GimbalrotorController : public PoseLinearController
{
public:
  GimbalrotorController();
  ~GimbalrotorController() = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate) override;

  const Eigen::MatrixXd getIntegratedMapInvTrans()
  {
    std::lock_guard<std::mutex> lock(trans_map_inv_mutex_);
    return integrated_map_inv_trans_;
  }
  const Eigen::MatrixXd getIntegratedMapInvRot()
  {
    std::lock_guard<std::mutex> lock(rot_map_inv_mutex_);
    return integrated_map_inv_rot_;
  }

private:
  std::mutex wrench_mutex_;
  std::mutex trans_map_inv_mutex_;
  std::mutex rot_map_inv_mutex_;

  ros::Publisher flight_cmd_pub_;
  ros::Publisher gimbal_control_pub_;
  ros::Publisher gimbal_state_pub_;
  ros::Publisher target_vectoring_force_pub_;
  ros::Publisher torque_allocation_matrix_inv_pub_;  // for spinal
  ros::Publisher gimbal_dof_pub_;                    // for spinal
  ros::Publisher wrench_allocation_matrix_pub_;      // physical wrench map for differential allocation
  ros::Publisher differential_allocation_config_pub_;
  bool hover_thrust_reference_enabled_ = false;

  boost::shared_ptr<GimbalrotorRobotModel> gimbalrotor_robot_model_;
  std::vector<float> target_base_thrust_;
  std::vector<float> target_full_thrust_;
  std::vector<double> target_gimbal_angles_;
  bool hovering_approximate_;
  Eigen::VectorXd target_vectoring_f_;
  Eigen::VectorXd target_vectoring_f_trans_;
  Eigen::VectorXd target_vectoring_f_rot_;
  Eigen::MatrixXd integrated_map_inv_trans_;
  Eigen::MatrixXd integrated_map_inv_rot_;
  Eigen::MatrixXd physical_integrated_map_;
  double candidate_yaw_term_;
  int gimbal_dof_;
  int rotor_coef_;
  bool underactuate_;
  bool gravity_compensation_;
  double gravity_compensation_acc_ = 0.0;
  double gravity_compensation_transition_rate_;
  double wrench_allocation_matrix_pub_stamp_ = 0.0;
  double wrench_allocation_matrix_pub_interval_ = 0.1;
  int differential_allocation_mode_ = spinal::DifferentialAllocationConfig::STATIC;
  double differential_allocation_damping_ = 0.03;
  double differential_allocation_wrench_error_gain_ = 20.0;
  double differential_allocation_wrench_rate_feedforward_gain_ = 1.0;
  double differential_allocation_wrench_rate_filter_ = 0.15;
  double differential_allocation_force_weight_ = 1.0;
  double differential_allocation_torque_weight_ = 5.0;
  double differential_allocation_thrust_rate_weight_ = 1.0;
  double differential_allocation_servo_rate_weight_ = 5.0;
  double differential_allocation_servo_center_gain_ = 0.2;
  double differential_allocation_thrust_balance_gain_ = 0.1;
  double differential_allocation_max_thrust_rate_ = 20.0;
  double differential_allocation_max_servo_rate_ = 2.0;
  double differential_allocation_max_servo_acceleration_ = 10.0;
  double differential_allocation_min_servo_angle_ = -4.0;
  double differential_allocation_max_servo_angle_ = 4.0;
  bool differential_allocation_servo_angle_limit_enabled_ = true;
  double differential_allocation_thrust_time_constant_ = 0.04;
  double differential_allocation_servo_time_constant_ = 0.05;
  double target_roll_ = 0.0, target_pitch_ = 0.0;

  void sendCmd() override;
  void sendFourAxisCommand();
  void sendGimbalCommand();
  void sendTorqueAllocationMatrixInv();
  void sendDifferentialAllocationData();

protected:
  static double calculateMaxYawScale(const Eigen::MatrixXd& integrated_map_inv_rot);

  bool gimbal_calc_in_fc_;
  bool i_term_rp_calc_in_pc_;
  ros::Publisher rpy_gain_pub_;             // for spinal
  ros::Subscriber send_att_gains_cmd_sub_;  // for spinal
  Eigen::VectorXd additional_wrench_acc_cog_term_;

  virtual void reset() override;
  virtual void setAttitudeGains();
  virtual void rosParamInit();
  virtual void controlCore() override;
  void sendAttGainCmdCallback(const std_msgs::Empty& msg);
  bool update() override;
};
};  // namespace aerial_robot_control
