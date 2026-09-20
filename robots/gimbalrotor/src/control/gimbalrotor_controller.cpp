#include <gimbalrotor/control/gimbalrotor_controller.h>

#include <cmath>

using namespace std;

namespace aerial_robot_control
{
double GimbalrotorController::calculateMaxYawScale(const Eigen::MatrixXd& integrated_map_inv_rot)
{
  if (integrated_map_inv_rot.cols() <= Z)
    return 0.0;

  constexpr double rosserial_scale = 1000.0;
  double max_abs_yaw_scale = 0.0;
  double signed_yaw_scale = 0.0;
  for (Eigen::Index i = 0; i < integrated_map_inv_rot.rows(); ++i)
  {
    const double yaw_scale = integrated_map_inv_rot(i, Z);
    const double scaled_yaw_scale = yaw_scale * rosserial_scale;
    if (!std::isfinite(yaw_scale) || std::abs(scaled_yaw_scale) > INT16_MAX)
      continue;

    /* Match the int16 quantization and row ordering seen by FC exactly. */
    const double quantized_yaw_scale = static_cast<int16_t>(scaled_yaw_scale) / rosserial_scale;
    const double abs_yaw_scale = std::abs(quantized_yaw_scale);
    if (abs_yaw_scale > max_abs_yaw_scale)
    {
      max_abs_yaw_scale = abs_yaw_scale;
      signed_yaw_scale = quantized_yaw_scale;
    }
  }
  return signed_yaw_scale;
}

GimbalrotorController::GimbalrotorController() : PoseLinearController()
{
}

void GimbalrotorController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                       boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                       boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                       boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                       double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  gimbalrotor_robot_model_ = boost::dynamic_pointer_cast<GimbalrotorRobotModel>(robot_model);

  GimbalrotorController::rosParamInit();

  rotor_coef_ = gimbal_dof_ + 1;  // number of virtual rotors in each rotor arm

  target_base_thrust_.resize(motor_num_ * rotor_coef_);
  target_full_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_ * gimbal_dof_, 0);

  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  if (!gimbal_calc_in_fc_)
    gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  gimbal_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  torque_allocation_matrix_inv_pub_ =
      nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  gimbal_dof_pub_ = nh_.advertise<std_msgs::UInt8>("gimbal_dof", 1);
  wrench_allocation_matrix_pub_ =
      nh_.advertise<spinal::WrenchAllocationMatrix>("wrench_allocation_matrix", 1);
  differential_allocation_config_pub_ =
      nh_.advertise<spinal::DifferentialAllocationConfig>("differential_allocation_config", 1, true);
  nh_.param("controller/differential_allocation/hover_thrust_reference_enabled",
            hover_thrust_reference_enabled_, false);
  send_att_gains_cmd_sub_ = nh_.subscribe("send_att_gain_cmd", 1, &GimbalrotorController::sendAttGainCmdCallback, this);
}

void GimbalrotorController::reset()
{
  PoseLinearController::reset();
  gravity_compensation_acc_ = 0.0;

  setAttitudeGains();
}

void GimbalrotorController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<int>(control_nh, "gimbal_dof", gimbal_dof_, 1);
  getParam<bool>(control_nh, "gimbal_calc_in_fc", gimbal_calc_in_fc_, true);
  getParam<bool>(control_nh, "i_term_rp_calc_in_pc", i_term_rp_calc_in_pc_, false);
  getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
  getParam<bool>(control_nh, "underactuate", underactuate_, false);
  getParam<bool>(control_nh, "gravity_compensation", gravity_compensation_, false);
  getParam<double>(control_nh, "gravity_compensation_transition_rate", gravity_compensation_transition_rate_, 2.0);
  gravity_compensation_transition_rate_ = std::max(0.0, gravity_compensation_transition_rate_);

  getParam<double>(control_nh, "wrench_allocation_matrix_pub_interval", wrench_allocation_matrix_pub_interval_, 0.1);
  std::string allocation_mode;
  getParam<std::string>(control_nh, "allocation_mode", allocation_mode, "static");
  if (allocation_mode == "differential")
    differential_allocation_mode_ = spinal::DifferentialAllocationConfig::DIFFERENTIAL;
  else if (allocation_mode == "dld")
    differential_allocation_mode_ = spinal::DifferentialAllocationConfig::DYNAMIC_LIMIT_DECOMPOSITION;
  else
  {
    differential_allocation_mode_ = spinal::DifferentialAllocationConfig::STATIC;
    if (allocation_mode != "static")
      ROS_WARN("[GimbalrotorController] unknown allocation_mode '%s'; use static", allocation_mode.c_str());
  }

  ros::NodeHandle differential_nh(control_nh, "differential_allocation");
  getParam<double>(differential_nh, "damping", differential_allocation_damping_, 0.03);
  getParam<double>(differential_nh, "wrench_error_gain", differential_allocation_wrench_error_gain_, 20.0);
  getParam<double>(differential_nh, "wrench_rate_feedforward_gain",
                   differential_allocation_wrench_rate_feedforward_gain_, 1.0);
  getParam<double>(differential_nh, "wrench_rate_filter", differential_allocation_wrench_rate_filter_, 0.15);
  getParam<double>(differential_nh, "force_weight", differential_allocation_force_weight_, 1.0);
  getParam<double>(differential_nh, "torque_weight", differential_allocation_torque_weight_, 5.0);
  getParam<double>(differential_nh, "thrust_rate_weight", differential_allocation_thrust_rate_weight_, 1.0);
  getParam<double>(differential_nh, "servo_rate_weight", differential_allocation_servo_rate_weight_, 5.0);
  getParam<double>(differential_nh, "servo_center_gain", differential_allocation_servo_center_gain_, 0.2);
  getParam<double>(differential_nh, "thrust_balance_gain", differential_allocation_thrust_balance_gain_, 0.1);
  getParam<double>(differential_nh, "max_thrust_rate", differential_allocation_max_thrust_rate_, 20.0);
  getParam<double>(differential_nh, "max_servo_rate", differential_allocation_max_servo_rate_, 2.0);
  getParam<double>(differential_nh, "max_servo_acceleration",
                   differential_allocation_max_servo_acceleration_, 10.0);
  getParam<double>(differential_nh, "min_servo_angle", differential_allocation_min_servo_angle_, -4.0);
  getParam<double>(differential_nh, "max_servo_angle", differential_allocation_max_servo_angle_, 4.0);
  getParam<bool>(differential_nh, "servo_angle_limit_enabled",
                 differential_allocation_servo_angle_limit_enabled_, true);
  getParam<double>(differential_nh, "thrust_time_constant",
                   differential_allocation_thrust_time_constant_, 0.04);
  getParam<double>(differential_nh, "servo_time_constant",
                   differential_allocation_servo_time_constant_, 0.05);

  differential_allocation_damping_ = std::max(0.0, differential_allocation_damping_);
  differential_allocation_wrench_rate_filter_ =
      std::min(1.0, std::max(0.0, differential_allocation_wrench_rate_filter_));
  differential_allocation_thrust_rate_weight_ = std::max(1e-6, differential_allocation_thrust_rate_weight_);
  differential_allocation_servo_rate_weight_ = std::max(1e-6, differential_allocation_servo_rate_weight_);
  differential_allocation_force_weight_ = std::max(1e-6, differential_allocation_force_weight_);
  differential_allocation_torque_weight_ = std::max(1e-6, differential_allocation_torque_weight_);
  differential_allocation_max_thrust_rate_ = std::max(0.0, differential_allocation_max_thrust_rate_);
  differential_allocation_max_servo_rate_ = std::max(0.0, differential_allocation_max_servo_rate_);
  differential_allocation_max_servo_acceleration_ =
      std::max(0.0, differential_allocation_max_servo_acceleration_);
  differential_allocation_thrust_time_constant_ =
      std::max(1e-3, differential_allocation_thrust_time_constant_);
  differential_allocation_servo_time_constant_ =
      std::max(1e-3, differential_allocation_servo_time_constant_);

  if (gravity_compensation_)
    ROS_INFO("[GimbalrotorController] world-frame gravity feed-forward is enabled");
  if (differential_allocation_mode_ == spinal::DifferentialAllocationConfig::DIFFERENTIAL)
    ROS_INFO("[GimbalrotorController] differential allocation is requested");
  else if (differential_allocation_mode_ ==
           spinal::DifferentialAllocationConfig::DYNAMIC_LIMIT_DECOMPOSITION)
    ROS_INFO("[GimbalrotorController] dynamic-limit decomposition allocation is requested");
}

bool GimbalrotorController::update()
{
  sendGimbalCommand();
  std_msgs::UInt8 msg;
  msg.data = gimbal_calc_in_fc_ ? gimbal_dof_ : 0;
  gimbal_dof_pub_.publish(msg);

  return PoseLinearController::update();
}

void GimbalrotorController::controlCore()
{
  if (!robot_model_->initialized())
  {
    ROS_WARN_THROTTLE(1.0, "[GimbalrotorController] robot model is not initialized yet, skip controlCore");
    return;
  }

  if (gravity_compensation_)
  {
    const uint8_t navi_state = navigator_->getNaviState();
    if (navigator_->getZControlMode() == aerial_robot_navigation::ACC_CONTROL_MODE)
    {
      // ACC mode defines target_acc_z as net inertial acceleration.  Complete
      // the gravity handover immediately so target_acc_z=0 remains hover and
      // no residual Z integral is needed by the contact controller.
      gravity_compensation_acc_ = robot_model_->getGravity()(Z);
      pid_controllers_.at(Z).setErrI(0.0);
    }
    const bool support_vehicle_weight =
        navi_state == aerial_robot_navigation::HOVER_STATE || navi_state == aerial_robot_navigation::LAND_STATE;

    if (support_vehicle_weight)
    {
      // Takeoff gains were tuned with gravity stored in the Z integrator.  Once
      // HOVER is reached, move that support continuously from the I term to an
      // explicit gravity feed-forward without changing the total acceleration
      // command.  Limiting the transfer to the available positive I term also
      // prevents a jump if HOVER is entered before the integrator reaches g.
      const double gravity_acc = robot_model_->getGravity()(Z);
      const double remaining_acc = std::max(0.0, gravity_acc - gravity_compensation_acc_);
      const double requested_transfer =
          std::min(remaining_acc, gravity_compensation_transition_rate_ * ctrl_loop_du_);
      const double i_gain = pid_controllers_.at(Z).getIGain();
      const double available_i_acc = std::max(0.0, pid_controllers_.at(Z).getITerm());
      const double transferred_acc = std::min(requested_transfer, available_i_acc);

      if (i_gain > 0.0 && transferred_acc > 0.0)
      {
        pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getErrI() - transferred_acc / i_gain);
        gravity_compensation_acc_ += transferred_acc;
      }
    }
  }

  PoseLinearController::controlCore();
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  if (gravity_compensation_)
  {
    // The position controller output is an acceleration command.  Add gravity
    // in the world frame before transforming it to the current CoG frame so
    // that a zero PID output corresponds to hover even when the vehicle is
    // tilted.  The translational allocation matrix contains 1 / mass, making
    // this acceleration feed-forward equivalent to an mg force command.
    target_acc_w.setZ(target_acc_w.z() + gravity_compensation_acc_);
  }
  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd target_wrench_acc_cog_for_est = Eigen::VectorXd::Zero(6);

  target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());
  target_wrench_acc_cog_for_est.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  if (underactuate_)
  {
    target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_dash.x(), target_acc_dash.y(), target_acc_dash.z());
    target_wrench_acc_cog_for_est.head(3) =
        Eigen::Vector3d(target_acc_dash.x(), target_acc_dash.y(), target_acc_dash.z());
  }
  else
  {
    target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());
    target_wrench_acc_cog_for_est.head(3) =
        Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());
  }

  double target_ang_acc_x, target_ang_acc_y, target_ang_acc_z;
  if (gimbal_calc_in_fc_ && i_term_rp_calc_in_pc_)
  {
    target_ang_acc_x = pid_controllers_.at(ROLL).getITerm();
    target_ang_acc_y = pid_controllers_.at(PITCH).getITerm();
    target_ang_acc_z = 0;
    target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, 0.0);
  }
  else
  {
    target_ang_acc_x = pid_controllers_.at(ROLL).result();
    target_ang_acc_y = pid_controllers_.at(PITCH).result();
    target_ang_acc_z = pid_controllers_.at(YAW).result();
    target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
  }

  double target_ang_acc_x_for_est = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y_for_est = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z_for_est = pid_controllers_.at(YAW).result();
  target_wrench_acc_cog_for_est.tail(3) =
      Eigen::Vector3d(target_ang_acc_x_for_est, target_ang_acc_y_for_est, target_ang_acc_z_for_est);

  setTargetWrenchAccCog(target_wrench_acc_cog_for_est);

  Eigen::Matrix3d inertia = gimbalrotor_robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::Vector3d omega;
  tf::vectorTFToEigen(omega_, omega);
  Eigen::Vector3d gyro = omega.cross(inertia * omega);

  if (gimbal_calc_in_fc_)
    target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
  else
    target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z) + gyro;

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3 * motor_num_);

  double mass_inv = 1 / gimbalrotor_robot_model_->getMass();

  Eigen::Matrix3d inertia_inv = inertia.inverse();

  std::vector<Eigen::Vector3d> rotors_origin_from_cog =
      gimbalrotor_robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto& rotor_direction = gimbalrotor_robot_model_->getRotorDirection();
  const double m_f_rate = gimbalrotor_robot_model_->getMFRate();

  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
  int last_col = 0;

  /* calculate normal allocation */
  for (int i = 0; i < motor_num_; i++)
  {
    wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i)) +
                                   rotor_direction.at(i + 1) * m_f_rate * Eigen::Matrix3d::Identity();
    full_q_mat.middleCols(last_col, 3) = wrench_map;
    last_col += 3;
  }

  // Preserve the force/torque map before normalizing it by mass and inertia.
  // The flight controller uses this physical map to recover the requested
  // wrench from its final virtual-force command and to form the differential
  // allocation Jacobian.
  const Eigen::MatrixXd physical_full_q_mat = full_q_mat;

  full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);

  /* calculate masked rotation matrix */
  std::vector<KDL::Rotation> thrust_coords_rot = gimbalrotor_robot_model_->getThrustCoordRot<KDL::Rotation>();
  std::vector<Eigen::MatrixXd> masked_rot;
  for (int i = 0; i < motor_num_; i++)
  {
    tf::Quaternion r;
    tf::quaternionKDLToTF(thrust_coords_rot.at(i), r);
    Eigen::Matrix3d conv_cog_from_thrust;
    tf::matrixTFToEigen(tf::Matrix3x3(r), conv_cog_from_thrust);
    if (gimbal_dof_ == 1)
    {
      Eigen::MatrixXd mask(3, 2);
      mask << 0, 0, 1, 0, 0, 1;
      masked_rot.push_back(conv_cog_from_thrust * mask);
    }
    else if (gimbal_dof_ == 2)
    {
      Eigen::MatrixXd mask = Eigen::Matrix3d::Identity();
      masked_rot.push_back(conv_cog_from_thrust * mask);
    }
  }

  /* mask integrated allocation */
  Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * motor_num_, rotor_coef_ * motor_num_);
  Eigen::MatrixXd integrated_map = Eigen::MatrixXd::Zero(6, (gimbal_dof_ + 1) * motor_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    integrated_rot.block(3 * i, rotor_coef_ * i, 3, rotor_coef_) = masked_rot[i];
  }
  if (full_q_mat.cols() != integrated_rot.rows())
  {
    ROS_WARN_THROTTLE(1.0,
                      "[GimbalrotorController] matrix dimension mismatch: full_q_mat(%d x %d) * "
                      "integrated_rot(%d x %d), skip",
                      static_cast<int>(full_q_mat.rows()), static_cast<int>(full_q_mat.cols()),
                      static_cast<int>(integrated_rot.rows()), static_cast<int>(integrated_rot.cols()));
    return;
  }
  integrated_map = full_q_mat * integrated_rot;
  physical_integrated_map_ = physical_full_q_mat * integrated_rot;

  /* extract controlled axis  */
  if (underactuate_)
  {
    target_wrench_acc_cog = target_wrench_acc_cog.tail(4);  // z, roll, pitch, yaw
    integrated_map = integrated_map.bottomRows(4);          // z, roll, pitch, yaw
  }

  /* vectoring force mapping */
  Eigen::MatrixXd integrated_map_inv = aerial_robot_model::pseudoinverse(integrated_map);
  integrated_map_inv_trans_ = integrated_map_inv.leftCols(underactuate_ ? 1 : 3);
  integrated_map_inv_rot_ = integrated_map_inv.rightCols(3);
  if (underactuate_)
    target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog(0);
  else
    target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog.topRows(3);
  target_vectoring_f_rot_ = integrated_map_inv_rot_ * target_wrench_acc_cog.bottomRows(3);  // debug
  target_vectoring_f_ = target_vectoring_f_trans_ + target_vectoring_f_rot_;
  last_col = 0;

  /* under actuated axis  */
  if (underactuate_)
  {
    if (hovering_approximate_)
    {
      target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;
      target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
      navigator_->setTargetRoll(target_roll_);
      navigator_->setTargetPitch(target_pitch_);
    }
    else
    {
      target_roll_ = atan2(-target_acc_dash.y(),
                           sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
      target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
      navigator_->setTargetRoll(target_roll_);
      navigator_->setTargetPitch(target_pitch_);
    }
  }
  else if (gimbal_calc_in_fc_)
  {
    target_roll_ = target_rpy_.x();
    target_pitch_ = target_rpy_.y();
  }
  else
  {
    target_roll_ = navigator_->getTargetRPY().x();
    target_pitch_ = navigator_->getTargetRPY().y();
  }

  /*  calculate target base thrust (considering only translational components)*/
  for (int i = 0; i < motor_num_; i++)
  {
    Eigen::VectorXd f_i;
    if (i_term_rp_calc_in_pc_)
      f_i = target_vectoring_f_.segment(last_col, rotor_coef_);
    else
      f_i = target_vectoring_f_trans_.segment(last_col, rotor_coef_);
    if (gimbal_dof_ == 1)
    {
      target_base_thrust_.at(rotor_coef_ * i) = f_i[0];
      target_base_thrust_.at(rotor_coef_ * i + 1) = f_i[1];
    }
    else if (gimbal_dof_ == 2)
    {
      target_base_thrust_.at(rotor_coef_ * i) = f_i[0];
      target_base_thrust_.at(rotor_coef_ * i + 1) = f_i[1];
      target_base_thrust_.at(rotor_coef_ * i + 2) = f_i[2];
    }
    last_col += rotor_coef_;
  }

  /*
   * FC reconstructs the yaw PI term using the signed yaw gain with the largest
   * magnitude across all virtual rotors.  Keeping the sign lets the scale stay
   * well-conditioned when the strongest gains become negative after inversion.
   */
  const double max_yaw_scale = calculateMaxYawScale(integrated_map_inv_rot_);
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;

  /* calculate target full thrusts and gimbal angles (considering full components)*/
  last_col = 0;
  for (int i = 0; i < motor_num_; i++)
  {
    Eigen::VectorXd f_i_integrated = target_vectoring_f_rot_.segment(last_col, rotor_coef_) +
                                     target_vectoring_f_trans_.segment(last_col, rotor_coef_);
    target_full_thrust_.at(i) = f_i_integrated.norm();
    if (gimbal_dof_ == 1)
    {
      target_gimbal_angles_.at(i) = atan2(-f_i_integrated[0], f_i_integrated[1]);
    }
    else if (gimbal_dof_ == 2)
    {
      if (f_i_integrated[0] == 0 || f_i_integrated[2] == 0)
        continue;

      double gimbal_roll = atan2(-f_i_integrated[1], f_i_integrated[2]);
      double gimbal_pitch =
          atan2(f_i_integrated[0], -f_i_integrated[1] * sin(gimbal_roll) + f_i_integrated[2] * cos(gimbal_roll));
      target_gimbal_angles_.at(2 * i) = gimbal_roll;
      target_gimbal_angles_.at(2 * i + 1) = gimbal_pitch;
    }
    last_col += rotor_coef_;
  }
}

void GimbalrotorController::sendCmd()
{
  PoseLinearController::sendCmd();

  sendFourAxisCommand();

  if (gimbal_calc_in_fc_)
  {
    sendTorqueAllocationMatrixInv();
    sendDifferentialAllocationData();
  }
  else
  {
    sensor_msgs::JointState gimbal_control_msg;
    gimbal_control_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < motor_num_; i++)
    {
      if (gimbal_dof_ == 1)
      {
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
      }
      else if (gimbal_dof_ == 2)
      {
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(2 * i));
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(2 * i + 1));
      }
    }
    gimbal_control_pub_.publish(gimbal_control_msg);

    std_msgs::Float32MultiArray target_vectoring_force_msg;
    target_vectoring_f_ = target_vectoring_f_trans_ + target_vectoring_f_rot_;
    for (int i = 0; i < target_vectoring_f_.size(); i++)
    {
      target_vectoring_force_msg.data.push_back(target_vectoring_f_(i));
    }
    target_vectoring_force_pub_.publish(target_vectoring_force_msg);
  }

  std_msgs::Float32MultiArray target_vectoring_force_msg;
  for (int i = 0; i < target_vectoring_f_.size(); i++)
  {
    target_vectoring_force_msg.data.push_back(target_vectoring_f_(i));
  }
  target_vectoring_force_pub_.publish(target_vectoring_force_msg);
}

void GimbalrotorController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;

  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;

  if (gimbal_calc_in_fc_)
  {
    flight_command_data.base_thrust = target_base_thrust_;
    flight_command_data.angles[2] = candidate_yaw_term_;
  }
  else
  {
    flight_command_data.base_thrust = target_full_thrust_;
  }

  flight_cmd_pub_.publish(flight_command_data);
}

void GimbalrotorController::sendGimbalCommand()
{
  sensor_msgs::JointState gimbal_state_msg;
  gimbal_state_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < motor_num_; i++)
  {
    if (gimbal_dof_ == 1)
    {
      gimbal_state_msg.position.push_back(target_gimbal_angles_.at(i));
      std::string gimbal_name = "gimbal" + std::to_string(i + 1);
      gimbal_state_msg.name.push_back(gimbal_name);
    }
    else if (gimbal_dof_ == 2)
    {
      gimbal_state_msg.position.push_back(target_gimbal_angles_.at(2 * i));
      gimbal_state_msg.position.push_back(target_gimbal_angles_.at(2 * i + 1));
      std::string gimbal_roll_name = "gimbal" + std::to_string(i + 1) + "_roll";
      std::string gimbal_pitch_name = "gimbal" + std::to_string(i + 1) + "_pitch";
      gimbal_state_msg.name.push_back(gimbal_roll_name);
      gimbal_state_msg.name.push_back(gimbal_pitch_name);
    }
  }
  // gimbal_state_pub_.publish(gimbal_state_msg);
}

void GimbalrotorController::sendTorqueAllocationMatrixInv()
{
  spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
  torque_allocation_matrix_inv_msg.rows.resize(motor_num_ * rotor_coef_);
  Eigen::MatrixXd torque_allocation_matrix_inv = integrated_map_inv_rot_;
  if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
    ROS_ERROR("Torque Allocation Matrix overflow");
  for (unsigned int i = 0; i < motor_num_ * rotor_coef_; i++)
  {
    torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i, 0) * 1000;
    torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i, 1) * 1000;
    torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i, 2) * 1000;
  }
  torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
}

void GimbalrotorController::sendDifferentialAllocationData()
{
  const double now = ros::Time::now().toSec();
  if (now - wrench_allocation_matrix_pub_stamp_ < wrench_allocation_matrix_pub_interval_)
    return;
  wrench_allocation_matrix_pub_stamp_ = now;

  if (physical_integrated_map_.rows() != 6 || physical_integrated_map_.cols() != motor_num_ * rotor_coef_)
  {
    ROS_WARN_THROTTLE(1.0, "[GimbalrotorController] physical wrench allocation matrix is not ready");
    return;
  }

  auto scaledInt16 = [](double value) -> int16_t
  {
    const double scaled = std::round(value * 1000.0);
    return static_cast<int16_t>(std::max<double>(INT16_MIN, std::min<double>(INT16_MAX, scaled)));
  };

  spinal::WrenchAllocationMatrix matrix_msg;
  matrix_msg.force_rows.resize(physical_integrated_map_.cols());
  matrix_msg.torque_rows.resize(physical_integrated_map_.cols());
  for (int col = 0; col < physical_integrated_map_.cols(); ++col)
  {
    matrix_msg.force_rows.at(col).x = scaledInt16(physical_integrated_map_(0, col));
    matrix_msg.force_rows.at(col).y = scaledInt16(physical_integrated_map_(1, col));
    matrix_msg.force_rows.at(col).z = scaledInt16(physical_integrated_map_(2, col));
    matrix_msg.torque_rows.at(col).x = scaledInt16(physical_integrated_map_(3, col));
    matrix_msg.torque_rows.at(col).y = scaledInt16(physical_integrated_map_(4, col));
    matrix_msg.torque_rows.at(col).z = scaledInt16(physical_integrated_map_(5, col));
  }
  wrench_allocation_matrix_pub_.publish(matrix_msg);

  spinal::DifferentialAllocationConfig config_msg;
  config_msg.mode = differential_allocation_mode_;
  config_msg.damping = differential_allocation_damping_;
  config_msg.wrench_error_gain = differential_allocation_wrench_error_gain_;
  config_msg.wrench_rate_feedforward_gain = differential_allocation_wrench_rate_feedforward_gain_;
  config_msg.wrench_rate_filter = differential_allocation_wrench_rate_filter_;
  config_msg.force_weight = differential_allocation_force_weight_;
  config_msg.torque_weight = differential_allocation_torque_weight_;
  config_msg.thrust_rate_weight = differential_allocation_thrust_rate_weight_;
  config_msg.servo_rate_weight = differential_allocation_servo_rate_weight_;
  config_msg.servo_center_gain = differential_allocation_servo_center_gain_;
  config_msg.thrust_balance_gain = differential_allocation_thrust_balance_gain_;
  config_msg.max_thrust_rate = differential_allocation_max_thrust_rate_;
  config_msg.max_servo_rate = differential_allocation_max_servo_rate_;
  config_msg.max_servo_acceleration = differential_allocation_max_servo_acceleration_;
  config_msg.min_servo_angle = differential_allocation_min_servo_angle_;
  config_msg.max_servo_angle = differential_allocation_max_servo_angle_;
  config_msg.servo_angle_limit_enabled = differential_allocation_servo_angle_limit_enabled_;
  config_msg.thrust_time_constant = differential_allocation_thrust_time_constant_;
  config_msg.servo_time_constant = differential_allocation_servo_time_constant_;
  config_msg.hover_thrust_reference_enabled = hover_thrust_reference_enabled_;
  double hover_roll, hover_pitch, hover_yaw;
  estimator_->getOrientation(Frame::BASELINK, estimate_mode_).getRPY(hover_roll, hover_pitch, hover_yaw);
  const tf::Vector3 hover_velocity = estimator_->getVel(Frame::COG, estimate_mode_);
  config_msg.hover_thrust_capture_allowed =
      navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE &&
      std::abs(hover_roll) < 0.0872665 && std::abs(hover_pitch) < 0.0872665 &&
      hover_velocity.length() < 0.1;
  differential_allocation_config_pub_.publish(config_msg);
}

void GimbalrotorController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg;  // for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  if (i_term_rp_calc_in_pc_ && gimbal_calc_in_fc_)
  {
    rpy_gain_msg.motors.resize(1);
    rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
    rpy_gain_msg.motors.at(0).roll_i = 0;
    rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_i = 0;
    rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
    rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
    rpy_gain_pub_.publish(rpy_gain_msg);
  }
  else
  {
    rpy_gain_msg.motors.resize(1);
    rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
    rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
    rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
    rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
    rpy_gain_pub_.publish(rpy_gain_msg);
  }
}

void GimbalrotorController::sendAttGainCmdCallback(const std_msgs::Empty& msg)
{
  setAttitudeGains();
}
}  // namespace aerial_robot_control

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorController, aerial_robot_control::ControlBase);
