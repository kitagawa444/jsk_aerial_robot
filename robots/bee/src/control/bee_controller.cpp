#include <bee/control/bee_controller.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <mutex>
#include <bee/BeeCommand.h>
#include <bee/ControllerStatus.h>
#include <bee/ForceFeedback.h>
#include <bee/SetForceControlParameters.h>
#include <bee/ObserverCalibrationStatus.h>
#include <std_msgs/Empty.h>
#include <memory>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <ros/ros.h>
#include <chrono>
#include <array>

namespace aerial_robot_control
{
BeeController::BeeController()
  : GimbalrotorController(),
    bee_controller_initialized_(false),
    pd_wrench_comp_mode_(false),
    prev_wrench_comp_active_(false),
    pre_module_state_(SEPARATED),
    des_wrench_pub_flag_(false)
{
}

void BeeController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                               boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                               boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                               boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                               double ctrl_loop_rate)
{
  GimbalrotorController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  wrench_pid_msg_.x.total.resize(1);
  wrench_pid_msg_.x.p_term.resize(1);
  wrench_pid_msg_.x.i_term.resize(1);
  wrench_pid_msg_.x.d_term.resize(1);
  wrench_pid_msg_.y.total.resize(1);
  wrench_pid_msg_.y.p_term.resize(1);
  wrench_pid_msg_.y.i_term.resize(1);
  wrench_pid_msg_.y.d_term.resize(1);
  wrench_pid_msg_.z.total.resize(1);
  wrench_pid_msg_.z.p_term.resize(1);
  wrench_pid_msg_.z.i_term.resize(1);
  wrench_pid_msg_.z.d_term.resize(1);
  wrench_pid_msg_.roll.total.resize(1);
  wrench_pid_msg_.roll.p_term.resize(1);
  wrench_pid_msg_.roll.i_term.resize(1);
  wrench_pid_msg_.roll.d_term.resize(1);
  wrench_pid_msg_.pitch.total.resize(1);
  wrench_pid_msg_.pitch.p_term.resize(1);
  wrench_pid_msg_.pitch.i_term.resize(1);
  wrench_pid_msg_.pitch.d_term.resize(1);
  wrench_pid_msg_.yaw.total.resize(1);
  wrench_pid_msg_.yaw.p_term.resize(1);
  wrench_pid_msg_.yaw.i_term.resize(1);
  wrench_pid_msg_.yaw.d_term.resize(1);

  bee_robot_model_ = boost::dynamic_pointer_cast<BeeRobotModel>(robot_model);
  bee_navigator_ = boost::dynamic_pointer_cast<aerial_robot_navigation::BeeNavigator>(navigator);
  external_wrench_lower_limit_ = Eigen::VectorXd::Zero(6);
  external_wrench_upper_limit_ = Eigen::VectorXd::Zero(6);
  rosParamInit();

  if(pd_wrench_comp_mode_) ROS_ERROR("PD & Wrench comp mode");
  external_wrench_compensation_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("external_wrench_compensation", 1);
  tagged_external_wrench_pub_ = nh_.advertise<beetle::TaggedWrench>("tagged_wrench", 1);
  whole_external_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("whole_wrench", 1);
  internal_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("internal_wrench", 1);
  wrench_comp_pid_pub_ = nh_.advertise<aerial_robot_msgs::PoseControlPid>("debug/wrench_comp/pid", 1);
  des_inter_wrench_pub_ = nh_.advertise<beetle::TaggedWrenches>("des_inter_wnrech", 1);

  int max_modules_num = bee_navigator_->getMaxModuleNum();
  for(int i = 0; i < max_modules_num; i++)
    {
      std::string module_name = std::string("/") + bee_navigator_->getMyName() + std::to_string(i + 1);
      est_wrench_subs_.insert(std::make_pair(module_name, nh_.subscribe(module_name + std::string("/tagged_wrench"), 1, &BeeController::estExternalWrenchCallback, this)));
      Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
      est_wrench_list_.insert(std::make_pair(i + 1, wrench));
      inter_wrench_list_.insert(std::make_pair(i + 1, wrench));
      wrench_comp_list_.insert(std::make_pair(i + 1, wrench));
      ff_inter_wrench_list_.insert(std::make_pair(i + 1, wrench));
      ff_inter_wrench_subs_.insert(std::make_pair(module_name, nh_.subscribe(module_name + std::string("/ff_inter_wrench"), 1, &BeeController::ffInterWrenchCallback, this)));
    }

  pid_controllers_.push_back(PID("f_x", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
  pid_controllers_.push_back(PID("f_y", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
  pid_controllers_.push_back(PID("f_z", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
  pid_controllers_.push_back(PID("t_x", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
  pid_controllers_.push_back(PID("t_y", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
  pid_controllers_.push_back(PID("t_z", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle wrench_nh(control_nh, "wrench_comp");
  std::vector<int> indices = {FX, FY, FZ, TX, TY, TZ};
  pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(wrench_nh));
  pid_reconf_servers_.back()->setCallback(boost::bind(&BeeController::cfgPidCallback, this, _1, _2, indices));

  prev_comp_update_time_ = -1;
  bool reference_enabled = true;
  nh_.param("controller/reference_enabled", reference_enabled, true);
  // getMass() is not initialized until the first model/joint-state update.
  // Derive startup mass from the same robot_description instead of a constant.
  std::string model_xml;
  if (!nh_.getParam("robot_description", model_xml))
    throw std::invalid_argument("Bee reference tracking requires robot_description");
  reference_tracker_.reset(new bee::control::ReferenceTracker(
      nh_, ros::NodeHandle(nh_, "controller/reference_tracking"), bee::control::modelMass(model_xml), reference_enabled));
  // PoseLinearController starts the observer thread and ros_control may start
  // calling update() before the most-derived initialize() has returned.  Only
  // publish this flag after all Gimbalrotor/Bee vectors and publishers exist.
  bee_controller_initialized_.store(true, std::memory_order_release);
}

bool BeeController::update()
{
  if(!bee_controller_initialized_.load(std::memory_order_acquire))
    {
      ROS_WARN_THROTTLE(1.0,
                        "[BeeController] initialization is incomplete, skip update");
      return false;
    }
  aerial_robot_msgs::FlightNav command;
  geometry_msgs::Vector3Stamped attitude;
  bool release = false;
  if(reference_tracker_ && reference_tracker_->update(command, attitude, release,
                                               bee_navigator_->getModuleState() == SEPARATED))
    bee_navigator_->applyBeeCommand(command, attitude, release);
  return GimbalrotorController::update();
}

void BeeController::controlCore()
{
  // BeeController performs inertia and interaction-wrench calculations before
  // delegating to GimbalrotorController.  The latter has its own model-ready
  // guard, but it is too late to protect the calculations below.  With several
  // vehicles starting concurrently, the control timer can run while the first
  // robot-model update is still building its matrices.
  if(!bee_robot_model_ || !bee_robot_model_->initialized())
    {
      ROS_WARN_THROTTLE(1.0,
                        "[BeeController] robot model is not initialized yet, skip controlCore");
      return;
    }

  std::map<int, bool> assembly_flag = bee_navigator_->getAssemblyFlags();
  int max_modules_num = bee_navigator_->getMaxModuleNum();
  int module_state = bee_navigator_->getModuleState();
  if(bee_navigator_->getControlFlag() &&
     module_state != SEPARATED)
    {
      calcInteractionWrench();
    }
  else
    {
      for(int i = 0; i < max_modules_num; i++)
        {
          est_wrench_list_[i + 1] = Eigen::VectorXd::Zero(6);
          inter_wrench_list_[i + 1] = Eigen::VectorXd::Zero(6);
          wrench_comp_list_[i + 1] = Eigen::VectorXd::Zero(6);
        }
    }

  double mass_inv = 1 / bee_robot_model_->getMass();
  Eigen::Matrix3d inertia_inv = (bee_robot_model_->getInertia<Eigen::Matrix3d>()).inverse();
  int my_id = bee_navigator_->getMyID();
  bool wrench_comp_active = module_state == FOLLOWER &&
                            pd_wrench_comp_mode_ &&
                            bee_navigator_->getControlFlag() &&
                            !bee_navigator_->pseudo_assembly_mode_;

  if(wrench_comp_active)
    {
      int module_num = 0;
      for(const auto& item : est_wrench_list_)
        {
          if(assembly_flag[item.first])
            {
              module_num++;
            }
        }

      std::vector<int> wrench_indices = {FX, FY, FZ, TX, TY, TZ};
      for(const auto& index : wrench_indices)
        {
          pid_controllers_.at(index).setPGain(wrench_comp_p_gain_ / std::pow(2, module_num - 2));
          pid_controllers_.at(index).setDGain(wrench_comp_d_gain_ / std::pow(2, module_num - 2));
          pid_controllers_.at(index).setIGain(wrench_comp_i_gain_ / std::pow(2, module_num - 2));
        }

      Eigen::VectorXd wrench_comp_term_cog = wrench_comp_list_[my_id];
      Eigen::Matrix3d cog_rot;
      tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);
      Eigen::VectorXd wrench_comp_term = wrench_comp_term_cog;
      wrench_comp_term.head(3) = cog_rot * wrench_comp_term.head(3);

      Eigen::VectorXd I_reconfig_acc_cog_term = Eigen::VectorXd::Zero(6);
      I_reconfig_acc_cog_term.head(3) = mass_inv * wrench_comp_term.head(3);
      I_reconfig_acc_cog_term.tail(3) = inertia_inv * wrench_comp_term.tail(3);

      double IGain_Fx = pid_controllers_.at(X).getIGain();
      double IGain_Fy = pid_controllers_.at(Y).getIGain();
      double IGain_Fz = pid_controllers_.at(Z).getIGain();
      double IGain_Tx = pid_controllers_.at(ROLL).getIGain();
      double IGain_Ty = pid_controllers_.at(PITCH).getIGain();
      double IGain_Tz = pid_controllers_.at(YAW).getIGain();

      double du;
      if(prev_comp_update_time_ < 0)
        {
          prev_comp_update_time_ = ros::Time::now().toSec();
          return;
        }
      else
        {
          du = ros::Time::now().toSec() - prev_comp_update_time_;
          prev_comp_update_time_ = ros::Time::now().toSec();
        }

      pid_controllers_.at(FX).update(I_reconfig_acc_cog_term(0) / IGain_Fx, du, 0);
      pid_controllers_.at(FY).update(I_reconfig_acc_cog_term(1) / IGain_Fy, du, 0);
      pid_controllers_.at(FZ).update(I_reconfig_acc_cog_term(2) / IGain_Fz, du, 0);
      pid_controllers_.at(TX).update(I_reconfig_acc_cog_term(3) / IGain_Tx, du, 0);
      pid_controllers_.at(TY).update(I_reconfig_acc_cog_term(4) / IGain_Ty, du, 0);
      pid_controllers_.at(TZ).update(I_reconfig_acc_cog_term(5) / IGain_Tz, du, 0);

      I_comp_Fx_ = pid_controllers_.at(FX).result();
      I_comp_Fy_ = pid_controllers_.at(FY).result();
      I_comp_Fz_ = pid_controllers_.at(FZ).result();
      I_comp_Tx_ = pid_controllers_.at(TX).result();
      I_comp_Ty_ = pid_controllers_.at(TY).result();
      I_comp_Tz_ = pid_controllers_.at(TZ).result();

      pid_controllers_.at(X).setErrI(I_comp_Fx_);
      pid_controllers_.at(Y).setErrI(I_comp_Fy_);
      pid_controllers_.at(Z).setErrI(I_comp_Fz_);
      pid_controllers_.at(ROLL).setErrI(I_comp_Tx_);
      pid_controllers_.at(PITCH).setErrI(I_comp_Ty_);
      pid_controllers_.at(YAW).setErrI(I_comp_Tz_);

      geometry_msgs::WrenchStamped wrench_msg;
      wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
      wrench_msg.wrench.force.x = I_reconfig_acc_cog_term(0);
      wrench_msg.wrench.force.y = I_reconfig_acc_cog_term(1);
      wrench_msg.wrench.force.z = I_reconfig_acc_cog_term(2);
      wrench_msg.wrench.torque.x = I_reconfig_acc_cog_term(3);
      wrench_msg.wrench.torque.y = I_reconfig_acc_cog_term(4);
      wrench_msg.wrench.torque.z = I_reconfig_acc_cog_term(5);
      external_wrench_compensation_pub_.publish(wrench_msg);

      wrench_pid_msg_.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
      wrench_pid_msg_.x.total.at(0) = pid_controllers_.at(FX).result();
      wrench_pid_msg_.x.p_term.at(0) = pid_controllers_.at(FX).getPTerm();
      wrench_pid_msg_.x.i_term.at(0) = pid_controllers_.at(FX).getITerm();
      wrench_pid_msg_.x.d_term.at(0) = pid_controllers_.at(FX).getDTerm();

      wrench_pid_msg_.y.total.at(0) = pid_controllers_.at(FY).result();
      wrench_pid_msg_.y.p_term.at(0) = pid_controllers_.at(FY).getPTerm();
      wrench_pid_msg_.y.i_term.at(0) = pid_controllers_.at(FY).getITerm();
      wrench_pid_msg_.y.d_term.at(0) = pid_controllers_.at(FY).getDTerm();

      wrench_pid_msg_.z.total.at(0) = pid_controllers_.at(FZ).result();
      wrench_pid_msg_.z.p_term.at(0) = pid_controllers_.at(FZ).getPTerm();
      wrench_pid_msg_.z.i_term.at(0) = pid_controllers_.at(FZ).getITerm();
      wrench_pid_msg_.z.d_term.at(0) = pid_controllers_.at(FZ).getDTerm();

      wrench_pid_msg_.roll.total.at(0) = pid_controllers_.at(TX).result();
      wrench_pid_msg_.roll.p_term.at(0) = pid_controllers_.at(TX).getPTerm();
      wrench_pid_msg_.roll.i_term.at(0) = pid_controllers_.at(TX).getITerm();
      wrench_pid_msg_.roll.d_term.at(0) = pid_controllers_.at(TX).getDTerm();

      wrench_pid_msg_.pitch.total.at(0) = pid_controllers_.at(TY).result();
      wrench_pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(TY).getPTerm();
      wrench_pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(TY).getITerm();
      wrench_pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(TY).getDTerm();

      wrench_pid_msg_.yaw.total.at(0) = pid_controllers_.at(TZ).result();
      wrench_pid_msg_.yaw.p_term.at(0) = pid_controllers_.at(TZ).getPTerm();
      wrench_pid_msg_.yaw.i_term.at(0) = pid_controllers_.at(TZ).getITerm();
      wrench_pid_msg_.yaw.d_term.at(0) = pid_controllers_.at(TZ).getDTerm();

      wrench_comp_pid_pub_.publish(wrench_pid_msg_);

      if(des_wrench_pub_flag_)
        {
          beetle::TaggedWrenches all_tagged_des_wrenche_msg;
          std::vector<int> assembled_ids = bee_navigator_->getAssemblyIds();
          all_tagged_des_wrenche_msg.tagged_wrenches.resize(assembled_ids.size());
          int cnt = 0;
          for(const auto id : assembled_ids)
            {
              beetle::TaggedWrench tagged_des_wrench_msg;
              geometry_msgs::WrenchStamped des_wrench_msg;
              Eigen::VectorXd des_wrench = ff_inter_wrench_list_[id];
              des_wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
              des_wrench_msg.wrench.force.x = des_wrench(0);
              des_wrench_msg.wrench.force.y = des_wrench(1);
              des_wrench_msg.wrench.force.z = des_wrench(2);
              des_wrench_msg.wrench.torque.x = des_wrench(3);
              des_wrench_msg.wrench.torque.y = des_wrench(4);
              des_wrench_msg.wrench.torque.z = des_wrench(5);

              tagged_des_wrench_msg.index = id;
              tagged_des_wrench_msg.wrench = des_wrench_msg;
              all_tagged_des_wrenche_msg.tagged_wrenches[cnt] = tagged_des_wrench_msg;
              cnt++;
            }
          des_inter_wrench_pub_.publish(all_tagged_des_wrenche_msg);
        }
    }
  else
    {
      pid_controllers_.at(FX).reset();
      pid_controllers_.at(FY).reset();
      pid_controllers_.at(FZ).reset();
      pid_controllers_.at(TX).reset();
      pid_controllers_.at(TY).reset();
      pid_controllers_.at(TZ).reset();

      if(prev_wrench_comp_active_)
        {
          pid_controllers_.at(X).setErrI(0.0);
          pid_controllers_.at(Y).setErrI(0.0);
          pid_controllers_.at(Z).setErrI(0.0);
          pid_controllers_.at(ROLL).setErrI(0.0);
          pid_controllers_.at(PITCH).setErrI(0.0);
          pid_controllers_.at(YAW).setErrI(0.0);
        }
    }

  GimbalrotorController::controlCore();
  prev_wrench_comp_active_ = wrench_comp_active;
  pre_module_state_ = module_state;
}

void BeeController::reset()
{
  GimbalrotorController::reset();
  pid_controllers_.at(FX).reset();
  pid_controllers_.at(FY).reset();
  pid_controllers_.at(FZ).reset();
  pid_controllers_.at(TX).reset();
  pid_controllers_.at(TY).reset();
  pid_controllers_.at(TZ).reset();
  pid_controllers_.at(X).setErrI(0.0);
  pid_controllers_.at(Y).setErrI(0.0);
  pid_controllers_.at(Z).setErrI(0.0);
  pid_controllers_.at(ROLL).setErrI(0.0);
  pid_controllers_.at(PITCH).setErrI(0.0);
  pid_controllers_.at(YAW).setErrI(0.0);
}

void BeeController::calcInteractionWrench()
{
  Eigen::VectorXd W_w = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd W_sum = Eigen::VectorXd::Zero(6);
  int module_num = 0;
  std::map<int, bool> assembly_flag = bee_navigator_->getAssemblyFlags();

  for(const auto& item : est_wrench_list_)
    {
      if(assembly_flag[item.first])
        {
          W_sum += item.second;
          module_num++;
        }
    }

  if(!module_num) return;

  W_w = W_sum / module_num;
  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
  wrench_msg.wrench.force.x = W_w(0);
  wrench_msg.wrench.force.y = W_w(1);
  wrench_msg.wrench.force.z = W_w(2);
  wrench_msg.wrench.torque.x = W_w(3);
  wrench_msg.wrench.torque.y = W_w(4);
  wrench_msg.wrench.torque.z = W_w(5);
  whole_external_wrench_pub_.publish(wrench_msg);

  Eigen::VectorXd left_inter_wrench = Eigen::VectorXd::Zero(6);
  for(const auto& item : est_wrench_list_)
    {
      if(assembly_flag[item.first])
        {
          Eigen::VectorXd right_inter_wrench = item.second - W_w + left_inter_wrench;
          inter_wrench_list_[item.first] = right_inter_wrench;
          left_inter_wrench = right_inter_wrench;
        }
      else
        {
          inter_wrench_list_[item.first] = Eigen::VectorXd::Zero(6);
        }
    }

  int my_id = bee_navigator_->getMyID();
  wrench_msg.wrench.force.x = inter_wrench_list_[my_id](0);
  wrench_msg.wrench.force.y = inter_wrench_list_[my_id](1);
  wrench_msg.wrench.force.z = inter_wrench_list_[my_id](2);
  wrench_msg.wrench.torque.x = inter_wrench_list_[my_id](3);
  wrench_msg.wrench.torque.y = inter_wrench_list_[my_id](4);
  wrench_msg.wrench.torque.z = inter_wrench_list_[my_id](5);
  internal_wrench_pub_.publish(wrench_msg);

  int leader_id = bee_navigator_->getLeaderID();
  Eigen::VectorXd wrench_comp_sum_left = Eigen::VectorXd::Zero(6);
  for(int i = leader_id - 1; i > 0; i--)
    {
      if(assembly_flag[i])
        {
          wrench_comp_sum_left += -ff_inter_wrench_list_[i] + inter_wrench_list_[i];
          wrench_comp_list_[i] = wrench_comp_sum_left;
        }
      else
        {
          wrench_comp_list_[i] = Eigen::VectorXd::Zero(6);
        }
    }

  int max_modules_num = bee_navigator_->getMaxModuleNum();
  int left_module_id = leader_id;
  Eigen::VectorXd wrench_comp_sum_right = Eigen::VectorXd::Zero(6);
  for(int i = leader_id + 1; i <= max_modules_num; i++)
    {
      if(assembly_flag[i])
        {
          wrench_comp_sum_right += ff_inter_wrench_list_[left_module_id] - inter_wrench_list_[left_module_id];
          wrench_comp_list_[i] = wrench_comp_sum_right;
          left_module_id = i;
        }
      else
        {
          wrench_comp_list_[i] = Eigen::VectorXd::Zero(6);
        }
    }
}

void BeeController::rosParamInit()
{
  GimbalrotorController::rosParamInit();
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "pd_wrench_comp_mode", pd_wrench_comp_mode_, false);

  double external_force_upper_limit, external_force_lower_limit, external_torque_upper_limit, external_torque_lower_limit;
  getParam<double>(control_nh, "external_force_upper_limit", external_force_upper_limit, 0.5);
  getParam<double>(control_nh, "external_force_lower_limit", external_force_lower_limit, -0.5);
  getParam<double>(control_nh, "external_torque_upper_limit", external_torque_upper_limit, 0.01);
  getParam<double>(control_nh, "external_torque_lower_limit", external_torque_lower_limit, -0.01);
  external_wrench_upper_limit_.head(3) = Eigen::Vector3d::Constant(external_force_upper_limit);
  external_wrench_upper_limit_.tail(3) = Eigen::Vector3d::Constant(external_torque_upper_limit);
  external_wrench_lower_limit_.head(3) = Eigen::Vector3d::Constant(external_force_lower_limit);
  external_wrench_lower_limit_.tail(3) = Eigen::Vector3d::Constant(external_torque_lower_limit);
  ROS_INFO_STREAM("upper limit of external wrench : " << external_wrench_upper_limit_.transpose());
  ROS_INFO_STREAM("lower limit of external wrench : " << external_wrench_lower_limit_.transpose());

  getParam<double>(control_nh, "comp_term_update_freq", comp_term_update_freq_, 10);

  ros::NodeHandle wrench_nh(control_nh, "wrench_comp");
  getParam<double>(wrench_nh, "p_gain", wrench_comp_p_gain_, 0.1);
  getParam<double>(wrench_nh, "i_gain", wrench_comp_i_gain_, 0.005);
  getParam<double>(wrench_nh, "d_gain", wrench_comp_d_gain_, 0.07);
}

void BeeController::externalWrenchEstimate()
{
  if(!bee_controller_initialized_.load(std::memory_order_acquire))
    {
      return;
    }

  // Use the actual acceleration/wrench target handed to the allocator.  XY
  // ACC_MODE is a direct feed-forward path and deliberately resets the XY PID
  // controllers, so reconstructing the target from PID::result() makes the
  // observer treat the commanded acceleration itself as an external force.
  const Eigen::VectorXd target_wrench_acc_cog = getTargetWrenchAccCog();

  // The estimator runs in a thread created by PoseLinearController during
  // controller initialization.  Do not read inertia/model data until the first
  // model update has completed, and accept only the six-axis target produced by
  // GimbalrotorController::controlCore().
  if(!robot_model_ || !robot_model_->initialized())
    {
      prev_est_wrench_timestamp_ = 0;
      integrate_term_ = Eigen::VectorXd::Zero(6);
      return;
    }

  if(navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE &&
     navigator_->getNaviState() != aerial_robot_navigation::TAKEOFF_STATE &&
     navigator_->getNaviState() != aerial_robot_navigation::LAND_STATE)
    {
      prev_est_wrench_timestamp_ = 0;
      integrate_term_ = Eigen::VectorXd::Zero(6);
      return;
    }
  else if(target_wrench_acc_cog.size() != 6)
    {
      ROS_WARN_THROTTLE(1.0,
                        "[BeeController] six-axis target wrench is not ready; skip estimation");
      prev_est_wrench_timestamp_ = 0;
      integrate_term_ = Eigen::VectorXd::Zero(6);
      return;
    }

  Eigen::Vector3d vel_w, omega_cog;
  tf::Vector3 vel_tf = estimator_->getVel(Frame::COG, estimate_mode_);
  tf::Vector3 omega_tf = estimator_->getAngularVel(Frame::COG, estimate_mode_);
  tf::vectorTFToEigen(vel_tf, vel_w);
  tf::vectorTFToEigen(omega_tf, omega_cog);
  Eigen::Matrix3d cog_rot;
  tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);

  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  double mass = robot_model_->getMass();

  Eigen::VectorXd sum_momentum = Eigen::VectorXd::Zero(6);
  sum_momentum.head(3) = mass * vel_w;
  sum_momentum.tail(3) = inertia * omega_cog;

  Eigen::VectorXd target_wrench_cog = Eigen::VectorXd::Zero(6);
  target_wrench_cog.head(3) = mass * target_wrench_acc_cog.head(3);
  target_wrench_cog.tail(3) = inertia * target_wrench_acc_cog.tail(3);

  Eigen::MatrixXd J_t = Eigen::MatrixXd::Identity(6, 6);
  J_t.topLeftCorner(3, 3) = cog_rot;

  Eigen::VectorXd N = mass * robot_model_->getGravity();
  N.tail(3) = aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);

  if(prev_est_wrench_timestamp_ == 0)
    {
      prev_est_wrench_timestamp_ = ros::Time::now().toSec();
      init_sum_momentum_ = sum_momentum;
    }

  double dt = ros::Time::now().toSec() - prev_est_wrench_timestamp_;
  integrate_term_ += (J_t * target_wrench_cog - N + est_external_wrench_) * dt;
  est_external_wrench_ = momentum_observer_matrix_ * (sum_momentum - init_sum_momentum_ - integrate_term_);

  Eigen::VectorXd est_external_wrench_cog = est_external_wrench_;
  est_external_wrench_cog.head(3) = cog_rot.inverse() * est_external_wrench_.head(3);

  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
  wrench_msg.header.frame_id = bee_navigator_->getMyName() +
    std::to_string(bee_navigator_->getMyID()) + "/cog";
  wrench_msg.wrench.force.x = est_external_wrench_cog(0);
  wrench_msg.wrench.force.y = est_external_wrench_cog(1);
  wrench_msg.wrench.force.z = est_external_wrench_cog(2);
  wrench_msg.wrench.torque.x = est_external_wrench_cog(3);
  wrench_msg.wrench.torque.y = est_external_wrench_cog(4);
  wrench_msg.wrench.torque.z = est_external_wrench_cog(5);
  estimate_external_wrench_pub_.publish(wrench_msg);

  // The base class starts the estimator thread before BeeController finishes
  // advertising Bee-specific topics.  The primary estimate publisher is
  // already ready; defer only the tagged copy until its publisher exists.
  if(tagged_external_wrench_pub_)
    {
      beetle::TaggedWrench tagged_wrench;
      tagged_wrench.index = bee_navigator_->getMyID();
      tagged_wrench.wrench = wrench_msg;
      tagged_external_wrench_pub_.publish(tagged_wrench);
    }

  prev_est_wrench_timestamp_ = ros::Time::now().toSec();
}

void BeeController::estExternalWrenchCallback(const beetle::TaggedWrench& msg)
{
  int id = msg.index;
  geometry_msgs::Wrench wrench_msg = msg.wrench.wrench;
  Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
  wrench(0) = wrench_msg.force.x;
  wrench(1) = wrench_msg.force.y;
  wrench(2) = wrench_msg.force.z;
  wrench(3) = wrench_msg.torque.x;
  wrench(4) = wrench_msg.torque.y;
  wrench(5) = wrench_msg.torque.z;
  est_wrench_list_[id] = wrench;
}

void BeeController::ffInterWrenchCallback(const beetle::TaggedWrench& msg)
{
  int id = msg.index;
  geometry_msgs::Wrench wrench_msg = msg.wrench.wrench;
  Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
  wrench(0) = wrench_msg.force.x;
  wrench(1) = wrench_msg.force.y;
  wrench(2) = wrench_msg.force.z;
  wrench(3) = wrench_msg.torque.x;
  wrench(4) = wrench_msg.torque.y;
  wrench(5) = wrench_msg.torque.z;
  ff_inter_wrench_list_[id] = wrench;
}

}  // namespace aerial_robot_control

namespace bee::control
{
void ForceConfig::validate() const
{
  for (double x : {mass, gravity, thrust_limit})
    if (!std::isfinite(x) || x <= 0) throw std::invalid_argument("invalid force-control limit");
  if (!kp.allFinite() || kp.minCoeff() < 0 || correction_reserve >= thrust_limit)
    throw std::invalid_argument("invalid force gain or thrust reserve");
  for (double x : {ki, damping, filter_tau, correction_reserve, integral_limit})
    if (!std::isfinite(x) || x < 0) throw std::invalid_argument("invalid force-control gain");
}

Vec3 limitThrust(const Vec3& acceleration, const ForceConfig& p)
{
  const Vec3 gravity(0, 0, -p.gravity);
  Vec3 thrust = p.mass * (acceleration - gravity);
  if (thrust.lpNorm<1>() > p.thrust_limit) thrust *= p.thrust_limit / thrust.lpNorm<1>();
  return gravity + thrust / p.mass;
}
// Observer reports force ON the Bee; target is force BY the Bee. FF and
// relative-velocity damping bypass the optional force-PI output low-pass filter.
Vec3 forceTrackingAcceleration(const Vec3& own_acceleration, const Vec3& velocity_error,
    const Vec3& target, const Vec3& external, const Eigen::Matrix3d& axes,
    const Vec3& lower, const Vec3& upper, bool bounded,
    ForceTrackingState& state, double dt, const ForceConfig& p, bool integrate)
{
  if (!integrate) state.integral.setZero();
  const Vec3 error = target + external;
  Vec3 candidate = state.integral + (integrate ? p.ki * dt : 0) * error;
  candidate = candidate.cwiseMax(-p.integral_limit).cwiseMin(p.integral_limit);
  state.unfiltered = axes * p.kp.asDiagonal() * axes.transpose() * error + candidate;
  if (p.filter_tau > 0)
    state.filtered += dt / (p.filter_tau + dt) * (state.unfiltered - state.filtered);
  else state.filtered = state.unfiltered;
  Vec3 correction = state.filtered + p.damping * velocity_error;
  const double force = p.mass * correction.lpNorm<1>();
  state.correction_saturated = force > p.correction_reserve;
  if (state.correction_saturated) correction *= p.correction_reserve / force;
  const Vec3 raw = own_acceleration + target / p.mass + correction;
  Vec3 acceleration = raw;
  if (bounded) acceleration = axes * (axes.transpose() * raw).cwiseMax(lower).cwiseMin(upper);
  acceleration = limitThrust(acceleration, p);
  state.acceleration_limited = (raw - acceleration).norm() >= 1e-9;
  if (!state.correction_saturated && !state.acceleration_limited) state.integral = candidate;
  return acceleration;
}

namespace
{
struct Wrench { Vec3 force = Vec3::Zero(), moment = Vec3::Zero(); };
struct Pose { Vec3 position = Vec3::Zero(); Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity(); };

// Owned by BeeController's generic reference tracker. The ROS callback only
// queues a request; update/applied are called under the owner's control-loop lock.
class ObserverCalibration
{
public:
  ObserverCalibration(ros::NodeHandle nh, ros::NodeHandle parameters,
                      const std::string& cog_frame, const std::string& world_frame)
  {
    parameters.param("calibration_duration", duration_, 2.0);
    parameters.param("calibration_min_samples", min_samples_, 20);
    if (!std::isfinite(duration_) || duration_ <= 0 || min_samples_ < 2)
      throw std::invalid_argument("invalid observer calibration duration/sample count");
    status_.header.frame_id = cog_frame;
    status_.world_frame = world_frame;
    pub_ = nh.advertise<bee::ObserverCalibrationStatus>("controller/observer_calibration", 1, true);
    sub_ = nh.subscribe<std_msgs::Empty>("controller/calibrate_observer", 1,
        [this](const std_msgs::Empty::ConstPtr&) { requested_.store(true); });
    publish(bee::ObserverCalibrationStatus::IDLE, "waiting for calibration request");
  }

  // Returns true exactly once when a manual collection succeeds. No navigation,
  // mode change, arming, flight-state or contact-state requirement is introduced here.
  bool update(double wall, double now, double timeout, double wrench_receipt, double wrench_stamp, const Wrench& raw,
              double cog_receipt, double cog_stamp, const Eigen::Quaterniond& orientation,
              Wrench& baseline, Vec3& world_baseline)
  {
    const auto fresh = [&](double receipt, double stamp) {
      return receipt > 0 && wall - receipt <= timeout && stamp > 0 &&
             now - stamp <= timeout && stamp - now <= .05;
    };
    const bool valid = fresh(wrench_receipt, wrench_stamp) && fresh(cog_receipt, cog_stamp) &&
        std::abs(wrench_stamp - cog_stamp) <= timeout && raw.force.allFinite() &&
        raw.moment.allFinite() && orientation.coeffs().allFinite() &&
        std::abs(orientation.squaredNorm() - 1) < .05;
    if (requested_.exchange(false))
    {
      if (collecting_)
        publish(bee::ObserverCalibrationStatus::REJECTED, "calibration already collecting; original request continues");
      else if (!valid)
        publish(bee::ObserverCalibrationStatus::REJECTED, "fresh cog odometry and observer wrench required");
      else
      {
        collecting_ = true; start_wall_ = wall; start_ros_ = now;
        last_stamp_ = wrench_stamp; first_stamp_ = 0;
        sum_ = Wrench{}; world_sum_.setZero(); samples_ = 0;
        status_.samples = 0;
        publish(bee::ObserverCalibrationStatus::COLLECTING, "collecting observer zero-reference samples");
      }
    }
    if (!collecting_) return false;
    if (!valid || now < start_ros_ || wrench_stamp < last_stamp_ ||
        wall - start_wall_ > std::max(5 * duration_, duration_ + timeout))
    {
      collecting_ = false;
      publish(bee::ObserverCalibrationStatus::FAILED,
              "calibration input stale, invalid or clock moved backwards");
      return false;
    }
    if (wrench_stamp > last_stamp_)
    {
      if (!samples_) first_stamp_ = wrench_stamp;
      sum_.force += raw.force; sum_.moment += raw.moment;
      world_sum_ += orientation.normalized() * raw.force;
      last_stamp_ = wrench_stamp; ++samples_;
      status_.samples = samples_;
    }
    if (now - start_ros_ < duration_) return false;
    collecting_ = false;
    if (samples_ < min_samples_ || last_stamp_ - first_stamp_ < duration_ - timeout)
    {
      publish(bee::ObserverCalibrationStatus::FAILED, "insufficient distinct samples or source-time coverage");
      return false;
    }
    baseline.force = sum_.force / samples_;
    baseline.moment = sum_.moment / samples_;
    world_baseline = world_sum_ / samples_;
    if (!baseline.force.allFinite() || !baseline.moment.allFinite() || !world_baseline.allFinite())
    {
      publish(bee::ObserverCalibrationStatus::FAILED, "nonfinite calibration average");
      return false;
    }
    // The owner commits first, then calls applied() to publish the shared result.
    return true;
  }

  void applied(const Wrench& baseline, const Vec3& world_baseline, unsigned samples,
               const std::string& detail)
  {
    collecting_ = false;
    status_.calibration_stamp = ros::Time::now();
    status_.calibration_start_stamp = ros::Time(start_ros_);
    status_.baseline_cog.force = message(baseline.force);
    status_.baseline_cog.torque = message(baseline.moment);
    status_.baseline_world_force = message(world_baseline);
    status_.samples = samples;
    publish(bee::ObserverCalibrationStatus::SUCCEEDED, detail);
  }
  unsigned samples() const { return samples_; }

private:
  static geometry_msgs::Vector3 message(const Vec3& v)
  {
    geometry_msgs::Vector3 m; m.x = v.x(); m.y = v.y(); m.z = v.z(); return m;
  }
  void publish(uint8_t state, const std::string& detail)
  {
    status_.header.stamp = ros::Time::now(); status_.state = state; status_.detail = detail;
    pub_.publish(status_);
  }
  ros::Publisher pub_;
  ros::Subscriber sub_;
  bee::ObserverCalibrationStatus status_;
  std::atomic<bool> requested_{false};
  bool collecting_ = false;
  double duration_, start_wall_ = 0, start_ros_ = 0, first_stamp_ = 0, last_stamp_ = 0;
  int min_samples_;
  unsigned samples_ = 0;
  Wrench sum_;
  Vec3 world_sum_ = Vec3::Zero();
};

double steadyNow()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
Vec3 vec(const geometry_msgs::Vector3& v) { return Vec3(v.x, v.y, v.z); }
geometry_msgs::Vector3 message(const Vec3& v)
{
  geometry_msgs::Vector3 m;
  m.x = v.x(); m.y = v.y(); m.z = v.z();
  return m;
}
std::string frame(std::string value)
{
  while (!value.empty() && value.front() == '/') value.erase(value.begin());
  return value;
}
}

// One instance per Bee. No access to other robots or the object pose topic.
// It receives generic references and uses only local odometry/observer data.
class ReferenceTracker::Impl
{
public:
  Impl(ros::NodeHandle nh, ros::NodeHandle parameters, double model_mass, bool enabled)
    : nh_(nh), priv_(parameters), enabled_(enabled)
  {
    config_.mass = model_mass;
    priv_.setParam("model_mass", config_.mass);  // Diagnostic; never read as configuration.
    priv_.setParam("gravity", config_.gravity);
    priv_.param("thrust_limit", config_.thrust_limit, config_.thrust_limit);
    priv_.param("correction_reserve", config_.correction_reserve, config_.correction_reserve);
    std::vector<double> kp;
    if (priv_.getParam("force_kp", kp))
    {
      if (kp.size() != 3) throw std::invalid_argument("force_kp must have 3 entries");
      config_.kp = Vec3(kp[0], kp[1], kp[2]);
    }
    priv_.param("force_ki", config_.ki, config_.ki);
    priv_.param("force_integral_limit", config_.integral_limit, config_.integral_limit);
    priv_.param("velocity_damping", config_.damping, config_.damping);
    priv_.param("force_filter_tau", config_.filter_tau, config_.filter_tau);
    config_.validate();
    priv_.param("external_filter_tau", external_tau_, .2);
    priv_.param("observer_force_scale", observer_force_scale_, 1.0);
    priv_.param("input_timeout", input_timeout_, .3);
    priv_.param("reference_timeout", reference_timeout_, .3);
    priv_.param("maximum_acceleration", max_acceleration_, 20.0);
    priv_.param("maximum_velocity_error", max_velocity_error_, .3);
    priv_.param("velocity_error_duration", velocity_error_duration_, 1.0);
    priv_.param("allow_empty_base_frame", allow_empty_base_frame_, false);
    priv_.param("world_frame", world_, std::string("world"));
    priv_.param("cog_frame", cog_frame_, frame(nh_.getNamespace()) + "/cog");
    if (!std::isfinite(external_tau_) || external_tau_ < 0)
      throw std::invalid_argument("invalid external filter time constant");
    for (double value : {input_timeout_, reference_timeout_, max_acceleration_,
                         max_velocity_error_, velocity_error_duration_})
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid tracking gain or limit");
    if (!std::isfinite(observer_force_scale_) || observer_force_scale_ == 0)
      throw std::invalid_argument("invalid observer force scale");
    std::string base_topic, cog_topic;
    priv_.param("base_topic", base_topic, std::string("uav/baselink/odom"));
    priv_.param("cog_topic", cog_topic, std::string("uav/cog/odom"));
    base_sub_ = nh_.subscribe<nav_msgs::Odometry>(base_topic, 1, [this](const nav_msgs::Odometry::ConstPtr& m) {
      odometry(*m, false);
    });
    cog_sub_ = nh_.subscribe<nav_msgs::Odometry>(cog_topic, 1, [this](const nav_msgs::Odometry::ConstPtr& m) {
      odometry(*m, true);
    });
    wrench_sub_ = nh_.subscribe<geometry_msgs::WrenchStamped>("estimated_external_wrench", 1,
        [this](const geometry_msgs::WrenchStamped::ConstPtr& m) {
      std::lock_guard<std::mutex> lock(mutex_);
      raw_force_ = vec(m->wrench.force);
      raw_moment_ = vec(m->wrench.torque);
      accept(2, m->header.stamp, raw_force_.allFinite() && frame(m->header.frame_id) == frame(cog_frame_));
    });
    state_sub_ = nh_.subscribe<std_msgs::UInt8>("flight_state", 1, [this](const std_msgs::UInt8::ConstPtr& m) {
      std::lock_guard<std::mutex> lock(mutex_);
      flight_state_ = m->data; receipt_[3] = steadyNow();
    });
    emergency_sub_ = nh_.subscribe<std_msgs::Empty>("/bee_cooperative_grasp/emergency_stop", 1,
        [this](const std_msgs::Empty::ConstPtr&) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (enabled_ && fault_.empty()) fail("operator emergency stop");
        });
    ref_sub_ = nh_.subscribe("command", 1, &Impl::reference, this);
    mode_service_ = nh_.advertiseService("controller/set_reference_mode", &Impl::setMode, this);
    mode_pub_ = nh_.advertise<std_msgs::Bool>("controller/reference_enabled", 1, true);
    publishMode();
    status_pub_ = nh_.advertise<bee::ControllerStatus>("controller/tracking_status", 1);
    feedback_pub_ = nh_.advertise<bee::ForceFeedback>("controller/force_feedback", 1);
    parameters_pub_ = nh_.advertise<bee::ForceControlParameters>("controller/force_control_parameters", 1, true);
    parameters_service_ = nh_.advertiseService("controller/set_force_control_parameters", &Impl::setForceParameters, this);
    parameters_pub_.publish(forceParameters());
    calibration_ = std::make_unique<ObserverCalibration>(nh_, priv_, cog_frame_, world_);

  }
  ~Impl() = default;
  bool update(aerial_robot_msgs::FlightNav& nav, geometry_msgs::Vector3Stamped& attitude, bool& release, bool available)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    controller_available_ = available;
    // One update path: the existing BeeController loop also advances force feedback.
    tick();
    if (!pending_) return false;
    nav = command_; attitude = attitude_; release = release_;
    pending_ = false;
    return true;
  }
private:
  bee::ForceControlParameters forceParameters() const
  {
    bee::ForceControlParameters p;
    p.force_kp = message(config_.kp); p.force_ki = config_.ki;
    p.force_integral_limit = config_.integral_limit; p.velocity_damping = config_.damping;
    p.force_filter_tau = config_.filter_tau; p.external_filter_tau = external_tau_;
    return p;
  }
  bool setForceParameters(bee::SetForceControlParameters::Request& req,
                          bee::SetForceControlParameters::Response& res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto& p = req.parameters;
    ForceConfig candidate = config_;
    candidate.kp = vec(p.force_kp); candidate.ki = p.force_ki;
    candidate.integral_limit = p.force_integral_limit; candidate.damping = p.velocity_damping;
    candidate.filter_tau = p.force_filter_tau;
    try
    {
      candidate.validate();
      if (!std::isfinite(p.external_filter_tau) || p.external_filter_tau < 0)
        throw std::invalid_argument("invalid external filter time constant");
    }
    catch (const std::invalid_argument& e)
    {
      res.success = false; res.message = e.what(); res.applied = forceParameters();
      return true;  // Transport succeeded; rejected settings have no side effects.
    }
    config_ = candidate; external_tau_ = p.external_filter_tau;
    if (req.reset_force_feedback) force_state_.reset();
    else
    {
      // I stores an acceleration contribution, not the unscaled error integral.
      // Preserve it across gain changes, clamp to a reduced limit, and clear
      // it when I is disabled. Retain both LPF outputs to avoid resetting them.
      force_state_.integral = force_state_.integral.cwiseMax(-config_.integral_limit)
          .cwiseMin(config_.integral_limit);
      if (config_.ki == 0) force_state_.integral.setZero();
    }
    res.success = true; res.message = "force parameters applied; used on the next control update";
    res.applied = forceParameters(); parameters_pub_.publish(res.applied);
    return true;
  }
  void publishMode()
  {
    std_msgs::Bool m; m.data = enabled_; mode_pub_.publish(m);
    nh_.setParam("controller/reference_enabled", enabled_);
  }
  bool setMode(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (req.data != enabled_)
    {
      if (!req.data) stop();
      enabled_ = req.data;
      active_ = false;  // Calibration persists until explicitly replaced.
      error_time_ = 0;
      // An explicit off/on cycle starts a new session; a reference alone cannot
      // clear a fault or revive an interrupted reference stream.
      if (enabled_) { fault_.clear(); reference_receipt_ = 0; }
      publishMode();
    }
    res.success = true;
    res.message = enabled_ ? "reference tracking enabled; waiting for fresh references" : "reference tracking disabled";
    return true;
  }
  void accept(int i, const ros::Time& stamp, bool good)
  {
    receipt_[i] = good ? steadyNow() : 0;
    source_[i] = stamp.toSec();
  }
  void odometry(const nav_msgs::Odometry& m, bool cog)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Pose p;
    p.position = Vec3(m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z);
    p.orientation = Eigen::Quaterniond(m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                                      m.pose.pose.orientation.y, m.pose.pose.orientation.z);
    const bool valid = p.position.allFinite() && p.orientation.coeffs().allFinite() &&
        std::abs(p.orientation.squaredNorm() - 1) < .05 && vec(m.twist.twist.linear).allFinite() &&
        vec(m.twist.twist.angular).allFinite() &&
        (frame(m.header.frame_id) == frame(world_) || (!cog && allow_empty_base_frame_ && m.header.frame_id.empty())) &&
        (!cog || frame(m.child_frame_id) == frame(cog_frame_));
    accept(cog ? 1 : 0, m.header.stamp, valid);
    if (!valid) return;
    p.orientation.normalize();
    if (cog) cog_ = p;
    else
    {
      base_ = p;
      velocity_ = vec(m.twist.twist.linear);
      omega_ = vec(m.twist.twist.angular);
    }
  }
  bool inputsReady(bool enable_force_control) const
  {
    const double now = steadyNow(), sim = ros::Time::now().toSec();
    if (!controller_available_) return false;
    for (int i = 0; i < 4; ++i)
    {
      if (i == 2 && !enable_force_control) continue;
      if (receipt_[i] <= 0 || now - receipt_[i] > input_timeout_ ||
          (i < 3 && (source_[i] <= 0 || sim - source_[i] > input_timeout_ || source_[i] - sim > .05)))
        return false;
    }
    return flight_state_ == 5;
  }
  void reference(const bee::BeeCommand::ConstPtr& m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_) return;
    if (m->stop)
    {
      stop(); active_ = false;
      return;
    }
    if (!fault_.empty()) return;  // restart required after a liveness/safety fault
    const auto& n = m->navigation;
    const auto translationMode = [](int mode) { return mode >= 0 && mode <= 4; };
    bool finite = true;
    for (double x : {n.target_pos_x, n.target_pos_y, double(n.target_pos_z), double(n.target_pos_diff_z),
          double(n.target_vel_x), double(n.target_vel_y), double(n.target_vel_z),
          double(n.target_acc_x), double(n.target_acc_y), double(n.target_acc_z),
          double(n.target_roll), double(n.target_pitch), double(n.target_yaw),
          double(n.target_omega_x), double(n.target_omega_y), double(n.target_omega_z)})
      finite = finite && std::isfinite(x);
    const Eigen::Quaterniond q(m->acceleration_orientation.w, m->acceleration_orientation.x,
                              m->acceleration_orientation.y, m->acceleration_orientation.z);
    const Vec3 input_acceleration(n.pos_xy_nav_mode == n.ACC_MODE ? n.target_acc_x : 0,
        n.pos_xy_nav_mode == n.ACC_MODE ? n.target_acc_y : 0,
        n.pos_z_nav_mode == n.ACC_MODE ? n.target_acc_z : 0);
    const bool valid = input_acceleration.norm() <= max_acceleration_ && frame(m->header.frame_id) == frame(world_) && finite &&
        vec(m->velocity).allFinite() && vec(m->velocity_point).allFinite() &&
        vec(m->baselink_rpy).allFinite() && vec(m->force).allFinite() &&
        translationMode(n.pos_xy_nav_mode) && translationMode(n.pos_z_nav_mode) &&
        n.yaw_nav_mode == n.NO_NAVIGATION && n.roll_nav_mode == n.NO_NAVIGATION && n.pitch_nav_mode == n.NO_NAVIGATION &&
        (n.control_frame == n.WORLD_FRAME || (!m->enable_force_control && n.control_frame == n.LOCAL_FRAME)) &&
        (n.target == n.COG || (!m->enable_force_control && n.target == n.BASELINK)) &&
        (!m->enable_force_control || ((n.pos_xy_nav_mode == n.ACC_MODE || n.pos_xy_nav_mode == n.POS_VEL_MODE) &&
          (n.pos_z_nav_mode == n.ACC_MODE || n.pos_z_nav_mode == n.POS_MODE))) &&
        (!m->acceleration_bounds || (q.coeffs().allFinite() && std::abs(q.squaredNorm()-1) < 1e-3 &&
            vec(m->acceleration_min).allFinite() && vec(m->acceleration_max).allFinite() &&
            (vec(m->acceleration_min).array() <= vec(m->acceleration_max).array()).all()));
    if (!valid) { fail("invalid reference"); return; }
    if (!active_) last_time_ = ros::Time::now().toSec();
    reference_ = *m;
    reference_receipt_ = steadyNow();
    active_ = true;
  }
  void stop()
  {
    force_state_.reset(); force_control_active_ = false;
    if (!active_) return;
    aerial_robot_msgs::FlightNav m;
    m.header.stamp = ros::Time::now(); m.target = m.COG; m.control_frame = m.WORLD_FRAME;
    m.pos_xy_nav_mode = m.pos_z_nav_mode = m.VEL_MODE;
    command_ = m; pending_ = true; release_ = true;
  }
  void fail(const std::string& reason)
  {
    if (fault_.empty()) ROS_ERROR_STREAM("Bee reference tracking: " << reason);
    fault_ = reason;
    stop(); active_ = false;
  }
  void tick()
  {
    const double now = ros::Time::now().toSec();
    Wrench baseline;
    Vec3 world_baseline;
    if (calibration_->update(steadyNow(), now, input_timeout_,
          receipt_[2], source_[2], Wrench{raw_force_, raw_moment_},
          receipt_[1], source_[1], cog_.orientation, baseline, world_baseline))
    {
      bias_ = baseline.force; calibrated_ = true;
      external_.setZero(); force_state_.reset();
      calibration_->applied(baseline, world_baseline, calibration_->samples(), "topic calibration applied");
    }
    const bool ready = inputsReady(active_ && reference_.enable_force_control);
    bee::ControllerStatus status = last_status_;
    status.header.stamp = ros::Time::now(); status.header.frame_id = world_;
    if (active_)
    {
      const double age = now - reference_.header.stamp.toSec();
      if (!ready) fail("local state missing, stale or not HOVER");
      else if (steadyNow() - reference_receipt_ > reference_timeout_ || age > reference_timeout_ || age < -.05)
        fail("central reference timeout");
      else if (now < last_time_ || now - last_time_ > input_timeout_) fail("control clock discontinuity");
      else if (now > last_time_)
      {
        const double dt = std::min(.05, now - last_time_);
        last_time_ = now;
        control(dt, status);
      }
    }
    status.ready = enabled_ && ready && fault_.empty(); status.active = active_;
    status.calibrated = calibrated_; status.fault = fault_;
    last_status_ = status; status_pub_.publish(status);
  }
  void control(double dt, bee::ControllerStatus& status)
  {
    const auto& r = reference_;
    auto nav = r.navigation;
    nav.header.stamp = ros::Time::now();
    if (receipt_[2] > 0 && steadyNow() - receipt_[2] <= input_timeout_)
    {
      const Vec3 measured = observer_force_scale_ * (cog_.orientation * (raw_force_ - bias_));
      external_ += dt / (external_tau_ + dt) * (measured - external_);
    }
    if (r.enable_force_control)
    {
      if (!force_control_active_ || r.reset_force_feedback) force_state_.reset();
      const Vec3 offset = base_.orientation * vec(r.velocity_point);
      const Vec3 point_velocity = velocity_ + (base_.orientation * omega_).cross(offset);
      const Vec3 error = vec(r.velocity) - point_velocity;
      Eigen::Matrix3d axes = Eigen::Matrix3d::Identity();
      if (r.acceleration_bounds)
        axes = Eigen::Quaterniond(r.acceleration_orientation.w, r.acceleration_orientation.x,
            r.acceleration_orientation.y, r.acceleration_orientation.z).normalized().toRotationMatrix();
      const Vec3 acceleration = forceTrackingAcceleration(
          Vec3(nav.pos_xy_nav_mode == nav.ACC_MODE ? nav.target_acc_x : 0.0,
               nav.pos_xy_nav_mode == nav.ACC_MODE ? nav.target_acc_y : 0.0, nav.pos_z_nav_mode == nav.ACC_MODE ? nav.target_acc_z : 0.0),
          error, vec(r.force), external_,
          axes, vec(r.acceleration_min), vec(r.acceleration_max), r.acceleration_bounds,
          force_state_, dt, config_, r.force_integral_enabled);
      bee::ForceFeedback diagnostic;
      diagnostic.header = nav.header; diagnostic.header.frame_id = world_;
      diagnostic.force_error = message(vec(r.force) + external_);
      diagnostic.pi_unfiltered = message(force_state_.unfiltered);
      diagnostic.pi_filtered = message(force_state_.filtered);
      diagnostic.integral = message(force_state_.integral);
      diagnostic.velocity_damping = message(config_.damping * error);
      diagnostic.correction_saturated = force_state_.correction_saturated;
      diagnostic.acceleration_limited = force_state_.acceleration_limited;
      diagnostic.state_reset = r.reset_force_feedback;
      feedback_pub_.publish(diagnostic);
      if (r.reset_force_feedback) force_state_.reset();
      nav.target_acc_x = acceleration.x(); nav.target_acc_y = acceleration.y(); nav.target_acc_z = acceleration.z();
      error_time_ = error.norm() > max_velocity_error_ ? error_time_ + dt : 0;
      if (!acceleration.allFinite() || acceleration.norm() > max_acceleration_ || error_time_ > velocity_error_duration_)
      { fail("excessive acceleration or persistent velocity error"); return; }
      status.velocity = message(point_velocity); status.velocity_error = message(error);
      status.external_force = message(external_); status.commanded_acceleration = message(acceleration);
    }
    else
    {
      error_time_ = 0; force_state_.reset();
      if (nav.pos_z_nav_mode == nav.POS_MODE) nav.target_acc_z = 0;
      if (nav.pos_xy_nav_mode == nav.POS_VEL_MODE) nav.target_acc_x = nav.target_acc_y = 0;
      status.velocity = message(velocity_); status.velocity_error = geometry_msgs::Vector3{};
      status.external_force = message(external_);
      status.commanded_acceleration = message(Vec3(nav.target_acc_x, nav.target_acc_y, nav.target_acc_z));
    }
    force_control_active_ = r.enable_force_control;
    geometry_msgs::Vector3Stamped attitude;
    attitude.header = nav.header; attitude.header.frame_id = world_; attitude.vector = r.baselink_rpy;
    command_ = nav; attitude_ = attitude; pending_ = true; release_ = false;
  }
  ros::NodeHandle nh_, priv_;
  ros::Subscriber base_sub_, cog_sub_, wrench_sub_, state_sub_, ref_sub_, emergency_sub_;
  ros::Publisher status_pub_, mode_pub_, feedback_pub_, parameters_pub_;
  ros::ServiceServer mode_service_, parameters_service_;
  std::mutex mutex_;
  bool enabled_ = false, pending_ = false, release_ = false, controller_available_ = false;
  aerial_robot_msgs::FlightNav command_;
  geometry_msgs::Vector3Stamped attitude_;
  bool force_control_active_ = false;
  ForceTrackingState force_state_;
  std::unique_ptr<ObserverCalibration> calibration_;
  ForceConfig config_;
  bee::ControllerStatus last_status_;
  bee::BeeCommand reference_;
  Pose base_, cog_;
  Vec3 velocity_ = Vec3::Zero(), omega_ = Vec3::Zero(), raw_force_ = Vec3::Zero();
  Vec3 bias_ = Vec3::Zero();
  Vec3 external_ = Vec3::Zero(), raw_moment_ = Vec3::Zero();
  std::array<double, 4> receipt_{};
  std::array<double, 3> source_{};
  std::string world_, cog_frame_, fault_;
  bool allow_empty_base_frame_ = false, active_ = false, calibrated_ = false;
  int flight_state_ = 0;
  double reference_receipt_ = 0, last_time_ = 0, error_time_ = 0;
  double external_tau_, observer_force_scale_, input_timeout_, reference_timeout_, max_acceleration_;
  double max_velocity_error_, velocity_error_duration_;
};

ReferenceTracker::ReferenceTracker(ros::NodeHandle nh, ros::NodeHandle parameters, double model_mass, bool enabled)
  : impl_(new Impl(nh, parameters, model_mass, enabled)) {}
ReferenceTracker::~ReferenceTracker() = default;
bool ReferenceTracker::update(aerial_robot_msgs::FlightNav& nav, geometry_msgs::Vector3Stamped& attitude,
                            bool& release, bool available)
{
  return impl_->update(nav, attitude, release, available);
}
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::BeeController, aerial_robot_control::ControlBase);
