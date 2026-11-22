// -*- mode: c++ -*-

#pragma once
#include <atomic>
#include <aerial_robot_model/model/gravity.h>
#include <bee/model/bee_robot_model.h>
#include <bee/bee_navigation.h>
#include <beetle/TaggedWrench.h>
#include <beetle/TaggedWrenches.h>
#include <gimbalrotor/control/gimbalrotor_controller.h>
#include <aerial_robot_msgs/FlightNav.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <memory>

namespace bee::control
{
using Vec3 = Eigen::Vector3d;
struct ForceConfig
{
  double mass = 1.84199, gravity = aerial_robot_model::kGravity, thrust_limit = 32, correction_reserve = 3;
  Vec3 kp = Vec3::Constant(.4);
  double ki = .1, integral_limit = .4, damping = 3, filter_tau = 0;
  void validate() const;
};
struct ForceTrackingState
{
  Vec3 integral = Vec3::Zero(), filtered = Vec3::Zero(), unfiltered = Vec3::Zero();
  bool correction_saturated = false, acceleration_limited = false;
  void reset() { *this = ForceTrackingState{}; }
};
Vec3 limitThrust(const Vec3& acceleration, const ForceConfig& p);
Vec3 forceTrackingAcceleration(const Vec3& own_acceleration, const Vec3& velocity_error,
    const Vec3& target, const Vec3& external, const Eigen::Matrix3d& axes,
    const Vec3& lower, const Vec3& upper, bool bounded,
    ForceTrackingState& state, double dt, const ForceConfig& p, bool integrate);

// Only BeeController owns the production instance. Runs on its update loop;
// no independent control timer, task phases or object/contact geometry.
class ReferenceTracker
{
public:
  ReferenceTracker(ros::NodeHandle nh, ros::NodeHandle parameters, double model_mass, bool enabled = true);
  ~ReferenceTracker();
  bool update(aerial_robot_msgs::FlightNav& nav, geometry_msgs::Vector3Stamped& attitude,
              bool& release, bool available, const std::optional<double>& hover_z_residual, double& acc_hover_trim,
              Eigen::Vector3d* force_point_torque_cog = nullptr);
private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}

namespace aerial_robot_control
{
  enum
    {
     FX = YAW +1,
     FY,
     FZ,
     TX,
     TY,
     TZ,
    };

  class BeeController: public GimbalrotorController
  {
  public:
    BeeController();
    ~BeeController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate
                    ) override;
    void setFfInterWrench(int id, Eigen::VectorXd des_int_wrench){ff_inter_wrench_list_[id] = des_int_wrench;}
  private:
    std::atomic<bool> bee_controller_initialized_;
    boost::shared_ptr<BeeRobotModel> bee_robot_model_;
    boost::shared_ptr<aerial_robot_navigation::BeeNavigator> bee_navigator_;
    std::unique_ptr<bee::control::ReferenceTracker> reference_tracker_;
    
    map<string, ros::Subscriber> ff_inter_wrench_subs_;

    aerial_robot_msgs::PoseControlPid wrench_pid_msg_;

    map<string, ros::Subscriber> est_wrench_subs_;
    
    void estExternalWrenchCallback(const beetle::TaggedWrench & msg);

  protected:
    std::map<int, Eigen::VectorXd> est_wrench_list_;
    std::map<int, Eigen::VectorXd> inter_wrench_list_;
    std::map<int, Eigen::VectorXd> wrench_comp_list_;
    std::map<int, Eigen::VectorXd> ff_inter_wrench_list_;

    /* external wrench compensation */
    bool pd_wrench_comp_mode_;
    bool prev_wrench_comp_active_;
    Eigen::VectorXd external_wrench_upper_limit_;
    Eigen::VectorXd external_wrench_lower_limit_;

    int pre_module_state_;

    bool des_wrench_pub_flag_;

    double comp_term_update_freq_;
    double prev_comp_update_time_;
    double wrench_comp_p_gain_;
    double wrench_comp_i_gain_;
    double wrench_comp_d_gain_;
    double I_comp_Fx_;
    double I_comp_Fy_;
    double I_comp_Fz_;
    double I_comp_Tx_;
    double I_comp_Ty_;
    double I_comp_Tz_;    

    virtual void calcInteractionWrench();
    ros::Publisher tagged_external_wrench_pub_;
    ros::Publisher external_wrench_compensation_pub_;
    ros::Publisher whole_external_wrench_pub_;
    ros::Publisher internal_wrench_pub_;
    ros::Publisher wrench_comp_pid_pub_;
    ros::Publisher des_inter_wrench_pub_;
    void controlCore() override;
    bool update() override;
    
    virtual void ffInterWrenchCallback(const beetle::TaggedWrench & msg);
    void rosParamInit();
    void externalWrenchEstimate();
    void reset() override;

    /* Wrench-estimation state and publisher are owned by
       PoseLinearController.  Keeping a second copy here races the base
       estimator thread during BeeController initialization. */
  };
};
