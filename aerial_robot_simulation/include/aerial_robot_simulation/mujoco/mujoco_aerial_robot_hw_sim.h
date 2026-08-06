#pragma once

#include <aerial_robot_simulation/mujoco/mujoco_spinal_interface.h>
#include <aerial_robot_simulation/noise_model.h>
#include <aerial_robot_msgs/ForceList.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <mujoco_ros_control/mujoco_default_robot_hw_sim.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

namespace mujoco_ros_control
{
  class AerialRobotHWSim : public mujoco_ros_control::DefaultRobotHWSim
  {
  public:
    AerialRobotHWSim() {};
    ~AerialRobotHWSim() {}

    bool init(const std::string& robot_namespace,
              ros::NodeHandle model_nh,
              mjModel* mujoco_model,
              mjData* mujoco_data
              ) override;

    void read(const ros::Time& time, const ros::Duration& period) override;

    void write(const ros::Time& time, const ros::Duration& period) override;

    void externalWrenchCallback(const geometry_msgs::WrenchStamped& msg);

  protected:
    struct ForceSiteSensor
    {
      std::string name;
      int data_address;
      int site_id;
    };

    struct GraspContactSensor
    {
      std::string name;
      int touch_data_address = -1;
      int compression_data_address = -1;
      int compression_velocity_data_address = -1;
    };

    hardware_interface::MujocoSpinalInterface spinal_interface_;

    std::vector<std::string> rotor_list_;
    std::string fc_site_name_;
    std::string acc_sensor_name_;
    std::string gyro_sensor_name_;
    std::string mag_sensor_name_;
    ros::Publisher ground_truth_pub_;
    ros::Publisher mocap_pub_;
    ros::Publisher foot_force_pub_;
    ros::Publisher grasp_force_pub_;
    ros::Publisher grasp_force_world_pub_;
    ros::Publisher grasp_contact_state_pub_;
    ros::Publisher grasp_object_pose_pub_;
    ros::Subscriber external_wrench_sub_;
    std::vector<ForceSiteSensor> force_site_sensors_;
    std::vector<ForceSiteSensor> grasp_force_site_sensors_;
    std::vector<GraspContactSensor> grasp_contact_sensors_;
    int grasp_object_body_id_ = -1;
    int root_body_id_ = -1;
    geometry_msgs::Wrench external_wrench_;
    ros::Time last_external_wrench_time_;
    double external_wrench_timeout_ = 0.1;
    double external_force_limit_ = 10.0;
    double external_torque_limit_ = 2.0;
    double ground_truth_pub_rate_;
    double mocap_pub_rate_;
    double force_sensor_pub_rate_;
    double mocap_rot_noise_, mocap_pos_noise_;
    double ground_truth_pos_noise_, ground_truth_vel_noise_, ground_truth_rot_noise_, ground_truth_angular_noise_;
    double ground_truth_rot_drift_, ground_truth_vel_drift_, ground_truth_angular_drift_;
    double ground_truth_rot_drift_frequency_, ground_truth_vel_drift_frequency_, ground_truth_angular_drift_frequency_;
    double joint_state_pub_rate_ = 0.02;

    ros::Time last_mocap_time_;
    ros::Time last_ground_truth_time_;
    ros::Time last_force_sensor_time_;
  };
}
