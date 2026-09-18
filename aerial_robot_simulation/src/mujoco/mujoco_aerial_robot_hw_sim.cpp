#include <aerial_robot_simulation/mujoco/mujoco_aerial_robot_hw_sim.h>

#include <algorithm>
#include <map>
#include <sstream>

namespace mujoco_ros_control
{

  bool AerialRobotHWSim::init(const std::string& robot_namespace,
                              ros::NodeHandle model_nh,
                              mjModel* mujoco_model,
                              mjData* mujoco_data
                              )
  {
    if(!DefaultRobotHWSim::init(robot_namespace, model_nh, mujoco_model, mujoco_data))
      {
        return false;
      }

    fc_site_name_ = name_prefix_.empty() ? std::string("fc") : name_prefix_ + "fc";
    acc_sensor_name_ = name_prefix_.empty() ? std::string("acc") : name_prefix_ + "acc";
    gyro_sensor_name_ = name_prefix_.empty() ? std::string("gyro") : name_prefix_ + "gyro";
    mag_sensor_name_ = name_prefix_.empty() ? std::string("mag") : name_prefix_ + "mag";

    rotor_list_.resize(0);

    // get entire mass
    float mass = 0.0;
    for(int i = 0; i < mujoco_model_->nbody; i++)
      {
       mass += mujoco_model_->body_mass[i];
      }
    ROS_INFO_STREAM("[mujoco] robot mass is " << mass);


    // get rotor names from mujoco model
    int motor_num = 0;
    for(int i = 0; i < mujoco_model_->nu; i++)
      {
        const char* actuator_name_cstr = mj_id2name(mujoco_model_, mjtObj_::mjOBJ_ACTUATOR, i);
        if(!actuator_name_cstr) continue;
        std::string actuator_name(actuator_name_cstr);
        if(matchesRobotNamespace(actuator_name) && actuator_name.find("rotor") != std::string::npos)
          {
            rotor_list_.push_back(actuator_name);
            registerManagedActuator(i);
            motor_num++;
          }
      }

    // init joints from rosparam
    XmlRpc::XmlRpcValue all_servos_params;
    model_nh.getParam("servo_controller", all_servos_params);
    std::string init_value_param_name = "init_value";
    for(auto servo_group_params: all_servos_params)
      {
        if (servo_group_params.second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
          continue;
        for(auto servo_params : servo_group_params.second)
          {
            if(servo_params.first.find("controller") != string::npos)
              {
                std::string servo_name = static_cast<std::string>(servo_params.second["name"]);
                double init_value = 0.0;

                // check simulation param exists
                if(!servo_group_params.second.hasMember("simulation") &&
                   !servo_params.second.hasMember("simulation"))
                  {
                    ROS_ERROR("please set mujoco servo parameters for %s, using sub namespace 'simulation:'", string(servo_params.second["name"]).c_str());
                    continue;
                  }

                // search init_value in servo params
                if(!servo_params.second.hasMember("simulation") ||
                   (servo_params.second.hasMember("simulation") && !servo_params.second["simulation"].hasMember(init_value_param_name)))
                  {
                    // search init_value in servo group params
                    if(!servo_group_params.second["simulation"].hasMember(init_value_param_name))
                      {
                        ROS_ERROR("can not find '%s' gazebo paramter for servo %s", init_value_param_name.c_str(),  string(servo_params.second["name"]).c_str());
                        return false;
                      }
                    // use param of servo group
                    init_value = static_cast<double>(servo_group_params.second["simulation"][init_value_param_name]);
                  }
                else
                  {
                    // use param of servo
                    init_value = static_cast<double>(servo_params.second["simulation"][init_value_param_name]);
                  }
                const std::string actuator_name = name_prefix_.empty() ? servo_name : name_prefix_ + servo_name;
                const int actuator_id = mj_name2id(mujoco_model_, mjtObj_::mjOBJ_ACTUATOR, actuator_name.c_str());
                if(actuator_id < 0)
                  {
                    ROS_ERROR("can not find actuator '%s' for servo %s", actuator_name.c_str(), servo_name.c_str());
                    return false;
                  }
                control_input_.at(actuator_id) = init_value;
              }
          }
      }

    /* Initialize spinal interface */
    spinal_interface_.init(model_nh, rotor_list_.size());
    registerInterface(&spinal_interface_);

    ros::NodeHandle simulation_nh = ros::NodeHandle(model_nh, "simulation");
    simulation_nh.param("ground_truth_pub_rate", ground_truth_pub_rate_, 0.01); // [sec]
    simulation_nh.param("ground_truth_pos_noise", ground_truth_pos_noise_, 0.0); // m
    simulation_nh.param("ground_truth_vel_noise", ground_truth_vel_noise_, 0.0); // m/s
    simulation_nh.param("ground_truth_rot_noise", ground_truth_rot_noise_, 0.0); // rad
    simulation_nh.param("ground_truth_angular_noise", ground_truth_angular_noise_, 0.0); // rad/s
    simulation_nh.param("ground_truth_rot_drift", ground_truth_rot_drift_, 0.0); // rad
    simulation_nh.param("ground_truth_vel_drift", ground_truth_vel_drift_, 0.0); // m/s
    simulation_nh.param("ground_truth_angular_drift", ground_truth_angular_drift_, 0.0); // rad/s
    simulation_nh.param("ground_truth_rot_drift_frequency", ground_truth_rot_drift_frequency_, 0.0); // 1/s
    simulation_nh.param("ground_truth_vel_drift_frequency", ground_truth_vel_drift_frequency_, 0.0); // 1/s
    simulation_nh.param("ground_truth_angular_drift_frequency", ground_truth_angular_drift_frequency_, 0.0); // 1/s

    simulation_nh.param("mocap_pub_rate", mocap_pub_rate_, 0.01); // [sec]
    simulation_nh.param("mocap_pos_noise", mocap_pos_noise_, 0.001); // m
    simulation_nh.param("mocap_rot_noise", mocap_rot_noise_, 0.001); // rad
    simulation_nh.param("force_sensor_pub_rate", force_sensor_pub_rate_, 0.01); // [sec]
    simulation_nh.param("external_wrench_timeout", external_wrench_timeout_, 0.1);
    simulation_nh.param("external_force_limit", external_force_limit_, 10.0);
    simulation_nh.param("external_torque_limit", external_torque_limit_, 2.0);
    ground_truth_pub_ = model_nh.advertise<nav_msgs::Odometry>("ground_truth", 1);
    mocap_pub_ = model_nh.advertise<geometry_msgs::PoseStamped>("mocap/pose", 1);
    external_wrench_sub_ = model_nh.subscribe(
      "mujoco/external_wrench", 1, &AerialRobotHWSim::externalWrenchCallback, this);
    if(root_joint_id_ >= 0)
      root_body_id_ = mujoco_model_->jnt_bodyid[root_joint_id_];

    force_site_sensors_.clear();
    grasp_force_site_sensors_.clear();
    grasp_contact_sensors_.clear();
    std::map<std::string, GraspContactSensor> grasp_contact_sensor_map;
    for(int sensor_id = 0; sensor_id < mujoco_model_->nsensor; ++sensor_id)
      {
        const char* sensor_name_cstr = mj_id2name(mujoco_model_, mjOBJ_SENSOR, sensor_id);
        if(!sensor_name_cstr || !matchesRobotNamespace(sensor_name_cstr)) continue;

        const std::string sensor_name = stripNamePrefix(sensor_name_cstr);
        const std::string foot_prefix = "spring_foot_";
        const std::string force_suffix = "_force";
        const std::string touch_suffix = "_touch";
        const std::string compression_suffix = "_compression";
        const std::string compression_velocity_suffix = "_compression_velocity";
        const bool is_foot_sensor = sensor_name.find(foot_prefix) == 0;
        if(!is_foot_sensor)
          {
            continue;
          }

        if(sensor_name.size() > force_suffix.size() &&
           sensor_name.compare(sensor_name.size() - force_suffix.size(),
                               force_suffix.size(), force_suffix) == 0 &&
           mujoco_model_->sensor_type[sensor_id] == mjSENS_FORCE &&
           mujoco_model_->sensor_dim[sensor_id] == 3 &&
           mujoco_model_->sensor_objtype[sensor_id] == mjOBJ_SITE)
          {
            ForceSiteSensor force_sensor;
            force_sensor.name = sensor_name;
            force_sensor.data_address = mujoco_model_->sensor_adr[sensor_id];
            force_sensor.site_id = mujoco_model_->sensor_objid[sensor_id];
            force_site_sensors_.push_back(force_sensor);
            continue;
          }

        if(mujoco_model_->sensor_dim[sensor_id] != 1)
          continue;

        std::string contact_name;
        enum ContactSensorField { NONE, TOUCH, COMPRESSION, COMPRESSION_VELOCITY } field = NONE;
        if(sensor_name.size() > compression_velocity_suffix.size() &&
           sensor_name.compare(sensor_name.size() - compression_velocity_suffix.size(),
                               compression_velocity_suffix.size(), compression_velocity_suffix) == 0 &&
           mujoco_model_->sensor_type[sensor_id] == mjSENS_JOINTVEL)
          {
            contact_name = sensor_name.substr(0, sensor_name.size() - compression_velocity_suffix.size());
            field = COMPRESSION_VELOCITY;
          }
        else if(sensor_name.size() > compression_suffix.size() &&
                sensor_name.compare(sensor_name.size() - compression_suffix.size(),
                                    compression_suffix.size(), compression_suffix) == 0 &&
                mujoco_model_->sensor_type[sensor_id] == mjSENS_JOINTPOS)
          {
            contact_name = sensor_name.substr(0, sensor_name.size() - compression_suffix.size());
            field = COMPRESSION;
          }
        else if(sensor_name.size() > touch_suffix.size() &&
                sensor_name.compare(sensor_name.size() - touch_suffix.size(),
                                    touch_suffix.size(), touch_suffix) == 0 &&
                mujoco_model_->sensor_type[sensor_id] == mjSENS_TOUCH)
          {
            contact_name = sensor_name.substr(0, sensor_name.size() - touch_suffix.size());
            field = TOUCH;
          }

        if(field == NONE) continue;
        GraspContactSensor& contact_sensor = grasp_contact_sensor_map[contact_name];
        contact_sensor.name = contact_name;
        if(field == TOUCH)
          contact_sensor.touch_data_address = mujoco_model_->sensor_adr[sensor_id];
        else if(field == COMPRESSION)
          contact_sensor.compression_data_address = mujoco_model_->sensor_adr[sensor_id];
        else
          contact_sensor.compression_velocity_data_address = mujoco_model_->sensor_adr[sensor_id];
      }

    const std::vector<std::string> grasp_order = {
      "spring_foot_front", "spring_foot_rear",
      "spring_foot_left", "spring_foot_right"
    };
    const auto grasp_order_index = [&](const std::string& sensor_name)
      {
        for(size_t index = 0; index < grasp_order.size(); ++index)
          if(sensor_name.find(grasp_order[index]) == 0) return index;
        return grasp_order.size();
      };
    std::sort(force_site_sensors_.begin(), force_site_sensors_.end(),
              [&](const ForceSiteSensor& lhs, const ForceSiteSensor& rhs)
              { return grasp_order_index(lhs.name) < grasp_order_index(rhs.name); });
    // The grasp interface is deliberately an alias of the four physical feet.
    // No additional grasp-only contact bodies are present in the model.
    grasp_force_site_sensors_ = force_site_sensors_;
    for(const std::string& contact_name : grasp_order)
      {
        const auto found = grasp_contact_sensor_map.find(contact_name);
        if(found == grasp_contact_sensor_map.end() ||
           found->second.touch_data_address < 0 ||
           found->second.compression_data_address < 0 ||
           found->second.compression_velocity_data_address < 0)
          {
            ROS_WARN_STREAM("[mujoco] incomplete grasp contact sensor set for " << contact_name);
            continue;
          }
        grasp_contact_sensors_.push_back(found->second);
      }

    if(!grasp_force_site_sensors_.empty())
      {
        grasp_force_pub_ = model_nh.advertise<aerial_robot_msgs::ForceList>("mujoco/grasp_forces", 1);
        grasp_force_world_pub_ = model_nh.advertise<aerial_robot_msgs::ForceList>("mujoco/grasp_forces_world", 1);
        std::ostringstream sensor_names;
        for(size_t i = 0; i < grasp_force_site_sensors_.size(); ++i)
          {
            if(i > 0) sensor_names << ", ";
            sensor_names << grasp_force_site_sensors_[i].name;
          }
        ROS_INFO_STREAM("[mujoco] grasp force order: [" << sensor_names.str() << "]");
      }

    if(!grasp_contact_sensors_.empty())
      {
        grasp_contact_state_pub_ = model_nh.advertise<sensor_msgs::JointState>("mujoco/grasp_contact_states", 1);
        std::ostringstream sensor_names;
        for(size_t i = 0; i < grasp_contact_sensors_.size(); ++i)
          {
            if(i > 0) sensor_names << ", ";
            sensor_names << grasp_contact_sensors_[i].name;
          }
        ROS_INFO_STREAM("[mujoco] grasp contact order: [" << sensor_names.str() << "]"
                        << " (position=compression, velocity=compression velocity, effort=touch)");
      }

    grasp_object_body_id_ = mj_name2id(mujoco_model_, mjOBJ_BODY, "grasp_prism");
    if(grasp_object_body_id_ >= 0)
      grasp_object_pose_pub_ = model_nh.advertise<geometry_msgs::PoseStamped>("mujoco/grasp_object_pose", 1);

    if(!force_site_sensors_.empty())
      {
        foot_force_pub_ = model_nh.advertise<aerial_robot_msgs::ForceList>("mujoco/foot_forces", 1);
        std::ostringstream sensor_names;
        for(size_t i = 0; i < force_site_sensors_.size(); ++i)
          {
            if(i > 0) sensor_names << ", ";
            sensor_names << force_site_sensors_[i].name;
          }
        ROS_INFO_STREAM("[mujoco] foot force order: [" << sensor_names.str() << "]");
      }

    return true;
  }

  void AerialRobotHWSim::read(const ros::Time& time, const ros::Duration& period)
  {
    int fc_id = mj_name2id(mujoco_model_, mjtObj_::mjOBJ_SITE, fc_site_name_.c_str());
    if(fc_id < 0)
      {
        ROS_ERROR_THROTTLE(1.0, "mujoco: site %s does not exist", fc_site_name_.c_str());
        DefaultRobotHWSim::read(time, period);
        return;
      }
    mjtNum* site_xpos = mujoco_data_->site_xpos;
    mjtNum* site_xmat = mujoco_data_->site_xmat;
    tf::Matrix3x3 fc_rot_mat = tf::Matrix3x3(site_xmat[9 * fc_id + 0], site_xmat[9 * fc_id + 1], site_xmat[9 * fc_id + 2],
                                             site_xmat[9 * fc_id + 3], site_xmat[9 * fc_id + 4], site_xmat[9 * fc_id + 5],
                                             site_xmat[9 * fc_id + 6], site_xmat[9 * fc_id + 7], site_xmat[9 * fc_id + 8]);
    tf::Quaternion fc_quat;
    fc_rot_mat.getRotation(fc_quat);

    tf::Vector3 acc, gyro, mag;
    for(int i = 0; i < mujoco_model_->nsensor; i++)
      {
        const char* sensor_name_cstr = mj_id2name(mujoco_model_, mjtObj_::mjOBJ_SENSOR, i);
        if(!sensor_name_cstr) continue;
        std::string sensor_name(sensor_name_cstr);
        if(sensor_name == acc_sensor_name_)
          {
            for(int j = 0; j < mujoco_model_->sensor_dim[i]; j++)
              {
                acc[j] = mujoco_data_->sensordata[mujoco_model_->sensor_adr[i] + j];
              }
          }
        if(sensor_name == gyro_sensor_name_)
          {
            for(int j = 0; j < mujoco_model_->sensor_dim[i]; j++)
              {
                gyro[j] = mujoco_data_->sensordata[mujoco_model_->sensor_adr[i] + j];
              }
          }
        if(sensor_name == mag_sensor_name_)
          {
            for(int j = 0; j < mujoco_model_->sensor_dim[i]; j++)
              {
                mag[j] = mujoco_data_->sensordata[mujoco_model_->sensor_adr[i] + j];
              }
          }
      }

    spinal_interface_.setImuValue(acc.x(), acc.y(), acc.z(), gyro.x(), gyro.y(), gyro.z());
    spinal_interface_.setMagValue(mag.x(), mag.y(), mag.z());

    spinal_interface_.stateEstimate();

    /* publish ground truth value */
    /* compute linear and angular velocity of the fc site using MuJoCo */
    mjtNum vel_site[6]; // [angular(3), linear(3)] in world frame
    mj_objectVelocity(mujoco_model_, mujoco_data_, mjtObj_::mjOBJ_SITE, fc_id, vel_site, 0); // flg_local=0 -> world frame

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = time;
    odom_msg.pose.pose.position.x = site_xpos[3 * fc_id + 0];
    odom_msg.pose.pose.position.y = site_xpos[3 * fc_id + 1];
    odom_msg.pose.pose.position.z = site_xpos[3 * fc_id + 2];
    odom_msg.pose.pose.orientation.x = fc_quat.x();
    odom_msg.pose.pose.orientation.y = fc_quat.y();
    odom_msg.pose.pose.orientation.z = fc_quat.z();
    odom_msg.pose.pose.orientation.w = fc_quat.w();
    odom_msg.twist.twist.linear.x = vel_site[3];
    odom_msg.twist.twist.linear.y = vel_site[4];
    odom_msg.twist.twist.linear.z = vel_site[5];
    /* angular velocity in body frame from gyro sensor */
    odom_msg.twist.twist.angular.x = gyro.x();
    odom_msg.twist.twist.angular.y = gyro.y();
    odom_msg.twist.twist.angular.z = gyro.z();

    if((time - last_ground_truth_time_).toSec() >= ground_truth_pub_rate_)
      {
        ground_truth_pub_.publish(odom_msg);
        last_ground_truth_time_ = time;
      }

    /* set ground truth for controller: use the value with noise */
    spinal_interface_.setGroundTruthStates(fc_quat.x(), fc_quat.y(), fc_quat.z(), fc_quat.w(),
                                           gyro.x(), gyro.y(), gyro.z());

    if((!force_site_sensors_.empty() || !grasp_force_site_sensors_.empty() ||
        !grasp_contact_sensors_.empty()) &&
       force_sensor_pub_rate_ > 0.0 &&
       (time - last_force_sensor_time_).toSec() >= force_sensor_pub_rate_)
      {
        const std::string frame_id = name_prefix_.empty() ? std::string("fc") : robot_namespace_ + "/fc";
        const auto publish_force_list = [&](const std::vector<ForceSiteSensor>& sensors,
                                            const ros::Publisher& publisher,
                                            const bool world_frame)
          {
            if(sensors.empty()) return;

            aerial_robot_msgs::ForceList force_list_msg;
            force_list_msg.header.stamp = time;
            force_list_msg.header.frame_id = world_frame ? "world" : frame_id;
            force_list_msg.forces.reserve(sensors.size());

            for(const ForceSiteSensor& sensor : sensors)
              {
                // Preserve MuJoCo's child-to-parent sign convention, then
                // express every contact reading in the common fc frame.
                const mjtNum* local_force = mujoco_data_->sensordata + sensor.data_address;
                mjtNum world_force[3];
                mjtNum fc_force[3];
                mju_rotVecMat(world_force, local_force, mujoco_data_->site_xmat + 9 * sensor.site_id);
                mju_rotVecMatT(fc_force, world_force, mujoco_data_->site_xmat + 9 * fc_id);

                const mjtNum* output_force = world_frame ? world_force : fc_force;
                geometry_msgs::Vector3 force;
                force.x = output_force[0];
                force.y = output_force[1];
                force.z = output_force[2];
                force_list_msg.forces.push_back(force);
              }

            publisher.publish(force_list_msg);
          };

        publish_force_list(force_site_sensors_, foot_force_pub_, false);
        publish_force_list(grasp_force_site_sensors_, grasp_force_pub_, false);
        publish_force_list(grasp_force_site_sensors_, grasp_force_world_pub_, true);

        if(!grasp_contact_sensors_.empty())
          {
            sensor_msgs::JointState contact_state;
            contact_state.header.stamp = time;
            contact_state.header.frame_id = frame_id;
            contact_state.name.reserve(grasp_contact_sensors_.size());
            contact_state.position.reserve(grasp_contact_sensors_.size());
            contact_state.velocity.reserve(grasp_contact_sensors_.size());
            contact_state.effort.reserve(grasp_contact_sensors_.size());
            for(const GraspContactSensor& sensor : grasp_contact_sensors_)
              {
                contact_state.name.push_back(sensor.name);
                contact_state.position.push_back(mujoco_data_->sensordata[sensor.compression_data_address]);
                contact_state.velocity.push_back(mujoco_data_->sensordata[sensor.compression_velocity_data_address]);
                contact_state.effort.push_back(mujoco_data_->sensordata[sensor.touch_data_address]);
              }
            grasp_contact_state_pub_.publish(contact_state);
          }

        if(grasp_object_body_id_ >= 0)
          {
            geometry_msgs::PoseStamped object_pose;
            object_pose.header.stamp = time;
            object_pose.header.frame_id = "world";
            object_pose.pose.position.x = mujoco_data_->xpos[3 * grasp_object_body_id_ + 0];
            object_pose.pose.position.y = mujoco_data_->xpos[3 * grasp_object_body_id_ + 1];
            object_pose.pose.position.z = mujoco_data_->xpos[3 * grasp_object_body_id_ + 2];
            object_pose.pose.orientation.w = mujoco_data_->xquat[4 * grasp_object_body_id_ + 0];
            object_pose.pose.orientation.x = mujoco_data_->xquat[4 * grasp_object_body_id_ + 1];
            object_pose.pose.orientation.y = mujoco_data_->xquat[4 * grasp_object_body_id_ + 2];
            object_pose.pose.orientation.z = mujoco_data_->xquat[4 * grasp_object_body_id_ + 3];
            grasp_object_pose_pub_.publish(object_pose);
          }

        last_force_sensor_time_ = time;
      }

    if((time - last_mocap_time_).toSec() >= mocap_pub_rate_)
      {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = time;
        pose_msg.pose.position.x = site_xpos[3 * fc_id + 0] + gazebo::gaussianKernel(mocap_pos_noise_);
        pose_msg.pose.position.y = site_xpos[3 * fc_id + 1] + gazebo::gaussianKernel(mocap_pos_noise_);
        pose_msg.pose.position.z = site_xpos[3 * fc_id + 2] + gazebo::gaussianKernel(mocap_pos_noise_);


        tf::Quaternion q_delta;
        q_delta.setRPY(gazebo::gaussianKernel(mocap_rot_noise_),
                       gazebo::gaussianKernel(mocap_rot_noise_),
                       gazebo::gaussianKernel(mocap_rot_noise_));
        tf::Quaternion q_noise = fc_quat * q_delta;
        pose_msg.pose.orientation.x = q_noise.x();
        pose_msg.pose.orientation.y = q_noise.y();
        pose_msg.pose.orientation.z = q_noise.z();
        pose_msg.pose.orientation.w = q_noise.w();

        mocap_pub_.publish(pose_msg);
        last_mocap_time_ = time;
      }

    DefaultRobotHWSim::read(time, period);
  }

  void AerialRobotHWSim::write(const ros::Time& time, const ros::Duration& period)
  {
    if(root_body_id_ >= 0)
      {
        mjtNum* applied = mujoco_data_->xfrc_applied + 6 * root_body_id_;
        for(int axis = 0; axis < 6; ++axis) applied[axis] = 0.0;
        if(!last_external_wrench_time_.isZero() &&
           (time - last_external_wrench_time_).toSec() <= external_wrench_timeout_)
          {
            applied[0] = external_wrench_.force.x;
            applied[1] = external_wrench_.force.y;
            applied[2] = external_wrench_.force.z;
            applied[3] = external_wrench_.torque.x;
            applied[4] = external_wrench_.torque.y;
            applied[5] = external_wrench_.torque.z;
          }
      }

    for(int i = 0; i < spinal_interface_.getMotorNum(); i++)
      {
        int rotor_id = mj_name2id(mujoco_model_, mjOBJ_ACTUATOR, rotor_list_.at(i).c_str());
        double rotor_force = spinal_interface_.getForce(i);
        control_input_.at(rotor_id) = rotor_force;
      }

      DefaultRobotHWSim::write(time, period);
  }

  void AerialRobotHWSim::externalWrenchCallback(const geometry_msgs::WrenchStamped& msg)
  {
    const auto clamp = [](const double value, const double limit)
      { return std::max(-limit, std::min(limit, value)); };
    external_wrench_.force.x = clamp(msg.wrench.force.x, external_force_limit_);
    external_wrench_.force.y = clamp(msg.wrench.force.y, external_force_limit_);
    external_wrench_.force.z = clamp(msg.wrench.force.z, external_force_limit_);
    external_wrench_.torque.x = clamp(msg.wrench.torque.x, external_torque_limit_);
    external_wrench_.torque.y = clamp(msg.wrench.torque.y, external_torque_limit_);
    external_wrench_.torque.z = clamp(msg.wrench.torque.z, external_torque_limit_);
    last_external_wrench_time_ = ros::Time::now();
  }

}

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::AerialRobotHWSim, mujoco_ros_control::RobotHWSim)
