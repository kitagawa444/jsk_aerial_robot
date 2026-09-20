/*
******************************************************************************
* File Name          : attitude_control.h
* Description        : attitude control interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "flight_control/attitude/attitude_control.h"

#ifdef SIMULATION
#include <cstdlib>
#include <sensor_msgs/JointState.h>
AttitudeController::AttitudeController(): DELTA_T(0), prev_time_(-1), sim_voltage_(0)
{
}

void AttitudeController::init(ros::NodeHandle* nh, StateEstimate* estimator)
{
  nh_ = nh;
  estimator_ = estimator;

  pwms_pub_ = nh_->advertise<spinal::Pwms>("motor_pwms", 1);
  control_term_pub_ = nh_->advertise<spinal::RollPitchYawTerms>("rpy/pid", 1);
  control_feedback_state_pub_ = nh_->advertise<spinal::RollPitchYawTerm>("rpy/feedback_state", 1);
  anti_gyro_pub_ = nh_->advertise<std_msgs::Float32MultiArray>("gyro_moment_compensation", 1);
  four_axis_cmd_sub_ = nh_->subscribe("four_axes/command", 1, &AttitudeController::fourAxisCommandCallback, this);
  pwm_info_sub_ = nh_->subscribe("motor_info", 1, &AttitudeController::pwmInfoCallback, this);
  rpy_gain_sub_ = nh_->subscribe("rpy/gain", 1, &AttitudeController::rpyGainCallback, this);
  p_matrix_pseudo_inverse_inertia_sub_ = nh_->subscribe("p_matrix_pseudo_inverse_inertia", 1, &AttitudeController::pMatrixInertiaCallback, this);
  pwm_test_sub_ = nh_->subscribe("pwm_test", 1, &AttitudeController::pwmTestCallback, this);
  att_control_srv_ = nh_->advertiseService("set_attitude_control", &AttitudeController::setAttitudeControlCallback, this);
  torque_allocation_matrix_inv_sub_ = nh_->subscribe("torque_allocation_matrix_inv", 1, &AttitudeController::torqueAllocationMatrixInvCallback, this);
  wrench_allocation_matrix_sub_ = nh_->subscribe("wrench_allocation_matrix", 1, &AttitudeController::wrenchAllocationMatrixCallback, this);
  differential_allocation_config_sub_ = nh_->subscribe("differential_allocation_config", 1, &AttitudeController::differentialAllocationConfigCallback, this);
  joint_state_sub_ = nh_->subscribe("joint_states", 1, &AttitudeController::jointStateCallback, this);
  sim_vol_sub_ = nh_->subscribe("set_sim_voltage", 1, &AttitudeController::setSimVolCallback, this);
  offset_rot_sub_ = nh_->subscribe("desire_coordinate", 1, &AttitudeController::offsetRotCallback, this);
  baseInit();
  gimbal_control_pub_ = nh_->advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  differential_allocation_debug_pub_ = nh_->advertise<std_msgs::Float32MultiArray>("debug/differential_allocation", 1);
}

#else

AttitudeController::AttitudeController():
  pwms_pub_("motor_pwms", &pwms_msg_),
  control_term_pub_("rpy/pid", &control_term_msg_),
  control_feedback_state_pub_("rpy/feedback_state", &control_feedback_state_msg_),
  four_axis_cmd_sub_("four_axes/command", &AttitudeController::fourAxisCommandCallback, this ),
  pwm_info_sub_("motor_info", &AttitudeController::pwmInfoCallback, this),
  rpy_gain_sub_("rpy/gain", &AttitudeController::rpyGainCallback, this),
  pwm_test_sub_("pwm_test", &AttitudeController::pwmTestCallback, this ),
  p_matrix_pseudo_inverse_inertia_sub_("p_matrix_pseudo_inverse_inertia", &AttitudeController::pMatrixInertiaCallback, this),
  torque_allocation_matrix_inv_sub_("torque_allocation_matrix_inv", &AttitudeController::torqueAllocationMatrixInvCallback, this),
  wrench_allocation_matrix_sub_("wrench_allocation_matrix", &AttitudeController::wrenchAllocationMatrixCallback, this),
  differential_allocation_config_sub_("differential_allocation_config", &AttitudeController::differentialAllocationConfigCallback, this),
  offset_rot_sub_("desire_coordinate", &AttitudeController::offsetRotCallback, this ),
  att_control_srv_("set_attitude_control", &AttitudeController::setAttitudeControlCallback, this),
  esc_telem_pub_("esc_telem", &esc_telem_msg_),
  gimbal_control_pub_("servo/target_states_fc", &gimbal_control_msg_)
{
}

void AttitudeController::init(TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim2, StateEstimate* estimator,
                              DShot* dshot, DirectServo* servo, BatteryStatus* bat, ros::NodeHandle* nh, osMutexId* mutex)
{

  pwm_htim1_ = htim1;
  pwm_htim2_ = htim2;
  nh_ = nh;
  estimator_ = estimator;
  dshot_ = dshot;
  servo_ = servo;
  bat_ = bat;
  mutex_ = mutex;

  if(!dshot_)
    {
      HAL_TIM_PWM_Stop(pwm_htim1_, TIM_CHANNEL_1);
      HAL_TIM_Base_Stop(pwm_htim1_);
      HAL_TIM_Base_DeInit(pwm_htim1_);

      pwm_htim1_->Init.Prescaler = 3;
      pwm_htim1_->Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
      pwm_htim1_->Init.Period = 50000;

      TIM_OC_InitTypeDef sConfigOC = {0};
      sConfigOC.OCMode = TIM_OCMODE_PWM1;
      sConfigOC.Pulse = 1000;
      sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
      sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

      while(HAL_TIM_Base_Init(pwm_htim1_) != HAL_OK);
      while(HAL_TIM_PWM_Init(pwm_htim1_) != HAL_OK);
      while(HAL_TIM_PWM_ConfigChannel(pwm_htim1_, &sConfigOC, TIM_CHANNEL_1) != HAL_OK);

      if (pwm_htim1_->hdma[TIM_DMA_ID_UPDATE] != NULL) {
        HAL_DMA_DeInit(pwm_htim1_->hdma[TIM_DMA_ID_UPDATE]);
        pwm_htim1_->hdma[TIM_DMA_ID_UPDATE] = NULL;
      }

      HAL_TIM_Base_Start(pwm_htim1_);

      HAL_TIM_PWM_Start(pwm_htim1_, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(pwm_htim1_, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(pwm_htim1_, TIM_CHANNEL_3);
      HAL_TIM_PWM_Start(pwm_htim1_, TIM_CHANNEL_4);
    }

  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_4);

  nh_->advertise(pwms_pub_);
  nh_->advertise(control_term_pub_);
  nh_->advertise(control_feedback_state_pub_);
  nh_->advertise(esc_telem_pub_);
  nh_->advertise(gimbal_control_pub_);

  nh_->subscribe(four_axis_cmd_sub_);
  nh_->subscribe(pwm_info_sub_);
  nh_->subscribe(rpy_gain_sub_);
  nh_->subscribe(pwm_test_sub_);
  nh_->subscribe(p_matrix_pseudo_inverse_inertia_sub_);
  nh_->subscribe(torque_allocation_matrix_inv_sub_);
  nh_->subscribe(wrench_allocation_matrix_sub_);
  nh_->subscribe(differential_allocation_config_sub_);
  nh_->subscribe(offset_rot_sub_);

  nh_->advertiseService(att_control_srv_);

  baseInit();
}
#endif

void AttitudeController::baseInit()
{
 // base param for uav model
  motor_number_ = 0;
  uav_model_ = -1;
  rotor_devider_ = 1;

  start_control_flag_ = false;
  force_landing_flag_ = false;
  att_control_flag_ = true;

  // pwm
  pwm_conversion_mode_ = -1;
  min_duty_ = IDLE_DUTY;
  max_duty_ = IDLE_DUTY; //should assign right value from PC(ros)
  min_thrust_ = 0;
  force_landing_thrust_ = 0;
  pwm_pub_last_time_ = 0;
  pwm_test_flag_ = false;

  // voltage
  motor_info_.resize(0);
  v_factor_ = 1;
  motor_ref_index_ = 0;
  voltage_update_last_time_ = 0;

  control_term_pub_last_time_ = 0;
  control_feedback_state_pub_last_time_ = 0;

  differential_allocation_mode_ = spinal::DifferentialAllocationConfig::STATIC;
  wrench_allocation_matrix_received_ = false;
  differential_allocator_initialized_ = false;
  differential_allocation_damping_ = 0.03f;
  differential_allocation_wrench_error_gain_ = 20.0f;
  differential_allocation_wrench_rate_feedforward_gain_ = 1.0f;
  differential_allocation_wrench_rate_filter_ = 0.15f;
  differential_allocation_force_weight_ = 1.0f;
  differential_allocation_torque_weight_ = 5.0f;
  differential_allocation_thrust_rate_weight_ = 1.0f;
  differential_allocation_servo_rate_weight_ = 5.0f;
  differential_allocation_servo_center_gain_ = 0.2f;
  differential_allocation_thrust_balance_gain_ = 0.1f;
  differential_allocation_max_thrust_rate_ = 20.0f;
  differential_allocation_max_servo_rate_ = 2.0f;
  differential_allocation_max_servo_acceleration_ = 10.0f;
  differential_allocation_min_servo_angle_ = -4.0f;
  differential_allocation_max_servo_angle_ = 4.0f;
  differential_allocation_servo_angle_limit_enabled_ = true;
  differential_allocation_thrust_time_constant_ = 0.04f;
  differential_allocation_servo_time_constant_ = 0.05f;
  differential_uniform_scale_ = 1.0f;
  differential_limiting_input_ = -1;
  differential_limiting_bound_reason_ = 0;
#ifdef SIMULATION
  differential_allocation_debug_pub_last_time_ = 0;
  gimbal_state_last_time_ = 0;
  differential_jacobian_min_singular_value_ = 0.0f;
  differential_jacobian_max_singular_value_ = 0.0f;
  differential_jacobian_condition_number_ = 0.0f;
  for(int rotor = 0; rotor < MAX_MOTOR_NUMBER; ++rotor)
    {
      gimbal_state_received_[rotor] = false;
      measured_gimbal_angle_[rotor] = 0.0f;
      differential_unscaled_normalized_rate_[rotor] = 0.0f;
    }
#endif
  for(int row = 0; row < 6; ++row)
    {
      previous_desired_wrench_[row] = 0.0f;
      filtered_desired_wrench_rate_[row] = 0.0f;
      for(int col = 0; col < MAX_MOTOR_NUMBER; ++col)
        wrench_allocation_matrix_[row][col] = 0.0f;
    }

  // frame
  offset_rot_.identity();

  reset();
}

void AttitudeController::pwmsControl(void)
{
  /* target thrust -> target pwm */
  pwmConversion();

#ifdef SIMULATION
  /* control result publish */
  if(HAL_GetTick() - control_term_pub_last_time_ > CONTROL_TERM_PUB_INTERVAL)
    {
      control_term_pub_last_time_ = HAL_GetTick();
      control_term_pub_.publish(control_term_msg_);
    }

  if(HAL_GetTick() - control_feedback_state_pub_last_time_ > CONTROL_FEEDBACK_STATE_PUB_INTERVAL)
    {
      control_feedback_state_pub_last_time_ = HAL_GetTick();
      control_feedback_state_pub_.publish(control_feedback_state_msg_);
    }

  if(HAL_GetTick() - pwm_pub_last_time_ > PWM_PUB_INTERVAL)
    {
      pwm_pub_last_time_ = HAL_GetTick();
      pwms_pub_.publish(pwms_msg_);
    }

#else
  /* control result publish */
  if(uav_model_ == spinal::UavInfo::DRAGON)
    {
      if(HAL_GetTick() - control_feedback_state_pub_last_time_ > CONTROL_FEEDBACK_STATE_PUB_INTERVAL)
        {
          control_feedback_state_pub_last_time_ = HAL_GetTick();
          control_feedback_state_pub_.publish(&control_feedback_state_msg_);
        }
    }
  else
    {
      if(HAL_GetTick() - control_term_pub_last_time_ > CONTROL_TERM_PUB_INTERVAL)
        {
          control_term_pub_last_time_ = HAL_GetTick();
          control_term_pub_.publish(&control_term_msg_);
        }
    }

  if(HAL_GetTick() - pwm_pub_last_time_ > PWM_PUB_INTERVAL)
    {
      pwm_pub_last_time_ = HAL_GetTick();
      pwms_pub_.publish(&pwms_msg_);
    }

  /* nerve comm type */
#if NERVE_COMM
  for(int i = 0; i < motor_number_; i++) {
#if MOTOR_TEST

    if (i == (HAL_GetTick() / 2000) % motor_number_)
      Spine::setMotorPwm(200, i);
    else
      Spine::setMotorPwm(0, i);
#else
    Spine::setMotorPwm(target_pwm_[i] * 2000 - 1000, i);
#endif
  }
#endif

  if(dshot_)
    {
      /* direct pwm type */
      uint16_t motor_value[4] = { 0, 0, 0, 0 };
      for (int i = 0; i < 4; i++)
        {
          // target_pwm_: 0.5 ~ 1.0
          uint16_t motor_v = (uint16_t)((target_pwm_[i] - 0.5) / 0.5 * DSHOT_RANGE + DSHOT_MIN_THROTTLE);

          if (motor_v > DSHOT_MAX_THROTTLE)
            motor_v = DSHOT_MAX_THROTTLE;
          else if (motor_v < DSHOT_MIN_THROTTLE)
            motor_v = DSHOT_MIN_THROTTLE;
    
          motor_value[i] = motor_v;
        }

      dshot_->write(motor_value, dshot_->is_telemetry_);

      if (dshot_->is_telemetry_)
        {
          if (dshot_->esc_reader_.is_update_all_msg_)
            {
              esc_telem_msg_.stamp = nh_->now();
              esc_telem_msg_.esc_telemetry_1 = dshot_->esc_reader_.esc_msg_1_;
              esc_telem_msg_.esc_telemetry_2 = dshot_->esc_reader_.esc_msg_2_;
              esc_telem_msg_.esc_telemetry_3 = dshot_->esc_reader_.esc_msg_3_;
              esc_telem_msg_.esc_telemetry_4 = dshot_->esc_reader_.esc_msg_4_;
              esc_telem_pub_.publish(&esc_telem_msg_);

              float voltage_ave = (float)(dshot_->esc_reader_.esc_msg_1_.voltage + dshot_->esc_reader_.esc_msg_2_.voltage +
                                          dshot_->esc_reader_.esc_msg_3_.voltage + dshot_->esc_reader_.esc_msg_4_.voltage) / 400.0;
              bat_->update(voltage_ave);

              dshot_->esc_reader_.is_update_all_msg_ = false;
            }
        }
    }
  else
    {
      pwm_htim1_->Instance->CCR1 = (uint32_t)(target_pwm_[0] * pwm_htim1_->Init.Period);
      pwm_htim1_->Instance->CCR2 = (uint32_t)(target_pwm_[1] * pwm_htim1_->Init.Period);
      pwm_htim1_->Instance->CCR3 = (uint32_t)(target_pwm_[2] * pwm_htim1_->Init.Period);
      pwm_htim1_->Instance->CCR4 = (uint32_t)(target_pwm_[3] * pwm_htim1_->Init.Period);
    }

  pwm_htim2_->Instance->CCR1 = (uint32_t)(target_pwm_[4] * pwm_htim2_->Init.Period);
  pwm_htim2_->Instance->CCR2 = (uint32_t)(target_pwm_[5] * pwm_htim2_->Init.Period);
  pwm_htim2_->Instance->CCR3 = (uint32_t)(target_pwm_[6] * pwm_htim2_->Init.Period);
  pwm_htim2_->Instance->CCR4 = (uint32_t)(target_pwm_[7] * pwm_htim2_->Init.Period);

#endif
}

void AttitudeController::update(void)
{
#ifndef SIMULATION
  /* mutex to wait for the completion of update of ros callback function */
  if(mutex_ != NULL)
    {
      osMutexWait(*mutex_, osWaitForever);
      osMutexRelease(*mutex_);
    }
#endif

  if(start_control_flag_ && att_control_flag_)
    {
      /* failsafe 1: check the timeout of the flight command receive process */
      if(failsafe_ && !force_landing_flag_ &&
         (int32_t)(HAL_GetTick() - flight_command_last_stamp_) > FLIGHT_COMMAND_TIMEOUT)
        {
          /* timeout => start force landing */
#ifdef SIMULATION
          ROS_ERROR("failsafe: cannot connect with ROS. time now: %d, time last stamp: %d", HAL_GetTick(), flight_command_last_stamp_);
#else
          nh_->logerror("failsafe: cannot connect with ROS");
#endif
          setForceLandingFlag(true);
        }

#ifdef SIMULATION

      if(prev_time_ < 0) DELTA_T = 0;
      else DELTA_T = ros::Time::now().toSec() - prev_time_;
      prev_time_ = ros::Time::now().toSec();
#endif

      ap::Matrix3f base_rot = estimator_->getAttEstimator()->getRotation();
      ap::Vector3f base_vel = estimator_->getAttEstimator()->getAngular();
      ap::Matrix3f rot = base_rot * offset_rot_.transposed();
      ap::Vector3f vel = offset_rot_ * base_vel;
      ap::Vector3f angles; // euler angles
      rot.to_euler(&angles.x, &angles.y, &angles.z);

      /* failsafe 3: too large tile angle */
      if(!force_landing_flag_  && (fabs(angles[X]) > MAX_TILT_ANGLE || fabs(angles[Y]) > MAX_TILT_ANGLE))
        {
#ifdef SIMULATION
          ROS_ERROR("failsafe: the roll pitch angles are too large, roll: %f (%f), pitch: %f (%f)",
                    angles[X], MAX_TILT_ANGLE, angles[Y], MAX_TILT_ANGLE);
#else
          nh_->logerror("failsafe: the roll pitch angles are too large");
#endif
          setForceLandingFlag(true);
          error_angle_i_[X] = 0;
          error_angle_i_[Y] = 0;
        }

      /* Force Landing Flag */
      if(force_landing_flag_)
        {
          target_angle_[X] = 0;
          target_angle_[Y] = 0;
          target_angle_[Z] = 0;

          for(int i = 0; i < motor_number_; i++) extra_yaw_pi_term_[i] = 0;
        }

      // linear control method
      {
        /* gyro moment */
        ap::Vector3f gyro_moment = vel % (inertia_ * vel);
#ifdef SIMULATION
        std_msgs::Float32MultiArray anti_gyro_msg;
#endif

        float error_angle[3];
        for(int axis = 0; axis < 3; axis++)
          {
            error_angle[axis] = target_angle_[axis] - angles[axis];
            if(integrate_flag_) error_angle_i_[axis] += error_angle[axis] * DELTA_T;

            if(axis == X)
              {
                control_feedback_state_msg_.roll_p = error_angle[axis] * 1000;
                control_feedback_state_msg_.roll_i = error_angle_i_[axis] * 1000;
                control_feedback_state_msg_.roll_d = vel[axis]  * 1000;
              }
            if(axis == Y)
              {
                control_feedback_state_msg_.pitch_p = error_angle[axis] * 1000;
                control_feedback_state_msg_.pitch_i = error_angle_i_[axis] * 1000;
                control_feedback_state_msg_.pitch_d = vel[axis]  * 1000;

              }
            if(axis == Z)
              {
                control_feedback_state_msg_.yaw_d = vel[axis] * 1000;
              }
          }

        float p_term = 0;
        float i_term = 0;
        float d_term = 0;
        for(int i = 0; i < motor_number_; i++)
          {
            for(int axis = 0; axis < 3; axis++)
              {
                p_term = error_angle[axis] * thrust_p_gain_[i][axis];
                i_term = error_angle_i_[axis] * thrust_i_gain_[i][axis];
                d_term = -vel[axis] * thrust_d_gain_[i][axis];
                if(axis == X)
                  {
                    roll_pitch_term_[i] = p_term + i_term + d_term; // [N]
                    control_term_msg_.motors[i].roll_p = p_term * 1000;
                    control_term_msg_.motors[i].roll_i= i_term * 1000;
                    control_term_msg_.motors[i].roll_d = d_term * 1000;
                  }
                if(axis == Y)
                  {
                    roll_pitch_term_[i] += (p_term + i_term + d_term); // [N]
                    control_term_msg_.motors[i].pitch_p = p_term * 1000;
                    control_term_msg_.motors[i].pitch_i = i_term * 1000;
                    control_term_msg_.motors[i].pitch_d = d_term * 1000;
                  }
                if(axis == Z)
                  {
                    yaw_term_[i] = extra_yaw_pi_term_[i] + d_term;
                    control_term_msg_.motors[i].yaw_d = d_term * 1000; //d_term;
                  }
              }

            /* gyro moment compensation */
            float gyro_moment_compensate =
              p_matrix_pseudo_inverse_[i][0] * gyro_moment.x +
              p_matrix_pseudo_inverse_[i][1] * gyro_moment.y +
              p_matrix_pseudo_inverse_[i][2] * gyro_moment.z;
            roll_pitch_term_[i] += gyro_moment_compensate;

#ifdef SIMULATION
            anti_gyro_msg.data.push_back(gyro_moment_compensate);
#endif
          }

#ifdef SIMULATION
        anti_gyro_pub_.publish(anti_gyro_msg);
#endif
      }

      if(force_landing_flag_)
        {
          float total_thrust = 0;
          /* sum */
          for(int i = 0; i < motor_number_; i++) total_thrust += base_thrust_term_[i];
          /* average */
          float average_thrust = total_thrust / motor_number_;

          if(average_thrust > force_landing_thrust_)
            {
              for(int i = 0; i < motor_number_; i++)
                base_thrust_term_[i] -= (base_thrust_term_[i] / average_thrust * FORCE_LANDING_INTEGRAL);
            }
        }
    }
  /* target thrust -> target pwm -> HAL */
  pwmsControl();
}


void AttitudeController::reset(void)
{
  for(int i = 0; i < MAX_MOTOR_NUMBER; i++)
    {
      target_thrust_[i] = 0;
      target_pwm_[i] = IDLE_DUTY;
      pwm_test_value_[i] = IDLE_DUTY;

      base_thrust_term_[i] = 0;
      roll_pitch_term_[i] = 0;
      yaw_term_[i] = 0;
      extra_yaw_pi_term_[i] = 0;
      allocator_thrust_[i] = 0.0f;
      allocator_gimbal_angle_[i] = 0.0f;
      allocator_thrust_command_[i] = 0.0f;
      allocator_gimbal_command_[i] = 0.0f;
      allocator_thrust_rate_[i] = 0.0f;
      allocator_gimbal_rate_[i] = 0.0f;

      for (int j = 0; j < 3; j++)
        {
          thrust_p_gain_[i][j] = 0;
          thrust_i_gain_[i][j] = 0;
          thrust_d_gain_[i][j] = 0;
          torque_allocation_matrix_inv_[i][j] = 0.0;
        }
    }

  for(int i = 0; i < 3; i++)
    {
      target_angle_[i] = 0;
      error_angle_i_[i] = 0;

      torque_p_gain_[i] = 0;
      torque_i_gain_[i] = 0;
      torque_d_gain_[i] = 0;
    }

  max_yaw_term_index_ = -1;
  integrate_flag_ = false;
  differential_allocator_initialized_ = false;
  for(int i = 0; i < 6; ++i)
    {
      previous_desired_wrench_[i] = 0.0f;
      filtered_desired_wrench_rate_[i] = 0.0f;
    }

  /* failsafe */
  failsafe_ = false;
  flight_command_last_stamp_ = HAL_GetTick();

#ifdef SIMULATION
  prev_time_ = -1;
#endif
  for(int i = 0; i < motor_number_ / (rotor_coef_); i++){
    target_gimbal_angles_[i]=0.0f;
    prev_gimbal_angles_[i]=0.0f;
  }

}

void AttitudeController::fourAxisCommandCallback( const spinal::FourAxisCommand &cmd_msg)
{
  if(!start_control_flag_) return; //do not receive command


  /* failsafe: if the pitch and roll angle is too big, start force landing */
  if(fabs(cmd_msg.angles[0]) > MAX_TILT_ANGLE || fabs(cmd_msg.angles[1]) > MAX_TILT_ANGLE )
    {
      setForceLandingFlag(true);
#ifdef SIMULATION
      ROS_ERROR("failsafe: target angles are too large, roll: %f (%f), pitch: %f ()%f",
                target_angle_[X], MAX_TILT_ANGLE, target_angle_[Y], MAX_TILT_ANGLE);
#else
      nh_->logerror("failsafe: target angles are too large");
#endif
    }

  /* check the number of motor which should be equal to the ros thrust */
#ifdef SIMULATION
  if(cmd_msg.base_thrust.size() != motor_number_)
    {
      ROS_ERROR("fource axis commnd: motor number is not identical between fc(%d) and pc(%ld)", motor_number_, cmd_msg.base_thrust.size());
      return;
    }
#else
  if(cmd_msg.base_thrust_length != motor_number_)
    {
      nh_->logerror("fource axis commnd: motor number is not identical between fc and pc");
      return;
    }
#endif

  if(force_landing_flag_)
    {
      float total_thrust = 0;
      for(int i = 0; i < motor_number_; i++) total_thrust += cmd_msg.base_thrust[i];
      float average_thrust = total_thrust / motor_number_;
      if(average_thrust < force_landing_thrust_) return;
    }


#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  /* start failsafe func if not activate */
  if(!failsafe_) failsafe_ = true;
  flight_command_last_stamp_ = HAL_GetTick();

  target_angle_[X] = cmd_msg.angles[0];
  target_angle_[Y] = cmd_msg.angles[1];
  int max_yaw_term_index = max_yaw_term_index_;
  float max_yaw_thrust_d_gain = 0;
  if(max_yaw_term_index != -1)
    max_yaw_thrust_d_gain = thrust_d_gain_[max_yaw_term_index][Z];
  for(int i = 0; i < motor_number_; i++)
    {
      // base thrust is about the z control
      base_thrust_term_[i] = cmd_msg.base_thrust[i];

      // reconstruct the pi term for yaw (temporary measure for pwm saturation avoidance)
      if(max_yaw_term_index != -1)
        extra_yaw_pi_term_[i] = cmd_msg.angles[Z] * thrust_d_gain_[i][Z] / max_yaw_thrust_d_gain;
      else
        extra_yaw_pi_term_[i] = 0;
    }

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::pwmInfoCallback( const spinal::PwmInfo &info_msg)
{
#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  force_landing_thrust_ = info_msg.force_landing_thrust;

  min_duty_ = info_msg.min_pwm;
  max_duty_ = info_msg.max_pwm;
  pwm_conversion_mode_ = info_msg.pwm_conversion_mode;

  min_thrust_ = info_msg.min_thrust; // make a variant min_duty_

  motor_info_.resize(0);

#ifdef SIMULATION
  for(int i = 0; i < info_msg.motor_info.size(); i++)
#else
    for(int i = 0; i < info_msg.motor_info_length; i++)
#endif
      {
        motor_info_.push_back(info_msg.motor_info[i]);
      }

#ifdef SIMULATION
  if(sim_voltage_== 0) sim_voltage_ = motor_info_[0].voltage;
#endif

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::rpyGainCallback( const spinal::RollPitchYawTerms &gain_msg)
{
  if(motor_number_ == 0) return; //not be activated

  /* check the number of motor which should be equal to the ros thrust */
#ifdef SIMULATION
  if(gain_msg.motors.size() != motor_number_ && gain_msg.motors.size() != 1)
    {
      ROS_ERROR("rpy gain: motor number is not identical between fc:%d and pc:%d", motor_number_, (int)gain_msg.motors.size());
      return;
    }
#else
  if(gain_msg.motors_length != motor_number_ && gain_msg.motors_length != 1)
    {
      nh_->logerror("rpy gain: motor number is not identical between fc and pc");
      return;
    }
#endif

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

#ifdef SIMULATION
  if(gain_msg.motors.size() == 1)
#else
  if(gain_msg.motors_length == 1)
#endif
    {
      torque_p_gain_[X] = gain_msg.motors[0].roll_p * 0.001f;
      torque_p_gain_[Y] = gain_msg.motors[0].pitch_p * 0.001f;
      torque_i_gain_[X] = gain_msg.motors[0].roll_i * 0.001f;
      torque_i_gain_[Y] = gain_msg.motors[0].pitch_i * 0.001f;
      torque_d_gain_[X] = gain_msg.motors[0].roll_d * 0.001f;
      torque_d_gain_[Y] = gain_msg.motors[0].pitch_d * 0.001f;
      torque_d_gain_[Z] = gain_msg.motors[0].yaw_d * 0.001f;

      thrustGainMapping(); // gain mapping
    }
  else
    {
      for(int i = 0; i < motor_number_; i++)
        {
          thrust_p_gain_[i][X] = gain_msg.motors[i].roll_p * 0.001f;
          thrust_i_gain_[i][X] = gain_msg.motors[i].roll_i * 0.001f;
          thrust_d_gain_[i][X] = gain_msg.motors[i].roll_d * 0.001f;
          thrust_p_gain_[i][Y] = gain_msg.motors[i].pitch_p * 0.001f;
          thrust_i_gain_[i][Y] = gain_msg.motors[i].pitch_i * 0.001f;
          thrust_d_gain_[i][Y] = gain_msg.motors[i].pitch_d * 0.001f;
          thrust_d_gain_[i][Z] = gain_msg.motors[i].yaw_d * 0.001f;
        }
    }
  maxYawGainIndex();

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::torqueAllocationMatrixInvCallback(const spinal::TorqueAllocationMatrixInv& msg)
{
  if(motor_number_ == 0 || !start_control_flag_) return;

#ifdef SIMULATION
  if(msg.rows.size() != motor_number_)
    {
      ROS_ERROR("torqueAllocationMatrixInvCallback: motor number is not identical between fc(%d) and pc(%ld)", motor_number_, msg.rows.size());
      return;
    }
#else
  if(msg.rows_length != motor_number_)
    {
      nh_->logerror("torqueAllocationMatrixInvCallback: motor number is not identical between fc and pc");
      return;
    }
#endif

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  for (int i = 0; i < motor_number_; i++)
    {
      torque_allocation_matrix_inv_[i][X] = msg.rows[i].x * 0.001f;
      torque_allocation_matrix_inv_[i][Y] = msg.rows[i].y * 0.001f;
      torque_allocation_matrix_inv_[i][Z] = msg.rows[i].z * 0.001f;
    }

  thrustGainMapping(); // gain mapping

  maxYawGainIndex();

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::wrenchAllocationMatrixCallback(const spinal::WrenchAllocationMatrix& msg)
{
  if(motor_number_ == 0) return;

#ifdef SIMULATION
  if(msg.force_rows.size() != motor_number_ || msg.torque_rows.size() != motor_number_)
    {
      ROS_ERROR("wrenchAllocationMatrixCallback: expected %d columns, received %ld force and %ld torque columns",
                motor_number_, msg.force_rows.size(), msg.torque_rows.size());
      return;
    }
#else
  if(msg.force_rows_length != motor_number_ || msg.torque_rows_length != motor_number_)
    {
      nh_->logerror("wrenchAllocationMatrixCallback: allocation matrix size mismatch");
      return;
    }
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  for(int col = 0; col < motor_number_; ++col)
    {
      wrench_allocation_matrix_[0][col] = msg.force_rows[col].x * 0.001f;
      wrench_allocation_matrix_[1][col] = msg.force_rows[col].y * 0.001f;
      wrench_allocation_matrix_[2][col] = msg.force_rows[col].z * 0.001f;
      wrench_allocation_matrix_[3][col] = msg.torque_rows[col].x * 0.001f;
      wrench_allocation_matrix_[4][col] = msg.torque_rows[col].y * 0.001f;
      wrench_allocation_matrix_[5][col] = msg.torque_rows[col].z * 0.001f;
    }
  wrench_allocation_matrix_received_ = true;

#ifndef SIMULATION
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::differentialAllocationConfigCallback(const spinal::DifferentialAllocationConfig& msg)
{
#ifndef SIMULATION
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  uint8_t mode = msg.mode;
  if(hover_thrust_reference_enabled_ != msg.hover_thrust_reference_enabled ||
     differential_allocation_mode_ != msg.mode)
    {
      hover_thrust_reference_ = 0.0f;
      hover_thrust_sample_time_ = 0.0f;
      hover_thrust_sample_integral_ = 0.0f;
    }
  hover_thrust_reference_enabled_ = msg.hover_thrust_reference_enabled;
  hover_thrust_capture_allowed_ = msg.hover_thrust_capture_allowed;
  hover_thrust_capture_stamp_ = HAL_GetTick();
  if(mode != spinal::DifferentialAllocationConfig::DIFFERENTIAL &&
     mode != spinal::DifferentialAllocationConfig::DYNAMIC_LIMIT_DECOMPOSITION)
    mode = spinal::DifferentialAllocationConfig::STATIC;
  if(mode != differential_allocation_mode_)
    differential_allocator_initialized_ = false;
  differential_allocation_mode_ = mode;
  differential_allocation_damping_ = fmaxf(0.0f, msg.damping);
  differential_allocation_wrench_error_gain_ = fmaxf(0.0f, msg.wrench_error_gain);
  differential_allocation_wrench_rate_feedforward_gain_ = fmaxf(0.0f, msg.wrench_rate_feedforward_gain);
  differential_allocation_wrench_rate_filter_ = fminf(1.0f, fmaxf(0.0f, msg.wrench_rate_filter));
  differential_allocation_force_weight_ = fmaxf(1e-6f, msg.force_weight);
  differential_allocation_torque_weight_ = fmaxf(1e-6f, msg.torque_weight);
  differential_allocation_thrust_rate_weight_ = fmaxf(1e-6f, msg.thrust_rate_weight);
  differential_allocation_servo_rate_weight_ = fmaxf(1e-6f, msg.servo_rate_weight);
  differential_allocation_servo_center_gain_ = fmaxf(0.0f, msg.servo_center_gain);
  differential_allocation_thrust_balance_gain_ = fmaxf(0.0f, msg.thrust_balance_gain);
  differential_allocation_max_thrust_rate_ = fmaxf(0.0f, msg.max_thrust_rate);
  differential_allocation_max_servo_rate_ = fmaxf(0.0f, msg.max_servo_rate);
  differential_allocation_max_servo_acceleration_ = fmaxf(0.0f, msg.max_servo_acceleration);
  differential_allocation_min_servo_angle_ = fminf(msg.min_servo_angle, msg.max_servo_angle);
  differential_allocation_max_servo_angle_ = fmaxf(msg.min_servo_angle, msg.max_servo_angle);
  differential_allocation_servo_angle_limit_enabled_ = msg.servo_angle_limit_enabled;
  differential_allocation_thrust_time_constant_ = fmaxf(1e-3f, msg.thrust_time_constant);
  differential_allocation_servo_time_constant_ = fmaxf(1e-3f, msg.servo_time_constant);

#ifndef SIMULATION
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

#ifdef SIMULATION
void AttitudeController::jointStateCallback(const sensor_msgs::JointState& msg)
{
  bool received = false;
  for(size_t joint = 0; joint < msg.name.size() && joint < msg.position.size(); ++joint)
    {
      const std::string& name = msg.name[joint];
      if(name.compare(0, 6, "gimbal") != 0) continue;
      const char* index_begin = name.c_str() + 6;
      char* index_end = NULL;
      const long one_based_index = strtol(index_begin, &index_end, 10);
      if(index_end == index_begin || one_based_index <= 0 || one_based_index > MAX_MOTOR_NUMBER) continue;
      const int rotor = static_cast<int>(one_based_index - 1);
      if(!std::isfinite(msg.position[joint])) continue;
      measured_gimbal_angle_[rotor] = static_cast<float>(msg.position[joint]);
      gimbal_state_received_[rotor] = true;
      received = true;
    }
  if(received) gimbal_state_last_time_ = HAL_GetTick();
}
#endif

void AttitudeController::thrustGainMapping()
{
  for(int i = 0; i < motor_number_; i++)
    {
      thrust_p_gain_[i][X] = torque_allocation_matrix_inv_[i][X] * torque_p_gain_[X];
      thrust_i_gain_[i][X] = torque_allocation_matrix_inv_[i][X] * torque_i_gain_[X];
      thrust_d_gain_[i][X] = torque_allocation_matrix_inv_[i][X] * torque_d_gain_[X];
      thrust_p_gain_[i][Y] = torque_allocation_matrix_inv_[i][Y] * torque_p_gain_[Y];
      thrust_i_gain_[i][Y] = torque_allocation_matrix_inv_[i][Y] * torque_i_gain_[Y];
      thrust_d_gain_[i][Y] = torque_allocation_matrix_inv_[i][Y] * torque_d_gain_[Y];
      thrust_d_gain_[i][Z] = torque_allocation_matrix_inv_[i][Z] * torque_d_gain_[Z];
    }

}

void AttitudeController::maxYawGainIndex()
{
  float max_abs_yaw_gain = 0;
  max_yaw_term_index_ = -1;
  for(int i = 0; i < motor_number_; i++)
    {
      const float abs_yaw_gain = fabs(thrust_d_gain_[i][Z]);
      /* Keep the original sign; only the magnitude determines the reference. */
      if(abs_yaw_gain > max_abs_yaw_gain)
        {
          max_abs_yaw_gain = abs_yaw_gain;
          max_yaw_term_index_ = i;
        }
    }
}

void AttitudeController::pwmTestCallback(const spinal::PwmTest& pwm_msg)
{
#ifndef SIMULATION  
  if(pwm_msg.pwms_length && !pwm_test_flag_)
    {
      pwm_test_flag_ = true;
      nh_->logwarn("Enter pwm test mode");
    }
  else if(!pwm_msg.pwms_length && pwm_test_flag_)
    {
      pwm_test_flag_ = false;
      nh_->logwarn("Escape from pwm test mode");
      return;
    }

  if(pwm_msg.motor_index_length)
    {
      /*Individual test mode*/
      if(pwm_msg.motor_index_length != pwm_msg.pwms_length)
        {
          nh_->logerror("The number of index does not match the number of pwms.");
          return;
        }
      for(int i = 0; i < pwm_msg.motor_index_length; i++){
        int motor_index = pwm_msg.motor_index[i];
                /*fail safe*/
        if (pwm_msg.pwms[i] >= IDLE_DUTY && pwm_msg.pwms[i] <= MAX_PWM)
          {
            pwm_test_value_[motor_index] = pwm_msg.pwms[i];
          }
        else
         {
            nh_->logwarn("FAIL SAFE!  Invaild PWM value for motor");
            pwm_test_value_[motor_index] = IDLE_DUTY;
          }
      }
    }
  else
    {
      /*Simultaneous test mode*/
      for(int i = 0; i < MAX_MOTOR_NUMBER; i++){
        /*fail safe*/
        if (pwm_msg.pwms[0] >= IDLE_DUTY && pwm_msg.pwms[0] <= MAX_PWM)
          {
            pwm_test_value_[i] = pwm_msg.pwms[0];
          }
        else
          {
            nh_->logwarn("FAIL SAFE!  Invaild PWM value for motors");
            pwm_test_value_[i] = IDLE_DUTY;
          }
      }
    }
#endif
}

void AttitudeController::setStartControlFlag(bool start_control_flag)
{
  start_control_flag_ = start_control_flag;

  if(!start_control_flag_) reset();
}

void AttitudeController::setMotorNumber(uint16_t motor_number)
{
  /* check the motor number which has spine system */
  if(motor_number_ > 0)
    {
      if(motor_number_ != motor_number)
        {
          motor_number_ = 0;
#ifdef SIMULATION
          ROS_ERROR("ATTENTION: motor number is 0");
#else
          nh_->logerror("ATTENTION: motor number is 0");
#endif
        }
    }
  else
    {
	  if(motor_number == 0) return;

      size_t control_term_msg_size  = motor_number;

#ifdef SIMULATION
      pwms_msg_.motor_value.resize(motor_number);
      control_term_msg_.motors.resize(control_term_msg_size);
#else
      pwms_msg_.motor_value_length = motor_number;
      control_term_msg_.motors_length = control_term_msg_size;
      pwms_msg_.motor_value = new uint16_t[motor_number];
      control_term_msg_.motors = new spinal::RollPitchYawTerm[control_term_msg_size];
#endif
      for(int i = 0; i < motor_number; i++) pwms_msg_.motor_value[i] = 0;

      /* the initialize order is important */
      motor_number_ = motor_number ;
    }
}

void  AttitudeController::setUavModel(int8_t uav_model)
{
  /* check the uav model which has spine system */
  uav_model_ = uav_model;

  if(uav_model_ == spinal::UavInfo::DRAGON) {
    rotor_devider_ = 2; // dual-rotor
  }
}

void AttitudeController::pMatrixInertiaCallback(const spinal::PMatrixPseudoInverseWithInertia& msg)
{
  if(motor_number_ == 0) return;

#ifdef SIMULATION
  if(msg.pseudo_inverse.size() != motor_number_)
    {
      if(motor_number_ > 0) ROS_ERROR("p matrix pseudo inverse and inertia commnd: motor number is not identical between fc and pc");
      return;
    }
#else
  if(msg.pseudo_inverse_length != motor_number_)
    {
      nh_->logerror("p matrix pseudo inverse and inertia commnd: motor number is not identical between fc and pc");
      return;
    }
#endif

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  for(int i = 0; i < motor_number_; i ++)
    {
      p_matrix_pseudo_inverse_[i][0] = msg.pseudo_inverse[i].r * 0.001f;
      p_matrix_pseudo_inverse_[i][1] = msg.pseudo_inverse[i].p * 0.001f;
      p_matrix_pseudo_inverse_[i][2] = msg.pseudo_inverse[i].y * 0.001f;
    }

  /* inertia */
  inertia_ = ap::Matrix3f(msg.inertia[0] * 0.001f, msg.inertia[3] * 0.001f, msg.inertia[5] * 0.001f,
                          msg.inertia[3] * 0.001f, msg.inertia[1] * 0.001f, msg.inertia[4] * 0.001f,
                          msg.inertia[5] * 0.001f, msg.inertia[4] * 0.001f, msg.inertia[2] * 0.001f);

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::offsetRotCallback(const spinal::DesireCoord& msg)
{
  offset_rot_.from_euler(msg.roll, msg.pitch, msg.yaw);
}

bool AttitudeController::activated()
{
  /* uav model check and motor property */
  if(motor_number_ > 0 && uav_model_ >= spinal::UavInfo::DRONE && max_duty_ > min_duty_) return true;
  else return false;
}

bool AttitudeController::solveDifferentialAllocationSystem(const float matrix[6][6], const float rhs[6],
                                                           float solution[6])
{
  for(int row = 0; row < 6; ++row)
    {
      for(int col = 0; col < 6; ++col) differential_solver_augmented_[row][col] = matrix[row][col];
      differential_solver_augmented_[row][6] = rhs[row];
    }

  for(int pivot_col = 0; pivot_col < 6; ++pivot_col)
    {
      int pivot_row = pivot_col;
      float pivot_abs = fabsf(differential_solver_augmented_[pivot_row][pivot_col]);
      for(int row = pivot_col + 1; row < 6; ++row)
        {
          const float candidate_abs = fabsf(differential_solver_augmented_[row][pivot_col]);
          if(candidate_abs > pivot_abs)
            {
              pivot_abs = candidate_abs;
              pivot_row = row;
            }
        }
      if(pivot_abs < 1e-9f || !std::isfinite(pivot_abs)) return false;

      if(pivot_row != pivot_col)
        for(int col = pivot_col; col < 7; ++col)
          {
            const float tmp = differential_solver_augmented_[pivot_col][col];
            differential_solver_augmented_[pivot_col][col] = differential_solver_augmented_[pivot_row][col];
            differential_solver_augmented_[pivot_row][col] = tmp;
          }

      const float pivot = differential_solver_augmented_[pivot_col][pivot_col];
      for(int col = pivot_col; col < 7; ++col) differential_solver_augmented_[pivot_col][col] /= pivot;
      for(int row = 0; row < 6; ++row)
        {
          if(row == pivot_col) continue;
          const float factor = differential_solver_augmented_[row][pivot_col];
          for(int col = pivot_col; col < 7; ++col)
            differential_solver_augmented_[row][col] -= factor * differential_solver_augmented_[pivot_col][col];
        }
    }

  for(int row = 0; row < 6; ++row)
    {
      solution[row] = differential_solver_augmented_[row][6];
      if(!std::isfinite(solution[row])) return false;
    }
  return true;
}

bool AttitudeController::differentialAllocationUpdate(float dt, float maximum_thrust)
{
  if(gimbal_dof_ != 1 || rotor_coef_ != 2 || !wrench_allocation_matrix_received_) return false;
  if(motor_number_ <= 0 || motor_number_ > MAX_MOTOR_NUMBER || motor_number_ % rotor_coef_ != 0) return false;

  const int physical_rotor_number = motor_number_ / rotor_coef_;
  const bool dynamic_limit_decomposition =
      differential_allocation_mode_ ==
      spinal::DifferentialAllocationConfig::DYNAMIC_LIMIT_DECOMPOSITION;
  float minimum_thrust = 0.0f;
#ifndef SIMULATION
  minimum_thrust = fminf(fmaxf(min_thrust_ * rotor_devider_, 0.0f), maximum_thrust);
#endif
  for(int input = 0; input < motor_number_; ++input)
    differential_desired_virtual_force_[input] = target_thrust_[input];

  for(int row = 0; row < 6; ++row)
    {
      differential_desired_wrench_[row] = 0.0f;
      for(int input = 0; input < motor_number_; ++input)
        differential_desired_wrench_[row] +=
            wrench_allocation_matrix_[row][input] * differential_desired_virtual_force_[input];
    }

  if(!differential_allocator_initialized_)
    {
      for(int rotor = 0; rotor < physical_rotor_number; ++rotor)
        {
          const float tangential_force = differential_desired_virtual_force_[2 * rotor];
          const float axial_force = differential_desired_virtual_force_[2 * rotor + 1];
          float angle = atan2f(-tangential_force, axial_force);
          const float previous_angle = target_gimbal_angles_[rotor];
          while(angle - previous_angle > M_PI) angle -= 2.0f * M_PI;
          while(angle - previous_angle < -M_PI) angle += 2.0f * M_PI;
          allocator_thrust_[rotor] = fminf(fmaxf(ap::pythagorous2(tangential_force, axial_force), 0.0f),
                                           maximum_thrust);
          allocator_gimbal_angle_[rotor] = angle;
          if(differential_allocation_servo_angle_limit_enabled_)
            allocator_gimbal_angle_[rotor] =
                fminf(fmaxf(angle, differential_allocation_min_servo_angle_),
                      differential_allocation_max_servo_angle_);
#ifdef SIMULATION
          if(dynamic_limit_decomposition && gimbal_state_received_[rotor] &&
             HAL_GetTick() - gimbal_state_last_time_ < 200)
            allocator_gimbal_angle_[rotor] = measured_gimbal_angle_[rotor];
#endif
          allocator_thrust_command_[rotor] = allocator_thrust_[rotor];
          allocator_gimbal_command_[rotor] = allocator_gimbal_angle_[rotor];
          allocator_thrust_rate_[rotor] = 0.0f;
          allocator_gimbal_rate_[rotor] = 0.0f;
          target_thrust_[rotor] = allocator_thrust_command_[rotor];
          target_gimbal_angles_[rotor] = allocator_gimbal_command_[rotor];
        }
      for(int row = 0; row < 6; ++row)
        {
          previous_desired_wrench_[row] = differential_desired_wrench_[row];
          filtered_desired_wrench_rate_[row] = 0.0f;
        }
      differential_allocator_initialized_ = true;
      return true;
    }

  // A delayed sample must not be integrated into a large actuator step.
  if(!std::isfinite(dt) || dt <= 1e-5f || dt > 0.05f)
    {
      for(int rotor = 0; rotor < physical_rotor_number; ++rotor)
        {
          target_thrust_[rotor] = dynamic_limit_decomposition ? allocator_thrust_command_[rotor]
                                                               : allocator_thrust_[rotor];
          target_gimbal_angles_[rotor] = dynamic_limit_decomposition ? allocator_gimbal_command_[rotor]
                                                                      : allocator_gimbal_angle_[rotor];
        }
      return true;
    }

  if(dynamic_limit_decomposition)
    {
      const float thrust_response = fminf(1.0f, dt / differential_allocation_thrust_time_constant_);
      const float servo_response = fminf(1.0f, dt / differential_allocation_servo_time_constant_);
      for(int rotor = 0; rotor < physical_rotor_number; ++rotor)
        {
          allocator_thrust_[rotor] +=
              thrust_response * (allocator_thrust_command_[rotor] - allocator_thrust_[rotor]);
#ifdef SIMULATION
          if(gimbal_state_received_[rotor] && HAL_GetTick() - gimbal_state_last_time_ < 200)
            allocator_gimbal_angle_[rotor] = measured_gimbal_angle_[rotor];
          else
#endif
            allocator_gimbal_angle_[rotor] +=
                servo_response * (allocator_gimbal_command_[rotor] - allocator_gimbal_angle_[rotor]);
        }
    }

  for(int input = 0; input < motor_number_; ++input)
    differential_current_virtual_force_[input] = 0.0f;
  for(int row = 0; row < 6; ++row)
    {
      differential_current_wrench_[row] = 0.0f;
      for(int input = 0; input < motor_number_; ++input)
        differential_jacobian_[row][input] = 0.0f;
    }
  for(int rotor = 0; rotor < physical_rotor_number; ++rotor)
    {
      const float thrust = allocator_thrust_[rotor];
      const float angle = allocator_gimbal_angle_[rotor];
      const float sin_angle = sinf(angle);
      const float cos_angle = cosf(angle);
      differential_current_virtual_force_[2 * rotor] = -thrust * sin_angle;
      differential_current_virtual_force_[2 * rotor + 1] = thrust * cos_angle;
      for(int row = 0; row < 6; ++row)
        {
          differential_jacobian_[row][2 * rotor] =
              wrench_allocation_matrix_[row][2 * rotor] * (-sin_angle) +
              wrench_allocation_matrix_[row][2 * rotor + 1] * cos_angle;
          differential_jacobian_[row][2 * rotor + 1] =
              wrench_allocation_matrix_[row][2 * rotor] * (-thrust * cos_angle) +
              wrench_allocation_matrix_[row][2 * rotor + 1] * (-thrust * sin_angle);
        }
    }
  for(int row = 0; row < 6; ++row)
    for(int input = 0; input < motor_number_; ++input)
      differential_current_wrench_[row] +=
          wrench_allocation_matrix_[row][input] * differential_current_virtual_force_[input];

  for(int row = 0; row < 6; ++row)
    {
      const float raw_desired_rate =
          (differential_desired_wrench_[row] - previous_desired_wrench_[row]) / dt;
      filtered_desired_wrench_rate_[row] += differential_allocation_wrench_rate_filter_ *
                                            (raw_desired_rate - filtered_desired_wrench_rate_[row]);
      previous_desired_wrench_[row] = differential_desired_wrench_[row];
      const float wrench_rate_reference =
          differential_allocation_wrench_rate_feedforward_gain_ * filtered_desired_wrench_rate_[row] +
          differential_allocation_wrench_error_gain_ *
              (differential_desired_wrench_[row] - differential_current_wrench_[row]);
      const float row_weight = row < 3 ? differential_allocation_force_weight_
                                       : differential_allocation_torque_weight_;
      differential_weighted_wrench_rate_reference_[row] = row_weight * wrench_rate_reference;
      for(int input = 0; input < motor_number_; ++input)
        differential_weighted_jacobian_[row][input] = row_weight * differential_jacobian_[row][input];
    }

  for(int rotor = 0; rotor < physical_rotor_number; ++rotor)
    {
      if(dynamic_limit_decomposition)
        {
          const float thrust_lower = fmaxf(-differential_allocation_max_thrust_rate_,
                                           (minimum_thrust - allocator_thrust_[rotor]) /
                                               differential_allocation_thrust_time_constant_);
          const float thrust_upper = fminf(differential_allocation_max_thrust_rate_,
                                           (maximum_thrust - allocator_thrust_[rotor]) /
                                               differential_allocation_thrust_time_constant_);
          differential_lower_limit_reason_[2 * rotor] =
              thrust_lower > -differential_allocation_max_thrust_rate_ ? 2 : 1;
          differential_upper_limit_reason_[2 * rotor] =
              thrust_upper < differential_allocation_max_thrust_rate_ ? 2 : 1;
          float servo_lower = -differential_allocation_max_servo_rate_;
          float servo_upper = differential_allocation_max_servo_rate_;
          differential_lower_limit_reason_[2 * rotor + 1] = 1;
          differential_upper_limit_reason_[2 * rotor + 1] = 1;
          if(differential_allocation_servo_angle_limit_enabled_)
            {
              const float angle_lower =
                  (differential_allocation_min_servo_angle_ -
                   allocator_gimbal_angle_[rotor]) /
                      differential_allocation_servo_time_constant_;
              const float angle_upper =
                  (differential_allocation_max_servo_angle_ -
                   allocator_gimbal_angle_[rotor]) /
                      differential_allocation_servo_time_constant_;
              if(angle_lower > servo_lower)
                {
                  servo_lower = angle_lower;
                  differential_lower_limit_reason_[2 * rotor + 1] = 2;
                }
              if(angle_upper < servo_upper)
                {
                  servo_upper = angle_upper;
                  differential_upper_limit_reason_[2 * rotor + 1] = 2;
                }
            }
          const float maximum_rate_change = differential_allocation_max_servo_acceleration_ * dt;
          const float acceleration_lower = allocator_gimbal_rate_[rotor] - maximum_rate_change;
          const float acceleration_upper = allocator_gimbal_rate_[rotor] + maximum_rate_change;
          if(acceleration_lower > servo_lower)
            {
              servo_lower = acceleration_lower;
              differential_lower_limit_reason_[2 * rotor + 1] = 3;
            }
          if(acceleration_upper < servo_upper)
            {
              servo_upper = acceleration_upper;
              differential_upper_limit_reason_[2 * rotor + 1] = 3;
            }
          if(servo_lower > servo_upper)
            {
              // Close to an angle limit, respecting the limit has priority
              // over the acceleration bound so that the command cannot wind
              // farther into saturation.
              if(allocator_gimbal_rate_[rotor] >= 0.0f)
                servo_lower = servo_upper;
              else
                servo_upper = servo_lower;
            }

          differential_rate_center_[2 * rotor] = 0.5f * (thrust_upper + thrust_lower);
          differential_rate_half_range_[2 * rotor] = 0.5f * fmaxf(0.0f, thrust_upper - thrust_lower);
          differential_rate_center_[2 * rotor + 1] = 0.5f * (servo_upper + servo_lower);
          differential_rate_half_range_[2 * rotor + 1] = 0.5f * fmaxf(0.0f, servo_upper - servo_lower);
          differential_inverse_rate_weight_[2 * rotor] = 1.0f;
          differential_inverse_rate_weight_[2 * rotor + 1] = 1.0f;
        }
      else
        {
          differential_inverse_rate_weight_[2 * rotor] =
              1.0f / (differential_allocation_thrust_rate_weight_ * differential_allocation_thrust_rate_weight_);
          differential_inverse_rate_weight_[2 * rotor + 1] =
              1.0f / (differential_allocation_servo_rate_weight_ * differential_allocation_servo_rate_weight_);
        }
    }

  if(dynamic_limit_decomposition)
    for(int input = 0; input < motor_number_; ++input)
      {
        for(int row = 0; row < 6; ++row)
          {
            differential_weighted_wrench_rate_reference_[row] -=
                differential_weighted_jacobian_[row][input] * differential_rate_center_[input];
            differential_weighted_jacobian_[row][input] *= differential_rate_half_range_[input];
          }
      }

  for(int row = 0; row < 6; ++row)
    for(int col = 0; col < 6; ++col)
      {
        differential_normal_matrix_[row][col] = 0.0f;
        for(int input = 0; input < motor_number_; ++input)
          differential_normal_matrix_[row][col] +=
              differential_weighted_jacobian_[row][input] * differential_inverse_rate_weight_[input] *
              differential_weighted_jacobian_[col][input];
        if(row == col)
          differential_normal_matrix_[row][col] +=
              differential_allocation_damping_ * differential_allocation_damping_;
      }
#ifdef SIMULATION
  if(dynamic_limit_decomposition) updateDifferentialAllocationConditionDebug();
#endif

  if(!solveDifferentialAllocationSystem(differential_normal_matrix_,
                                        differential_weighted_wrench_rate_reference_,
                                        differential_task_dual_))
    {
#ifdef SIMULATION
      ROS_WARN_THROTTLE(1.0, "differential allocation solve failed; hold the last actuator command");
#else
      nh_->logwarn("differential allocation solve failed");
#endif
      for(int rotor = 0; rotor < physical_rotor_number; ++rotor)
        {
          target_thrust_[rotor] = dynamic_limit_decomposition ? allocator_thrust_command_[rotor]
                                                               : allocator_thrust_[rotor];
          target_gimbal_angles_[rotor] = dynamic_limit_decomposition ? allocator_gimbal_command_[rotor]
                                                                      : allocator_gimbal_angle_[rotor];
        }
      return true;
    }

  for(int input = 0; input < motor_number_; ++input)
    {
      differential_actuator_rate_[input] = 0.0f;
      for(int row = 0; row < 6; ++row)
        differential_actuator_rate_[input] +=
            differential_inverse_rate_weight_[input] * differential_weighted_jacobian_[row][input] *
            differential_task_dual_[row];
    }

  // Use redundancy only for centering the gimbals and balancing rotor thrust.
  float average_thrust = 0.0f;
  for(int rotor = 0; rotor < physical_rotor_number; ++rotor) average_thrust += allocator_thrust_[rotor];
  average_thrust /= physical_rotor_number;
  if(hover_thrust_reference_enabled_ && hover_thrust_reference_ <= 0.0f)
    {
      const bool capture = hover_thrust_capture_allowed_ &&
          HAL_GetTick() - hover_thrust_capture_stamp_ < 500 &&
          std::isfinite(average_thrust) && average_thrust > 0.1f;
      if(!capture || (hover_thrust_sample_time_ > 0.0f &&
                     fabsf(average_thrust - hover_thrust_sample_start_) > 0.05f * hover_thrust_sample_start_))
        {
          hover_thrust_sample_time_ = 0.0f;
          hover_thrust_sample_integral_ = 0.0f;
        }
      if(capture)
        {
          if(hover_thrust_sample_time_ == 0.0f) hover_thrust_sample_start_ = average_thrust;
          hover_thrust_sample_time_ += dt;
          hover_thrust_sample_integral_ += average_thrust * dt;
          if(hover_thrust_sample_time_ >= 2.0f)
            hover_thrust_reference_ = hover_thrust_sample_integral_ / hover_thrust_sample_time_;
        }
    }
  const float thrust_reference = hover_thrust_reference_enabled_ ? hover_thrust_reference_ : average_thrust;
  for(int rotor = 0; rotor < physical_rotor_number; ++rotor)
    {
      differential_secondary_rate_[2 * rotor] = -differential_allocation_thrust_balance_gain_ *
                                                 (allocator_thrust_[rotor] - thrust_reference);
      // No guessed reference during takeoff or before stable hover capture.
      if(hover_thrust_reference_enabled_ && hover_thrust_reference_ <= 0.0f)
        differential_secondary_rate_[2 * rotor] = 0.0f;
      differential_secondary_rate_[2 * rotor + 1] = -differential_allocation_servo_center_gain_ *
                                                     allocator_gimbal_angle_[rotor];
      if(dynamic_limit_decomposition)
        {
          const float thrust_range = differential_rate_half_range_[2 * rotor];
          const float servo_range = differential_rate_half_range_[2 * rotor + 1];
          differential_secondary_rate_[2 * rotor] =
              thrust_range > 1e-6f ? differential_secondary_rate_[2 * rotor] / thrust_range : 0.0f;
          differential_secondary_rate_[2 * rotor + 1] =
              servo_range > 1e-6f ? differential_secondary_rate_[2 * rotor + 1] / servo_range : 0.0f;
        }
    }

  for(int row = 0; row < 6; ++row)
    {
      differential_secondary_wrench_rate_[row] = 0.0f;
      for(int input = 0; input < motor_number_; ++input)
        differential_secondary_wrench_rate_[row] +=
            differential_weighted_jacobian_[row][input] * differential_secondary_rate_[input];
    }
  if(solveDifferentialAllocationSystem(differential_normal_matrix_,
                                       differential_secondary_wrench_rate_,
                                       differential_secondary_dual_))
    for(int input = 0; input < motor_number_; ++input)
      {
        float task_space_component = 0.0f;
        for(int row = 0; row < 6; ++row)
          task_space_component += differential_inverse_rate_weight_[input] *
                                  differential_weighted_jacobian_[row][input] *
                                  differential_secondary_dual_[row];
        differential_actuator_rate_[input] += differential_secondary_rate_[input] - task_space_component;
      }

  differential_uniform_scale_ = 1.0f;
  if(dynamic_limit_decomposition)
    {
      float maximum_normalized_rate = 1.0f;
      differential_limiting_input_ = -1;
      differential_limiting_bound_reason_ = 0;
      for(int input = 0; input < motor_number_; ++input)
        {
#ifdef SIMULATION
          differential_unscaled_normalized_rate_[input] = differential_actuator_rate_[input];
#endif
          const float absolute_normalized_rate = fabsf(differential_actuator_rate_[input]);
          if(absolute_normalized_rate > maximum_normalized_rate)
            {
              maximum_normalized_rate = absolute_normalized_rate;
              differential_limiting_input_ = input;
              differential_limiting_bound_reason_ =
                  differential_actuator_rate_[input] < 0.0f ?
                      differential_lower_limit_reason_[input] :
                      differential_upper_limit_reason_[input];
            }
        }
      differential_uniform_scale_ = 1.0f / maximum_normalized_rate;
      for(int input = 0; input < motor_number_; ++input)
        differential_actuator_rate_[input] =
            differential_rate_center_[input] + differential_rate_half_range_[input] *
                                                   differential_uniform_scale_ *
                                                   differential_actuator_rate_[input];
    }

  for(int rotor = 0; rotor < physical_rotor_number; ++rotor)
    {
      float thrust_rate = differential_actuator_rate_[2 * rotor];
      float gimbal_rate = differential_actuator_rate_[2 * rotor + 1];
      if(!std::isfinite(thrust_rate) || !std::isfinite(gimbal_rate))
        {
          thrust_rate = 0.0f;
          gimbal_rate = 0.0f;
        }
      if(!dynamic_limit_decomposition)
        {
          thrust_rate = fminf(fmaxf(thrust_rate, -differential_allocation_max_thrust_rate_),
                              differential_allocation_max_thrust_rate_);
          thrust_rate = fminf(thrust_rate, (maximum_thrust - allocator_thrust_[rotor]) / dt);
          thrust_rate = fmaxf(thrust_rate, -allocator_thrust_[rotor] / dt);

          const float maximum_rate_change = differential_allocation_max_servo_acceleration_ * dt;
          gimbal_rate = fminf(gimbal_rate, allocator_gimbal_rate_[rotor] + maximum_rate_change);
          gimbal_rate = fmaxf(gimbal_rate, allocator_gimbal_rate_[rotor] - maximum_rate_change);
          gimbal_rate = fminf(fmaxf(gimbal_rate, -differential_allocation_max_servo_rate_),
                             differential_allocation_max_servo_rate_);
          if(differential_allocation_servo_angle_limit_enabled_)
            {
              gimbal_rate = fminf(gimbal_rate,
                                  (differential_allocation_max_servo_angle_ - allocator_gimbal_angle_[rotor]) / dt);
              gimbal_rate = fmaxf(gimbal_rate,
                                  (differential_allocation_min_servo_angle_ - allocator_gimbal_angle_[rotor]) / dt);
            }
        }

      allocator_thrust_rate_[rotor] = thrust_rate;
      allocator_gimbal_rate_[rotor] = gimbal_rate;
      if(dynamic_limit_decomposition)
        {
          allocator_thrust_command_[rotor] =
              fminf(fmaxf(allocator_thrust_[rotor] +
                              differential_allocation_thrust_time_constant_ * thrust_rate,
                          minimum_thrust),
                    maximum_thrust);
          allocator_gimbal_command_[rotor] =
              allocator_gimbal_angle_[rotor] +
              differential_allocation_servo_time_constant_ * gimbal_rate;
          if(differential_allocation_servo_angle_limit_enabled_)
            allocator_gimbal_command_[rotor] =
                fminf(fmaxf(allocator_gimbal_command_[rotor],
                            differential_allocation_min_servo_angle_),
                      differential_allocation_max_servo_angle_);
          target_thrust_[rotor] = allocator_thrust_command_[rotor];
          target_gimbal_angles_[rotor] = allocator_gimbal_command_[rotor];
        }
      else
        {
          allocator_thrust_[rotor] =
              fminf(fmaxf(allocator_thrust_[rotor] + dt * thrust_rate, 0.0f), maximum_thrust);
          allocator_gimbal_angle_[rotor] =
              allocator_gimbal_angle_[rotor] + dt * gimbal_rate;
          if(differential_allocation_servo_angle_limit_enabled_)
            allocator_gimbal_angle_[rotor] =
                fminf(fmaxf(allocator_gimbal_angle_[rotor],
                            differential_allocation_min_servo_angle_),
                      differential_allocation_max_servo_angle_);
          allocator_thrust_command_[rotor] = allocator_thrust_[rotor];
          allocator_gimbal_command_[rotor] = allocator_gimbal_angle_[rotor];
          target_thrust_[rotor] = allocator_thrust_[rotor];
          target_gimbal_angles_[rotor] = allocator_gimbal_angle_[rotor];
        }
    }

#ifdef SIMULATION
  publishDifferentialAllocationDebug(differential_desired_wrench_, differential_current_wrench_,
                                     physical_rotor_number);
#endif
  return true;
}

#ifdef SIMULATION
void AttitudeController::updateDifferentialAllocationConditionDebug()
{
  // The normal matrix is J_normalized * J_normalized^T plus lambda^2 I.
  // A fixed-size Jacobi eigensolver keeps this diagnostic independent of
  // Eigen and out of the embedded build. Singular values are the square roots
  // of the undamped Gram-matrix eigenvalues.
  float gram[6][6];
  const float damping_squared =
      differential_allocation_damping_ * differential_allocation_damping_;
  for(int row = 0; row < 6; ++row)
    for(int col = 0; col < 6; ++col)
      gram[row][col] = differential_normal_matrix_[row][col] -
                       (row == col ? damping_squared : 0.0f);

  for(int iteration = 0; iteration < 48; ++iteration)
    {
      int pivot_row = 0;
      int pivot_col = 1;
      float largest_off_diagonal = fabsf(gram[pivot_row][pivot_col]);
      for(int row = 0; row < 6; ++row)
        for(int col = row + 1; col < 6; ++col)
          if(fabsf(gram[row][col]) > largest_off_diagonal)
            {
              largest_off_diagonal = fabsf(gram[row][col]);
              pivot_row = row;
              pivot_col = col;
            }
      if(largest_off_diagonal < 1e-8f) break;

      const float angle = 0.5f * atan2f(2.0f * gram[pivot_row][pivot_col],
                                        gram[pivot_col][pivot_col] -
                                        gram[pivot_row][pivot_row]);
      const float cosine = cosf(angle);
      const float sine = sinf(angle);
      const float diagonal_row = gram[pivot_row][pivot_row];
      const float diagonal_col = gram[pivot_col][pivot_col];
      const float off_diagonal = gram[pivot_row][pivot_col];

      for(int index = 0; index < 6; ++index)
        {
          if(index == pivot_row || index == pivot_col) continue;
          const float row_value = gram[index][pivot_row];
          const float col_value = gram[index][pivot_col];
          gram[index][pivot_row] = cosine * row_value - sine * col_value;
          gram[pivot_row][index] = gram[index][pivot_row];
          gram[index][pivot_col] = sine * row_value + cosine * col_value;
          gram[pivot_col][index] = gram[index][pivot_col];
        }
      gram[pivot_row][pivot_row] =
          cosine * cosine * diagonal_row - 2.0f * sine * cosine * off_diagonal +
          sine * sine * diagonal_col;
      gram[pivot_col][pivot_col] =
          sine * sine * diagonal_row + 2.0f * sine * cosine * off_diagonal +
          cosine * cosine * diagonal_col;
      gram[pivot_row][pivot_col] = 0.0f;
      gram[pivot_col][pivot_row] = 0.0f;
    }

  float minimum_eigenvalue = fmaxf(0.0f, gram[0][0]);
  float maximum_eigenvalue = minimum_eigenvalue;
  for(int index = 1; index < 6; ++index)
    {
      const float eigenvalue = fmaxf(0.0f, gram[index][index]);
      minimum_eigenvalue = fminf(minimum_eigenvalue, eigenvalue);
      maximum_eigenvalue = fmaxf(maximum_eigenvalue, eigenvalue);
    }
  differential_jacobian_min_singular_value_ = sqrtf(minimum_eigenvalue);
  differential_jacobian_max_singular_value_ = sqrtf(maximum_eigenvalue);
  differential_jacobian_condition_number_ =
      differential_jacobian_min_singular_value_ > 1e-6f ?
          differential_jacobian_max_singular_value_ /
              differential_jacobian_min_singular_value_ : 1e9f;
}

void AttitudeController::publishDifferentialAllocationDebug(const float desired_wrench[6],
                                                            const float current_wrench[6],
                                                            int physical_rotor_number)
{
  if(HAL_GetTick() - differential_allocation_debug_pub_last_time_ < 100) return;
  differential_allocation_debug_pub_last_time_ = HAL_GetTick();
  std_msgs::Float32MultiArray msg;
  // desired wrench[6], realized wrench[6], thrust, gimbal angle,
  // thrust rate, and gimbal rate for every physical rotor.
  for(int row = 0; row < 6; ++row) msg.data.push_back(desired_wrench[row]);
  for(int row = 0; row < 6; ++row) msg.data.push_back(current_wrench[row]);
  for(int rotor = 0; rotor < physical_rotor_number; ++rotor) msg.data.push_back(allocator_thrust_[rotor]);
  for(int rotor = 0; rotor < physical_rotor_number; ++rotor) msg.data.push_back(allocator_gimbal_angle_[rotor]);
  for(int rotor = 0; rotor < physical_rotor_number; ++rotor) msg.data.push_back(allocator_thrust_rate_[rotor]);
  for(int rotor = 0; rotor < physical_rotor_number; ++rotor) msg.data.push_back(allocator_gimbal_rate_[rotor]);
  // Appended fields preserve the original layout: commanded thrust/angle,
  // allocator mode, and DLD's common normalized-rate scale factor.
  for(int rotor = 0; rotor < physical_rotor_number; ++rotor) msg.data.push_back(allocator_thrust_command_[rotor]);
  for(int rotor = 0; rotor < physical_rotor_number; ++rotor) msg.data.push_back(allocator_gimbal_command_[rotor]);
  msg.data.push_back(static_cast<float>(differential_allocation_mode_));
  msg.data.push_back(differential_uniform_scale_);
  // DLD diagnostics appended after the legacy 38 entries:
  // unscaled normalized rate[2*N], physical lower rate[2*N], physical upper
  // rate[2*N], limiting input (-1 means no scaling), sigma_min, sigma_max,
  // limiting-bound reason, sigma_min, sigma_max, and condition number. Even
  // inputs are thrust; odd inputs are gimbal rate. Reason: 0=none, 1=rate,
  // 2=thrust/angle position, 3=servo acceleration.
  for(int input = 0; input < 2 * physical_rotor_number; ++input)
    msg.data.push_back(differential_unscaled_normalized_rate_[input]);
  for(int input = 0; input < 2 * physical_rotor_number; ++input)
    msg.data.push_back(differential_rate_center_[input] - differential_rate_half_range_[input]);
  for(int input = 0; input < 2 * physical_rotor_number; ++input)
    msg.data.push_back(differential_rate_center_[input] + differential_rate_half_range_[input]);
  msg.data.push_back(static_cast<float>(differential_limiting_input_));
  msg.data.push_back(static_cast<float>(differential_limiting_bound_reason_));
  msg.data.push_back(differential_jacobian_min_singular_value_);
  msg.data.push_back(differential_jacobian_max_singular_value_);
  msg.data.push_back(differential_jacobian_condition_number_);
  // Appended diagnostics: [67] fixed hover reference (N, zero=uncaptured),
  // [68] stable sample duration (s), [69] fixed-reference mode enabled.
  msg.data.push_back(hover_thrust_reference_);
  msg.data.push_back(hover_thrust_sample_time_);
  msg.data.push_back(hover_thrust_reference_enabled_ ? 1.0f : 0.0f);
  differential_allocation_debug_pub_.publish(msg);
}
#endif

void AttitudeController::pwmConversion()
{
  auto convert = [this](float target_thrust)
    {
      float scaled_thrust = v_factor_ * target_thrust / rotor_devider_;
      float target_pwm = 0;
      if (scaled_thrust < 0) scaled_thrust = 0;

      switch(pwm_conversion_mode_)
        {
        case spinal::MotorInfo::SQRT_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^2 f] */
            float sqrt_tmp = motor_info_[motor_ref_index_].polynominal[1] * motor_info_[motor_ref_index_].polynominal[1] - 4 * 10 * motor_info_[motor_ref_index_].polynominal[2] * (motor_info_[motor_ref_index_].polynominal[0] - scaled_thrust); //special decimal order shift (x10)
            target_pwm = (-motor_info_[motor_ref_index_].polynominal[1] + sqrt_tmp * ap::inv_sqrt(sqrt_tmp)) / (2 * motor_info_[motor_ref_index_].polynominal[2]);
            break;
          }
        case spinal::MotorInfo::POLYNOMINAL_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^1.5 f] */
            float tenth_scaled_thrust = scaled_thrust * 0.1f; //special decimal order shift (x0.1)
            /* 4 dimensional */
            int max_dimenstional = 4;
            target_pwm = motor_info_[motor_ref_index_].polynominal[max_dimenstional];
            for (int j = max_dimenstional - 1; j >= 0; j--)
              target_pwm = target_pwm * tenth_scaled_thrust + motor_info_[motor_ref_index_].polynominal[j];
            break;
          }
        default:
          {
            break;
          }
        }
      return target_pwm / 100; // target_pwm is [%]
    };

  if(pwm_test_flag_) /* motor pwm test */
    {
      for(int i = 0; i < MAX_MOTOR_NUMBER; i++)
        {
          target_pwm_[i] = pwm_test_value_[i];
        }
      return;
    }

  if(motor_info_.size() == 0) return;

  /* update the factor regarding the robot voltage */
  if(HAL_GetTick() - voltage_update_last_time_ > 500) //[500ms = 0.5s]
    {
#ifdef SIMULATION
      float voltage = sim_voltage_;
#else
      float voltage = bat_->getVoltage();
#endif

      /* find the best reference voltage */
      float min_voltage_diff = 1e6;
      for(int i = 0; i < motor_info_.size(); i++)
        {
          float voltage_diff = fabs(voltage - motor_info_[i].voltage);
          if(min_voltage_diff > voltage_diff)
            {
              motor_ref_index_ = i;
              min_voltage_diff = voltage_diff;
            }
        }

      switch(pwm_conversion_mode_)
        {
        case spinal::MotorInfo::SQRT_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^2 f] */
            v_factor_ = (motor_info_[motor_ref_index_].voltage / voltage) *  (motor_info_[motor_ref_index_].voltage / voltage) ;
            break;
          }
        case spinal::MotorInfo::POLYNOMINAL_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^1.5 f] */
            v_factor_ = motor_info_[motor_ref_index_].voltage / voltage * ap::inv_sqrt(voltage / motor_info_[motor_ref_index_].voltage);
            break;
          }
        default:
          {
            break;
          }
        }

      if(min_thrust_> 0) min_duty_ = convert(min_thrust_);

      voltage_update_last_time_ = HAL_GetTick();
    }

  /* pwm saturation avoidance */
  /* get the decreasing rate for the thrust to avoid the divergence because of the pwm saturation */
  float base_thrust_decreasing_rate = 0;
  float yaw_decreasing_rate = 0;
  float thrust_limit = motor_info_[motor_ref_index_].max_thrust / v_factor_;

  /* check saturation level 2: z control saturation */
  float max_thrust = 0;
  int max_thrust_index = 0;
  for(int i = 0; i < motor_number_ / rotor_coef_; i++)
    {
      float thrust;
      switch(gimbal_dof_)
        {
        case 2:
          thrust = ap::pythagorous3(base_thrust_term_[rotor_coef_ * i] + roll_pitch_term_[rotor_coef_ *i],base_thrust_term_[rotor_coef_ * i+1] + roll_pitch_term_[rotor_coef_ * i+1], base_thrust_term_[rotor_coef_ * i+2] + roll_pitch_term_[rotor_coef_ *i+2]);
          break;
        case 1:
          thrust = ap::pythagorous2(base_thrust_term_[rotor_coef_ * i] + roll_pitch_term_[rotor_coef_ *i],base_thrust_term_[rotor_coef_ * i+1] + roll_pitch_term_[rotor_coef_ * i+1]);
          break;
        case 0:
          thrust = base_thrust_term_[i] + roll_pitch_term_[i];
          break;
        default:
          break;
        }
      if(max_thrust < thrust)
        {
          max_thrust = thrust;
          max_thrust_index = i;
        }
    }
  if(start_control_flag_)
    {
      float residual_term = thrust_limit - max_thrust / rotor_devider_;

      if(residual_term < 0 && base_thrust_term_[max_thrust_index] > 0)
        {
          base_thrust_decreasing_rate = residual_term / (base_thrust_term_[max_thrust_index] / rotor_devider_);
          yaw_decreasing_rate = -1; // also, we have to ignore the yaw control
        }
      else
        {
          if(max_yaw_term_index_ != -1 && fabs(base_thrust_term_[0]) > 0 )
            {
              /* check saturation level1: yaw control saturation */
              max_thrust = 0;
              float min_thrust = 10000;
              int min_thrust_index = 0;
              for(int i = 0; i < motor_number_ / (rotor_coef_); i++)
                {
                  float thrust;
                  switch(gimbal_dof_)
                    {
                    case 2:
                      thrust = ap::pythagorous3(base_thrust_term_[rotor_coef_ * i] + roll_pitch_term_[rotor_coef_ *i],base_thrust_term_[rotor_coef_ * i+1] + roll_pitch_term_[rotor_coef_ * i+1], base_thrust_term_[rotor_coef_ * i+2] + roll_pitch_term_[rotor_coef_ *i+2]);
                      break;
                    case 1:
                      thrust = ap::pythagorous2(base_thrust_term_[rotor_coef_ * i] + roll_pitch_term_[rotor_coef_ *i],base_thrust_term_[rotor_coef_ * i+1] + roll_pitch_term_[rotor_coef_ * i+1]);
                      break;
                    case 0:
                      thrust = base_thrust_term_[i] + roll_pitch_term_[i];
                      break;
                    default:
                      break;
                    }
                  if(max_thrust < thrust)
                    {
                      max_thrust = thrust;
                      max_thrust_index = i;
                    }
                  if(min_thrust > thrust)
                    {
                      min_thrust = thrust;
                      min_thrust_index = i;
                    }
                }

              float residual_term_max =  thrust_limit - max_thrust / rotor_devider_;
              float residual_term_min =  min_thrust / rotor_devider_ - min_thrust_;
              int thrust_index = 0;
              if (residual_term_min < residual_term_max)
                {
                  residual_term = residual_term_min;
                  thrust_index = min_thrust_index;
                }
              else
                {
                  residual_term = residual_term_max;
                  thrust_index = max_thrust_index;
                }

              if(residual_term < 0)
                {
                  yaw_decreasing_rate = residual_term / (fabs(yaw_term_[thrust_index]) / rotor_devider_);
                }

              if(yaw_decreasing_rate < -1) yaw_decreasing_rate = -1;
              if(yaw_decreasing_rate > 0) yaw_decreasing_rate = 0;
            }
          else
            {
              yaw_decreasing_rate = -1;
            }
        }
    }
  
  for(int i = 0; i < motor_number_; i++)
    target_thrust_[i] = roll_pitch_term_[i] + (1 + base_thrust_decreasing_rate) * base_thrust_term_[i] + (1 + yaw_decreasing_rate) * yaw_term_[i];

  // Static allocation is used while the vehicle is supported by the ground.
  if(!start_control_flag_ || !integrate_flag_)
    {
      hover_thrust_reference_ = 0.0f;
      hover_thrust_sample_time_ = 0.0f;
      hover_thrust_sample_integral_ = 0.0f;
    }
  // Reinitialize from the measured actuator state whenever differential
  // allocation is enabled again so that takeoff/landing transitions are
  // bumpless rather than resuming an old actuator-rate state.
  if(!integrate_flag_ &&
     differential_allocation_mode_ != spinal::DifferentialAllocationConfig::STATIC)
    differential_allocator_initialized_ = false;

  // DLD handles actuator limits itself. Give it the unreduced wrench, not
  // the legacy static allocator's base-thrust/yaw saturation correction.
  // Keep the legacy command available if DLD cannot run on this cycle.
  const bool use_unreduced_dld_command =
      start_control_flag_ && integrate_flag_ &&
      differential_allocation_mode_ == spinal::DifferentialAllocationConfig::DYNAMIC_LIMIT_DECOMPOSITION;
  if(use_unreduced_dld_command)
    for(int i = 0; i < motor_number_; i++)
      target_thrust_[i] = roll_pitch_term_[i] + base_thrust_term_[i] + yaw_term_[i];

  const bool differential_allocation_active =
      start_control_flag_ && integrate_flag_ &&
      differential_allocation_mode_ != spinal::DifferentialAllocationConfig::STATIC &&
      differentialAllocationUpdate(DELTA_T, thrust_limit * rotor_devider_);

  if(use_unreduced_dld_command && !differential_allocation_active)
    for(int i = 0; i < motor_number_; i++)
      target_thrust_[i] = roll_pitch_term_[i] + (1 + base_thrust_decreasing_rate) * base_thrust_term_[i] +
                          (1 + yaw_decreasing_rate) * yaw_term_[i];

  /* convert to target pwm and calculate target gimbal angles */
  /* TODO: adjust not only for gimbalrotor but also for fixed rotor */
  for(int i = 0; i < motor_number_ / (rotor_coef_); i++)
    {
      if(start_control_flag_)
        {
          if(!differential_allocation_active)
            {
              switch(gimbal_dof_)
                {
                case 2:
                  {
                    ap::Vector3f f_i;
                    f_i.x = target_thrust_[i*3];
                    f_i.y = target_thrust_[i*3+1];
                    f_i.z = target_thrust_[i*3+2];

                    float gimbal_candidate_roll = atan2f(-f_i.y, f_i.z);
                    float gimbal_candidate_pitch = atan2f(f_i.x, -f_i.y * sin(gimbal_candidate_roll) + f_i.z * cos(gimbal_candidate_roll));
                    target_thrust_[i] = ap::pythagorous3(f_i.x,f_i.y,f_i.z);

                    /* simple lpf */
                    if(std::isfinite(gimbal_candidate_roll) && std::isfinite(gimbal_candidate_pitch)){
                      target_gimbal_angles_[2*i] =(target_gimbal_angles_[2*i]+ gimbal_candidate_roll)/2;
                      target_gimbal_angles_[2*i+1] =(target_gimbal_angles_[2*i+1]+ gimbal_candidate_pitch)/2;

                    }
                    break;
                  }
                case 1:
                  {
                    ap::Vector3f f_i;
                    f_i.x = target_thrust_[i*2];
                    f_i.z = target_thrust_[i*2+1];
                    float gimbal_candidate = atan2f(-f_i.x, f_i.z);
                    target_thrust_[i] = ap::pythagorous2(f_i.x,f_i.z);

                    if (std::isfinite(gimbal_candidate)) {
                      prev_gimbal_angles_[i] = target_gimbal_angles_[i];

                      float delta = gimbal_candidate - prev_gimbal_angles_[i];
                      if (delta > M_PI) {
                        gimbal_candidate -= 2.0f * M_PI;
                      } else if (delta < -M_PI) {
                        gimbal_candidate += 2.0f * M_PI;
                      }

                      if (gimbal_candidate > 4.0f) {
                        gimbal_candidate -= 2.0f * M_PI; // Wrap down
                      } else if (gimbal_candidate < -4.0f) {
                        gimbal_candidate += 2.0f * M_PI; // Wrap up
                      }

                      target_gimbal_angles_[i] = gimbal_candidate;
                    }
                    break;
                  }
                default:
                  break;
                }
            }

          target_pwm_[i] = convert(target_thrust_[i]);

          /* constraint */
          if(target_pwm_[i] < min_duty_) target_pwm_[i]  = min_duty_;
          else if(target_pwm_[i]  > max_duty_) target_pwm_[i]  = max_duty_;
        }

      /* for ros */
      pwms_msg_.motor_value[i] = (target_pwm_[i] * 2000);
    }
  //TODO: send target gimbal angles in real machiene
#ifdef SIMULATION
  //TODO: directly send target gimbal angles to gazebo
  switch(gimbal_dof_)
    {
    case 2:
      {
        sensor_msgs::JointState gimbal_control_msg;
        gimbal_control_msg.header.stamp = ros::Time::now();
        for(int i = 0; i < motor_number_ / (rotor_coef_); i++){
          gimbal_control_msg.position.push_back(target_gimbal_angles_[2*i]);
          gimbal_control_msg.position.push_back(target_gimbal_angles_[2*i+1]);
        }
        gimbal_control_pub_.publish(gimbal_control_msg);
        break;
      }
    case 1:
      {
        sensor_msgs::JointState gimbal_control_msg;
        gimbal_control_msg.header.stamp = ros::Time::now();
        for(int i = 0; i < motor_number_ / (rotor_coef_); i++){
          gimbal_control_msg.position.push_back(target_gimbal_angles_[i]);
        }
        gimbal_control_pub_.publish(gimbal_control_msg);
        break;
      }
    default:
      break;
    }

#else
  switch(gimbal_dof_)
    {
    case 2:
      {
        std::map<uint8_t, float> gimbal_map;
        gimbal_control_msg_.index_length = motor_number_ / rotor_coef_ * 2;
        gimbal_control_msg_.angles_length = motor_number_ / rotor_coef_ * 2;
        gimbal_control_msg_.index = gimbal_control_indices_;
        gimbal_control_msg_.angles = gimbal_control_angles_;
        for(int i = 0; i < motor_number_ / (rotor_coef_); i++){
          if(start_control_flag_)
            {
              gimbal_map[2*i] = target_gimbal_angles_[2*i];
              gimbal_map[2*i+1] = target_gimbal_angles_[2*i+1];
            }
          else
            {
              gimbal_map[2*i] =  0;
              gimbal_map[2*i+1] = 0;
            }
        }
        if(start_control_flag_)
          {
            servo_->setGoalAngle(gimbal_map,ValueType::RADIAN);
            gimbal_control_msg_.index_length = motor_number_ / rotor_coef_ * 2;
            gimbal_control_msg_.angles_length = motor_number_ / rotor_coef_ * 2;
            gimbal_control_msg_.index = gimbal_control_indices_;
            gimbal_control_msg_.angles = gimbal_control_angles_;
            for(int i = 0; i < motor_number_ / (rotor_coef_); i++){
              const auto& servo_roll = servo_->getServoHnadler().getServo()[2*i];
              const auto& servo_pitch = servo_->getServoHnadler().getServo()[2*i + 1];
              gimbal_control_indices_[2*i] = 2*i;
              gimbal_control_indices_[2*i+1] = 2*i + 1;
              gimbal_control_angles_[2*i] = static_cast<int16_t>((servo_roll.getGoalPosition() + servo_roll.internal_offset_) / servo_roll.resolution_ratio_);
              gimbal_control_angles_[2*i+1] = static_cast<int16_t>((servo_pitch.getGoalPosition() + servo_pitch.internal_offset_) / servo_pitch.resolution_ratio_);
            }
            gimbal_control_pub_.publish(&gimbal_control_msg_);
          }
        else
          servo_->torqueEnable(gimbal_map);
        break;
      }
    case 1:
      {
        std::map<uint8_t, float> gimbal_map;
        gimbal_control_msg_.index_length = motor_number_ / rotor_coef_;
        gimbal_control_msg_.angles_length = motor_number_ / rotor_coef_;
        gimbal_control_msg_.index = gimbal_control_indices_;
        gimbal_control_msg_.angles = gimbal_control_angles_;
        for(int i = 0; i < motor_number_ / (rotor_coef_); i++){
          if(start_control_flag_)
            gimbal_map[i] = target_gimbal_angles_[i];
          else
            gimbal_map[i] = 0;
        }
        if(start_control_flag_)
          {
            servo_->setGoalAngle(gimbal_map,ValueType::RADIAN);
            gimbal_control_msg_.index_length = motor_number_ / rotor_coef_;
            gimbal_control_msg_.angles_length = motor_number_ / rotor_coef_;
            gimbal_control_msg_.index = gimbal_control_indices_;
            gimbal_control_msg_.angles = gimbal_control_angles_;
            for(int i = 0; i < motor_number_ / (rotor_coef_); i++){
              const auto& servo = servo_->getServoHnadler().getServo()[i];
              gimbal_control_indices_[i] = i;
              gimbal_control_angles_[i] = static_cast<int16_t>((servo.getGoalPosition() + servo.internal_offset_) / servo.resolution_ratio_);
            }
            gimbal_control_pub_.publish(&gimbal_control_msg_);
          }
        else
          servo_->torqueEnable(gimbal_map);
        break;
      }
    default:
      break;
    }
#endif
}
