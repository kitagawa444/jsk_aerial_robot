/*
******************************************************************************
* File Name          : attitude_control.h
* Description        : attitude control interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __ATTITUDE_CONTROL_H
#define __ATTITUDE_CONTROL_H

#ifndef SIMULATION
#include "config.h"
#include <ros.h>
#else
#include <ros/ros.h>
#endif
#include <sensor_msgs/JointState.h>

#include <math/AP_Math.h>
#include <vector>

#ifndef SIMULATION
/* state estimate  */
#if NERVE_COMM
#include <Spine/spine.h>
#endif
/* battery status */
#include "battery_status/battery_status.h"
/* RTOS */
#include "cmsis_os.h"
/* dshot esc */
#include "dshot_esc/dshot.h"
/* servo */
#include "servo/servo.h"
#endif

#include "state_estimate/state_estimate.h"

#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/SetBool.h>
#include <spinal/Pwms.h>
#include <spinal/PwmTest.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/PwmInfo.h>
#include <spinal/UavInfo.h>
#include <spinal/PMatrixPseudoInverseWithInertia.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <spinal/WrenchAllocationMatrix.h>
#include <spinal/DifferentialAllocationConfig.h>
#include <spinal/ServoControlCmd.h>

#define IDLE_DUTY 0.5f
#define FORCE_LANDING_INTEGRAL 0.0025f // 500Hz * 0.0025 = 1.25 N / sec

#define MAX_MOTOR_NUMBER 10

/* fail safe */
#define FLIGHT_COMMAND_TIMEOUT 500 //500ms
#define MAX_TILT_ANGLE 6.28 // rad
#define MAX_PWM 1.0f // duty

#define CONTROL_TERM_PUB_INTERVAL 100
#define CONTROL_FEEDBACK_STATE_PUB_INTERVAL 25
#define PWM_PUB_INTERVAL 100 //100ms

#define MOTOR_TEST 0

enum AXIS {
  X = 0,
  Y = 1,
  Z = 2,
};

class AttitudeController
{
public:
  AttitudeController();
  ~AttitudeController(){}

#ifdef SIMULATION
  void init(ros::NodeHandle* nh, StateEstimate* estimator);
#else
  void init(TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim2, StateEstimate* estimator,
            DShot* dshot, DirectServo* servo, BatteryStatus* bat, ros::NodeHandle* nh, osMutexId* mutex = NULL);
#endif

  void baseInit(); // common part in both pc and board
  void update();

  void setStartControlFlag(bool start_control_flag);
  void setUavModel(int8_t uav_model);
  inline uint16_t getMotorNumber(){return motor_number_;}
  inline const ap::Matrix3f getOffsetRotation()  { return offset_rot_; }

  void setMotorNumber(uint16_t motor_number);
  void setGimbalDof(uint8_t gimbal_dof){gimbal_dof_ = gimbal_dof; }
  void setRotorCoef(uint8_t rotor_coef){rotor_coef_ = rotor_coef; }
  uint16_t getGimbalDof(){return gimbal_dof_; }
  void setPwmTestMode(bool pwm_test_flag){pwm_test_flag_ = pwm_test_flag; }
  bool getIntegrateFlag(){return integrate_flag_; }
  void setIntegrateFlag(bool integrate_flag){integrate_flag_ = integrate_flag; }
  bool getForceLandingFlag() {return force_landing_flag_;}

  void setForceLandingFlag(bool force_landing_flag) { force_landing_flag_ = force_landing_flag; }
  float getPwm(uint8_t index) {return target_pwm_[index];}
  float getForce(uint8_t index) {return target_thrust_[index];}

  bool activated();

private:

#ifndef SIMULATION
  TIM_HandleTypeDef* pwm_htim1_;
  TIM_HandleTypeDef* pwm_htim2_;
#endif

  ros::NodeHandle* nh_;
  ros::Publisher pwms_pub_;
  ros::Publisher control_term_pub_;
  ros::Publisher control_feedback_state_pub_;
  spinal::Pwms pwms_msg_;
  spinal::RollPitchYawTerms control_term_msg_;
  spinal::RollPitchYawTerm control_feedback_state_msg_;

#ifdef SIMULATION
  ros::Subscriber four_axis_cmd_sub_;
  ros::Subscriber pwm_info_sub_;
  ros::Subscriber rpy_gain_sub_;
  ros::Subscriber pwm_test_sub_;
  ros::Subscriber p_matrix_pseudo_inverse_inertia_sub_;
  ros::Subscriber torque_allocation_matrix_inv_sub_;
  ros::Subscriber wrench_allocation_matrix_sub_;
  ros::Subscriber differential_allocation_config_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber sim_vol_sub_;
  ros::Subscriber offset_rot_sub_;
  ros::Publisher anti_gyro_pub_;
  ros::Publisher gimbal_control_pub_;
  ros::Publisher differential_allocation_debug_pub_;
  ros::ServiceServer att_control_srv_;

  bool setAttitudeControlCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) { att_control_flag_ = req.data; return true;}
  void setSimVolCallback(const std_msgs::Float32 vol_msg) { sim_voltage_ = vol_msg.data; }
  float sim_voltage_;

#else
  ros::Subscriber<spinal::FourAxisCommand, AttitudeController> four_axis_cmd_sub_;
  ros::Subscriber<spinal::PwmInfo, AttitudeController> pwm_info_sub_;
  ros::Subscriber<spinal::RollPitchYawTerms, AttitudeController> rpy_gain_sub_;
  ros::Subscriber<spinal::PwmTest, AttitudeController> pwm_test_sub_;
  ros::Subscriber<spinal::PMatrixPseudoInverseWithInertia, AttitudeController> p_matrix_pseudo_inverse_inertia_sub_;
  ros::Subscriber<spinal::TorqueAllocationMatrixInv, AttitudeController> torque_allocation_matrix_inv_sub_;
  ros::Subscriber<spinal::WrenchAllocationMatrix, AttitudeController> wrench_allocation_matrix_sub_;
  ros::Subscriber<spinal::DifferentialAllocationConfig, AttitudeController> differential_allocation_config_sub_;
  ros::Subscriber<spinal::DesireCoord, AttitudeController> offset_rot_sub_;
  ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response, AttitudeController> att_control_srv_;

  ros::Publisher esc_telem_pub_;
  ros::Publisher gimbal_control_pub_;
  spinal::ESCTelemetryArray esc_telem_msg_;
  spinal::ServoControlCmd gimbal_control_msg_;
  uint8_t gimbal_control_indices_[MAX_MOTOR_NUMBER];
  int16_t gimbal_control_angles_[MAX_MOTOR_NUMBER];

  void setAttitudeControlCallback(const std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    att_control_flag_ = req.data;
  }

  BatteryStatus* bat_;
  osMutexId* mutex_;
  DShot* dshot_;
  DirectServo* servo_;
#endif

  StateEstimate* estimator_;

  int8_t uav_model_;
  uint16_t motor_number_;
  uint8_t gimbal_dof_ {0};
  uint8_t rotor_coef_ {1};
  bool start_control_flag_;
  bool pwm_test_flag_;
  bool integrate_flag_;
  bool force_landing_flag_;
  bool att_control_flag_;

  float target_angle_[3];
  float error_angle_i_[3];
  float error_angle_i_limit_[3];

  float torque_p_gain_[3];
  float torque_i_gain_[3];
  float torque_d_gain_[3];
  float thrust_p_gain_[MAX_MOTOR_NUMBER][3];
  float thrust_i_gain_[MAX_MOTOR_NUMBER][3];
  float thrust_d_gain_[MAX_MOTOR_NUMBER][3];
  float torque_allocation_matrix_inv_[MAX_MOTOR_NUMBER][3];
  float base_thrust_term_[MAX_MOTOR_NUMBER]; //[N]
  float roll_pitch_term_[MAX_MOTOR_NUMBER]; //[N]
  float yaw_term_[MAX_MOTOR_NUMBER]; //[N]
  float extra_yaw_pi_term_[MAX_MOTOR_NUMBER]; //[N]
  int max_yaw_term_index_{-1};

  // Offset Rotation from the control frame to the estimation frame
  ap::Matrix3f offset_rot_;

  // Gyro Moment Compensation
  float p_matrix_pseudo_inverse_[MAX_MOTOR_NUMBER][4];
  ap::Matrix3f inertia_;

  // Failsafe
  bool failsafe_;
  uint32_t flight_command_last_stamp_;

  // Thrust PWM Conversion
  float target_thrust_[MAX_MOTOR_NUMBER];
  float target_pwm_[MAX_MOTOR_NUMBER];
  float target_gimbal_angles_[MAX_MOTOR_NUMBER];
  float prev_gimbal_angles_[MAX_MOTOR_NUMBER];

  /* Differential allocation for one-DoF tilting rotors.  The allocation
   * state is the commanded thrust and gimbal angle; measured actuator states
   * can be incorporated later without changing the allocator interface. */
  uint8_t differential_allocation_mode_;
  bool wrench_allocation_matrix_received_;
  bool differential_allocator_initialized_;
  float wrench_allocation_matrix_[6][MAX_MOTOR_NUMBER];
  float allocator_thrust_[MAX_MOTOR_NUMBER];
  bool hover_thrust_reference_enabled_ = false;
  bool hover_thrust_capture_allowed_ = false;
  uint32_t hover_thrust_capture_stamp_ = 0;
  float hover_thrust_reference_ = 0.0f;
  float hover_thrust_sample_time_ = 0.0f;
  float hover_thrust_sample_integral_ = 0.0f;
  float hover_thrust_sample_start_ = 0.0f;
  float allocator_gimbal_angle_[MAX_MOTOR_NUMBER];
  float allocator_thrust_command_[MAX_MOTOR_NUMBER];
  float allocator_gimbal_command_[MAX_MOTOR_NUMBER];
  float allocator_thrust_rate_[MAX_MOTOR_NUMBER];
  float allocator_gimbal_rate_[MAX_MOTOR_NUMBER];
  float previous_desired_wrench_[6];
  float filtered_desired_wrench_rate_[6];
  /* Keep the fixed-size solver workspace out of the flight-control task
   * stack.  In particular, the legacy F7 target only allocates 1 KiB to
   * that task. */
  float differential_desired_virtual_force_[MAX_MOTOR_NUMBER];
  float differential_desired_wrench_[6];
  float differential_current_virtual_force_[MAX_MOTOR_NUMBER];
  float differential_current_wrench_[6];
  float differential_jacobian_[6][MAX_MOTOR_NUMBER];
  float differential_weighted_jacobian_[6][MAX_MOTOR_NUMBER];
  float differential_weighted_wrench_rate_reference_[6];
  float differential_inverse_rate_weight_[MAX_MOTOR_NUMBER];
  float differential_normal_matrix_[6][6];
  float differential_task_dual_[6];
  float differential_actuator_rate_[MAX_MOTOR_NUMBER];
  float differential_secondary_rate_[MAX_MOTOR_NUMBER];
  float differential_rate_center_[MAX_MOTOR_NUMBER];
  float differential_rate_half_range_[MAX_MOTOR_NUMBER];
  uint8_t differential_lower_limit_reason_[MAX_MOTOR_NUMBER];
  uint8_t differential_upper_limit_reason_[MAX_MOTOR_NUMBER];
  float differential_secondary_wrench_rate_[6];
  float differential_secondary_dual_[6];
  float differential_solver_augmented_[6][7];
  float differential_allocation_damping_;
  float differential_allocation_wrench_error_gain_;
  float differential_allocation_wrench_rate_feedforward_gain_;
  float differential_allocation_wrench_rate_filter_;
  float differential_allocation_force_weight_;
  float differential_allocation_torque_weight_;
  float differential_allocation_thrust_rate_weight_;
  float differential_allocation_servo_rate_weight_;
  float differential_allocation_servo_center_gain_;
  float differential_allocation_thrust_balance_gain_;
  float differential_allocation_max_thrust_rate_;
  float differential_allocation_max_servo_rate_;
  float differential_allocation_max_servo_acceleration_;
  float differential_allocation_min_servo_angle_;
  float differential_allocation_max_servo_angle_;
  bool differential_allocation_servo_angle_limit_enabled_;
  float differential_allocation_thrust_time_constant_;
  float differential_allocation_servo_time_constant_;
  float differential_uniform_scale_;
  int differential_limiting_input_;
  uint8_t differential_limiting_bound_reason_;
#ifdef SIMULATION
  uint32_t differential_allocation_debug_pub_last_time_;
  uint32_t gimbal_state_last_time_;
  bool gimbal_state_received_[MAX_MOTOR_NUMBER];
  float measured_gimbal_angle_[MAX_MOTOR_NUMBER];
  float differential_unscaled_normalized_rate_[MAX_MOTOR_NUMBER];
  float differential_jacobian_min_singular_value_;
  float differential_jacobian_max_singular_value_;
  float differential_jacobian_condition_number_;
#endif
  float min_duty_;
  float max_duty_;
  float min_thrust_; // max thrust is variant according to the voltage
  float force_landing_thrust_;
  int8_t rotor_devider_;
  int8_t pwm_conversion_mode_;
  std::vector<spinal::MotorInfo> motor_info_;
  uint8_t motor_ref_index_;
  float v_factor_;
  uint32_t voltage_update_last_time_;
  uint32_t control_term_pub_last_time_, control_feedback_state_pub_last_time_;
  uint32_t pwm_pub_last_time_;
  float pwm_test_value_[MAX_MOTOR_NUMBER]; // PWM Test

  void fourAxisCommandCallback( const spinal::FourAxisCommand &cmd_msg);
  void pwmInfoCallback( const spinal::PwmInfo &info_msg);
  void rpyGainCallback( const spinal::RollPitchYawTerms &gain_msg);
  void pMatrixInertiaCallback(const spinal::PMatrixPseudoInverseWithInertia& msg);
  void torqueAllocationMatrixInvCallback(const spinal::TorqueAllocationMatrixInv& msg);
  void wrenchAllocationMatrixCallback(const spinal::WrenchAllocationMatrix& msg);
  void differentialAllocationConfigCallback(const spinal::DifferentialAllocationConfig& msg);
#ifdef SIMULATION
  void jointStateCallback(const sensor_msgs::JointState& msg);
#endif
  void offsetRotCallback(const spinal::DesireCoord& msg);

  bool differentialAllocationUpdate(float dt, float maximum_thrust);
  bool solveDifferentialAllocationSystem(const float matrix[6][6], const float rhs[6], float solution[6]);
#ifdef SIMULATION
  void publishDifferentialAllocationDebug(const float desired_wrench[6], const float current_wrench[6],
                                          int physical_rotor_number);
  void updateDifferentialAllocationConditionDebug();
#endif

  void thrustGainMapping();
  void maxYawGainIndex();
  void pwmTestCallback(const spinal::PwmTest& pwm_msg);
  void pwmConversion(void);
  void pwmsControl(void);

  void reset(void);

#ifdef SIMULATION
  uint32_t HAL_GetTick(){ return ros::Time::now().toSec() * 1000; }
public:
  float DELTA_T;
  double prev_time_;
#endif
};
#endif
