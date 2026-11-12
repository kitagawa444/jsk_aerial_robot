#include <bee/control/grasp_mode.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <mutex>
#include <bee/GraspReference.h>
#include <bee/GraspTrackingStatus.h>
#include <bee/cooperative_grasp/types.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <ros/ros.h>
#include <chrono>
#include <array>

namespace bee::grasp
{
namespace
{
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
// It receives cooperative references and uses only local odometry/observer data.
class GraspMode::Impl
{
public:
  Impl(ros::NodeHandle nh, ros::NodeHandle parameters, bool enabled)
    : nh_(nh), priv_(parameters), enabled_(enabled)
  {
#define GRASP_PARAM(name) priv_.param("controller/" #name, config_.name, config_.name);
#include <bee/cooperative_grasp/control_parameters.inc>
#undef GRASP_PARAM
    std::vector<double> offset;
    if (priv_.getParam("controller/contact_offset", offset))
    {
      if (offset.size() != 3) throw std::invalid_argument("contact_offset must have 3 elements");
      config_.contact_offset = Vec3(offset[0], offset[1], offset[2]);
    }
    config_.validate();
    priv_.param("tracker/velocity_gain_xy", velocity_gain_xy_, 3.0);
    priv_.param("tracker/velocity_gain_z", velocity_gain_z_, 3.0);
    priv_.param("tracker/external_filter_tau", external_tau_, .2);
    priv_.param("tracker/input_timeout", input_timeout_, .3);
    priv_.param("tracker/reference_timeout", reference_timeout_, .3);
    priv_.param("tracker/maximum_acceleration", max_acceleration_, 20.0);
    priv_.param("tracker/maximum_velocity_error", max_velocity_error_, .3);
    priv_.param("tracker/velocity_error_duration", velocity_error_duration_, 1.0);
    priv_.param("allow_empty_base_frame", allow_empty_base_frame_, false);
    priv_.param("world_frame", world_, std::string("world"));
    priv_.param("cog_frame", cog_frame_, frame(nh_.getNamespace()) + "/cog");
    for (double value : {velocity_gain_xy_, velocity_gain_z_, external_tau_, input_timeout_, reference_timeout_,
                         max_acceleration_, max_velocity_error_, velocity_error_duration_})
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid tracker gain or limit");
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
      accept(2, m->header.stamp, raw_force_.allFinite() && frame(m->header.frame_id) == frame(cog_frame_));
    });
    state_sub_ = nh_.subscribe<std_msgs::UInt8>("flight_state", 1, [this](const std_msgs::UInt8::ConstPtr& m) {
      std::lock_guard<std::mutex> lock(mutex_);
      flight_state_ = m->data; receipt_[3] = steadyNow();
    });
    ref_sub_ = nh_.subscribe("grasp/reference", 1, &Impl::reference, this);
    mode_service_ = nh_.advertiseService("grasp/set_mode", &Impl::setMode, this);
    mode_pub_ = nh_.advertise<std_msgs::Bool>("grasp/enabled", 1, true);
    publishMode();
    status_pub_ = nh_.advertise<bee::GraspTrackingStatus>("grasp/tracking_status", 1);
    timer_ = nh_.createWallTimer(ros::WallDuration(.01), &Impl::tick, this);
  }
  ~Impl() { timer_.stop(); }
  bool takeCommand(aerial_robot_msgs::FlightNav& nav, geometry_msgs::Vector3Stamped& attitude, bool& release, bool available)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    controller_receipt_ = steadyNow();
    controller_available_ = available;
    if (!pending_) return false;
    nav = command_; attitude = attitude_; release = release_;
    pending_ = false;
    return true;
  }
private:
  void publishMode()
  {
    std_msgs::Bool m; m.data = enabled_; mode_pub_.publish(m);
    nh_.setParam("controller/grasp_mode", enabled_);
  }
  bool setMode(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (req.data != enabled_)
    {
      if (!req.data) stop();
      enabled_ = req.data;
      active_ = calibrating_ = calibrated_ = false;
      error_time_ = 0;
      // An explicit off/on cycle starts a new session; a reference alone cannot
      // clear a fault or revive an interrupted grasp.
      if (enabled_) { fault_.clear(); reference_receipt_ = 0; }
      publishMode();
    }
    res.success = true;
    res.message = enabled_ ? "grasp mode enabled; waiting for fresh references" : "normal mode";
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
  bool inputsReady() const
  {
    const double now = steadyNow(), sim = ros::Time::now().toSec();
    if (!controller_available_ || now - controller_receipt_ > input_timeout_) return false;
    for (int i = 0; i < 4; ++i)
      if (receipt_[i] <= 0 || now - receipt_[i] > input_timeout_ ||
          (i < 3 && (source_[i] <= 0 || sim - source_[i] > input_timeout_ || source_[i] - sim > .05)))
        return false;
    return flight_state_ == 5;
  }
  void reference(const bee::GraspReference::ConstPtr& m)
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
    const bool valid = frame(m->header.frame_id) == frame(world_) &&
        vec(m->velocity).allFinite() && vec(m->acceleration).allFinite() && vec(m->normal).allFinite() &&
        vec(m->object_position).allFinite() && vec(m->baselink_rpy).allFinite() &&
        std::isfinite(m->normal_force) && m->normal_force >= 0 && m->normal_force <= config_.max_normal_force &&
        std::isfinite(m->minimum_radius) && std::isfinite(m->support_force) && m->support_force >= 0 &&
        std::isfinite(m->observer_blend) && m->observer_blend >= 0 && m->observer_blend <= 1 &&
        std::isfinite(m->support_scale) && m->support_scale >= 0 && m->support_scale <= 1 &&
        (!m->contact_tracking || (std::abs(vec(m->normal).norm() - 1) < 1e-3 && std::abs(m->normal.z) < 1e-3)) &&
        (!m->contact_tracking || (vec(m->velocity).norm() <= config_.reference_velocity_limit + 1e-6 &&
                                 vec(m->acceleration).norm() <= config_.reference_acceleration_limit + 1e-6)) &&
        (!m->support_tracking || m->contact_tracking) &&
        std::isfinite(n.target_acc_x) && std::isfinite(n.target_acc_y) && std::isfinite(n.target_acc_z) &&
        std::isfinite(n.target_pos_z) && std::isfinite(n.target_vel_z) && std::isfinite(n.target_yaw) &&
        n.control_frame == n.WORLD_FRAME && n.target == n.COG;
    if (!valid) { fail("invalid reference"); return; }
    if (!active_) last_time_ = ros::Time::now().toSec();
    reference_ = *m;
    reference_receipt_ = steadyNow();
    active_ = true;
  }
  void stop()
  {
    if (!active_) return;
    aerial_robot_msgs::FlightNav m;
    m.header.stamp = ros::Time::now(); m.target = m.COG; m.control_frame = m.WORLD_FRAME;
    m.pos_xy_nav_mode = m.pos_z_nav_mode = m.VEL_MODE;
    command_ = m; pending_ = true; release_ = true;
  }
  void fail(const std::string& reason)
  {
    if (fault_.empty()) ROS_ERROR_STREAM("Bee grasp mode: " << reason);
    fault_ = reason;
    stop(); active_ = false;
  }
  void tick(const ros::WallTimerEvent&)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = ros::Time::now().toSec();
    const bool ready = inputsReady();
    bee::GraspTrackingStatus status;
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
    status_pub_.publish(status);
  }
  void control(double dt, bee::GraspTrackingStatus& status)
  {
    const auto& r = reference_;
    if (r.calibrate && !calibrating_)
    {
      samples_ = 0; sum_.setZero(); world_sum_.setZero(); calibrated_ = false;
      last_calibration_stamp_ = -1;
    }
    if (r.calibrate && source_[2] != last_calibration_stamp_)
    {
      sum_ += raw_force_; world_sum_ += cog_.orientation * raw_force_; ++samples_;
      last_calibration_stamp_ = source_[2];
    }
    if (!r.calibrate && calibrating_)
    {
      if (samples_ < 2) { fail("insufficient observer calibration"); return; }
      bias_ = sum_ / samples_; world_bias_ = world_sum_ / samples_;
      external_.setZero(); normal_force_ = 0; calibrated_ = true;
    }
    calibrating_ = r.calibrate;
    auto nav = r.navigation;
    nav.header.stamp = ros::Time::now();
    if (r.contact_tracking)
    {
      if (!calibrated_) { fail("contact tracking requires observer calibration"); return; }
      const auto& p = config_;
      const Vec3 n = vec(r.normal);
      const Vec3 offset = base_.orientation * p.contact_offset;
      const Vec3 contact_velocity = velocity_ + (base_.orientation * omega_).cross(offset);
      const Vec3 error = vec(r.velocity) - contact_velocity;
      const Vec3 measured_force = p.observer_force_sign * p.observer_force_scale *
                                 (cog_.orientation * (raw_force_ - bias_));
      external_ += dt / (external_tau_ + dt) * (measured_force - external_);
      const double measured_normal = std::max(0.0, p.observer_force_sign * p.observer_force_scale *
          (cog_.orientation * raw_force_ - world_bias_).dot(n));
      normal_force_ += p.force_filter_alpha * (measured_normal - normal_force_);
      // Normal axis is force controlled; never cancel that reaction a second time.
      double radial = -r.normal_force / p.robot_mass - p.normal_force_kp * (r.normal_force - normal_force_) +
                      p.radial_kd * error.dot(n);
      const double radius = (base_.position - vec(r.object_position)).dot(n);
      if (radius < r.minimum_radius)
        radial = p.press_radial_kp * (r.minimum_radius + p.radius_margin - radius) + p.radial_kd * error.dot(n);
      const Vec3 tangent(-n.y(), n.x(), 0);
      Vec3 acceleration = vec(r.acceleration);
      const double tangential = acceleration.dot(tangent) + velocity_gain_xy_ * error.dot(tangent) -
                                (r.support_tracking ? external_.dot(tangent) / p.robot_mass : 0);
      acceleration = clamp(radial + acceleration.dot(n), p.press_limit) * n +
                     clamp(tangential, p.approach_limit) * tangent;
      nav.target_acc_x = acceleration.x(); nav.target_acc_y = acceleration.y();
      if (r.support_tracking)
      {
        // Bootstrap with planned load; replace it by measured load continuously.
        // These are weighted alternatives, not two additive full compensations.
        const double support = r.support_scale * ((1 - r.observer_blend) * r.support_force -
                                                 r.observer_blend * external_.z());
        acceleration.z() = clamp(r.acceleration.z + velocity_gain_z_ * error.z() + support / p.robot_mass,
                                 p.common_z_limit + p.relative_z_limit + p.tilt_limit);
        nav.pos_z_nav_mode = nav.ACC_MODE;
        nav.target_acc_z = acceleration.z();
      }
      // Monitor the motion-controlled axes, not commanded inward contact force.
      const double tracking_error = r.support_tracking ? std::hypot(error.dot(tangent), error.z()) : 0;
      error_time_ = tracking_error > max_velocity_error_ ? error_time_ + dt : 0;
      if (!acceleration.allFinite() || acceleration.norm() > max_acceleration_ || error_time_ > velocity_error_duration_)
      { fail("excessive acceleration or persistent velocity error"); return; }
      status.velocity = message(contact_velocity); status.velocity_error = message(error);
      status.external_force = message(external_); status.commanded_acceleration = message(acceleration);
    }
    else error_time_ = 0;
    geometry_msgs::Vector3Stamped attitude;
    attitude.header = nav.header; attitude.header.frame_id = world_; attitude.vector = r.baselink_rpy;
    command_ = nav; attitude_ = attitude; pending_ = true; release_ = false;
  }
  ros::NodeHandle nh_, priv_;
  ros::Subscriber base_sub_, cog_sub_, wrench_sub_, state_sub_, ref_sub_;
  ros::Publisher status_pub_, mode_pub_;
  ros::ServiceServer mode_service_;
  std::mutex mutex_;
  bool enabled_ = false, pending_ = false, release_ = false, controller_available_ = false;
  double controller_receipt_ = 0;
  aerial_robot_msgs::FlightNav command_;
  geometry_msgs::Vector3Stamped attitude_;
  ros::WallTimer timer_;
  ControlConfig config_;
  bee::GraspReference reference_;
  Pose base_, cog_;
  Vec3 velocity_ = Vec3::Zero(), omega_ = Vec3::Zero(), raw_force_ = Vec3::Zero();
  Vec3 bias_ = Vec3::Zero(), world_bias_ = Vec3::Zero(), sum_ = Vec3::Zero(), world_sum_ = Vec3::Zero();
  Vec3 external_ = Vec3::Zero();
  std::array<double, 4> receipt_{};
  std::array<double, 3> source_{};
  std::string world_, cog_frame_, fault_;
  bool allow_empty_base_frame_ = false, active_ = false, calibrating_ = false, calibrated_ = false;
  int flight_state_ = 0, samples_ = 0;
  double reference_receipt_ = 0, last_time_ = 0, normal_force_ = 0, error_time_ = 0, last_calibration_stamp_ = -1;
  double velocity_gain_xy_, velocity_gain_z_, external_tau_, input_timeout_, reference_timeout_, max_acceleration_;
  double max_velocity_error_, velocity_error_duration_;
};
}
namespace bee::grasp
{
GraspMode::GraspMode(ros::NodeHandle nh, ros::NodeHandle parameters, bool enabled)
  : impl_(new Impl(nh, parameters, enabled)) {}
GraspMode::~GraspMode() = default;
bool GraspMode::takeCommand(aerial_robot_msgs::FlightNav& nav, geometry_msgs::Vector3Stamped& attitude,
                            bool& release, bool available)
{
  return impl_->takeCommand(nav, attitude, release, available);
}
}
