#include <bee/cooperative_grasp/controller.h>
#include <bee/GraspReference.h>
#include <bee/GraspTrackingStatus.h>
#include <bee/cooperative_grasp/phase_manager.h>
#include <bee/cooperative_grasp/safety_monitor.h>
#include <bee/cooperative_grasp/state_estimator.h>
#include <aerial_robot_msgs/FlightNav.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Trigger.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <set>
#include <thread>

namespace bee::grasp
{
namespace
{
double wallNow()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string frameName(std::string name)
{
  while (!name.empty() && name.front() == '/')
    name.erase(name.begin());
  return name;
}
Vec3 vector(const geometry_msgs::Vector3& v)
{
  return Vec3(v.x, v.y, v.z);
}
Pose pose(const geometry_msgs::Pose& p)
{
  return { Vec3(p.position.x, p.position.y, p.position.z),
           Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z) };
}
bool validPose(const Pose& p)
{
  return p.position.allFinite() && p.orientation.coeffs().allFinite() &&
         std::abs(p.orientation.squaredNorm() - 1) < .05;
}
Vec3 getVector(ros::NodeHandle& nh, const std::string& key)
{
  std::vector<double> v;
  if (!nh.getParam(key, v) || v.size() != 3)
    throw std::invalid_argument("expected 3-vector: " + key);
  Vec3 result(v[0], v[1], v[2]);
  if (!result.allFinite())
    throw std::invalid_argument("non-finite vector: " + key);
  return result;
}
struct Receipt
{
  double wall = 0, source = 0;
  bool valid = false;
};
}  // namespace

class CooperativeGraspNode
{
public:
  CooperativeGraspNode() : private_("~")
  {
    private_.param("execute", execute_, false);
    bool simulation = false;
    ros::param::param("/use_sim_time", simulation, false);
    // Hardware execution remains in the validated Python entry point until
    // numerical and closed-loop migration acceptance is complete.
    if (execute_ && !simulation)
      throw std::invalid_argument("C++ grasp execution currently requires simulation time");
    private_.param("world_frame", world_frame_, std::string("world"));
    private_.param("allow_empty_base_frame", allow_empty_base_frame_, false);
    if (world_frame_.empty())
      throw std::invalid_argument("world_frame must not be empty");
    std::string scene_namespace;
    private_.param("scene_namespace", scene_namespace, std::string("/bee_grasp/scene"));
    ros::NodeHandle scene_nh(scene_namespace);
    bool available = false;
    double deadline = wallNow() + 30;
    while (ros::ok() && wallNow() < deadline)
    {
      scene_nh.param("available", available, false);
      if (available)
        break;
      ros::WallDuration(.05).sleep();
    }
    if (!available)
      throw std::runtime_error("resolved scene manifest unavailable at " + scene_namespace);
    int version = 0;
    scene_nh.getParam("schema_version", version);
    if (version != 1)
      throw std::invalid_argument("unsupported scene manifest version");
    scene_nh.getParam("robot_names", scene_.robot_names);
    scene_nh.getParam("face_angles", scene_.face_angles);
    scene_nh.getParam("face_radii", scene_.face_radii);
    if (!scene_nh.getParam("object_mass", scene_.object_mass) ||
        !scene_nh.getParam("object_height", scene_.object_height) ||
        !scene_nh.getParam("staging_clearance", scene_.staging_clearance) ||
        !scene_nh.getParam("has_destination", scene_.has_destination))
      throw std::invalid_argument("incomplete scene manifest");
    scene_.object_position = getVector(scene_nh, "object_position");
    scene_.destination_position = getVector(scene_nh, "destination_position");
    scene_.validate();
#define GRASP_PARAM(name) private_.param("controller/" #name, control_.name, control_.name);
#include <bee/cooperative_grasp/control_parameters.inc>
#undef GRASP_PARAM
    if (private_.hasParam("controller/contact_offset"))
      control_.contact_offset = getVector(private_, "controller/contact_offset");
#define GRASP_PARAM(name) private_.param("task/" #name, task_.name, task_.name);
#include <bee/cooperative_grasp/task_parameters.inc>
#undef GRASP_PARAM
    private_.param("task/automatic_transport", task_.automatic_transport, true);
#define GRASP_PARAM(name) private_.param("safety/" #name, safety_config_.name, safety_config_.name);
#include <bee/cooperative_grasp/safety_parameters.inc>
#undef GRASP_PARAM
    control_.validate();
    task_.validate();
#define GRASP_PARAM(name)                                                                                              \
  if (!std::isfinite(safety_config_.name) || safety_config_.name <= 0)                                                 \
    throw std::invalid_argument("invalid safety/" #name);
#include <bee/cooperative_grasp/safety_parameters.inc>
#undef GRASP_PARAM
    if (private_.hasParam("task/normal_force"))
      throw std::invalid_argument("task/normal_force was replaced by friction_coefficient, normal_force_margin and design_vertical_acceleration");
    if (task_.automatic_transport && !scene_.has_destination)
      throw std::invalid_argument("automatic transport requires a destination pedestal");
    private_.param("control_period", period_, .01);
    if (!std::isfinite(period_) || period_ <= 0 || period_ > .05)
      throw std::invalid_argument("control_period must be in (0, .05]");
    private_.param("command_timeout", command_timeout_, .3);
    if (!std::isfinite(command_timeout_) || command_timeout_ < period_ || command_timeout_ > 1)
      throw std::invalid_argument("invalid command_timeout");
    estimator_ = std::make_unique<GraspStateEstimator>(scene_, control_);
    controller_ = std::make_unique<CooperativeGraspController>(scene_, control_);
    phases_ = std::make_unique<GraspPhaseManager>(scene_, control_, task_);
    private_.setParam("normal_force_target", phases_->normalForceTarget());
    ROS_INFO_STREAM("Calculated normal force target: " << phases_->normalForceTarget() << " N per Bee");
    safety_ = std::make_unique<GraspSafetyMonitor>(safety_config_);
    input_.vehicles.resize(scene_.robot_names.size());
    tracker_receipts_.resize(scene_.robot_names.size());
    tracker_states_.resize(scene_.robot_names.size());
    receipts_.resize(1 + 4 * input_.vehicles.size());
    cog_frames_.resize(input_.vehicles.size());
    std::string object_topic;
    private_.param("object_pose_topic", object_topic, std::string("/bee1/mujoco/grasp_object_pose"));
    subscribers_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>(
        object_topic, 1, [this](const geometry_msgs::PoseStamped::ConstPtr& m) {
          const Pose p = pose(m->pose);
          const bool good = validPose(p) && frameName(m->header.frame_id) == frameName(world_frame_);
          receipts_[0] = { wallNow(), m->header.stamp.toSec(), good };
          if (good)
          {
            input_.object = p;
            input_.object.orientation.normalize();
            input_.object_stamp = m->header.stamp.toSec();
          }
        }));
    std::set<std::string> namespaces;
    for (std::size_t i = 0; i < input_.vehicles.size(); ++i)
    {
      const auto& name = scene_.robot_names[i];
      std::string root, base_topic, cog_topic, wrench_topic, state_topic;
      private_.param("robot_namespaces/" + name, root, name);
      if (root.empty())
        throw std::invalid_argument("empty robot namespace");
      root = "/" + frameName(root);
      while (root.size() > 1 && root.back() == '/')
        root.pop_back();
      if (root == "/" || !namespaces.insert(root).second)
        throw std::invalid_argument("robot namespaces must be distinct and nonempty");
      private_.param("cog_frames/" + name, cog_frames_[i], name + "/cog");
      private_.param("base_topics/" + name, base_topic, root + "/ground_truth");
      private_.param("cog_topics/" + name, cog_topic, root + "/uav/cog/odom");
      private_.param("wrench_topics/" + name, wrench_topic, root + "/estimated_external_wrench");
      private_.param("state_topics/" + name, state_topic, root + "/flight_state");
      subscribers_.push_back(nh_.subscribe<nav_msgs::Odometry>(
          base_topic, 1, [this, i](const nav_msgs::Odometry::ConstPtr& m) { receiveOdometry(i, false, *m); }));
      subscribers_.push_back(nh_.subscribe<nav_msgs::Odometry>(
          cog_topic, 1, [this, i](const nav_msgs::Odometry::ConstPtr& m) { receiveOdometry(i, true, *m); }));
      subscribers_.push_back(nh_.subscribe<geometry_msgs::WrenchStamped>(
          wrench_topic, 1, [this, i](const geometry_msgs::WrenchStamped::ConstPtr& m) {
            Wrench wrench{ vector(m->wrench.force), vector(m->wrench.torque) };
            const bool good = wrench.force.allFinite() && wrench.moment.allFinite() &&
                              frameName(m->header.frame_id) == frameName(cog_frames_[i]);
            receipts_[1 + 4 * i + 2] = { wallNow(), m->header.stamp.toSec(), good };
            if (good)
              input_.vehicles[i].observer_cog = wrench;
          }));
      subscribers_.push_back(
          nh_.subscribe<std_msgs::UInt8>(state_topic, 1, [this, i](const std_msgs::UInt8::ConstPtr& m) {
            input_.vehicles[i].flight_state = m->data;
            flight_states_[i].store(m->data);
            receipts_[1 + 4 * i + 3] = { wallNow(), 0, true };
          }));
      flight_states_[i].store(0);
      preview_reference_.push_back(private_.advertise<bee::GraspReference>("preview/" + name + "/reference", 1));
      if (control_.distributed_tracking && execute_)
      {
        references_.push_back(nh_.advertise<bee::GraspReference>(root + "/grasp/reference", 1));
        subscribers_.push_back(nh_.subscribe<bee::GraspTrackingStatus>(root + "/grasp/tracking_status", 1,
            [this, i](const bee::GraspTrackingStatus::ConstPtr& m) {
          tracker_states_[i] = *m;
          tracker_receipts_[i] = wallNow();
        }));
      }
      preview_.push_back(private_.advertise<aerial_robot_msgs::FlightNav>("preview/" + name + "/nav", 1));
      preview_attitude_.push_back(
          private_.advertise<geometry_msgs::Vector3Stamped>("preview/" + name + "/attitude", 1));
      if (execute_ && !control_.distributed_tracking)
      {
        nav_.push_back(nh_.advertise<aerial_robot_msgs::FlightNav>(root + "/uav/nav", 1));
        attitude_.push_back(nh_.advertise<geometry_msgs::Vector3Stamped>(root + "/final_target_baselink_rpy", 1));
      }
    }
    phase_pub_ = private_.advertise<std_msgs::String>("phase", 1, true);
    reason_pub_ = private_.advertise<std_msgs::String>("reason", 1, true);
    metrics_ = private_.advertise<std_msgs::Float64MultiArray>("metrics", 1);
    geometry_errors_pub_ = private_.advertise<std_msgs::Float64MultiArray>("contact_geometry_errors", 1);
    start_service_ = private_.advertiseService("start", &CooperativeGraspNode::start, this);
    transport_service_ = private_.advertiseService("transport", &CooperativeGraspNode::transport, this);
    abort_service_ = private_.advertiseService("abort", &CooperativeGraspNode::abort, this);
    timer_ = nh_.createWallTimer(ros::WallDuration(period_), &CooperativeGraspNode::tick, this);
    watchdog_ = std::thread([this] {
      while (!exit_.load())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (active_.load() && wallNow() - heartbeat_.load() > command_timeout_)
          latchAbort("control loop heartbeat lost");
      }
    });
    publishPhase();
    ROS_INFO("Cooperative grasp ready (%s); start via ~start after all vehicles reach HOVER",
             execute_ ? "simulation execution" : "preview only");
  }
  ~CooperativeGraspNode()
  {
    exit_.store(true);
    if (watchdog_.joinable())
      watchdog_.join();
    if (active_.load())
    {
      std::lock_guard<std::mutex> guard(command_mutex_);
      stopCommands();
    }
  }

private:
  void receiveOdometry(std::size_t i, bool cog, const nav_msgs::Odometry& m)
  {
    Pose p = pose(m.pose.pose);
    const Vec3 velocity = vector(m.twist.twist.linear), omega = vector(m.twist.twist.angular);
    const bool good = validPose(p) && velocity.allFinite() && omega.allFinite() &&
                      (frameName(m.header.frame_id) == frameName(world_frame_) ||
                       (!cog && allow_empty_base_frame_ && m.header.frame_id.empty())) &&
                      (!cog || frameName(m.child_frame_id) == frameName(cog_frames_[i]));
    receipts_[1 + 4 * i + (cog ? 1 : 0)] = { wallNow(), m.header.stamp.toSec(), good };
    if (!good)
      return;
    p.orientation.normalize();
    if (cog)
      input_.vehicles[i].cog = p;
    else
    {
      input_.vehicles[i].base = p;
      input_.vehicles[i].velocity_world = velocity;
      input_.vehicles[i].angular_velocity_base = omega;
    }
  }
  std::string inputError() const
  {
    const double wall = wallNow(), source = ros::Time::now().toSec();
    if (execute_ && control_.distributed_tracking)
      for (std::size_t i = 0; i < tracker_states_.size(); ++i)
      {
        const auto& t = tracker_states_[i];
        if (wall - tracker_receipts_[i] > command_timeout_ || !t.ready || !t.fault.empty() ||
            source - t.header.stamp.toSec() > safety_config_.input_timeout || t.header.stamp.toSec() - source > .05)
          return "tracker " + scene_.robot_names[i] + " not ready: " + t.fault;
      }
    for (std::size_t i = 0; i < receipts_.size(); ++i)
    {
      const auto& r = receipts_[i];
      const bool state_message = i > 0 && (i - 1) % 4 == 3;
      if (!r.valid || wall - r.wall > safety_config_.input_timeout ||
          (!state_message &&
           (r.source <= 0 || source - r.source > safety_config_.input_timeout || r.source - source > .05)))
        return "invalid, missing or stale input " + std::to_string(i) +
               " (0=object; then base/cog/wrench/state per robot)";
    }
    for (const auto& v : input_.vehicles)
      if (v.flight_state != 5)
        return "all vehicles must be in HOVER";
    return {};
  }
  bool start(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
  {
    if (aborted_.load() || phases_->phase() != Phase::Idle)
    {
      response.message = "node is not Idle or abort is latched";
      return true;
    }
    response.message = inputError();
    if (!response.message.empty())
      return true;
    if (execute_)
      for (const auto& pub : references_)
        if (!pub.getNumSubscribers())
        { response.message = "missing local grasp-mode subscriber"; return true; }
    if (execute_)
      for (std::size_t i = 0; i < nav_.size(); ++i)
        if (!nav_[i].getNumSubscribers() || !attitude_[i].getNumSubscribers())
        {
          response.message = "missing flight command subscriber";
          return true;
        }
    controller_->reset();
    estimator_->reset();
    const auto s = estimator_->update(input_);
    previous_time_ = ros::Time::now().toSec();
    phases_->start(s, previous_time_);
    heartbeat_.store(wallNow());
    active_.store(true);
    response.success = true;
    response.message = execute_ ? "started" : "preview started; flight topics are disabled";
    return true;
  }
  bool transport(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
  {
    response.success = phases_->phase() == Phase::Hold && scene_.has_destination;
    if (response.success)
      phases_->requestTransport();
    response.message = response.success ? "transport requested" : "requires Hold and a destination";
    return true;
  }
  bool abort(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
  {
    latchAbort("operator abort");
    response.success = true;
    return true;
  }
  void latchAbort(const std::string& reason)
  {
    std::lock_guard<std::mutex> guard(command_mutex_);
    if (!aborted_.load())
    {
      abort_reason_ = reason;
      aborted_.store(true);
      ROS_ERROR("Cooperative grasp aborted: %s", reason.c_str());
    }
    if (active_.load())
      stopCommands();
  }
  void stopCommands()
  {
    if (!execute_)
      return;
    for (const auto& publisher : references_)
    {
      bee::GraspReference reference;
      reference.header.stamp = ros::Time::now(); reference.header.frame_id = world_frame_;
      reference.stop = true;
      publisher.publish(reference);
    }
    for (std::size_t i = 0; i < nav_.size(); ++i)
    {
      if (flight_states_[i].load() != 5)
        continue;
      aerial_robot_msgs::FlightNav msg;
      msg.header.stamp = ros::Time::now();
      msg.target = msg.COG;
      msg.control_frame = msg.WORLD_FRAME;
      msg.pos_xy_nav_mode = msg.VEL_MODE;
      msg.pos_z_nav_mode = msg.VEL_MODE;
      nav_[i].publish(msg);  // preserve attitude; do not interfere with LAND/STOP
    }
  }
  void publishPhase()
  {
    std_msgs::String msg;
    msg.data = phaseName(phases_->phase());
    phase_pub_.publish(msg);
    msg.data = phases_->fault();
    reason_pub_.publish(msg);
  }
  void tick(const ros::WallTimerEvent&)
  {
    if (aborted_.load())
    {
      {
        std::lock_guard<std::mutex> guard(command_mutex_);
        phases_->abort(abort_reason_);
      }
      active_.store(false);
      publishPhase();
      return;
    }
    if (!active_.load())
      return;
    // Liveness belongs to the steady clock. Repeated /clock values must not
    // look like a blocked control loop; input receipt/source ages are checked
    // separately, and no controller state is integrated while dt == 0.
    heartbeat_.store(wallNow());
    const std::string error = inputError();
    if (!error.empty())
    {
      latchAbort(error);
      return;
    }
    const double now = ros::Time::now().toSec(), dt = now - previous_time_;
    if (dt < 0 || dt > safety_config_.input_timeout)
    {
      latchAbort("control clock discontinuity");
      return;
    }
    if (dt == 0)
      return;
    previous_time_ = now;
    try
    {
      const GraspState state = estimator_->update(input_);
      const Phase previous = phases_->phase();
      Reference reference = phases_->step(state, last_output_.status, now);
      if (reference.phase == Phase::Aborted)
      {
        latchAbort(phases_->fault());
        return;
      }
      if (reference.calibrate && previous != Phase::Calibrate)
        estimator_->beginCalibration();
      if (reference.calibrate)
        estimator_->addCalibrationSample(input_);
      if (previous == Phase::Calibrate && !reference.calibrate && !estimator_->finishCalibration())
      {
        latchAbort("empty observer calibration");
        return;
      }
      const ControlOutput output = controller_->step(state, reference, dt);
      const std::string reason = safety_->check(state, reference, output, std::min(dt, .05));
      if (!reason.empty())
      {
        latchAbort(reason);
        return;
      }
      {
        std::lock_guard<std::mutex> guard(command_mutex_);
        if (aborted_.load())
          return;
        if (reference.phase == Phase::Complete)
        {
          stopCommands();
          active_.store(false);
        }
        else
          publishCommands(output);
        heartbeat_.store(wallNow());
      }
      last_output_ = output;
      if (previous != reference.phase)
        ROS_INFO("Cooperative grasp: %s", phaseName(reference.phase));
      publishPhase();
      std_msgs::Float64MultiArray metrics;
      metrics.data = {
        output.status.normal_force,          output.status.reference_z,   output.status.reference_vz,
        output.status.reference_az,          output.status.z_integral,    output.status.vertical_force_target,
        output.status.common_z_acceleration, output.status.support_ratio, output.status.common_slip
      };
      metrics_.publish(metrics);
      std_msgs::Float64MultiArray geometry_errors;
      for (const auto& e : output.status.contact_geometry_errors)
        for (int axis = 0; axis < 3; ++axis)
          geometry_errors.data.push_back(e[axis]);
      geometry_errors_pub_.publish(geometry_errors);
    }
    catch (const std::exception& e)
    {
      latchAbort(e.what());
    }
  }
  void publishCommands(const ControlOutput& output)
  {
    for (std::size_t i = 0; i < output.commands.size(); ++i)
    {
      const auto& c = output.commands[i];
      aerial_robot_msgs::FlightNav msg;
      msg.header.stamp = ros::Time::now();
      msg.control_frame = msg.WORLD_FRAME;
      msg.target = msg.COG;
      msg.pos_xy_nav_mode = msg.ACC_MODE;
      msg.target_acc_x = c.acceleration.x();
      msg.target_acc_y = c.acceleration.y();
      msg.yaw_nav_mode = msg.POS_MODE;
      msg.target_yaw = c.yaw;
      msg.roll_nav_mode = msg.pitch_nav_mode = msg.POS_MODE;
      if (c.z_mode == ZMode::Acceleration)
      {
        msg.pos_z_nav_mode = msg.ACC_MODE;
        msg.target_acc_z = c.z;
      }
      else if (c.z_mode == ZMode::Velocity)
      {
        msg.pos_z_nav_mode = msg.VEL_MODE;
        msg.target_vel_z = c.z;
      }
      else
      {
        msg.pos_z_nav_mode = msg.POS_MODE;
        msg.target_pos_z = c.z;
      }
      geometry_msgs::Vector3Stamped attitude;
      attitude.header = msg.header;
      attitude.header.frame_id = world_frame_;
      attitude.vector.x = c.baselink_rpy.x();
      attitude.vector.y = c.baselink_rpy.y();
      attitude.vector.z = c.baselink_rpy.z();
      preview_[i].publish(msg);
      preview_attitude_[i].publish(attitude);
      bee::GraspReference reference;
      reference.header = attitude.header;
      reference.navigation = msg;
      reference.baselink_rpy = attitude.vector;
      auto copy = [](const Vec3& v, geometry_msgs::Vector3& m) { m.x = v.x(); m.y = v.y(); m.z = v.z(); };
      copy(c.velocity, reference.velocity); copy(c.acceleration, reference.acceleration);
      copy(c.normal, reference.normal); copy(c.object_position, reference.object_position);
      reference.contact_tracking = c.contact_tracking; reference.support_tracking = c.support_tracking;
      reference.calibrate = c.calibrate; reference.normal_force = c.normal_force;
      reference.minimum_radius = c.minimum_radius; reference.support_force = c.support_force;
      reference.observer_blend = c.observer_blend; reference.support_scale = c.support_scale;
      preview_reference_[i].publish(reference);
      if (execute_ && control_.distributed_tracking)
        references_[i].publish(reference);
      if (execute_ && !control_.distributed_tracking)
      {
        nav_[i].publish(msg);
        attitude_[i].publish(attitude);
      }
    }
  }
  ros::NodeHandle nh_, private_;
  SceneConfig scene_;
  ControlConfig control_;
  TaskConfig task_;
  SafetyConfig safety_config_;
  std::unique_ptr<GraspStateEstimator> estimator_;
  std::unique_ptr<CooperativeGraspController> controller_;
  std::unique_ptr<GraspPhaseManager> phases_;
  std::unique_ptr<GraspSafetyMonitor> safety_;
  InputState input_;
  ControlOutput last_output_;
  std::vector<Receipt> receipts_;
  std::vector<std::string> cog_frames_;
  std::vector<ros::Subscriber> subscribers_;
  std::vector<ros::Publisher> nav_, attitude_, preview_, preview_attitude_;
  std::vector<ros::Publisher> references_, preview_reference_;
  std::vector<bee::GraspTrackingStatus> tracker_states_;
  std::vector<double> tracker_receipts_;
  ros::Publisher phase_pub_, reason_pub_, metrics_, geometry_errors_pub_;
  ros::ServiceServer start_service_, transport_service_, abort_service_;
  ros::WallTimer timer_;
  bool execute_ = false, allow_empty_base_frame_ = false;
  double period_ = .01, command_timeout_ = .3, previous_time_ = 0;
  std::string world_frame_, abort_reason_;
  std::atomic<bool> active_{ false }, aborted_{ false }, exit_{ false };
  std::atomic<double> heartbeat_{ 0 };
  std::atomic<int> flight_states_[5];
  std::mutex command_mutex_;
  std::thread watchdog_;
};
}  // namespace bee::grasp
int main(int argc, char** argv)
{
  ros::init(argc, argv, "bee_cooperative_grasp");
  try
  {
    bee::grasp::CooperativeGraspNode node;
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("Cooperative grasp: %s", e.what());
    return 1;
  }
  return 0;
}
