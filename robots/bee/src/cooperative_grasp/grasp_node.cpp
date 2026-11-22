#include <bee/cooperative_grasp/grasp_node.h>
#include <bee/cooperative_grasp/grasp_controller.h>
#include <bee/model/bee_robot_model.h>
#include <bee/ObserverCalibrationStatus.h>
#include <bee/BeeCommand.h>
#include <bee/ControllerStatus.h>
#include <std_msgs/Empty.h>
#include <smach_msgs/SmachContainerStructure.h>
#include <smach_msgs/SmachContainerStatus.h>
#include <visualization_msgs/MarkerArray.h>
#include <bee/cooperative_grasp/phase_manager.h>
#include <bee/cooperative_grasp/grasp_state_estimator.h>
#include <aerial_robot_msgs/FlightNav.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Cholesky>
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
std::vector<Vec3> getVectors(ros::NodeHandle& nh, const std::string& key)
{
  std::vector<double> values;
  if (!nh.getParam(key, values) || values.empty() || values.size() % 3)
    throw std::invalid_argument("expected flattened vector array: " + key);
  std::vector<Vec3> result;
  for (std::size_t i = 0; i < values.size(); i += 3)
    result.emplace_back(values[i], values[i + 1], values[i + 2]);
  return result;
}
visualization_msgs::Marker forceArrow(const std_msgs::Header& header, const std::string& name,
                                     int id, const Vec3& origin, const Vec3& force, bool observed)
{
  visualization_msgs::Marker m;
  m.header = header;
  m.ns = name;
  m.id = id;
  m.type = m.ARROW;
  m.pose.orientation.w = 1;
  m.lifetime = ros::Duration(0.4);
  if (force.squaredNorm() < 1e-10)
  {
    m.action = m.DELETE;
    return m;
  }
  const Vec3 end = origin + 0.05 * force;
  for (const auto& v : {origin, end})
  {
    geometry_msgs::Point point;
    point.x = v.x(); point.y = v.y(); point.z = v.z();
    m.points.push_back(point);
  }
  m.scale.x = observed ? 0.008 : 0.014;
  m.scale.y = 2 * m.scale.x;
  m.color.r = observed ? 1.0 : 0.0;
  m.color.g = observed ? 0.4 : 0.75;
  m.color.b = observed ? 0.0 : 1.0;
  m.color.a = observed ? 1.0 : 0.55;
  return m;
}
}  // namespace

class GraspNode
{
public:
  GraspNode() : private_("~")
  {
    private_.param("execute", execute_, false);
    bool simulation = false;
    ros::param::param("/use_sim_time", simulation, false);
    private_.param("world_frame", world_frame_, std::string("world"));
    private_.param("allow_empty_base_frame", allow_empty_base_frame_, false);
    if (world_frame_.empty())
      throw std::invalid_argument("world_frame must not be empty");
    std::string scene_namespace;
    private_.param("scene_namespace", scene_namespace, std::string("/bee_grasp/scene"));
    if (execute_ && !simulation && scene_namespace == "/bee_grasp/scene")
      throw std::invalid_argument("set object_model to the payload configuration for hardware execution");
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
    if (version != 1 && version != 2 && version != 3)
      throw std::invalid_argument("unsupported scene manifest version");
    if (version >= 2)
      for (const auto& key : {"object_shape", "surface_radius", "surface_top_radius",
                              "surface_length", "object_bottom_offset"})
        if (!scene_nh.hasParam(key))
          throw std::invalid_argument(std::string("incomplete v2 surface geometry: ") + key);
    scene_nh.getParam("robot_names", scene_.robot_names);
    scene_nh.getParam("face_angles", scene_.face_angles);
    scene_nh.getParam("face_radii", scene_.face_radii);
    if (scene_nh.hasParam("surface_radii")) scene_.surface_radii = getVector(scene_nh, "surface_radii");
    scene_nh.param("object_shape", scene_.object_shape, std::string(""));
    if (scene_.object_shape == "rock")
    {
      if (version != 3) throw std::invalid_argument("rock requires scene manifest v3");
      scene_.surface_vertices = getVectors(scene_nh, "surface_vertices");
      scene_.contact_normals = getVectors(scene_nh, "contact_normals");
    }
    scene_nh.param("surface_radius", scene_.surface_radius, 0.0);
    scene_nh.param("surface_top_radius", scene_.surface_top_radius, 0.0);
    scene_nh.param("surface_length", scene_.surface_length, 0.0);
    scene_nh.param("object_bottom_offset", scene_.object_bottom_offset, -1.0);
    scene_nh.param("contact_height", scene_.contact_height, 0.0);
    if (!scene_nh.getParam("object_mass", scene_.object_mass) ||
        !scene_nh.getParam("object_height", scene_.object_height) ||
        !scene_nh.getParam("staging_clearance", scene_.staging_clearance) ||
        !scene_nh.getParam("has_destination", scene_.has_destination))
      throw std::invalid_argument("incomplete scene manifest");
    scene_.object_position = getVector(scene_nh, "object_position");
    scene_.destination_position = getVector(scene_nh, "destination_position");
    if (scene_nh.hasParam("object_inertia"))
    {
      std::vector<double> inertia;
      scene_nh.getParam("object_inertia", inertia);
      if (inertia.size() != 9) throw std::invalid_argument("object_inertia must have 9 elements");
      for (int j = 0; j < 9; ++j) scene_.object_inertia(j/3,j%3) = inertia[j];
      Eigen::LLT<Eigen::Matrix3d> check(scene_.object_inertia);
      if (!scene_.object_inertia.allFinite() || !scene_.object_inertia.isApprox(scene_.object_inertia.transpose()) ||
          check.info() != Eigen::Success) throw std::invalid_argument("invalid object inertia");
    }
    scene_.validate();
#define GRASP_PARAM(name) private_.param("controller/" #name, control_.name, control_.name);
#include <bee/cooperative_grasp/control_parameters.inc>
#undef GRASP_PARAM
    // Resolve the same URDFs used by the individual Bee models. The allocator
    // currently assumes identical vehicle masses; reject unsupported mixtures.
    for (std::size_t i = 0; i < scene_.robot_names.size(); ++i)
    {
      const auto& name = scene_.robot_names[i];
      std::string root, xml;
      private_.param("robot_namespaces/" + name, root, name);
      const std::string parameter = "/" + frameName(root) + "/robot_description";
      if (!nh_.getParam(parameter, xml))
        throw std::invalid_argument("missing model: " + parameter);
      if (execute_ && !simulation && !nh_.param<bool>("/" + frameName(root) + "/controller/gravity_compensation", false))
        throw std::invalid_argument("net acceleration commands require gravity_compensation for " + name);
      const double mass = bee::control::modelMass(xml);
      if (i && std::abs(mass - control_.robot_mass) > 1e-6)
        throw std::invalid_argument("allocator requires equal Bee model masses");
      control_.robot_mass = mass;
    }
    // Resolved value for recorders/identification, never an authoritative input.
    private_.setParam("controller/robot_mass", control_.robot_mass);
    private_.setParam("controller/gravity", control_.gravity);
    private_.setParam("model_mass", control_.robot_mass);
    if (private_.hasParam("controller/contact_offset"))
      control_.contact_offset = getVector(private_, "controller/contact_offset");
#define GRASP_PARAM(name) private_.param("task/" #name, task_.name, task_.name);
#include <bee/cooperative_grasp/task_parameters.inc>
#undef GRASP_PARAM
    private_.param("task/automatic_transport", task_.automatic_transport, true);
    if (control_.wrench_control && !scene_nh.hasParam("object_inertia"))
      throw std::invalid_argument("wrench control requires object_inertia in scene manifest");
    if (control_.wrench_control && control_.allocation_mu > task_.friction_coefficient)
      throw std::invalid_argument("allocation friction exceeds task design friction");
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
    private_.param("tracker/velocity_gain_xy", legacy_velocity_gain_xy_, 3.0);
    private_.param("tracker/velocity_gain_z", legacy_velocity_gain_z_, 3.0);
    if (!std::isfinite(legacy_velocity_gain_xy_) || legacy_velocity_gain_xy_ < 0 ||
        !std::isfinite(legacy_velocity_gain_z_) || legacy_velocity_gain_z_ < 0)
      throw std::invalid_argument("invalid upper-level velocity gain");
    estimator_ = std::make_unique<GraspStateEstimator>(scene_, control_);
    controller_ = std::make_unique<GraspController>(scene_, control_);
    phases_ = std::make_unique<GraspPhaseManager>(scene_, control_, task_);
    private_.setParam("normal_force_target", phases_->normalForceTarget());
    ROS_INFO_STREAM("Calculated normal force target: " << phases_->normalForceTarget() << " N per Bee");
    safety_ = std::make_unique<GraspSafetyMonitor>(safety_config_);
    input_.vehicles.resize(scene_.robot_names.size());
    tracker_receipts_.resize(scene_.robot_names.size());
    tracker_states_.resize(scene_.robot_names.size());
    calibration_starts_.resize(scene_.robot_names.size(), 0);
    external_filters_.resize(scene_.robot_names.size(), Vec3::Zero());
    receipts_.resize(1 + 4 * input_.vehicles.size());
    cog_frames_.resize(input_.vehicles.size());
    std::string object_topic;
    private_.param("object_pose_topic", object_topic, std::string("/mocap/object/pose"));
    Vec3 marker_translation = Vec3::Zero(), marker_rpy = Vec3::Zero();
    if (scene_nh.hasParam("marker_to_model_xyz"))
      marker_translation = getVector(scene_nh, "marker_to_model_xyz");
    if (scene_nh.hasParam("marker_to_model_rpy"))
      marker_rpy = getVector(scene_nh, "marker_to_model_rpy");
    const Eigen::Quaterniond marker_rotation = Eigen::AngleAxisd(marker_rpy.z(), Vec3::UnitZ()) *
        Eigen::AngleAxisd(marker_rpy.y(), Vec3::UnitY()) * Eigen::AngleAxisd(marker_rpy.x(), Vec3::UnitX());
    subscribers_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>(
        object_topic, 1, [this, marker_translation, marker_rotation](const geometry_msgs::PoseStamped::ConstPtr& m) {
          const Pose p = pose(m->pose);
          const bool good = validPose(p) && frameName(m->header.frame_id) == frameName(world_frame_);
          receipts_[0] = { wallNow(), m->header.stamp.toSec(), good };
          if (good)
          {
            const auto rotation = p.orientation.normalized();
            input_.object.position = p.position + rotation * marker_translation;
            input_.object.orientation = rotation * marker_rotation;
            input_.object_stamp = m->header.stamp.toSec();
          }
        }));
    std::set<std::string> namespaces;
    std::string base_odom_suffix;
    private_.param("base_odom_suffix", base_odom_suffix, std::string("ground_truth"));
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
      private_.param("base_topics/" + name, base_topic, root + "/" + base_odom_suffix);
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
      calibration_stamps_.push_back(0);
      if (control_.distributed_tracking)
        subscribers_.push_back(nh_.subscribe<bee::ObserverCalibrationStatus>(root + "/controller/observer_calibration", 1,
            [this, i](const bee::ObserverCalibrationStatus::ConstPtr& m) {
          const double stamp = m->calibration_stamp.toSec();
          if (m->calibration_start_stamp.isZero() || m->calibration_start_stamp > m->calibration_stamp ||
              stamp <= calibration_stamps_[i] || stamp > ros::Time::now().toSec() + .05 ||
              frameName(m->header.frame_id) != frameName(cog_frames_[i]) ||
              frameName(m->world_frame) != frameName(world_frame_)) return;
          const Wrench baseline{vector(m->baseline_cog.force), vector(m->baseline_cog.torque)};
          if (estimator_->applyCalibration(i, baseline, vector(m->baseline_world_force)))
          {
            calibration_stamps_[i] = stamp;
            calibration_starts_[i] = m->calibration_start_stamp.toSec();
          }
        }));
      preview_reference_.push_back(private_.advertise<bee::BeeCommand>("preview/" + name + "/reference", 1));
      if (control_.distributed_tracking && execute_)
      {
        references_.push_back(nh_.advertise<bee::BeeCommand>(root + "/command", 1));
        calibration_requests_.push_back(nh_.advertise<std_msgs::Empty>(root + "/controller/calibrate_observer", 1));
        subscribers_.push_back(nh_.subscribe<bee::ControllerStatus>(root + "/controller/tracking_status", 1,
            [this, i](const bee::ControllerStatus::ConstPtr& m) {
          tracker_states_[i] = *m; tracker_receipts_[i] = wallNow();
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
    smach_structure_ = private_.advertise<smach_msgs::SmachContainerStructure>("smach/container_structure", 10, true);
    smach_status_ = private_.advertise<smach_msgs::SmachContainerStatus>("smach/container_status", 10, true);
    force_markers_ = private_.advertise<visualization_msgs::MarkerArray>("force_markers", 1);
    publishStructure();
    smach_timer_ = private_.createWallTimer(ros::WallDuration(1), [this](const ros::WallTimerEvent&) {
      if (smach_structure_.getNumSubscribers()) publishStructure();
      reportPhase(true);
    });
    phase_pub_ = private_.advertise<std_msgs::String>("phase", 1, true);
    reason_pub_ = private_.advertise<std_msgs::String>("reason", 1, true);
    metrics_ = private_.advertise<std_msgs::Float64MultiArray>("metrics", 1);
    geometry_errors_pub_ = private_.advertise<std_msgs::Float64MultiArray>("contact_geometry_errors", 1);
    preload_diagnostics_pub_ = private_.advertise<std_msgs::Float64MultiArray>("preload_diagnostics", 1);
    subscribers_.push_back(private_.subscribe<std_msgs::Empty>("operator_start", 1,
        [this](const std_msgs::Empty::ConstPtr&) {
          std_srvs::Trigger request;
          start(request.request, request.response);
          if (!request.response.success) ROS_WARN_STREAM("Joy grasp start: " << request.response.message);
        }));
    subscribers_.push_back(private_.subscribe<std_msgs::Empty>("emergency_stop", 1,
        [this](const std_msgs::Empty::ConstPtr&) { latchAbort("operator emergency stop", true); }));
    subscribers_.push_back(private_.subscribe<std_msgs::Empty>("operator_realign", 1,
        [this](const std_msgs::Empty::ConstPtr&) {
          if (!aborted_.load() && phases_->requestRealign(ros::Time::now().toSec()))
          {
            controller_->reset();
            last_output_ = {};
            ROS_INFO("Operator requested realignment");
          }
        }));
    start_service_ = private_.advertiseService("start", &GraspNode::start, this);
    transport_service_ = private_.advertiseService("transport", &GraspNode::transport, this);
    abort_service_ = private_.advertiseService("abort", &GraspNode::abort, this);
    timer_ = nh_.createWallTimer(ros::WallDuration(period_), &GraspNode::tick, this);
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
             execute_ ? "command output enabled" : "preview only");
  }
  ~GraspNode()
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
    if (!aborted_.load() && phases_->phase() == Phase::Realign)
    {
      response.success = phases_->requestResume();
      response.message = response.success ? "resume requested" : "waiting for all Bees to realign";
      return true;
    }
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
    std::fill(calibration_stamps_.begin(), calibration_stamps_.end(), ros::Time::now().toSec());
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
    latchAbort("operator abort", true);
    response.success = true;
    return true;
  }
  void latchAbort(const std::string& reason, bool operator_cancel = false)
  {
    std::lock_guard<std::mutex> guard(command_mutex_);
    if (!aborted_.load())
    {
      abort_reason_ = reason;
      aborted_.store(true);
      ROS_ERROR("Cooperative grasp aborted: %s", reason.c_str());
    }
    if (active_.load())
      stopCommands(operator_cancel);
  }
  void stopCommands(bool operator_cancel = false)
  {
    if (!execute_)
      return;
    for (const auto& publisher : references_)
    {
      bee::BeeCommand reference;
      reference.header.stamp = ros::Time::now(); reference.header.frame_id = world_frame_;
      reference.stop = true; reference.operator_cancel = operator_cancel;
      publisher.publish(reference);
    }
    if (control_.distributed_tracking) return;  // The tracker hands control back to local position hold.
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
  struct Stage
  {
    std::string name;
    std::vector<Phase> phases;
  };
  const std::vector<Stage> stages_{
    {"Prepare", {Phase::Staging, Phase::Orient, Phase::Calibrate}},
    {"Grasp", {Phase::Approach, Phase::Preload, Phase::SupportTransfer, Phase::Realign}},
    {"Carry", {Phase::Lift, Phase::Hold, Phase::Transport, Phase::DestinationSettle}},
    {"Place", {Phase::Descend, Phase::Unload, Phase::Release, Phase::Retreat, Phase::Unwind, Phase::Verify}}
  };
  void publishStructure()
  {
    smach_msgs::SmachContainerStructure root;
    root.header.stamp = ros::Time::now();
    root.path = "/BeeGrasp";
    root.children = {"Idle"};
    auto edge = [](auto& graph, const std::string& from, const std::string& to, const std::string& outcome) {
      graph.internal_outcomes.push_back(outcome);
      graph.outcomes_from.push_back(from);
      graph.outcomes_to.push_back(to);
    };
    for (const auto& stage : stages_)
    {
      edge(root, root.children.back(), stage.name, root.children.size() == 1 ? "start" : "done");
      root.children.push_back(stage.name);
      edge(root, stage.name, "Aborted", "abort");
      smach_msgs::SmachContainerStructure sub;
      sub.header = root.header;
      sub.path = root.path + "/" + stage.name;
      sub.container_outcomes = {"done", "abort"};
      std::string previous;
      for (Phase phase : stage.phases)
      {
        const std::string name = phaseName(phase);
        sub.children.push_back(name);
        if (phase == Phase::Realign)
        {
          edge(sub, "Approach", name, "realign");
          edge(sub, "Preload", name, "realign");
          edge(sub, name, "Approach", "resume");
          continue;
        }
        if (!previous.empty()) edge(sub, previous, name, "next");
        previous = name;
      }
      edge(sub, previous, "done", "done");
      smach_structure_.publish(sub);
    }
    edge(root, root.children.back(), "Complete", "done");
    edge(root, "Idle", "Aborted", "abort");
    root.children.insert(root.children.end(), {"Complete", "Aborted"});
    smach_structure_.publish(root);
  }
  void publishSmachStatus(const std::string& path, const std::string& initial, const std::string& active)
  {
    smach_msgs::SmachContainerStatus status;
    status.header.stamp = ros::Time::now();
    status.path = path;
    status.initial_states.push_back(initial);
    if (!active.empty()) status.active_states.push_back(active);
    if (path == "/BeeGrasp") status.info = path;
    status.local_data = "gAJ9cQAu";
    smach_status_.publish(status);
  }
  void reportPhase(bool refresh)
  {
    const Phase phase = phases_->phase();
    const std::string name = phaseName(phase);
    const bool changed = last_smach_phase_ != name;
    if (!refresh && !changed) return;
    std::string group = name;
    for (const auto& stage : stages_)
    {
      const bool active = std::find(stage.phases.begin(), stage.phases.end(), phase) != stage.phases.end();
      if (active) group = stage.name;
      publishSmachStatus("/BeeGrasp/" + stage.name, phaseName(stage.phases.front()), active ? name : "");
    }
    publishSmachStatus("/BeeGrasp", "Idle", group);
    const std::string label = group == name ? name : group + "/" + name;
    const double now = wallNow();
    if (changed || (active_.load() && now - last_phase_log_ >= 5))
    {
      const auto& reason = phase == Phase::Aborted ? phases_->fault() : phases_->advisory();
      ROS_INFO_STREAM("[BeeGrasp] " << label << (reason.empty() ? "" : " | " + reason));
      last_phase_log_ = now;
    }
    last_smach_phase_ = name;
  }
  void publishPhase()
  {
    std_msgs::String msg;
    msg.data = phaseName(phases_->phase());
    phase_pub_.publish(msg);
    reportPhase(false);
    msg.data = phases_->phase() == Phase::Aborted ? phases_->fault() : phases_->advisory();
    if (!msg.data.empty() && phases_->phase() != Phase::Aborted)
      ROS_WARN_STREAM_THROTTLE(3, msg.data);
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
      ControlStatus phase_status = last_output_.status;
      if (execute_ && control_.distributed_tracking && previous == Phase::Calibrate)
      {
        phase_status.calibration_ready = calibration_requested_ > 0;
        for (double started : calibration_starts_)
          phase_status.calibration_ready = phase_status.calibration_ready && started >= calibration_requested_;
        if (!phase_status.calibration_ready && wallNow() - calibration_request_wall_ > std::max(10.0, 5 * task_.baseline_duration))
        { latchAbort("observer calibration did not complete on all Bees"); return; }
      }
      Reference reference = phases_->step(state, phase_status, now);
      if(!reference.preload_diagnostics.empty())
      {
        std_msgs::Float64MultiArray diagnostic;
        diagnostic.data=reference.preload_diagnostics;
        preload_diagnostics_pub_.publish(diagnostic);
      }
      if (reference.phase == Phase::Aborted)
      {
        latchAbort(phases_->fault());
        return;
      }
      if (execute_ && control_.distributed_tracking)
      {
        if (reference.calibrate && previous != Phase::Calibrate)
        {
          calibration_requested_ = now; calibration_request_wall_ = wallNow();
          for (const auto& publisher : calibration_requests_) publisher.publish(std_msgs::Empty{});
        }
      }
      else
      {
        if (reference.calibrate && previous != Phase::Calibrate) estimator_->beginCalibration();
        if (reference.calibrate) estimator_->addCalibrationSample(input_);
        if (previous == Phase::Calibrate && !reference.calibrate && !estimator_->finishCalibration())
        { latchAbort("empty observer calibration"); return; }
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
          publishCommands(output, state, dt);
        heartbeat_.store(wallNow());
      }
      last_output_ = output;
      publishPhase();
      std_msgs::Float64MultiArray metrics;
      metrics.data = {
        output.status.normal_force,          output.status.reference_z,   output.status.reference_vz,
        output.status.reference_az,          output.status.z_integral,    output.status.vertical_force_target,
        output.status.common_z_acceleration, output.status.support_ratio, output.status.common_slip,
        output.status.wrench_force_error, output.status.wrench_moment_error, output.status.allocation_violation,
        double(output.status.allocation_iterations), output.status.allocation_seconds, output.status.allocation_primal,
        output.status.allocation_stationarity, output.status.allocation_complementarity,
        double(output.status.allocation_restarts)
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
  void publishCommands(const ControlOutput& output, const GraspState& state, double dt)
  {
    visualization_msgs::MarkerArray markers;
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
      external_filters_[i] += dt / (control_.support_filter_tau + dt) *
                             (state.contacts[i].bee_world.force - external_filters_[i]);
      const auto generic = makeBeeCommand(c, state.input.vehicles[i], state.contacts[i], control_,
                                              msg, attitude.header, external_filters_[i], legacy_velocity_gain_xy_, legacy_velocity_gain_z_);
      preview_reference_[i].publish(generic);
      if (force_markers_.getNumSubscribers())
      {
        const auto& contact = state.contacts[i];
        const Vec3 target = generic.enable_force_control ? vector(generic.force) : Vec3::Zero();
        markers.markers.push_back(forceArrow(generic.header, "command", i, contact.position, target, false));
        markers.markers.push_back(forceArrow(generic.header, "observer", i, contact.position, -contact.bee_world.force, true));
      }
      if (execute_ && control_.distributed_tracking) references_[i].publish(generic);
      if (execute_ && !control_.distributed_tracking)
      {
        nav_[i].publish(msg);
        attitude_[i].publish(attitude);
      }
    }
    if (!markers.markers.empty()) force_markers_.publish(markers);
  }
  std::string last_smach_phase_;
  double last_phase_log_ = 0;
  ros::WallTimer smach_timer_;
  ros::Publisher smach_status_, smach_structure_, force_markers_;
  ros::NodeHandle nh_, private_;
  SceneConfig scene_;
  ControlConfig control_;
  TaskConfig task_;
  SafetyConfig safety_config_;
  std::unique_ptr<GraspStateEstimator> estimator_;
  std::unique_ptr<GraspController> controller_;
  std::unique_ptr<GraspPhaseManager> phases_;
  std::unique_ptr<GraspSafetyMonitor> safety_;
  InputState input_;
  ControlOutput last_output_;
  std::vector<Receipt> receipts_;
  std::vector<std::string> cog_frames_;
  std::vector<ros::Subscriber> subscribers_;
  std::vector<ros::Publisher> nav_, attitude_, preview_, preview_attitude_;
  std::vector<ros::Publisher> references_, preview_reference_;
  std::vector<bee::ControllerStatus> tracker_states_;
  std::vector<ros::Publisher> calibration_requests_;
  std::vector<double> calibration_starts_;
  std::vector<Vec3> external_filters_;
  double calibration_requested_ = 0, calibration_request_wall_ = 0;
  double legacy_velocity_gain_xy_ = 3, legacy_velocity_gain_z_ = 3;
  std::vector<double> tracker_receipts_;
  std::vector<double> calibration_stamps_;
  ros::Publisher phase_pub_, reason_pub_, metrics_, geometry_errors_pub_, preload_diagnostics_pub_;
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
    bee::grasp::GraspNode node;
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("Cooperative grasp: %s", e.what());
    return 1;
  }
  return 0;
}
