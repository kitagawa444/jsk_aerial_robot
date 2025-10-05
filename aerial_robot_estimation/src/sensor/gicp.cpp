// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Dragon Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <aerial_robot_estimation/sensor/gicp.h>

#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace sensor_plugin
{

  GlobalICP::GlobalICP() = default;
  GlobalICP::~GlobalICP() = default;

  void GlobalICP::initialize(ros::NodeHandle nh,
                             boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                             boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                             std::string sensor_name, int index)
  {
    SensorBase::initialize(nh, robot_model, estimator, sensor_name, index);

    // パラメータ（Base の getParam を利用）
    getParam<double>("localization_freq", localization_freq_, 0.5);
    getParam<bool>("oneshot", oneshot_, false);

    // 初期推定値
    std::vector<double> pos{0,0,0}, rpy{0,0,0};
    nhp_.param("initial_guess/pos", pos, pos);
    nhp_.param("initial_guess/rpy", rpy, rpy);

    tf::Transform init_tf;
    init_tf.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));
    tf::Matrix3x3 R;
    R.setRPY(rpy[0], rpy[1], rpy[2]);
    init_tf.setBasis(R);
    tfToEigen(init_tf, T_map_to_odom_);

    // registration フェーズ
    loadPhase("init",  0.4, 0.1, 1.0, 0.60, 15.0, 8.0);
    loadPhase("float", 0.3, 0.1, 0.6, 0.75, 12.0, 6.0);
    loadPhase("fix",   0.2, 0.1, 0.3, 0.85, 10.0, 5.0);

    // pubs
    pub_pc_in_map_   = nh_.advertise<sensor_msgs::PointCloud2>("cur_scan_in_map", 1);
    pub_submap_      = nh_.advertise<sensor_msgs::PointCloud2>("submap", 1);
    pub_map_to_odom_ = nh_.advertise<nav_msgs::Odometry>("map_to_odom", 1);

    // subs
    sub_scan_ = nh_.subscribe("cloud_registered", 1, &GlobalICP::cbScan, this);
    sub_odom_ = nh_.subscribe("Odometry", 1, &GlobalICP::cbOdom, this);

    // Global map の取得
    ROS_WARN("[%s] waiting for global map...", indexed_nhp_.getNamespace().c_str());
    if (auto msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("threeD_map", nh_)) {
      initGlobalMap(*msg);
      setStatus(Status::INIT);
      ROS_INFO("[%s] global map loaded", indexed_nhp_.getNamespace().c_str());
    } else {
      setStatus(Status::INVALID);
      ROS_ERROR("[%s] failed to get global map", indexed_nhp_.getNamespace().c_str());
    }
   

    // タイマ
    if (oneshot_) {
      timer_ = nh_.createTimer(ros::Duration(0.01), &GlobalICP::timerOnce, this, true, true);
    } else {
      const double hz = std::max(0.01, localization_freq_);
      timer_ = nh_.createTimer(ros::Duration(1.0 / hz), &GlobalICP::timerCB, this);
    }
  }

  /*** helpers ***/
  void GlobalICP::tfToEigen(const tf::Transform& t, Eigen::Matrix4f& out)
  {
    out.setIdentity();
    tf::Matrix3x3 R = t.getBasis();
    for (int r=0; r<3; ++r)
      for (int c=0; c<3; ++c)
        out(r,c) = R[r][c];
    out(0,3) = t.getOrigin().x();
    out(1,3) = t.getOrigin().y();
    out(2,3) = t.getOrigin().z();
  }

  tf::Transform GlobalICP::eigenToTf(const Eigen::Matrix4f& T)
  {
    tf::Matrix3x3 R;
    for (int r=0; r<3; ++r)
      for (int c=0; c<3; ++c)
        R[r][c] = T(r,c);
    tf::Vector3 p(T(0,3), T(1,3), T(2,3));
    return tf::Transform(R, p);
  }

  void GlobalICP::loadPhase(const std::string& name,
                            double map_voxel, double scan_voxel, double max_corr,
                            double thresh, double map_range, double scan_range)
  {
    PhaseParam p;
    nhp_.param(("registration/" + name + "/map_voxel_size").c_str(),        p.map_voxel, map_voxel);
    nhp_.param(("registration/" + name + "/scan_voxel_size").c_str(),       p.scan_voxel, scan_voxel);
    nhp_.param(("registration/" + name + "/max_corres_dist").c_str(),       p.max_corr, max_corr);
    nhp_.param(("registration/" + name + "/localization_thresh").c_str(),   p.thresh, thresh);
    nhp_.param(("registration/" + name + "/map_range").c_str(),             p.map_range, map_range);
    nhp_.param(("registration/" + name + "/scan_range").c_str(),            p.scan_range, scan_range);
    phase_[name] = p;
  }

  void GlobalICP::initGlobalMap(const sensor_msgs::PointCloud2& msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msg, *raw);

    for (auto& kv : phase_) {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setLeafSize(kv.second.map_voxel, kv.second.map_voxel, kv.second.map_voxel);
      vg.setInputCloud(raw);
      pcl::PointCloud<pcl::PointXYZ>::Ptr d(new pcl::PointCloud<pcl::PointXYZ>);
      vg.filter(*d);
      global_map_[kv.first] = d;
    }
  }

  void GlobalICP::cbOdom(const nav_msgs::Odometry::ConstPtr& od)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cur_odom_ = od;
    updateHealthStamp(0);
  }

  void GlobalICP::cbScan(const sensor_msgs::PointCloud2::ConstPtr& cloud)
  {
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::fromROSMsg(*cloud, tmp);
    std::lock_guard<std::mutex> lk(mtx_);
    *cur_scan_ = tmp;
    have_new_scan_ = true;

    pub_pc_in_map_.publish(*cloud);
    updateHealthStamp(0);
  }

  void GlobalICP::timerOnce(const ros::TimerEvent&) { doLocalization(); }
  void GlobalICP::timerCB(const ros::TimerEvent&)   { doLocalization(); }

  bool GlobalICP::ready()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return (cur_odom_ && have_new_scan_);
  }

  void GlobalICP::doLocalization()
  {
    if (!ready()) return;

    // BaseLink->Sensor TF を（必要なら）更新
    updateBaseLink2SensorTransform();

    nav_msgs::Odometry::ConstPtr od;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
    {
      std::lock_guard<std::mutex> lk(mtx_);
      od.reset(new nav_msgs::Odometry(*cur_odom_));
      *scan = *cur_scan_;
      have_new_scan_ = false;
    }

    const std::string phase_name = (phase_idx_==0)? "init" : (phase_idx_==1? "float" : "fix");
    const PhaseParam& P = phase_.at(phase_name);

    auto map_crop  = cropMapAroundBase(P, od);
    auto scan_crop = cropScanAroundBase(P, od, scan);
    if (map_crop->empty() || scan_crop->empty()) {
      ROS_WARN_THROTTLE(1.0, "[%s] empty crop: map=%zu, scan=%zu",
                        indexed_nhp_.getNamespace().c_str(),
                        map_crop->size(), scan_crop->size());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ds(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ds(new pcl::PointCloud<pcl::PointXYZ>);
    voxel(map_crop, P.map_voxel, map_ds);
    voxel(scan_crop, P.scan_voxel, scan_ds);

    Eigen::Matrix4f init = T_map_to_odom_;
    Eigen::Matrix4f T_est;
    double fitness;
    icpAlign(scan_ds, map_ds, P.max_corr, init, T_est, fitness);

    ROS_INFO("[%s] phase=%s fitness=%.3f",
             indexed_nhp_.getNamespace().c_str(), phase_name.c_str(), fitness);

    if (fitness < P.thresh) {
      ROS_WARN("[%s] not converged in phase %s (%.3f < %.3f)",
               indexed_nhp_.getNamespace().c_str(), phase_name.c_str(), fitness, P.thresh);
      phase_idx_ = std::max(0, phase_idx_-1);
      converge_cnt_ = 0;
      return;
    }

    if (phase_idx_ == 0) {
      phase_idx_ = 1;
    } else if (phase_idx_ == 1) {
      const double fix_thresh = phase_.at("fix").thresh;
      if (fitness > fix_thresh) {
        if (++converge_cnt_ > 5) {
          phase_idx_ = 2;
          ROS_INFO("[%s] shift to FIX", indexed_nhp_.getNamespace().c_str());
        }
      } else {
        converge_cnt_ = 0;
      }
    }
    T_map_to_odom_ = T_est;

    publishOdom(od->header.stamp, T_map_to_odom_);
    pushGroundTruthToEstimator(T_map_to_odom_, *od);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr
  GlobalICP::cropMapAroundBase(const PhaseParam& P, const nav_msgs::Odometry::ConstPtr& od)
  {
    tf::Vector3 p_b(od->pose.pose.position.x, od->pose.pose.position.y, od->pose.pose.position.z);
    tf::Quaternion q_b(od->pose.pose.orientation.x, od->pose.pose.orientation.y,
                       od->pose.pose.orientation.z, od->pose.pose.orientation.w);
    tf::Transform T_ob(q_b, p_b);

    tf::Transform T_mo = eigenToTf(T_map_to_odom_);
    tf::Transform T_mb = T_mo * T_ob;
    const tf::Vector3 ref = T_mb.getOrigin();

    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
    const auto& gm = *global_map_.at((phase_idx_==0)? "init" : (phase_idx_==1? "float" : "fix"));
    const double r2 = P.map_range * P.map_range;
    out->reserve(gm.size());
    for (const auto& pt : gm) {
      const double dx = pt.x - ref.x();
      const double dy = pt.y - ref.y();
      const double dz = pt.z - ref.z();
      if (dx*dx + dy*dy + dz*dz < r2) out->push_back(pt);
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*out, msg);
    msg.header = od->header;
    msg.header.frame_id = "world";
    pub_submap_.publish(msg);
    return out;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr
  GlobalICP::cropScanAroundBase(const PhaseParam& P,
                                const nav_msgs::Odometry::ConstPtr& od,
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan)
  {
    tf::Vector3 ref(od->pose.pose.position.x, od->pose.pose.position.y, od->pose.pose.position.z);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
    const double r2 = P.scan_range * P.scan_range;
    out->reserve(scan->size());
    for (const auto& pt : *scan) {
      const double dx = pt.x - ref.x();
      const double dy = pt.y - ref.y();
      const double dz = pt.z - ref.z();
      if (dx*dx + dy*dy + dz*dz < r2) out->push_back(pt);
    }
    return out;
  }

  void GlobalICP::voxel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in, double leaf,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& out)
  {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(leaf, leaf, leaf);
    vg.setInputCloud(in);
    vg.filter(*out);
  }

  void GlobalICP::icpAlign(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src,
                           const pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt,
                           double max_corr_dist,
                           const Eigen::Matrix4f& init,
                           Eigen::Matrix4f& T_out,
                           double& fitness_out)
  {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.setMaxCorrespondenceDistance(max_corr_dist);
    icp.setMaximumIterations(100);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, init);

    T_out = icp.getFinalTransformation();
    // PCL の FitnessScore は平均対応点距離に比例するので、簡易に 1/(1+e) 的な正規化でもOK
    const double e = icp.getFitnessScore();
    fitness_out = 1.0 / (1.0 + e);
  }

  void GlobalICP::publishOdom(const ros::Time& stamp, const Eigen::Matrix4f& Tmo)
  {
    nav_msgs::Odometry od;
    od.header.stamp = stamp;
    od.header.frame_id = "map";

    tf::Transform tfmo = eigenToTf(Tmo);
    od.pose.pose.position.x = tfmo.getOrigin().x();
    od.pose.pose.position.y = tfmo.getOrigin().y();
    od.pose.pose.position.z = tfmo.getOrigin().z();
    tf::Quaternion q;
    tfmo.getBasis().getRotation(q);
    od.pose.pose.orientation.x = q.x();
    od.pose.pose.orientation.y = q.y();
    od.pose.pose.orientation.z = q.z();
    od.pose.pose.orientation.w = q.w();
    pub_map_to_odom_.publish(od);
  }

  void GlobalICP::pushGroundTruthToEstimator(const Eigen::Matrix4f& Tmo,
                                             const nav_msgs::Odometry& cur_odom)
  {
    if (!estimator_) return;

    // ^wT_b = ^wT_o * ^oT_b  （ここでは w=map, o=odom）
    tf::Transform tfmo = eigenToTf(Tmo);
    tf::Transform tfob(tf::Quaternion(cur_odom.pose.pose.orientation.x,
                                      cur_odom.pose.pose.orientation.y,
                                      cur_odom.pose.pose.orientation.z,
                                      cur_odom.pose.pose.orientation.w),
                       tf::Vector3(cur_odom.pose.pose.position.x,
                                   cur_odom.pose.pose.position.y,
                                   cur_odom.pose.pose.position.z));
    tf::Transform tfmb = tfmo * tfob;

    tf::Vector3 p = tfmb.getOrigin();
    estimator_->setPos(Frame::BASELINK, aerial_robot_estimation::GROUND_TRUTH, p);

    tf::Matrix3x3 R = tfmb.getBasis();
    estimator_->setOrientation(Frame::BASELINK, aerial_robot_estimation::GROUND_TRUTH, R);
    estimator_->setStateStatus(State::X_BASE, aerial_robot_estimation::GROUND_TRUTH, true);
    estimator_->setStateStatus(State::Y_BASE, aerial_robot_estimation::GROUND_TRUTH, true);
    estimator_->setStateStatus(State::Z_BASE, aerial_robot_estimation::GROUND_TRUTH, true);
    estimator_->setStateStatus(State::Base::Rot, aerial_robot_estimation::GROUND_TRUTH, true);
    setStatus(Status::ACTIVE);
  }

} // namespace sensor_plugin

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::GlobalICP, sensor_plugin::SensorBase)
