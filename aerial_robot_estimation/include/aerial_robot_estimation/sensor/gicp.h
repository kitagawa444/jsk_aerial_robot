// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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

#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mutex>
#include <map>
#include <string>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <aerial_robot_estimation/sensor/base_plugin.h>

namespace sensor_plugin
{
  class GlobalICP : public sensor_plugin::SensorBase
  {
  public:
    GlobalICP();
    ~GlobalICP() override;

   virtual void initialize(ros::NodeHandle nh,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    std::string sensor_name, int index) override;

  protected:
    void estimateProcess() override {}  // 使わない

  private:
    struct PhaseParam {
      double map_voxel, scan_voxel, max_corr, thresh, map_range, scan_range;
    };

    // helpers
    static void tfToEigen(const tf::Transform& t, Eigen::Matrix4f& out);
    static tf::Transform eigenToTf(const Eigen::Matrix4f& T);
    void loadPhase(const std::string& name,
                   double map_voxel, double scan_voxel, double max_corr,
                   double thresh, double map_range, double scan_range);
    void initGlobalMap(const sensor_msgs::PointCloud2& msg);

    void cbOdom(const nav_msgs::Odometry::ConstPtr& od);
    void cbScan(const sensor_msgs::PointCloud2::ConstPtr& cloud);

    void timerOnce(const ros::TimerEvent&);
    void timerCB(const ros::TimerEvent&);
    bool ready();
    void doLocalization();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropMapAroundBase(const PhaseParam& P, const nav_msgs::Odometry::ConstPtr& od);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropScanAroundBase(const PhaseParam& P,
                                                           const nav_msgs::Odometry::ConstPtr& od,
                                                           const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan);

    static void voxel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in, double leaf,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& out);

    static void icpAlign(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt,
                         double max_corr_dist,
                         const Eigen::Matrix4f& init,
                         Eigen::Matrix4f& T_out,
                         double& fitness_out);

    void publishOdom(const ros::Time& stamp, const Eigen::Matrix4f& Tmo);
    void pushGroundTruthToEstimator(const Eigen::Matrix4f& Tmo,
                                    const nav_msgs::Odometry& cur_odom);

  private:
    // pubs/subs/timer（NodeHandle は Base の nh_ / nhp_ / indexed_nhp_ を使用）
    ros::Subscriber sub_scan_, sub_odom_;
    ros::Publisher  pub_pc_in_map_, pub_submap_, pub_map_to_odom_;
    tf2_ros::TransformBroadcaster tfbr_;
    ros::Timer timer_;

    // params
    std::map<std::string, PhaseParam> phase_;
    int  phase_idx_ = 0;  // 0:init, 1:float, 2:fix
    int  converge_cnt_ = 0;

    std::string target_frame_;
    bool oneshot_{false};
    double localization_freq_{0.5};

    // data
    std::mutex mtx_;
    nav_msgs::Odometry::ConstPtr cur_odom_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_scan_{new pcl::PointCloud<pcl::PointXYZ>};
    bool have_new_scan_{false};

    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> global_map_;
    Eigen::Matrix4f T_map_to_odom_ = Eigen::Matrix4f::Identity();
  };

} // namespace sensor_plugin

