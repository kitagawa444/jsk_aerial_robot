#pragma once
#include <aerial_robot_msgs/FlightNav.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <memory>

namespace bee::grasp
{
// Owned by BeeController. The reference loop queues commands; only the existing
// flight-control loop applies them to navigation. No self-published FlightNav.
class GraspMode
{
public:
  GraspMode(ros::NodeHandle nh, ros::NodeHandle parameters, bool enabled);
  ~GraspMode();
  bool takeCommand(aerial_robot_msgs::FlightNav& nav, geometry_msgs::Vector3Stamped& attitude,
                   bool& release, bool available);
private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
