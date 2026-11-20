// Test-only host: observes returned commands on private test topics. No actuators or nav publisher.
#include <bee/control/bee_controller.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "reference_tracker_harness");
  ros::NodeHandle nh, parameters("~");
  // Explicit synthetic model mass; production passes the mass derived from robot_description.
  bee::control::ReferenceTracker mode(nh, parameters, 1.84199, false);
  auto nav_pub = parameters.advertise<aerial_robot_msgs::FlightNav>("observed_nav", 1);
  auto attitude_pub = parameters.advertise<geometry_msgs::Vector3Stamped>("observed_attitude", 1);
  auto timer = nh.createWallTimer(ros::WallDuration(.01), [&](const ros::WallTimerEvent&) {
    aerial_robot_msgs::FlightNav nav;
    geometry_msgs::Vector3Stamped attitude;
    bool release;
    if (mode.update(nav, attitude, release, true))
    {
      nav_pub.publish(nav); attitude_pub.publish(attitude);
    }
  });
  ros::spin();
  return 0;
}
