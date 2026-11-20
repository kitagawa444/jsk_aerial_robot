#include <aerial_robot_control/util/joy_parser.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <algorithm>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bee_grasp_joy");
  ros::NodeHandle nh;
  auto start = nh.advertise<std_msgs::Empty>("/bee_cooperative_grasp/operator_start", 1);
  auto stop = nh.advertise<std_msgs::Empty>("/bee_cooperative_grasp/emergency_stop", 1);
  bool start_armed = false;
  auto input = nh.subscribe<sensor_msgs::Joy>("joy", 10, [&](const sensor_msgs::Joy::ConstPtr& msg) {
    const auto joy = joyParse(*msg);
    if (joy.buttons.size() != JOY_BUTTON_SIZE) { start_armed = false; return; }
    const auto& buttons = joy.buttons;
    if (buttons[JOY_BUTTON_REAR_LEFT_2] && buttons[JOY_BUTTON_REAR_RIGHT_2])
    {
      start_armed = false;
      stop.publish(std_msgs::Empty{});
      return;
    }
    if (!buttons[JOY_BUTTON_REAR_RIGHT_1] && !buttons[JOY_BUTTON_REAR_LEFT_1])
      start_armed = true;
    if (buttons[JOY_BUTTON_REAR_RIGHT_1] && buttons[JOY_BUTTON_REAR_LEFT_1])
    {
      const double age = (ros::Time::now() - msg->header.stamp).toSec();
      if (start_armed && !msg->header.stamp.isZero() && age >= -.05 && age <= .5 &&
          std::count(buttons.begin(), buttons.end(), 1) == 2)
        start.publish(std_msgs::Empty{});
      start_armed = false;
    }
  });
  ros::spin();
}
