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
  auto realign = nh.advertise<std_msgs::Empty>("/bee_cooperative_grasp/operator_realign", 1);
  bool start_armed = false, realign_armed = false;
  auto input = nh.subscribe<sensor_msgs::Joy>("joy", 10, [&](const sensor_msgs::Joy::ConstPtr& msg) {
    const auto joy = joyParse(*msg);
    if (joy.buttons.size() != JOY_BUTTON_SIZE) { start_armed = realign_armed = false; return; }
    const auto& buttons = joy.buttons;
    if (buttons[JOY_BUTTON_REAR_LEFT_2] && buttons[JOY_BUTTON_REAR_RIGHT_2])
    {
      start_armed = realign_armed = false;
      stop.publish(std_msgs::Empty{});
      return;
    }
    const double age = (ros::Time::now() - msg->header.stamp).toSec();
    const bool fresh_pair = !msg->header.stamp.isZero() && age >= -.05 && age <= .5 &&
                            std::count(buttons.begin(), buttons.end(), 1) == 2;
    if (!buttons[JOY_BUTTON_REAR_RIGHT_1] && !buttons[JOY_BUTTON_ACTION_SQUARE]) realign_armed = true;
    if (buttons[JOY_BUTTON_REAR_RIGHT_1] && buttons[JOY_BUTTON_ACTION_SQUARE])
    {
      if (realign_armed && fresh_pair) realign.publish(std_msgs::Empty{});
      realign_armed = start_armed = false;
      return;
    }
    if (!buttons[JOY_BUTTON_REAR_RIGHT_1] && !buttons[JOY_BUTTON_REAR_LEFT_1])
      start_armed = true;
    if (buttons[JOY_BUTTON_REAR_RIGHT_1] && buttons[JOY_BUTTON_REAR_LEFT_1])
    {
      if (start_armed && fresh_pair)
        start.publish(std_msgs::Empty{});
      start_armed = false;
    }
  });
  ros::spin();
}
