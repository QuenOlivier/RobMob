#pragma once

#include <ros/ros.h>
#include <robmob_msgs/CommandStatus.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

using geometry_msgs::Twist;
using robmob_msgs::CommandStatus;
using nav_msgs::Odometry;

namespace trajectory
{

class TrajectoryFollower {

public:
  TrajectoryFollower(ros::NodeHandle *nh);
  ~TrajectoryFollower(){};

  void update();

private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_odom_;
  ros::Subscriber sub_trajectory_;
  ros::Subscriber sub_command_status_;
  ros::Subscriber sub_manual_cmd_;

  ros::Publisher pub_cmd_;

  ros::Time last_manual_msg_stamp_;
  Twist last_manual_cmd_;
  CommandStatus current_state_;
  Odometry last_robot_odometry_;

  void onCommandStatus(CommandStatus &msg);
  void onManualCommand(Twist &msg);
  void onOdom(Odometry &msg);
};

}
