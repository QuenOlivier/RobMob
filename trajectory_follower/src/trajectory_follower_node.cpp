#include <minilab_manager/interface.hpp>

#include <iostream>
#include <string>

using std_srvs::Empty;
using std_srvs::Trigger;

namespace minilab_manager
{


TrajectoryFollower::TrajectoryFollower(ros::NodeHandle *nh) :
nh_(*nh)
{
  sub_odom_ = nh_.subscribe("/odom", 1, &TrajectoryFollower::onOdom, this);
  sub_command_status_ = nh_.subscribe("/system/state", 1, &TrajectoryFollower::onCommandStatus, this);
  sub_manual_cmd_ = nh_.subscribe("/teleoperation/cmd_vel", 1, &TrajectoryFollower::onManualCommand, this);

  pub_cmd_ = nh_.advertise<Twist>("/cmd_vel", 1);

  last_manual_msg_stamp_ = ros::Time::now();
  last_manual_cmd_.linear.x = 0.0;
  last_manual_cmd_.angular.z = 0.0;

  current_state_.command_status = CommandStatus::MANUAL;
}

void TrajectoryFollower::onManualCommand(Twist &msg){
  last_manual_cmd_ = msg;
  last_manual_msg_stamp_ = ros::Time::now();
}

void TrajectoryFollower::onCommandStatus(CommandStatus &msg){
  current_state_ = msg;
}

void TrajectoryFollower::onOdom(Odometry &msg){
  last_robot_odometry_ = msg;
}

void TrajectoryFollower::update(){
  if( current_state_.command_status == CommandStatus::MANUAL)
  {
    if ( (last_manual_msg_stamp_ - ros::Time::now).toSec > 0.2 )
    {
      last_manual_cmd_.linear.x = 0.0;
      last_manual_cmd_.angular.z = 0.0;
    }
    pub_cmd_.publish(last_manual_cmd_);
  }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_follower");
  ros::NodeHandle* node = new ros::NodeHandle();

  trajectory::TrajectoryFollower interface(node);

  ros::Rate rate(10);
  while (ros::ok())
  {
    interface.update();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}