#include <minilab_manager/interface.hpp>

#include <iostream>
#include <string>


namespace minilab_manager
{


MinilabManagerInterface::MinilabManagerInterface(ros::NodeHandle *nh) :
nh_(*nh),
map_received_(false),
last_trajectory_msg_(ros::Time::now())
{
  // sub_map_;
  // sub_trajectory_;
  auto_request_server_ = nh_.advertiseService("/system/auto_mode_request", &MinilabManagerInterface::onAutoRequest, this);
  manual_request_server_ = nh_.advertiseService("/system/manual_mode_request", &MinilabManagerInterface::onManualRequest, this);

  pub_command_state_ = nh_.advertise<robmob_msgs::CommandStatus>("/system/state", 1);

  current_state_.command_status = CommandStatus::MANUAL;
  last_trajectory_msg_ = ros::Time::now();
}

bool MinilabManagerInterface::onAutoRequest(Trigger::Request &req, Trigger::Response &res){
  bool map_ok = isMapOk();
  bool traj_ok = isTrajectoryOk();
  if( current_state_.command_status != CommandStatus::AUTO && map_ok && traj_ok){
    updateState(CommandStatus::AUTO);
    res.success = true;
  } else if (current_state_.command_status == CommandStatus::AUTO) {
    res.success = false;
    res.message = "The State is already in AUTO mode";
  } else {
    res.success = false;
    std::string map_state = map_ok ? "ok" : "not ready";
    std::string traj_state = traj_ok ? "ok" : "not ready";
    res.message = "One or more modules are not ready (map: " + map_state +  ", trajectory: " + traj_state + ")";
  }

  return res.success;
}

bool MinilabManagerInterface::onManualRequest(Empty::Request &req, Empty::Response &res){
  updateState(CommandStatus::MANUAL);
}

bool MinilabManagerInterface::isMapOk(){
  return map_received_;
}

bool MinilabManagerInterface::isTrajectoryOk(){
  return (last_trajectory_msg_ - ros::Time::now).toSec() < 0.2;
}

void MinilabManagerInterface::updateState(uint8 state){
  current_state_ = state;
}

void MinilabManagerInterface::update(){
  pub_command_state_.publish(current_state_);
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "minilab_manager");
  ros::NodeHandle* node = new ros::NodeHandle();

  minilab_manager::MinilabManagerInterface interface(node);

  ros::Rate rate(10);
  while (ros::ok())
  {
    interface.update();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
