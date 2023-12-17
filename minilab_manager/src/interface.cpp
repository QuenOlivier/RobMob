#include <minilab_manager/interface.hpp>

#include <iostream>
#include <string>

using std_srvs::Empty;
using std_srvs::Trigger;

namespace minilab_manager
{


MinilabManagerInterface::MinilabManagerInterface(ros::NodeHandle *nh) :
nh_(*nh),
map_received_(false),
last_trajectory_msg_(ros::Time::now()),
current_state_(State::MANUAL)
{
  // sub_map_;
  // sub_trajectory_;
  auto_request_server_ = nh_.advertiseService("/system/auto_mode_request", &MinilabManagerInterface::onAutoRequest, this);
  manual_request_server_ = nh_.advertiseService("/system/manual_mode_request", &MinilabManagerInterface::onManualRequest, this);

  pub_command_state_ = nh_.advertise<robmob_msgs::CommandStatus>("/system/state", 1);
}

bool MinilabManagerInterface::onAutoRequest(Trigger::Request &req, Trigger::Response &res){
  bool map_ok = isMapOk();
  bool traj_ok = isTrajectoryOk();
  if( current_state_ != State::AUTO && map_ok && traj_ok){
    updateState(State::AUTO);
    res.success = true;
  } else if (current_state_ == State::AUTO) {
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
  updateState(State::MANUAL);
}

bool MinilabManagerInterface::isMapOk(){
  return map_received_;
}

bool MinilabManagerInterface::isTrajectoryOk(){
  return (last_trajectory_msg_ - ros::Time::now).toSec() < 0.2;
}

void MinilabManagerInterface::updateState(State state){
  current_state_ = state;
}

void MinilabManagerInterface::update(){
  robmob_msgs::CommandStatus msg;
  msg.command_status = current_state_;
  pub_command_state_.publish(msg);
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
