#pragma once

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <robmob_msgs/CommandStatus.h>


using std_srvs::Empty;
using std_srvs::Trigger;
using robmob_msgs::CommandStatus;

namespace minilab_manager
{

class MinilabManagerInterface {

public:
  MinilabManagerInterface(ros::NodeHandle *nh);
  virtual ~MinilabManagerInterface();

  void update();

private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_map_;
  ros::Subscriber sub_trajectory_;
  ros::ServiceServer auto_request_server_;
  ros::ServiceServer manual_request_server_;

  ros::Publisher pub_command_state_;

  ros::Time last_trajectory_msg_;
  bool map_received_;
  CommandStatus current_state_;

  bool onAutoRequest(Trigger::Request &req, Trigger::Response &res);
  bool onManualRequest(Empty::Request &req, Empty::Response &res);
  bool isMapOk();
  bool isTrajectoryOk();
  void updateState(uint state);
};

}
