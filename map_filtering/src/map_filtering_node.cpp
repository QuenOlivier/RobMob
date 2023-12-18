#include <map_filtering/map_filtering_node.hpp>

#include <iostream>
#include <string>


MapFilteringNode::MapFilteringNode(ros::NodeHandle *nh) :
nh_(*nh)
{
  sub_map_ = nh_.subscribe("/map/raw_map", 1, &MapFilteringNode::onOccupancyGrid, this);

  pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map/filtered_map", 1);

  client = nh_.serviceClient<nav_msgs::GetMap>("/dynamic_map");
}

void MapFilteringNode::onOccupancyGrid(const nav_msgs::OccupancyGrid::SharedPtr &msg){
  last_map_ = msg;
}

void MapFilteringNode::update(){
  if( current_state_.command_status == CommandStatus::MANUAL)
  {
    if ( (last_manual_msg_stamp_ - ros::Time::now() ).toSec() > 0.2 )
    {
      last_manual_cmd_.linear.x = 0.0;
      last_manual_cmd_.angular.z = 0.0;
    }
    pub_cmd_.publish(last_manual_cmd_);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_follower");
  ros::NodeHandle* node = new ros::NodeHandle();

  MapFilteringNode interface(node);

  ros::Rate rate(10);
  while (ros::ok())
  {
    interface.update();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
