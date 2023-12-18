#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>


class MapFilteringNode {

public:
  MapFilteringNode(ros::NodeHandle *nh);
  ~MapFilteringNode(){};

  void update();

private:
  ros::NodeHandle nh_;
  ros::ServiceClient map_client_;
  ros::Subscriber sub_map_;
  ros::Publisher pub_map_;

  nav_msgs::OccupancyGrid::SharedPtr last_map_;

  void onOccupancyGrid(const nav_msgs::OccupancyGrid::SharedPtr &msg);
};
