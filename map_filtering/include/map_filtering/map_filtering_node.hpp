#pragma once

#include <ros/ros.h>
#include <map_filtering/cv_filter.hpp>
#include <map_filtering/mapping_utils.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <memory>


class MapFilteringNode {

public:
  MapFilteringNode(ros::NodeHandle *nh);
  ~MapFilteringNode(){};

  void update();

private:

  ros::NodeHandle nh_;
  ros::Publisher pub_map_;

  CvMapFilter raw_image_filter_;
  MapHelper map_helper_;

  std::shared_ptr<nav_msgs::OccupancyGrid> map_;
};
