#include <map_filtering/map_filtering_node.hpp>

#include <iostream>
#include <string>
#include <vector>

MapFilteringNode::MapFilteringNode(ros::NodeHandle *nh) :
nh_(*nh),
map_(nullptr)
{
  pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map/filtered_map", 1);

  // get parameters
  std::string image_path;
  nh_.getParam("/map_filtering_node/image_folder_path", image_path);
  std::string image_name;
  nh_.getParam("image", image_name);
  int filter_size;
  nh_.getParam("/map_filtering_node/filter_size", filter_size);
  std::vector<float> origin_pose;
  nh_.getParam("origin", origin_pose);

  image_path = image_path + image_name;
  raw_image_filter_ = CvMapFilter(filter_size);
  raw_image_filter_.setPath(image_path);

  cv::Size image_size = raw_image_filter_.getImageSize();

  nav_msgs::MapMetaData metadata;

  nh_.getParam("resolution", metadata.resolution);
  metadata.width = image_size.width;
  metadata.height = image_size.height;
  metadata.origin.position.x = origin_pose.at(0);
  metadata.origin.position.y = origin_pose.at(1);
  metadata.origin.position.z = origin_pose.at(2);
  metadata.map_load_time = ros::Time::now();
  map_helper_.setMapMetadata(metadata);

}

void MapFilteringNode::update(){
  if(!map_)
  {
    cv::Mat filtered_map_cv = raw_image_filter_.filter();
    cv::imshow("Test window", filtered_map_cv);
    cv::waitKey(0);
    *map_ = map_helper_.toOccupancyGrid(filtered_map_cv);
    map_->header.frame_id = "/world";
  }
  map_->header.stamp = ros::Time::now();
  pub_map_.publish(*map_);
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
