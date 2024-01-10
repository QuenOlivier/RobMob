#include "map_filtering/mapping_utils.hpp"

cv::Mat MapHelper::toCvMat(const nav_msgs::OccupancyGrid map)
{
  return cv::Mat();
}

cv::Point MapHelper::toCvCoordinates(const geometry_msgs::Pose pose)
{
  return cv::Point(0, 0);
}

nav_msgs::OccupancyGrid MapHelper::toOccupancyGrid(const cv::Mat matrix)
{
  nav_msgs::OccupancyGrid return_grid;
  return_grid.info = map_metadata_;
  for(int i = 0; i < map_metadata_.height; i++){
    for(int j = 0; j < map_metadata_.width; j++){
      int cell_value = matrix.at<int>(cv::Point(j,i));
      float grid_cell_value = 255* (float)cell_value / 100.0;
      return_grid.data.push_back(grid_cell_value);
    }
  }
  return return_grid;
}

geometry_msgs::Point MapHelper::toMapCoordinates(const cv::Point pose)
{
  return geometry_msgs::Point();
}
