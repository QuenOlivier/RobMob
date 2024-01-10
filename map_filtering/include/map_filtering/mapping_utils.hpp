#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/opencv.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

class MapHelper
{
public:
  MapHelper(){};
  ~MapHelper(){};

  void setMapMetadata(const nav_msgs::MapMetaData data){ map_metadata_ = data; };
  nav_msgs::MapMetaData getMapMetadata() const { return map_metadata_; };

  cv::Mat toCvMat(const nav_msgs::OccupancyGrid map);
  cv::Point toCvCoordinates(const geometry_msgs::Pose pose);
  nav_msgs::OccupancyGrid toOccupancyGrid(const cv::Mat matrix);
  geometry_msgs::Point toMapCoordinates(const cv::Point pose);

private:
  nav_msgs::MapMetaData map_metadata_;
};
