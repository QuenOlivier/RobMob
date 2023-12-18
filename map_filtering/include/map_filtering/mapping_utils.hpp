#pragma once

cv::Mat toCvMat(const nav_msgs::OccupancyGrid map);
cv::Point toCvCoordinates(const geometry_msgs::Pose pose, const nav_msgs::MapMetaData data);
nav_msgs::OccupancyGrid toOccupancyGrid(const cv::Mat matrix);
geometry_msgs::Point toMapCoordinates(const cv::Point pose, const nav_msgs::MapMetaData data);
