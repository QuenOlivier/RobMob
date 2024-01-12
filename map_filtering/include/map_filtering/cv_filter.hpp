#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/opencv.hpp>

#define OPENING 2
#define CLOSING 3

class CvMapFilter {

public:
  CvMapFilter();
  CvMapFilter(int size);
  ~CvMapFilter(){};

  cv::Mat filter();
  void setPath(std::string path);
  cv::Size getImageSize() { return raw_image_.size(); };
  void setFilterSize(int size){ size_ = size; };

private:
  void removeEmptyCells();
  bool findDataRow(int row);
  bool findDataCol(int col);
  int findMaxRowPixel(int low_interval, int high_interval, cv::Mat image);
  int findMaxColPixel(int low_interval, int high_interval);
  int findMinRowPixel(int low_interval, int high_interval);
  int findMinColPixel(int low_interval, int high_interval);

  cv::Mat raw_image_;
  cv::Mat roi_image_;
  std::string image_path_;
  int size_;

};
