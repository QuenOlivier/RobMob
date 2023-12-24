#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/opencv.hpp>

#define OPENING 2
#define CLOSING 3

class CvMapFilter {

public:
  CvMapFilter(int size);
  ~CvMapFilter(){};

  cv::Mat filter();
  void setPath(std::string path);
  cv::Size getSize() { return raw_image_.size(); };

private:
  cv::Mat raw_image_;
  std::string image_path_;
  int size_,

};
