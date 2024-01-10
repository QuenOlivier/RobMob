#include "map_filtering/cv_filter.hpp"
#include <sys/stat.h>

inline bool fileExist (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

CvMapFilter::CvMapFilter() :
size_(1)
{
};

CvMapFilter::CvMapFilter(int size) :
size_(size)
{
};

void CvMapFilter::setPath(std::string path)
{
  if(!fileExist(path))
  {
    throw std::invalid_argument("File does not exist");
  }
  raw_image_ = cv::imread(path.c_str(), cv::IMREAD_GRAYSCALE);
  if (raw_image_.empty())
  {
     throw std::invalid_argument("Could not open source image");
  }
  cv::imshow("Test window", raw_image_);
  cv::waitKey(0);
}

cv::Mat CvMapFilter::filter()
{

  cv::Mat result;
  cv::cvtColor(raw_image_, result, cv::COLOR_RGB2GRAY);

  cv::Mat element = cv::getStructuringElement( 2, cv::Size( 2*size_ + 1, 2*size_+1 ), cv::Point( size_, size_ ) );

  cv::morphologyEx(result, result, OPENING, element);
  cv::morphologyEx(result, result, CLOSING, element);

  return result;

}
