#include "map_filtering/cv_filter.hpp"

CvMapFilter::CvMapFilter(int size) :
size_(size)
{

};

void CvMapFilter::setPath(std::string path)
{
  raw_image_ = cv::imread(image_path_, cv::IMREAD_GRAYSCALE);

  if (raw_image_.empty())
  {
     throw std::invalid_argument("Could not open source image");
  }
}

cv::Mat CvMapFilter::filter()
{

  cv::Mat result;

  cv::Mat element = cv::getStructuringElement( 2, cv::Size( 2*size_ + 1, 2*size_+1 ), cv::Point( size_, size_ ) );

  cv::MorphologyEx(raw_image_, result, OPENING, element);
  cv::MorphologyEx(result, result, CLOSING, element);

  return result;

}
