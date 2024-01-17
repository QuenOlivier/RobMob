#include "map_filtering/cv_filter.hpp"
#include <sys/stat.h>
#include <limits>

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

bool CvMapFilter::findDataRow(int row, cv::Mat image)
{
  cv::Size map_size = image.size();
  int cnt = 0;
  for(int i = (map_size.width) /2; i>=0 && i<map_size.width; cnt%2==0 ? i+=cnt : i-=cnt )
  {
    if(image.at<char>(row,i) == 0%255)
    {
      return true;
    }
    cnt ++;
  }
  return false;
}

bool CvMapFilter::findDataCol(int col, cv::Mat image)
{
  cv::Size map_size = image.size();
  int cnt = 0;
  for(int i = (map_size.height) /2; i>=0 && i<map_size.height; cnt%2==0 ? i+=cnt : i-=cnt )
  {
    if(image.at<char>(i,col) == 0%255)
    {
      return true;
    }
    cnt ++;
  }
  return false;
}

int CvMapFilter::findMaxRowPixel(int low_interval, int high_interval, cv::Mat image)
{
  if(high_interval == low_interval)
  {
    return findDataRow(low_interval, image) ? low_interval : -1;
  }
  if(high_interval == low_interval + 1)
  {
    bool top_row = findDataRow(high_interval, image);
    if(top_row){
      return high_interval;
    } else {
      bool found_data = findDataRow(low_interval, image);
      return found_data ? low_interval : -1;
    }
  }
  int test_row = low_interval + (high_interval - low_interval)/2;
  int top_half = findMaxRowPixel(test_row, high_interval, image);
  return top_half != -1 ? top_half : findMaxRowPixel(low_interval, test_row-1, image);
}

int CvMapFilter::findMaxColPixel(int low_interval, int high_interval, cv::Mat image)
{
  if(high_interval == low_interval + 1)
  {
    return std::min(
      findDataCol(low_interval, image) ? low_interval : -1,
      findDataCol(high_interval, image) ? high_interval : -1
    );
  }
  if(high_interval == low_interval)
  {
    return findDataCol(low_interval, image) ? low_interval : -1;
  }
  int test_col = low_interval + (high_interval - low_interval)/2;
  int top_half = findMaxColPixel(test_col, high_interval, image);
  return top_half != -1 ? top_half : findMaxColPixel(low_interval, test_col-1, image);
}

int CvMapFilter::findMinRowPixel(int low_interval, int high_interval, cv::Mat image)
{
  if(high_interval == low_interval)
  {
    return findDataRow(low_interval, image) ? low_interval : std::numeric_limits<int>::max();
  }
  if(high_interval == low_interval + 1)
  {
    bool low_row = findDataRow(low_interval, image);
    if(low_row){
      return low_interval;
    } else {
      bool found_data = findDataRow(high_interval, image);
      return found_data ? high_interval : std::numeric_limits<int>::max();
    }
  }
  int test_row = low_interval + (high_interval - low_interval)/2;
  int lower_half = findMinRowPixel(low_interval, test_row-1, image);
  return lower_half < image.size().height ? lower_half : findMinRowPixel(test_row, high_interval, image);
}

int CvMapFilter::findMinColPixel(int low_interval, int high_interval, cv::Mat image)
{
  if(high_interval == low_interval)
  {
    return findDataCol(low_interval, image) ? low_interval : std::numeric_limits<int>::max();
  }
  if(high_interval == low_interval + 1)
  {
    bool low_row = findDataCol(low_interval, image);
    if(low_row){
      return low_interval;
    } else {
      bool found_data = findDataCol(high_interval, image);
      return found_data ? high_interval : std::numeric_limits<int>::max();
    }
  }
  int test_col = low_interval + (high_interval - low_interval)/2;
  int lower_half = findMinColPixel(low_interval, test_col-1, image);
  return lower_half < image.size().width ? lower_half : findMinColPixel(test_col, high_interval, image);
}

void CvMapFilter::removeEmptyCells()
{
  cv::Size map_size = getImageSize();

  int maxRow = 0, minRow = 0, maxCol = 0, minCol = 0;
  maxRow = findMaxRowPixel(0, map_size.height-1, raw_image_);
  minRow = findMinRowPixel(0, map_size.height-1, raw_image_);
  std::cout << "Dimensions for rows: "<< minRow<<", "<< maxRow<< '\n';
  cv::Mat pre_reduced_image = raw_image_( cv::Range(minRow, maxRow), cv::Range(0, map_size.width-1));
  maxCol = findMaxColPixel(0, pre_reduced_image.size().width-1, pre_reduced_image);
  minCol = findMinColPixel(0, pre_reduced_image.size().width-1, pre_reduced_image);
  std::cout << "Dimensions: "<< minRow<<", "<< maxRow<< ", "<< minCol<< ", "<< maxCol<<'\n';


  roi_image_ = pre_reduced_image( cv::Range(0, pre_reduced_image.size().height), cv::Range(minCol, maxCol));
}

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
  removeEmptyCells();
  cv::threshold(roi_image_, roi_image_, 250, 255, cv::THRESH_BINARY);
}

cv::Mat CvMapFilter::filter()
{

  cv::Mat result;

  cv::Mat element = cv::getStructuringElement( 2, cv::Size( 2*size_ + 1, 2*size_+1 ), cv::Point( size_, size_ ) );
  cv::Mat element_erode = cv::getStructuringElement( 2, cv::Size( 5, 5 ), cv::Point( 2, 2 ) );

  cv::erode(roi_image_, result, element_erode);

  cv::morphologyEx(result, result, OPENING, element);
  cv::morphologyEx(result, result, CLOSING, element);

  return result;

}
