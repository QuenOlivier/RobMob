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

bool CvMapFilter::findDataRow(int row)
{
  cv::Size map_size = getImageSize();
  int cnt = 0;
  for(int i = (map_size.width) /2; i>=0 && i<map_size.width; cnt%2==0 ? i+=cnt : i-=cnt )
  {
    if(raw_image_.at<int>(row,i) == 0%255)
    {
      return true;
    }
    cnt ++;
  }
  return false;
}

bool CvMapFilter::findDataCol(int col)
{
  cv::Size map_size = getImageSize();
  int cnt = 0;
  if(col>3997){
    std::cout << "Current col " << col;
  }
  for(int i = (map_size.height) /2; i>=0 && i<map_size.height; cnt%2==0 ? i+=cnt : i-=cnt )
  {
    if(col>3997){
      std::cout << "Current idx " << i;
    }
    if(raw_image_.at<int>(i,col) == 0%255)
    {
      return true;
    }
    cnt ++;
  }
  return false;
}

int CvMapFilter::findMaxRowPixel(int low_interval, int high_interval)
{

  std::cout << "Testing interval " << low_interval << ", " << high_interval <<"\n";
  if(high_interval == low_interval)
  {
    return findDataRow(low_interval) ? low_interval : -1;
  }
  if(high_interval == low_interval + 1)
  {
    return std::max(
      findDataRow(low_interval) ? low_interval : -1,
      findDataRow(high_interval) ? high_interval : -1
    );
  }
  int test_row = low_interval + (high_interval - low_interval)/2;
  int top_half = findMaxRowPixel(test_row, high_interval);
  return top_half != -1 ? top_half : findMaxRowPixel(low_interval, test_row-1);
}

int CvMapFilter::findMaxColPixel(int low_interval, int high_interval)
{
  std::cout << "Testing interval " << low_interval << ", " << high_interval <<"\n";
  if(high_interval == low_interval + 1)
  {
    return std::max(
      findDataCol(low_interval) ? low_interval : -1,
      findDataCol(high_interval) ? high_interval : -1
    );
  }
  if(high_interval == low_interval)
  {
    return findDataCol(low_interval) ? low_interval : -1;
  }
  int test_col = low_interval + (high_interval - low_interval)/2;
  int top_half = findMaxColPixel(test_col, high_interval);
  return top_half != -1 ? top_half : findMaxColPixel(low_interval, test_col-1);
}

int CvMapFilter::findMinRowPixel(int low_interval, int high_interval)
{
  if(high_interval == low_interval)
  {
    return findDataRow(low_interval) ? low_interval : std::numeric_limits<int>::max();
  }
  if(high_interval == low_interval + 1)
  {
    return std::max(
      findDataRow(low_interval) ? low_interval : std::numeric_limits<int>::max(),
      findDataRow(high_interval) ? high_interval : std::numeric_limits<int>::max()
    );
  }
  int test_row = (high_interval - low_interval)/2, max_row = 0;
  if(!findDataRow(test_row))
  {
    return findMinRowPixel(test_row, high_interval);
  } else {
    return findMinRowPixel(low_interval, test_row -1);
  }
}

int CvMapFilter::findMinColPixel(int low_interval, int high_interval)
{
  if(high_interval == low_interval)
  {
    return findDataCol(low_interval) ? low_interval : -1;
  }
  if(high_interval == low_interval + 1)
  {
    return std::max(
      findDataCol(low_interval) ? low_interval : std::numeric_limits<int>::max(),
      findDataCol(high_interval) ? high_interval : std::numeric_limits<int>::max()
    );
  }
  int test_row = (high_interval - low_interval)/2, max_row = 0;
  if(!findDataCol(test_row))
  {
    return findMinColPixel(test_row, high_interval);
  } else {
    return findMinColPixel(low_interval, test_row -1);
  }
}

void CvMapFilter::removeEmptyCells()
{
  cv::Size map_size = getImageSize();

  int maxRow = 0, minRow = 0, maxCol = 0, minCol = 0;
  maxRow = findMaxRowPixel(0, map_size.height-1);
  std::cout << "Max Row " << maxRow << std::endl;
  // int minRow = findMinRowPixel(0, map_size.height-1);
  maxCol = findMaxColPixel(0, map_size.width-1);
  // int minCol = findMinColPixel(0, map_size.width-1);
  std::cout << "Dimensions: "<< minRow<<", "<< maxRow<< ", "<< minCol<< ", "<< maxCol<<'\n';

  raw_image_ = raw_image_( cv::Range(minRow, maxRow+1), cv::Range(minCol, maxCol+1));
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
