#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>
#include <map>
#include <sstream>

#include <Eigen/Core>
#include <sys/time.h>
#include <pcl/console/time.h>
#include <opencv2/core/eigen.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <cv.h>

class PathDrawer
{
public:

  PathDrawer(int height, int width);
  cv::Mat U_Turn(int road_width, int line_length, int turning_radius);

private:
  int height_;
  int width_;

};
