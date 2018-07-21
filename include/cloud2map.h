#ifndef CLOUD2MAP_H_
#define CLOUD2MAP_H_
#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>
#include <map>

#include <Eigen/Core>
#include <sys/time.h>
#include <pcl/console/time.h>
#include <opencv2/core/eigen.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <cv.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include "box_type.h"
#include <nav_msgs/OccupancyGrid.h>

class GridMap
{
public:
  GridMap();
  void init(int width, int height, int offset_x, int offset_y, int offset_z, float grid_size, int queue_size);
  void update(pcl::PointCloud<pcl::PointXYZI>::ConstPtr origin_cloud, Eigen::Affine3d transform);
  cv::Mat toImg();
  nav_msgs::OccupancyGrid toNavMsg();
private:
  int H;
  int W;
  float grid_size_;
  int queue_size_;
  double offset_x_;
  double offset_y_;
  double offset_z_;

  cv::Mat img_map_;
  std::vector<pcl::PointCloud<pcl::PointXYZI> > cloud_queue_;
  std::vector<Eigen::Affine3d> transform_queue_;
  std::vector<int> vector_map_;

  void mapOperation(cv::Mat& img);
  void setOccupancyGrid(nav_msgs::OccupancyGrid *og);
  Obstacle getObstacle(std::vector<Vertex> rect);
  std::vector<Obstacle> mapToObstacle(cv::Mat img);

};

#endif