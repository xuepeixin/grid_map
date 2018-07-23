#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>
#include <map>
#include <chrono>

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
#include <nav_msgs/OccupancyGrid.h>

#include "cloud2map.h"
#include "DBSCAN.h"
#include "convex_hull.h"

constexpr double HEIGHT_LIMIT = 2.0;  // from sensor
constexpr double CAR_LENGTH = 4.95; // LINCOLN MKZ
constexpr double CAR_WIDTH = 1.864; //LINCOLN MKZ

GridMap::GridMap():grid_size_(0),
                  offset_x_(0),
                  offset_y_(0),
                  offset_z_(0),
                  map_center_x_(0),
                  map_center_y_(0)
{

}

void GridMap::init(  int width, 
                     int height,
                     int offset_x,
                     int offset_y,
                     int offset_z, 
                     float grid_size,
                     int queue_size)
{
  occupy_blank_ratio_ = 4.0;
  blank_occupy_ratio_ = 0.25;
  W = width;
  H = height;
  offset_x_ = offset_x;
  offset_y_ = offset_y;
  offset_z_ = offset_z;
  img_map_ = cv::Mat::zeros(height, width, CV_8UC1);
  vector_map_ = std::vector<int>(height * width, 0);
  grid_size_ = grid_size;
  queue_size_ = queue_size;

  grid_info_.resize(W * H);
  map_center_x_ = (W / 2.0) * grid_size_ - offset_x_; //地图中
  map_center_y_ = (H / 2.0) * grid_size_ - offset_y_;
  grid_center_x_ = map_center_x_ / grid_size_;
  grid_center_y_ = map_center_y_ / grid_size_;
  
  grid_info_ = initGridInfo(W, H, cv::Point(map_center_x_/grid_size_, map_center_y_/grid_size_));
}

// 初始化带有角度和距离信息的空地图
std::vector<GridInfo> GridMap::initGridInfo(int width, int height, cv::Point origin)
{
  std::vector<GridInfo> grid_info(width*height);
  for(int j = 0; j < height; j++)
  {
    for(int i = 0; i < width; i++)
    {
      if(j >= origin.y && i >= origin.x)
      {
        grid_info[j * width + i].theta_min = atan2(j - origin.y, i - origin.x + 1);
        grid_info[j * width + i].theta_max = atan2(j - origin.y + 1, i - origin.x);
        grid_info[j * width + i].distance = sqrt((i - origin.x + 0.5)*(i - origin.x + 0.5) + 
                                                 (j - origin.y + 0.5)*(j - origin.y + 0.5));
      }
      else if(j <= origin.y && i >= origin.x)
      {
        grid_info[j * width + i].theta_min = atan2(j - origin.y, i - origin.x );
        grid_info[j * width + i].theta_max = atan2(j - origin.y + 1, i - origin.x + 1);
        grid_info[j * width + i].distance = sqrt((i - origin.x + 0.5)*(i - origin.x + 0.5) + 
                                                 (j - origin.y + 0.5)*(j - origin.y + 0.5));
      }
      else if(j >= origin.y && i <= origin.x)
      {
        grid_info[j * width + i].theta_min = atan2(j - origin.y + 1, i - origin.x + 1);
        grid_info[j * width + i].theta_max = atan2(j - origin.y, i - origin.x);
        grid_info[j * width + i].distance = sqrt((i - origin.x + 0.5)*(i - origin.x + 0.5) + 
                                                 (j - origin.y + 0.5)*(j - origin.y + 0.5));
      }
      else{
        grid_info[j * width + i].theta_min = atan2(j - origin.y + 1, i - origin.x);
        grid_info[j * width + i].theta_max = atan2(j - origin.y, i - origin.x + 1);
        grid_info[j * width + i].distance = sqrt((i - origin.x + 0.5)*(i - origin.x + 0.5) + 
                                                 (j - origin.y + 0.5)*(j - origin.y + 0.5));
      }
    }
  }
  
  return grid_info;
}

void GridMap::update(pcl::PointCloud<pcl::PointXYZI>::ConstPtr origin_cloud, Eigen::Affine3d transform)
{ 
  auto start = system_clock::now();

  img_map_ = cv::Mat(H, W, CV_32FC1,1);
  vector_map_ = std::vector<int>(H * W, 0);
  // cv::Mat img = cv::Mat::zeros(300,300,CV_8UC1);
  // std::vector<cv::Point> points = getPointsInLine(cv::Point(1,200), cv::Point(300,1));
  // for(int i = 0; i < 100; i++)
  // {
  //   img.at<uchar>(points[i]) = 255;
  // }
  // cv::imshow("11", img);
  // cv::waitKey(0);
  if(cloud_queue_.size() == queue_size_)
  {
    cloud_queue_.erase(cloud_queue_.begin());
    transform_queue_.erase(transform_queue_.begin());
  }

  cloud_queue_.push_back(*origin_cloud);
  transform_queue_.push_back(transform);
  Eigen::Affine3d transform_base = *(transform_queue_.end()-1);


  for(int i = 0; i < transform_queue_.size(); i++)
  {
    Eigen::Vector3d translation_deta = transform_queue_[i].translation() -
                                        transform_base.translation();

    Eigen::Quaterniond q_max(transform_base.linear());
    Eigen::Quaterniond q_min(transform_queue_[i].linear());
    Eigen::Quaterniond q1(q_max.conjugate() * q_min);
    q1.normalize();
    translation_deta = q_max.conjugate() * translation_deta;
    Eigen::Translation3d t1(translation_deta);
    Eigen::Affine3d delta_trans = t1 * q1;

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(cloud_queue_[i], *transformed, delta_trans.matrix());

    cv::Mat img_temp = toSingleMap(transformed);
    for(int q = 0; q < H; q++)
    {
      for(int p = 0; p < W; p++)
      {
        img_map_.at<float>(q,p) = img_map_.at<float>(q,p) * img_temp.at<float>(q,p);
      }
    }
    img_map_ += img_temp;
  }

  // 还原为概率分布
  for(int q = 0; q < H; q++)
  {
    for(int p = 0; p < W; p++)
    {
      img_map_.at<float>(q,p) = img_map_.at<float>(q,p) / (1 + img_map_.at<float>(q,p));
    }
  }

  for(int i = 0; i < W; i++)
  {
    for(int j = 0; j < H; j++)
    {
        vector_map_[j * W + i] = img_map_.at<float>(j, i)*100;
    }
  }
  auto end = system_clock::now();
  auto duration = duration_cast<microseconds>(end - start);
  std::cout <<  "spend" << double(duration.count()) * microseconds::period::num / microseconds::period::den << "s" << endl;

}

std::vector<cv::Point> GridMap::getPointsInLine(cv::Point pt1, cv::Point pt2)
{
  std::vector<cv::Point> points_in_line;
  bool steep = (abs(pt2.y- pt1.y) > abs(pt2.x-pt1.x));
  if(steep)
  {
    swap(pt1.x, pt1.y);
    swap(pt2.x, pt2.y);
  }
  if(pt1.x > pt2.x)
  {
    swap(pt1.x, pt2.x);
    swap(pt1.y, pt2.y);
  }
  int deltax = pt2.x - pt1.x;
  int deltay = abs(pt2.y - pt1.y);
  int error = deltax / 2;
  int ystep;
  int y = pt1.y;
  if(pt1.y < pt2.y)
  {
    ystep = 1;
  }
  else{
    ystep = -1;
  }
  for(int x = pt1.x; x < pt2.x; x++)
  {
   if(steep)
   {
      points_in_line.push_back(cv::Point(y,x));
   } 
   else{
      points_in_line.push_back(cv::Point(x,y));    
   }

   error = error - deltay;
   if(error < 0)
   {
     y = y + ystep;
     error = error + deltax;
   }
  }

  return points_in_line;
}

cv::Mat GridMap::toSingleMap(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  cv::Mat img_temp(H, W, CV_32FC1,1);
  for(int j = 0; j < cloud->points.size(); j++)
  {
    // int width;
    // int height;
    if (cloud->points[j].z > HEIGHT_LIMIT)
      continue;
    if (std::fabs(cloud->points[j].x) < (CAR_LENGTH )&& std::fabs(cloud->points[j].y) < (CAR_WIDTH))
      continue;
    int grid_x = (cloud->points[j].x + map_center_x_) / grid_size_;
    int grid_y = (cloud->points[j].y + map_center_y_) / grid_size_;

    if(grid_y < H && grid_y >= 0 && grid_x >= 0 && grid_x < W)
    {
      img_temp.at<float>(grid_y, grid_x) = occupy_blank_ratio_;
    }
  }

  for(int i = 0; i < H; i++)
  {
    setOccupyInLine(img_temp, cv::Point(0,i));
    setOccupyInLine(img_temp, cv::Point(W-1, i));
  }
  for(int i = 0; i < W; i++)
  {
    setOccupyInLine(img_temp, cv::Point(i, 0));
    setOccupyInLine(img_temp, cv::Point(i, H-1));
  }
  return img_temp;
}

void GridMap::setOccupyInLine(cv::Mat& img_temp, cv::Point end_point)
{
  std::vector<cv::Point> points_in_line = getPointsInLine(end_point, 
                                                               cv::Point(grid_center_x_, grid_center_y_));                    
  int len = points_in_line.size();
  int inc_count = 0;
  int dec_count = len-1;
  if(points_in_line[0] != cv::Point(grid_center_x_, grid_center_y_))
  {
    std::reverse(points_in_line.begin(),points_in_line.end());
  }
  while(dec_count >= 0)
  {
    if(img_temp.at<float>(points_in_line[dec_count]) == occupy_blank_ratio_) break;
    img_temp.at<float>(points_in_line[dec_count]) = 1;
    dec_count--;
  }

  while(inc_count < len)
  {
    if(img_temp.at<float>(points_in_line[inc_count]) == occupy_blank_ratio_) break;
    img_temp.at<float>(points_in_line[inc_count]) = blank_occupy_ratio_;
    inc_count++;
  }
}

void GridMap::setOccupancyGrid(nav_msgs::OccupancyGrid *og)
{
  og->info.resolution = grid_size_;
  og->info.width = W;
  og->info.height = H;
  og->info.origin.position.x = (-1) * (W / 2.0) * grid_size_ + offset_x_;
  og->info.origin.position.y = (-1) * (H / 2.0) * grid_size_ + offset_y_;
  og->info.origin.position.z = offset_z_;
  og->info.origin.orientation.x = 0.0;
  og->info.origin.orientation.y = 0.0;
  og->info.origin.orientation.z = 0.0;
  og->info.origin.orientation.w = 1.0;
}

cv::Mat GridMap::toImg()
{
  return img_map_;
}

nav_msgs::OccupancyGrid GridMap::toNavMsg()
{
  nav_msgs::OccupancyGrid og;
  setOccupancyGrid(&og);
  og.data.insert(og.data.end(), vector_map_.begin(), vector_map_.end());
  return og;
}

void GridMap::mapOperation(cv::Mat& img)
{
  cv::Mat kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4,4));
  cv::dilate(img, img,kernal);
  cv::erode(img, img, kernal);
}

Obstacle GridMap::getObstacle(std::vector<Vertex> rect)
{
  Obstacle ob;
  ob.x = (rect[0].x + rect[1].x + rect[2].x + rect[3].x) / 4;
  ob.y = (rect[0].y + rect[1].y + rect[2].y + rect[3].y) / 4;

  float dis1 = sqrt((rect[0].x-rect[1].x)*(rect[0].x-rect[1].x) + (rect[0].y-rect[1].y)*(rect[0].y-rect[1].y));
  float dis2 = sqrt((rect[2].x-rect[1].x)*(rect[2].x-rect[1].x) + (rect[2].y-rect[1].y)*(rect[2].y-rect[1].y));
  if(dis1 > dis2)
  {
    ob.length = dis1;
    ob.width = dis2;
    ob.orientation = atan((rect[0].y-rect[1].y) / (rect[0].x-rect[1].x));
  }  
  else
  {
    ob.length = dis2;
    ob.width = dis1;
    ob.orientation = atan((rect[1].y-rect[2].y) / (rect[1].x-rect[2].x));
  }
  return ob;
}

std::vector<Obstacle> GridMap::mapToObstacle(cv::Mat img)
{
  int height_half = img.size().height/2;
  int width_half = img.size().width/2;
  cv::Mat img_copy;
  img.copyTo(img_copy);
  std::vector<std::vector<Vertex> > vertex_clusters;
  std::vector<Obstacle> obstacles;
  Cluster* cluster(new Cluster());
  cluster->DBScanPlane(img, vertex_clusters, offset_x_, offset_y_, 
                      grid_size_, 0.6, 4);

  for(int i = 0; i < vertex_clusters.size(); i++)
  {
    //避免出现无边框的情况
    if(vertex_clusters[i].size() < 6) continue;
    std::vector<Vertex> rect;
    ConvexHull* convex_hull(new ConvexHull(vertex_clusters[i]));
    if(convex_hull->vertices_.size() < 3) continue;
    rect = convex_hull->toRec1();
    for(int i = 0; i < rect.size()-1; i++)
    {
    cv::line(img_copy, cv::Point(rect[i].x/grid_size_ + width_half, 
                                  rect[i].y/grid_size_ + height_half),
                      cv::Point(rect[i+1].x/grid_size_ + width_half, 
                      rect[i+1].y/grid_size_ + height_half),
                      cv::Scalar(255,0,0),1);
    }
    cv::line(img_copy, cv::Point((rect.end()-1)->x/grid_size_ + width_half,
                                 (rect.end()-1)->y/grid_size_ + height_half),
                      cv::Point(rect.begin()->x/grid_size_ + width_half, 
                                rect.begin()->y/grid_size_ + height_half),
                      cv::Scalar(255,0,0),1);
    Obstacle ob = getObstacle(rect);
    obstacles.push_back(ob);

  }
  cv::namedWindow(" ", CV_WINDOW_NORMAL);
  cv::imshow(" ", img_copy);
  cv::waitKey(10);
  return obstacles;
} 

