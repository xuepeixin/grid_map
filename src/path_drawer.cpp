#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>
#include <map>
#include <sstream>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <sys/time.h>
#include <opencv2/core/eigen.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <cv.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ros/ros.h>

#include <path_drawer.h>

const float GRIDSIZE = 0.25;
const int SGSIZE = 200;

ros::Publisher costmap_pub_;
cv::Mat MG;
bool is_loc_;
Eigen::Vector2d start_position_;
cv::Point start_point_;

PathDrawer::PathDrawer(int height, int width):height_(height),
                                              width_(width)
{

}

cv::Mat PathDrawer::U_Turn(int road_width, int line_length, int turning_radius)
{
  static int height_half = height_ / 2;
  static int width_half = width_ / 2;
  static int inner_radius = turning_radius - road_width;
  cv::Mat MG = cv::Mat::zeros(height_, width_, CV_8UC1);

  cv::line(MG, cv::Point(0, height_half - turning_radius), 
                cv::Point(line_length + inner_radius - 1, height_half - turning_radius),100,2);
  cv::line(MG, cv::Point(0, height_half + turning_radius), 
                cv::Point(line_length + inner_radius - 1, height_half + turning_radius),100,2);

  cv::line(MG, cv::Point(0, height_half - inner_radius), 
                cv::Point(line_length-1, height_half - inner_radius),100,2);
  cv::line(MG, cv::Point(0, height_half + inner_radius), 
                cv::Point(line_length-1, height_half + inner_radius),100,2);
  
  cv::ellipse(MG, cv::Point(line_length - 1, height_half), 
                  cv::Size(inner_radius, inner_radius), -90.0, 0.0, 180.0, 100,2);
  cv::ellipse(MG, cv::Point(line_length + inner_radius - 1, height_half), 
                  cv::Size(turning_radius, turning_radius), -90.0, 0.0, 180.0, 100,2);
  return MG;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &gps_pose)
{
  Eigen::Quaterniond row_heading;
  row_heading.coeffs() << gps_pose->pose.pose.orientation.x, 
                          gps_pose->pose.pose.orientation.y, 
                          gps_pose->pose.pose.orientation.z, 
                          gps_pose->pose.pose.orientation.w;

  Eigen::Matrix3d mat = row_heading.toRotationMatrix();
  Eigen::Vector3d temp = mat.eulerAngles(2,1,0);
  float rotation = temp(0) * 180 / M_PI;
  Eigen::Vector2d position(gps_pose->pose.pose.position.x, gps_pose->pose.pose.position.y);
  if(!is_loc_)
  {
    start_position_ = position;
    is_loc_ = true;
  }
  Eigen::Vector2d trans = position - start_position_;

  cv::Mat sg = cv::Mat::zeros(SGSIZE, SGSIZE, CV_8UC1);
  cv::Mat mg_copy;
  MG.copyTo(mg_copy);

  cv::Mat t_mat =cv::Mat::zeros(2, 3, CV_32FC1);
  t_mat.at<float>(0, 0) = 1;
	t_mat.at<float>(0, 2) = -trans(0)/GRIDSIZE; //水平平移量
	t_mat.at<float>(1, 1) = 1;
	t_mat.at<float>(1, 2) = -trans(1)/GRIDSIZE; //竖直平移量
  cv::warpAffine(mg_copy, mg_copy, t_mat, cv::Size(500,500));

  cv::Mat rot = cv::getRotationMatrix2D(start_point_, -rotation, 1);
  cv::warpAffine(mg_copy,mg_copy, rot, cv::Size(500, 500));
  std::vector<int> vector_map(SGSIZE*SGSIZE,0);
  for(int i = 0; i < SGSIZE; i++)
  {
    for(int j = 0; j < SGSIZE; j++)
    {
      sg.at<uchar>(j,i) = mg_copy.at<uchar>(start_point_.y - SGSIZE/2 + j, i);
      vector_map[j * SGSIZE + i] = sg.at<uchar>(j,i);
    }
  }
  static int count = 0;
  static nav_msgs::OccupancyGrid og;
  if (!count)
  {
    og.info.resolution = GRIDSIZE;
    og.info.width = SGSIZE;
    og.info.height = SGSIZE;
    og.info.origin.position.x = -1;
    og.info.origin.position.y = (-1) * (SGSIZE / 2.0) * GRIDSIZE;
    og.info.origin.position.z = 0;
    // og.info.origin.orientation.x = gps_pose->pose.pose.orientation.x;
    // og.info.origin.orientation.y = gps_pose->pose.pose.orientation.y;
    // og.info.origin.orientation.z = gps_pose->pose.pose.orientation.z;
    // og.info.origin.orientation.w = gps_pose->pose.pose.orientation.w;
    og.info.origin.orientation.x = 0;
    og.info.origin.orientation.y = 0;
    og.info.origin.orientation.z = 0;
    og.info.origin.orientation.w = 1;
  }
  
  og.header = gps_pose->header;
  og.header.frame_id = "vehicle/base_footprint";
  og.data.insert(og.data.end(), vector_map.begin(), vector_map.end());
  costmap_pub_.publish(og);
  og.data.clear();
  count++;
  // cv::imshow(" ", sg);
  // cv::waitKey(50);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_drawer");
  ros::NodeHandle nh; 
  is_loc_ = false;
  PathDrawer* path_drawer(new PathDrawer(500, 500));

  MG = path_drawer->U_Turn(20, 200, 50);
  start_point_ = cv::Point(4, 210);
  costmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("realtime_cost_map1", 10);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/vehicle/odom", 1, odomCallback);
  ros::spin();
}