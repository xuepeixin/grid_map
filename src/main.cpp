#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <cmath>  
#include <stack> 
#include <limits>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/transforms.h>

#include "cloud2map.h"
#include "ground_filter.h"


using namespace message_filters;
using namespace chrono;

// static std::unique_ptr<tf::TransformListener> g_tf_listener_left;
// static std::unique_ptr<tf::TransformListener> g_tf_listener_right;
// static std::unique_ptr<tf::TransformListener> g_tf_listener_middle;

// ros::Publisher filtered_pub;
static std::unique_ptr<tf::TransformListener> g_tf_listener_left;
static std::unique_ptr<tf::TransformListener> g_tf_listener_right;
static std::unique_ptr<tf::TransformListener> g_tf_listener_middle;

pcl::PointCloud<pcl::PointXYZI>::Ptr last_frame_(new pcl::PointCloud<pcl::PointXYZI>());

Eigen::Affine3d transform_last_;
GridMap* grid_map;
ros::Publisher costmap_pub_;

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& ns1, 
                    const sensor_msgs::PointCloud2::ConstPtr& ns2, 
                    const sensor_msgs::PointCloud2::ConstPtr& ns3,
                    const nav_msgs::Odometry::ConstPtr &gps_trans)
{
  auto start = system_clock::now();
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan1_(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan2_(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan3_(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan3(new pcl::PointCloud<pcl::PointXYZI>());//M
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan1(new pcl::PointCloud<pcl::PointXYZI>());//R
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan2(new pcl::PointCloud<pcl::PointXYZI>());//L
  std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters;
  pcl::fromROSMsg(*ns1,*scan1_);
  pcl::fromROSMsg(*ns2,*scan2_);
  pcl::fromROSMsg(*ns3,*scan3_);

  g_tf_listener_left->waitForTransform("base_footprint", ns2->header.frame_id, ns2->header.stamp, ros::Duration(5.0));
  pcl_ros::transformPointCloud("base_footprint", *scan2_, *scan2, *g_tf_listener_left);

  g_tf_listener_right->waitForTransform("base_footprint", ns1->header.frame_id, ns1->header.stamp, ros::Duration(5.0));
  pcl_ros::transformPointCloud("base_footprint", *scan1_, *scan1, *g_tf_listener_right);

  g_tf_listener_middle->waitForTransform("base_footprint", ns3->header.frame_id, ns3->header.stamp, ros::Duration(5.0));
  pcl_ros::transformPointCloud("base_footprint", *scan3_, *scan3, *g_tf_listener_middle);

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_two(new pcl::PointCloud<pcl::PointXYZI>());

  *scan = *scan1 + *scan2;
  *scan = *scan + *scan3;
  *scan_two = *scan1 + *scan2;
  *scan = removePointsByRange(*scan, 0, 50.0);

  pcl::PointCloud<pcl::PointXYZI>::Ptr removeground_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  ground_filter(scan,scan_two,removeground_scan_ptr);


  Eigen::Quaterniond rotation;
  rotation.coeffs() << gps_trans->pose.pose.orientation.x,
                      gps_trans->pose.pose.orientation.y,
                      gps_trans->pose.pose.orientation.z,
                      gps_trans->pose.pose.orientation.w;
  // Eigen::Vector3d euler_angles = mat.eulerAngles(2,1,0);
  Eigen::Translation3d translation(gps_trans->pose.pose.position.x,
                             gps_trans->pose.pose.position.y,
                             gps_trans->pose.pose.position.z);
  Eigen::Affine3d R(rotation.toRotationMatrix());
  Eigen::Affine3d T(translation);
  Eigen::Affine3d transform = T*R;

  // std::cout<<"transform: \n" << transform.matrix()<<std::endl;
  if(last_frame_->points.size() != 0)
  {
    // Eigen::Affine3d delta_trans = transform * transform_last_.inverse();
    // Eigen::Vector3d translation_deta = transform_last_.translation() -
    //               transform.translation();

    // Eigen::Quaterniond q_max(transform.linear());
    // Eigen::Quaterniond q_min(transform_last_.linear());
    // Eigen::Quaterniond q1(q_max.conjugate() * q_min);
    // q1.normalize();
    // translation_deta = q_max.conjugate() * translation_deta;
    // Eigen::Translation3d t1(translation_deta);
    // Eigen::Affine3d delta_trans = t1 * q1;
    // delta_trans = delta_trans.inverse();
    // std::cout<<"deta: \n" << delta_trans.matrix()<<std::endl;

    // pcl::PointCloud<pcl::PointXYZI>::Ptr transformed2 (new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::transformPointCloud(*removeground_scan_ptr, *transformed2, delta_trans.matrix());
    grid_map->update(removeground_scan_ptr, transform);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_two (new pcl::PointCloud<pcl::PointXYZI>);
    // *cloud_two = *transformed2 + *grid;

    // sensor_msgs::PointCloud2 voxel_msg;
    // pcl::toROSMsg(*dynamic_cloud1, voxel_msg);
    // voxel_msg.header = ns3->header;
    // filtered_pub.publish(voxel_msg);
  }
  else{
    transform_last_ = transform;
    grid_map->init(400, 400, 0, 0, 0, 0.25, 1);
    grid_map->update(removeground_scan_ptr, transform);
  }
  *last_frame_ = *removeground_scan_ptr;

  nav_msgs::OccupancyGrid og = grid_map->toNavMsg();
  og.header = ns3->header;
  og.header.frame_id = "vehicle/base_footprint";
  costmap_pub_.publish(og);

  // cv::Mat img = grid_map->toImg();
  // cv::Mat img1;
  // img.convertTo(img1,CV_8U,255);
  // cv::namedWindow("_", CV_WINDOW_NORMAL);
  // cv::imshow("_", img1);
  // cv::waitKey(50);

  auto end = system_clock::now();
  auto duration = duration_cast<microseconds>(end - start);
  std::cout <<  "spend" << double(duration.count()) * microseconds::period::num / microseconds::period::den << "s" << endl;
  // *last_frame_ = *grid;



  // cout <<  "花费了spend" << double(duration.count()) * microseconds::period::num / microseconds::period::den << "ssss秒" << endl;


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detection");
  grid_map = new GridMap();
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  g_tf_listener_left.reset(new tf::TransformListener());
  g_tf_listener_right.reset(new tf::TransformListener());
  g_tf_listener_middle.reset(new tf::TransformListener());

  // bbox_pub = nh.advertise<visualization_msgs::MarkerArray>("prisms_disp", 1000, true);
  // bbox_pub1 = nh.advertise<visualization_msgs::MarkerArray>("cuboids_disp", 1000, true);
  // filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);
  costmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("realtime_cost_map", 10);

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub1(nh, "/lidar/vlp16_right/PointCloud2", 5);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub2(nh,"/lidar/vlp16_left/PointCloud2", 5);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub3(nh,"/lidar/vlp32_middle/PointCloud2", 5);
  message_filters::Subscriber<nav_msgs::Odometry> gps_trans(nh, "/pioneer_sensors/EKF_Localization_RS232/filteredodometry", 5);

  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub1, cloud_sub2,cloud_sub3,gps_trans);
  sync.registerCallback(boost::bind(&lidar_callback, _1, _2, _3,_4));
  ros::spin();
  return 0;
}