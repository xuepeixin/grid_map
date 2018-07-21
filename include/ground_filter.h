// #ifdef _INCLUDE_GROUNDFILTER_H_
// #define _INCLUDE_GROUNDFILTER_H_
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

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>   
#include <pcl/sample_consensus/model_types.h>   
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/point_cloud.h>


#define DISTANCETHRESHOLD 0.3

using namespace std;
void init();
void ground_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr &a,pcl::PointCloud<pcl::PointXYZI>::Ptr &b, pcl::PointCloud<pcl::PointXYZI>::Ptr& c);
pcl::PointCloud<pcl::PointXYZI> removePointsByRange(pcl::PointCloud<pcl::PointXYZI> scan, double min_range, double max_range);
// #endif