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
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include "ground_filter.h"

#define DISTANCETHRESHOLD 0.2

using namespace std;


//pcl::PointCloud<pcl::PointXYZI>::Ptr removeground_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
//static ros::Publisher filtered_points_pub;

void init_conditional_removal(pcl::ConditionalRemoval<pcl::PointXYZI>& condrem)
{
    pcl::ConditionOr<pcl::PointXYZI>::Ptr car_range_cond(new pcl::ConditionOr<pcl::PointXYZI>);
	pcl::FieldComparison<pcl::PointXYZI>::ConstPtr x_cond_L(new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::LT,-0.8));
	pcl::FieldComparison<pcl::PointXYZI>::ConstPtr x_cond_G(new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::GT,3.6));
	pcl::FieldComparison<pcl::PointXYZI>::ConstPtr y_cond_L(new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::LT,-1.0));
	pcl::FieldComparison<pcl::PointXYZI>::ConstPtr y_cond_G(new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::GT,0.8));
	car_range_cond->addComparison(x_cond_L);
	car_range_cond->addComparison(x_cond_G);
	car_range_cond->addComparison(y_cond_L);
	car_range_cond->addComparison(y_cond_G);

    pcl::ConditionOr<pcl::PointXYZI>::Ptr rearview_range_cond(new pcl::ConditionOr<pcl::PointXYZI>);
	pcl::FieldComparison<pcl::PointXYZI>::ConstPtr rx_cond_L(new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::LT,1.8));
	pcl::FieldComparison<pcl::PointXYZI>::ConstPtr rx_cond_G(new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::GT,2.8));
	pcl::FieldComparison<pcl::PointXYZI>::ConstPtr ry_cond_L(new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::LT,-1.3));
	pcl::FieldComparison<pcl::PointXYZI>::ConstPtr ry_cond_G(new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::GT,1.2));
	rearview_range_cond->addComparison(rx_cond_L);
	rearview_range_cond->addComparison(rx_cond_G);
	rearview_range_cond->addComparison(ry_cond_L);
	rearview_range_cond->addComparison(ry_cond_G);

    pcl::ConditionAnd<pcl::PointXYZI>::Ptr car_full_range_cond(new pcl::ConditionAnd<pcl::PointXYZI>);
    car_full_range_cond->addCondition(car_range_cond);
    car_full_range_cond->addCondition(rearview_range_cond);

    condrem.setCondition(car_full_range_cond);
	condrem.setKeepOrganized(false);
}

void ground_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr &filtered_scan_ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr &scan_two, pcl::PointCloud<pcl::PointXYZI>::Ptr& removeground_scan_ptr)
{
    pcl::IndicesPtr indices_sky(new std::vector<int>());
    pcl::PassThrough<pcl::PointXYZI> ptfilter(false);
    ptfilter.setInputCloud(filtered_scan_ptr);
    ptfilter.setFilterFieldName("z");
    ptfilter.setFilterLimits(-100.0, 2.0);
    ptfilter.filter (*indices_sky);

    pcl::PointCloud<pcl::PointXYZI>::Ptr removecar(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
    init_conditional_removal(condrem);

    condrem.setInputCloud(filtered_scan_ptr);
    condrem.setIndices(indices_sky);
    condrem.filter(*removecar);

    pcl::IndicesPtr indices_ground_only(new std::vector<int>());
    pcl::PassThrough<pcl::PointXYZI> ptfilter_ground(false);
    ptfilter_ground.setInputCloud(scan_two);
    ptfilter_ground.setFilterFieldName("z");
    ptfilter_ground.setFilterLimits(-100.0, 0.7);
    ptfilter_ground.filter (*indices_ground_only);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    Eigen::Vector3f ground_vec;
    ground_vec<<0, 0, 1;
    seg.setAxis(ground_vec);
    seg.setEpsAngle(0.15);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(DISTANCETHRESHOLD); 
    seg.setInputCloud(scan_two);
    seg.setIndices(indices_ground_only);
    seg.segment(*inliers, *coefficients);

    pcl::PointCloud<pcl::PointXYZI>::Ptr remove_ground(new pcl::PointCloud<pcl::PointXYZI>()); 
    for(size_t i=0; i<removecar->points.size(); i++)
    {
        if(fabs(removecar->points[i].x * coefficients->values[0] +
                removecar->points[i].y * coefficients->values[1] +
                removecar->points[i].z * coefficients->values[2] +
                                         coefficients->values[3] )>=0.25 )
            remove_ground->points.push_back(removecar->points[i]);
    }

    // pcl::ExtractIndices<pcl::PointXYZI> extract;
    // extract.setInputCloud(removecar);
    // extract.setIndices(inliers);
    // extract.setNegative(true);
    // extract.filter(*remove_ground);
  
    pcl::IndicesPtr indices_ground(new std::vector<int>());
    pcl::PassThrough<pcl::PointXYZI> ptfilter2(false);
    ptfilter2.setInputCloud(remove_ground);
    ptfilter2.setFilterFieldName("z");
    ptfilter2.setFilterLimits(0.2,2.0);
    ptfilter2.filter(*removeground_scan_ptr);
}

pcl::PointCloud<pcl::PointXYZI> removePointsByRange(pcl::PointCloud<pcl::PointXYZI> scan, double min_range, double max_range)
{
  pcl::PointCloud<pcl::PointXYZI> narrowed_scan;
  narrowed_scan.header = scan.header;

  double square_min_range = min_range * min_range;
  double square_max_range = max_range * max_range;

  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator iter = scan.begin(); iter != scan.end(); ++iter)
  {
    pcl::PointXYZI p;
    p.x = iter->x;
    p.y = iter->y;
    p.z = iter->z;
    p.intensity = iter->intensity;
    double square_distance = p.x * p.x + p.y * p.y;

    if(square_min_range <= square_distance && square_distance <= square_max_range){
      narrowed_scan.points.push_back(p);
    }
  }

  return narrowed_scan;
}
