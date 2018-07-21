#ifndef _INCLUDE_DBSCAN_H_
#define _INCLUDE_DBSCAN_H_

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

#include <opencv2/core/eigen.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <cv.h>

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

#include "box_type.h"
// #define EPS 1.2
// #define MINPTS 10

using namespace std;
using namespace chrono;



class DBPoint{  
public:  
    float x;  
    float y; 
    float z; 
    int cluster = 0;  
    int pointType=1;  //0 noise  1 core  
    int pts = 0;  //points in MinPts   
    vector<int> corepts;  
    int visited = 0;  
    DBPoint (){}  
    DBPoint (float a,float b,float c){  
        x = a;  
        y = b;
        z = c;  
        //cluster = c1;  
    }
};

class Cluster{


public:
    Cluster(){}
    void DBScan(pcl::PointCloud<pcl::PointXYZI>::Ptr removestatic_scan_ptr, 
                std::vector<pcl::PointCloud<pcl::PointXYZ> >& clusters,
                float Eps = 1.2,
                int MinPts = 10);
    void DBScanPlane(cv::Mat img, 
                    std::vector<std::vector<Vertex> >& clusters,
                    int offset_x, 
                    int offset_y, 
                    float grid_size, 
                    float Eps = 1.2,
                    int MinPts = 10);
private:
    float squareDistance(const DBPoint& a, const DBPoint& b);
    void ExpandCluster(vector<DBPoint> &dataset,DBPoint &data,int c);
};
#endif
