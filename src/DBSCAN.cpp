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
#include <boost/make_shared.hpp>
#include "DBSCAN.h"


//inline float squareDistance
float Cluster::squareDistance(const DBPoint& a, const DBPoint& b){  
    return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);  
} 


// void Cluster::ExpandCluster(vector<DBPoint> &dataset,DBPoint &data,float Eps,int MinPts,int c)
// {
//     vector<DBPoint> neighbors;
//     int len=data.corepts.size();
//     neighbors.resize(len);
//     for(int i=0;i<len;i++)
//     {
//         neighbors[i]=dataset[data.corepts[i]];
//     }
//     for(int i=0;i<len;i++)
//     {
//         //if(neighbors[i].visited==0) continue;

//         if(neighbors[i].pts>=MinPts)
//         {
//             for(int j=0;j<neighbors[i].corepts.size();j++)
//             {
//                 if(dataset[neighbors[i].corepts[j]].visited==0)
//                 {
//                     dataset[data.corepts[i]].cluster=c;
//                     dataset[neighbors[i].corepts[j]].cluster=c;
//                     dataset[neighbors[i].corepts[j]].visited=1;
//                     dataset[neighbors[i].corepts[j]].pointType=1;
//                 }
//             }
//         }
//         //if(neighbors[i].cluster==0)
//         else
//         {
//             data.cluster=c;
//             //dataset[i].pointType=1;
//             dataset[data.corepts[i]].visited=1;
//             dataset[data.corepts[i]].pointType=1;
//         }
//     } 
// }

void Cluster::ExpandCluster(vector<DBPoint> &dataset, DBPoint &data, int c)
{
    for(int i = 0; i < data.pts; i++)
    {
        if(dataset[data.corepts[i]].visited == 0)
        {
            dataset[data.corepts[i]].visited = 1;
            dataset[data.corepts[i]].cluster = c;
            dataset[data.corepts[i]].pointType = 1;
            ExpandCluster(dataset, dataset[data.corepts[i]], c);
        }
        else continue;
    }
}
//int C=1;

void Cluster::DBScan(pcl::PointCloud<pcl::PointXYZI>::Ptr removestatic_scan_ptr, 
                    std::vector<pcl::PointCloud<pcl::PointXYZ> >& clusters,
                    float Eps,
                    int MinPts )
{
    //vector<DBPoint> dataset;
    vector<DBPoint> dataset;
    for(int i = 0;i < removestatic_scan_ptr->points.size();++i)
    {
        DBPoint p(removestatic_scan_ptr->points[i].x,removestatic_scan_ptr->points[i].y,removestatic_scan_ptr->points[i].z);
        dataset.push_back(p);
    }
    int C=1;
    int coreflag;
    vector<DBPoint> dataset2;
    int len=dataset.size();
    vector<int> index;
    for(int i=0;i<len;i++){  
        for(int j=i+1;j<len;j++){  
            if(squareDistance(dataset[i],dataset[j])<Eps*Eps)
            {
                dataset[i].pts++;  
                dataset[j].pts++;
                dataset[i].corepts.push_back(j);  
                dataset[j].corepts.push_back(i);  
            }
        }  
    }  
    for(int i=0;i<len;i++)
    {
        if(dataset[i].visited==1) continue;
        if(dataset[i].pts>=MinPts)
        {
            coreflag=1;
            dataset[i].cluster=C;
            dataset[i].visited=1;
            dataset[i].pointType=1;
            DBPoint& p=dataset[i];
            ExpandCluster(dataset,p,C);
        }
        else
        {
            coreflag=0;
            //dataset[i].visited=1;
            dataset[i].cluster=0;
            dataset[i].pointType=0;
        }
        C=C+coreflag;
    }
    clusters.resize(C);
    for(int i = 0; i < len; i++)
    {
        if(dataset[i].cluster  == 0 ) continue;
        clusters[dataset[i].cluster-1].push_back(pcl::PointXYZ(dataset[i].x, dataset[i].y, dataset[i].z));
    }    
}

void Cluster::DBScanPlane(cv::Mat img, 
                        std::vector<std::vector<Vertex> >& clusters,
                        int offset_x, 
                        int offset_y, 
                        float grid_size, 
                        float Eps,
                        int MinPts )
{
    std::vector<pcl::PointCloud<pcl::PointXYZ> > cloud_clusters;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(int i = 0; i < img.size().height; i++)
    {
        for(int j = 0; j < img.size().width; j++)
        {
            if(img.at<uchar>(i,j) != 0)
            {
                pcl::PointXYZI point;
                point.x = (j - img.size().width/2) * grid_size - offset_x;
                point.y = (i - img.size().height/2) * grid_size - offset_y;
                point.z = 0;
                point.intensity = 0;

                map_cloud->points.push_back(point);
            }
        }
    }
    DBScan(map_cloud, cloud_clusters, Eps, MinPts);

    clusters.resize(cloud_clusters.size());
    for(int i = 0; i < cloud_clusters.size(); i++)
    {
        for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = cloud_clusters[i].points.begin();
            pt < cloud_clusters[i].points.end();
            pt++)
        {
            clusters[i].push_back(Vertex(pt->x, pt->y));
        }
    }
}
