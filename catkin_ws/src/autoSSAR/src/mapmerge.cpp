#include <mapMerge/mapmerge.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/common/projection_matrix.h>


void mergeMaps(pcl::PointCloud<pcl::PointXYZ>& map_in, sensor_msgs::PointCloud2& map_out)
{
    ROS_WARN("Merging maps");
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_in_ptr(new pcl::PointCloud<pcl::PointXYZ>(map_in));
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_out_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_out_ptr_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(map_out, *map_out_ptr_tmp);
    //Remove NAN points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(map_in_ptr, map_in_ptr, indices);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(map_in_ptr);
    icp.setInputTarget(map_out_ptr);
    icp.align(*map_out_ptr_tmp);
    if(icp.hasConverged())
    {
        ROS_WARN("ICP has converged");
        icp.getFitnessScore();
        icp.getFinalTransformation();
        pcl::transformPointCloud(*map_in_ptr, *map_out_ptr_tmp, icp.getFinalTransformation());
        pcl::concatenateFields(*map_out_ptr_tmp, *map_out_ptr, *map_out_ptr);
        pcl::toROSMsg(*map_out_ptr, map_out);
    }
    else
    {
        ROS_WARN("ICP has not converged");
    }
    ROS_WARN("ICP done");
}


