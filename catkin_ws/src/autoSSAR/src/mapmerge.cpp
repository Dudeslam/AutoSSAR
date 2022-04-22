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


void mergeMaps(pcl::PointCloud<pcl::PointXYZ>& map_in, octomap_msgs::Octomap map_out)
{
    ROS_WARN("Merging maps");

    //Remove NAN points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloudMap, cloudMap, indices);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloudMap);
    icp.setInputTarget(own_globalMap_pcd);
    icp.align(cloudMap);
    if(icp.hasConverged())
    {
        ROS_WARN("ICP has converged");
        icp.getFitnessScore();
        icp.getFinalTransformation();
        own_globalMap_pcd = cloudMap;
    }
    else
    {
        ROS_WARN("ICP has not converged");
    }
    ROS_WARN("ICP done");
    //convert to ROS message
    pcl::toROSMsg(map_out, globalMap_pcd);
    globalMap_pcd.header.frame_id = "map";

}


