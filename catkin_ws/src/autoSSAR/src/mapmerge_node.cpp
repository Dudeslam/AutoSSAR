#include "ros/ros.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/common/projection_matrix.h>

pcl::PointCloud<pcl::PointXYZ> own_globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> local_map_pcd;
sensor_msgs::PointCloud2 rcv_globalMap_pcd2;
sensor_msgs::PointCloud2* globalMap_pcd2;

void getGlobalMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //GOBack
    //conversations between octomap and pcl
    ROS_WARN("Received map");
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::fromROSMsg(*msg, cloudMap);
    pcl::toROSMsg(own_globalMap_pcd, *globalMap_pcd2);
    //merge maps
    mergeMaps(cloudMap, *globalMap_pcd2);
    pcl::fromROSMsg(*globalMap_pcd2, own_globalMap_pcd);
    ROS_WARN("Merged with own map");
}



void getLocalMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ROS_WARN("Received local map");
    sensor_msgs::PointCloud2 localMap_pcd = *msg;
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::fromROSMsg(localMap_pcd, cloudMap);
    local_map_pcd += cloudMap;
    own_globalMap_pcd += cloudMap;
}

void mergeMaps(pcl::PointCloud<pcl::PointXYZ>& map_in, sensor_msgs::PointCloud2 map_out)
{
    ROS_WARN("Merging maps");
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_in_ptr(new pcl::PointCloud<pcl::PointXYZ>(map_in));
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_out_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_out_ptr_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(map_out, *map_out_ptr_tmp);
    //Remove NAN points
    std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*map_in_ptr, *map_in_ptr, indices);

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


int main (int argc, char* argv[]){
    ros::init(argc, argv, "map_merger");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    std::string cloud_topic;
    std::string Publish_topic;

    // Parameter Handles
    nh.param("Cloud_in", cloud_topic);
    nh.param("Publish_out", Publish_topic);
    //Pub Subs
    ros::Subscriber map_local = nh.subscribe(cloud_topic, 1, getLocalMapCallback);
    ros::Subscriber map_global = nh.subscribe("/octomap_full", 1, getGlobalMapCallback);
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>(Publish_topic, 1);
    //merge local received map with own global map

    //Publish merged map
    pcl::toROSMsg(own_globalMap_pcd, *globalMap_pcd2);
    
    ROS_WARN("Publishing map");
    map_pub.publish(*globalMap_pcd2);

    ros::Rate loop_rate(10);
    ros::spin();
    return 0;
}