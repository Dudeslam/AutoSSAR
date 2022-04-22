// #include "include/mapmerge.h"
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


using namespace std;

pcl::PointCloud<pcl::PointXYZ> own_globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> local_map_pcd;
sensor_msgs::PointCloud2 rcv_globalMap_pcd;
sensor_msgs::PointCloud2 globalMap_pcd;


void getMapCallback(const octomap_msgs::octomap msg)
{
    //conversations between octomap and pcl
    ROS_WARN("Received map");
    sensor_msgs::PointCloud2 temp_pcd;
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    octomap_msgs::octomaptoPointCloud2(msg, temp_pcd);
    pcl::fromROSMsg(temp_pcd, cloudMap);
    //merge maps
    mergeMaps(cloudMap);
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


void mergeMaps(pcl::PointCloud<pcl::PointXYZ>& cloudMap)
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
    pcl::toROSMsg(own_globalMap_pcd, globalMap_pcd);
    globalMap_pcd.header.frame_id = "map";

}


int main (int argc, char** argv){
    ros::init(argc, argv, "map_merger");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //Pub Subs
    ros::Subscriber map_local = nh.subscribe("/pcl_render_node/local_map", 1, getLocalMapCallback);
    ros::Subscriber map_global = nh.subscribe("/octomap_full", 1, getMapCallback);
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_merged", 1);
    //merge local received map with own global map

    //Publish merged map
    pcl::toROSMsg(own_globalMap_pcd, globalMap_pcd);
    
    ROS_WARN("Publishing map");
    map_pub.publish(globalMap_pcd);

    ros::Rate loop_rate(10);
    ros::spin();
    return 0;
}