
#include <mapMerge/mapmerge.h>
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
sensor_msgs::PointCloud2 globalMap_pcd2;

void getGlobalMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //conversations between octomap and pcl
    ROS_WARN("Received map");
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::fromROSMsg(*msg, cloudMap);
    pcl::toPCLPointCloud2(own_globalMap_pcd, globalMap_pcd2);
    //merge maps
    mapMerge::mergeMaps(cloudMap, globalMap_pcd2);
    pcl::fromROSMsg(globalMap_pcd2, own_globalMap_pcd);
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
    pcl::toROSMsg(own_globalMap_pcd, globalMap_pcd2);
    
    ROS_WARN("Publishing map");
    map_pub.publish(globalMap_pcd2);

    ros::Rate loop_rate(10);
    ros::spin();
    return 0;
}