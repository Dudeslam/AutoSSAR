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
#include <Eigen/Eigen>
#include <random>


using namespace std;

sensor_msgs::PointCloud2 own_globalMap_pcd;
sensor_msgs::PointCloud2 rcv_globalMap_pcd;
sensor_msgs::PointCloud2 localMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;


void getLocalMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    localMap_pcd = *msg;
    cloudMap.clear();
    pcl::fromROSMsg(rcv_globalMap_pcd, cloudMap);
}

void getGlobalMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    rcv_globalMap_pcd = *msg;
    cloudMap.clear();
    pcl::fromROSMsg(own_globalMap_pcd, cloudMap);
}

void mergeMaps(sensor_msgs::PointCloud2 own, sensor_msgs::PointCloud2 recv)
{
    if(own != 0){
        pcl::PointCloud<pcl::PointXYZ> cloudOwn;
        pcl::PointCloud<pcl::PointXYZ> cloudRecv;
        pcl::PointCloud<pcl::PointXYZ> cloudMerged;

        cloudOwn.clear();
        cloudRecv.clear();
        cloudMerged.clear();

        pcl::fromROSMsg(own, cloudOwn);
        pcl::fromROSMsg(recv, cloudRecv);

        cloudMerged = cloudOwn + cloudRecv;
        own_globalMap_pcd = cloudMerged;
        rcv_globalMap_pcd.clear();
    }
    else(
        own_globalMap_pcd = recv;
    )
}




int main (int argc, char** argv){
    ros::init(argc, argv, "map_merger");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //Local map subscriber
    ros::Subscriber map_sub = nh.subscribe("/pcl_render_node/local_map", 1, getLocalMapCallback);
    ros::Subscriber map_sub2 = nh.subscribe("/map_generator/global_cloud", 1, getGlobalMapCallback);
    //merge local received map with own global map
    if(localMap_pcd.data.size() == 0){
        ROS_WARN("No map received");
    }
    else{
        ROS_WARN("Map received");
        mergeMaps(own_globalMap_pcd, localMap_pcd);
    }

    //Glocal map subscriber

    // ROS_WARN("Subscribed");
    if(rcv_globalMap_pcd.data.size() == 0){
        ROS_WARN("No map received");
    }
    else{
        ROS_WARN("Map received");
        mergeMaps(own_globalMap_pcd, rcv_globalMap_pcd);
    }


    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_merged", 1);
    ros::Rate loop_rate(10);

    return 0;
}