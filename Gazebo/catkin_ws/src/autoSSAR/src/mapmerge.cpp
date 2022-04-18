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


void getMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    rcv_globalMap_pcd = *msg;
    cloudMap.clear();
    pcl::fromROSMsg(rcv_globalMap_pcd, cloudMap);
}

int main (int argc, char** argv)){
    ros::init(argc, argv, "map_merger");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Subscriber map_sub = nh.subscribe("/map_merger/global_cloud", 1, getMapCallback);
    ROSWARN("Subscribed");

    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_merged", 1);
    ros::Rate loop_rate(10);

    return 0;
}