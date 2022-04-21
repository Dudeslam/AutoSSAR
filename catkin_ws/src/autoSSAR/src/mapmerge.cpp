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

pcl::PointCloud<pcl::PointXYZ> own_globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> local_map_pcd;
sensor_msgs::PointCloud2 rcv_globalMap_pcd;
sensor_msgs::PointCloud2 globalMap_pcd;


void getMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ROS_WARN("Received map");
    sensor_msgs::PointCloud2 localMap_pcd = *msg;
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::fromROSMsg(localMap_pcd, cloudMap);
    //merge maps

}


void getLocalMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ROS_WARN("Received local map");
    sensor_msgs::PointCloud2 localMap_pcd = *msg;
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::fromROSMsg(localMap_pcd, cloudMap);
    Eigen::Transform<Scalar, 3, Eigen::Affine> cloudMap_mat(received.sensor_origin)
    local_map_pcd += cloudMap;
    own_globalMap_pcd += cloudMap;
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