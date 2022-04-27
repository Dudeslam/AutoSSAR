#include "ros/ros.h"
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <tf/transform_broadcaster.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/io.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

pcl::PointCloud<pcl::PointXYZ> own_globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> local_map_pcd;
sensor_msgs::PointCloud2 Global_Publish;
geometry_msgs::PoseStamped UAV_pose;
geometry_msgs::PoseStamped Global_Pose;
std::string UAV_frame;

//placeholder for inRange flag, to be set
bool inRange = true;
bool finishState = false;

//functions here
bool concatePCL(pcl::PointCloud<pcl::PointXYZ> cloud1, pcl::PointCloud<pcl::PointXYZ> cloud2, pcl::PointCloud<pcl::PointXYZ>& cloud_out)
{
    // Make the resultant point cloud take the newest stamp
    cloud1.header.stamp = std::max (cloud1.header.stamp, cloud2.header.stamp);

    // libstdc++ (GCC) on calling reserve allocates new memory, copies and deallocates old vector
    // This causes a drastic performance hit. Prefer not to use reserve with libstdc++ (default on clang)
    cloud1.insert (cloud1.end (), cloud2.begin (), cloud2.end ());

    cloud1.width    = cloud1.size ();
    cloud1.height   = 1;
    cloud1.is_dense = cloud1.is_dense && cloud2.is_dense;
    cloud_out = cloud1;
    return true;


}

void mergeMaps(pcl::PointCloud<pcl::PointXYZ>& map_in, pcl::PointCloud<pcl::PointXYZ>& map_out)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_in_ptr(new pcl::PointCloud<pcl::PointXYZ>(map_in));
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_out_ptr(new pcl::PointCloud<pcl::PointXYZ>(map_out));
    pcl::PointCloud<pcl::PointXYZ> Final;
    //pcl::fromROSMsg(map_out, *map_out_ptr_tmp);
    //Remove NAN points
    std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*map_in_ptr, *map_in_ptr, indices);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // ROS_WARN("Set input cloud");
    icp.setInputSource(map_in_ptr);
    icp.setInputTarget(map_out_ptr);
    icp.align(Final);
    if(icp.hasConverged())
    {
        // ROS_WARN("ICP has converged");
        icp.getFitnessScore();
        pcl::transformPointCloud(*map_in_ptr, Final, icp.getFinalTransformation());
        concatePCL(Final, *map_out_ptr, map_out);
    }
    else
    {
        ROS_WARN("ICP has not converged");
    }
}


//Callbacks

void getGlobalMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if(!finishState){
        //conversations between octomap and pcl
        pcl::PointCloud<pcl::PointXYZ> cloudMap;
        pcl::fromROSMsg(*msg, cloudMap);

        //will be removed when local map update works
        UAV_frame = msg->header.frame_id;

        if(own_globalMap_pcd.size() > 0)
        {
            // ROS_WARN("Own Global map is not empty");
            mergeMaps(cloudMap, own_globalMap_pcd);
        }
        else            
        {
            // ROS_WARN("Own Global map is empty");
            own_globalMap_pcd = cloudMap;
        }
    }
}



void getLocalMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if(!finishState){
        // ROS_WARN("Received local map");
        pcl::PointCloud<pcl::PointXYZ> cloudMap;
        pcl::fromROSMsg(*msg, cloudMap);
        UAV_frame = msg->header.frame_id;
        if(cloudMap.size() > 0 && own_globalMap_pcd.size() > 0)
        {
            // ROS_WARN("Local map is not empty");
            local_map_pcd = cloudMap;
            own_globalMap_pcd += cloudMap;
        }
        else
        {
            // ROS_WARN("Local map is empty");
            local_map_pcd = cloudMap;
            own_globalMap_pcd = cloudMap;
        }
    }
}



int main (int argc, char* argv[]){
    ros::init(argc, argv, "map_merger");
    ros::NodeHandle nh;
    // ros::NodeHandle nh_private("~");
    // std::string cloud_local_topic;
    // std::string cloud_global_topic;
    // std::string Publish_topic;

    // // Parameter Handles
    // nh.param("Cloud_in_local", cloud_local_topic);
    // nh.param("Cloud_in_global", cloud_global_topic);
    // nh.param("Publish_out", Publish_topic);
    // //Pub Subs for roslaunch
    // ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>(Publish_topic, 1);
    // ros::Subscriber map_local = nh.subscribe(cloud_local_topic, 1, getLocalMapCallback);
    // ros::Subscriber map_global = nh.subscribe(cloud_global_topic, 1, getGlobalMapCallback);

    // Static pubsub
	ROS_WARN("Trying to subscribe");
    ros::Subscriber map_local = nh.subscribe("/sdf_map/occupancy_all", 10, getLocalMapCallback);
    // ros::Subscriber map_global = nh.subscribe("/sdf_map/occupancy_all", 1, getGlobalMapCallback);
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/MergedMap", 1000);
    ros::Rate loop_rate(10);
    //merge local received map with own global map
	ROS_WARN("Have subscribed");


    while(ros::ok())
    {
        if(own_globalMap_pcd.size() > 0  && inRange == true)
        {
            Global_Publish.header.frame_id = "/map";
            //This to publish Global
			pcl::toROSMsg(own_globalMap_pcd, Global_Publish);

            map_pub.publish(Global_Publish);
            // Global_Publish.clear();
            loop_rate.sleep();
        }
        ros::spinOnce();
        
    }
    
    ros::spin();


    return 0;
}