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
#include <std_msgs/String.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <tf/transform_broadcaster.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
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

//placeholder for inRange flag, to be set
bool otherUAV0InRange_  = false;
bool otherUAV1InRange_  = false;
bool finishState = false;


std::string selfUAV;
std::string otherUAV0 = "nan";
std::string otherUAV1 = "nan";

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
    pcl::removeNaNFromPointCloud(*map_in_ptr, *map_in_ptr, indices);
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

void downsample(pcl::PointCloud<pcl::PointXYZ>& cloud_in, pcl::PointCloud<pcl::PointXYZ>& cloud_out)
{
    // std::cout << "Cloud in before downsample: " << cloud_in.size() << std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_in.makeShared());
    sor.setLeafSize (0.1, 0.1, 0.1);
    sor.filter (cloud_out);
    // std::cout << "Cloud in after downsample: " << cloud_out.size() << std::endl;
}


//Callbacks

void getGlobalMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //can only merge if other UAVs are in range
    if(otherUAV0InRange_  == true || otherUAV1InRange_ == true){
        //conversations between octomap and pcl
        pcl::PointCloud<pcl::PointXYZ> cloudMap;
        pcl::fromROSMsg(*msg, cloudMap);

        if(own_globalMap_pcd.size() > 0)
        {
            // ROS_WARN("Own Global map is not empty");
            mergeMaps(cloudMap, own_globalMap_pcd);
            downsample(own_globalMap_pcd, own_globalMap_pcd);
            otherUAV0InRange_ = false;
            otherUAV1InRange_  = false;
        }
        else            
        {
            // ROS_WARN("Own Global map is empty");
            own_globalMap_pcd = cloudMap;
            downsample(own_globalMap_pcd, own_globalMap_pcd);
            otherUAV0InRange_ = false;
            otherUAV1InRange_  = false;
        }
    }
}



void getLocalMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if(!finishState){
        // ROS_WARN("Received local map");
        pcl::PointCloud<pcl::PointXYZ> cloudMap;
        pcl::fromROSMsg(*msg, cloudMap);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(cloudMap, cloudMap, indices);
        if(cloudMap.size() > 0 && own_globalMap_pcd.size() > 0)
        {
            // ROS_WARN("Local map is not empty");
            local_map_pcd = cloudMap;
            own_globalMap_pcd += cloudMap;
            downsample(own_globalMap_pcd, own_globalMap_pcd);
        }   
        else
        {
            // ROS_WARN("Local map is empty");
            local_map_pcd = cloudMap;
            own_globalMap_pcd = cloudMap;
            downsample(own_globalMap_pcd, own_globalMap_pcd);
        }
    }
}

void getWithinRangeCallback(const std_msgs::String& msg){


    std::string str = msg.data;

    // std::cout << "getWithinRangeCallback" << std::endl;
    // std::cout << str << std::endl;
    // std::cout << otherUAV0 << std::endl;
    // std::cout << otherUAV1 << std::endl;


    if( str == otherUAV0 ){
        otherUAV0InRange_ = true;
    }

    if( str == otherUAV1 ){
        otherUAV1InRange_ = true;
    }

}


int main (int argc, char* argv[]){
    ros::init(argc, argv, "map_merger");
    ros::NodeHandle nh;



    selfUAV = nh.getNamespace().c_str();
    nh.getParam(selfUAV+"/mapmerge/otherUAV0", otherUAV0);
    nh.getParam(selfUAV+"/mapmerge/otherUAV1", otherUAV1);

	ROS_WARN("Trying to subscribe");
    ros::Subscriber map_global_own = nh.subscribe(selfUAV+"/sdf_map/occupancy_all", 10, getLocalMapCallback);
    ros::Subscriber map_global_uav1 = nh.subscribe(otherUAV0+"/MergedMap", 10, getGlobalMapCallback);
    ros::Subscriber map_global_uav2 = nh.subscribe(otherUAV1+"/MergedMap", 10, getGlobalMapCallback);
    ros::Subscriber within_range = nh.subscribe(selfUAV+"/within_range", 10, getWithinRangeCallback);
    ROS_WARN("Have subscribed");

    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>(selfUAV+"/MergedMap", 1000);

    ros::Rate loop_rate(20);

    while(ros::ok())
    {
        if(own_globalMap_pcd.size() > 0)
        {
            // Always publish own global map if it is not empty
            Global_Publish.header.frame_id = "/map";
            pcl::toROSMsg(own_globalMap_pcd, Global_Publish);
            map_pub.publish(Global_Publish);

            // // Publish debug
            // pcl::toROSMsg(other0GlobalMap_, Global_Publish);
            // map_pub2.publish(Global_Publish);


        }
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    
    ros::spin();


    return 0;
}