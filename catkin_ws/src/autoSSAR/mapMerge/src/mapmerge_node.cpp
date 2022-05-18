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
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include "coverage.h"

pcl::PointCloud<pcl::PointXYZ> own_globalMap_pcd;
// pcl::PointCloud<pcl::PointXYZ> received_map_;
size_t last_point_cloud_size_ = 0;
sensor_msgs::PointCloud2 Global_Publish;
geometry_msgs::PoseStamped UAV_pose;
geometry_msgs::PoseStamped Global_Pose;

//placeholder for inRange flag, to be set
bool otherUAV0InRange_  = false;
bool otherUAV1InRange_  = false;
bool finishState = false;
bool recentlyMerged_1, recentlyMerged_2 = false;


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
    if(!finishState){
            // ROS_WARN("Received local map");
            pcl::PointCloud<pcl::PointXYZ> cloudMap;
            pcl::fromROSMsg(*msg, cloudMap);
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(cloudMap, cloudMap, indices);
            if(cloudMap.size() != last_point_cloud_size_){
                mergeMaps(cloudMap, own_globalMap_pcd);
                downsample(own_globalMap_pcd, own_globalMap_pcd);
                last_point_cloud_size_ = cloudMap.size();
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
            own_globalMap_pcd += cloudMap;
            downsample(own_globalMap_pcd, own_globalMap_pcd);
        }   
        else
        {
            // ROS_WARN("Local map is empty");
            own_globalMap_pcd = cloudMap;
            downsample(own_globalMap_pcd, own_globalMap_pcd);
        }
    }
}

void getWithinRangeCallback(const nav_msgs::Odometry::ConstPtr& msg){
    if((*msg).child_frame_id == ""){return;}

    if((*msg).child_frame_id == otherUAV0 ){
        otherUAV0InRange_ = true;
    }

    if( (*msg).child_frame_id == otherUAV1 ){
        otherUAV1InRange_ = true;
    }

}

void getFinishCallback(const std_msgs::String& msg){
    std::string str = msg.data;
    if(str == "finish"){
        finishState = true;
    }
}

void mergeTimerCallback(const ros::TimerEvent& event){
    if(recentlyMerged_1)
    {
        recentlyMerged_1 = false;
    }
    if(recentlyMerged_2)
    {
        recentlyMerged_2 = false;
    }
}


int main (int argc, char* argv[]){
    ros::init(argc, argv, "map_merger");
    ros::NodeHandle nh;

    coverage mapcover;
    mapcover.init(nh);

    selfUAV = nh.getNamespace().c_str();
    nh.getParam(selfUAV+"/mapmerge/otherUAV0", otherUAV0);
    nh.getParam(selfUAV+"/mapmerge/otherUAV1", otherUAV1);

	ROS_WARN("Trying to subscribe");
    ros::Subscriber map_global_own = nh.subscribe(selfUAV+"/sdf_map/occupancy_all", 10, getLocalMapCallback);
    ros::Subscriber map_global_uav1 = nh.subscribe(otherUAV0+"/MergedMap", 10, getGlobalMapCallback);
    ros::Subscriber map_global_uav2 = nh.subscribe(otherUAV1+"/MergedMap", 10, getGlobalMapCallback); 
    ros::Subscriber within_range = nh.subscribe(selfUAV+"/within_range", 10, getWithinRangeCallback);
    ros::Subscriber finish = nh.subscribe(selfUAV+"/planning/state", 10, getFinishCallback);
    ROS_WARN("Have subscribed");

    ros::Publisher other_pub = nh.advertise<sensor_msgs::PointCloud2>(selfUAV+"/MergedMap", 10);
    ros::Publisher own_publish = nh.advertise<sensor_msgs::PointCloud2>(selfUAV+"/pcl_render_node/cloud", 10);
    ros::Timer merge_timer = nh.createTimer(ros::Duration(2), mergeTimerCallback);
    // ros::Publisher debugger_own = nh.advertise<sensor_msgs::PointCloud2>(selfUAV+"/debugger/cloud", 1000);
    ros::Rate loop_rate(20);
    Global_Publish.header.frame_id = "/map";
    
    while(ros::ok())
    {
        if(own_globalMap_pcd.size() > 0)
        {
            if(!finishState){
                // Always publish own global map if it is not empty
                pcl::toROSMsg(own_globalMap_pcd, Global_Publish);
                own_publish.publish(Global_Publish);
                // debugger_own.publish(Global_Publish);
                // ROS_WARN("Publish own global map");
            }


                if(otherUAV0InRange_ && !recentlyMerged_1)
                {
                    // ROS_WARN("Other UAV0 is in range");
                    pcl::toROSMsg(own_globalMap_pcd, Global_Publish);
                    other_pub.publish(Global_Publish);
                    otherUAV0InRange_ = false;
                    recentlyMerged_1 = true;
                }

                if(otherUAV1InRange_ && !recentlyMerged_2)
                {
                    // ROS_WARN("Other UAV1 is in range");
                    pcl::toROSMsg(own_globalMap_pcd, Global_Publish);
                    other_pub.publish(Global_Publish);
                    otherUAV1InRange_ = false;
                    recentlyMerged_2 = true;
                }
        }
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    
    ros::spin();


    return 0;
}