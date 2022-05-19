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
#include <std_msgs/Float64.h>
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
#include <nav_msgs/Odometry.h>


pcl::PointCloud<pcl::PointXYZ> ownGlobalMap_;
//pcl::PointCloud<pcl::PointXYZ> otherGlobalMap_;

pcl::PointCloud<pcl::PointXYZ> other0GlobalMap_;
pcl::PointCloud<pcl::PointXYZ> other1GlobalMap_;
sensor_msgs::PointCloud2 Global_Publish;

std::string selfUAV;
std::string otherUAV0 = "nan";
std::string otherUAV1 = "nan";

//placeholder for inRange flag, to be set
bool otherUAV0InRange_ = false;
bool otherUAV1InRange_ = false;

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
    //pcl::removeNaNFromPointCloud(*map_in_ptr, *map_in_ptr, indices);
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
    //std::cout << "Cloud in before downsample: " << cloud_in.size() << std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_in.makeShared());
    sor.setLeafSize (0.1, 0.1, 0.1);
    sor.filter (cloud_out);
    //std::cout << "Cloud in after downsample: " << cloud_out.size() << std::endl;
}


//Callbacks
// void getOtherGlobalCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
//     pcl::PointCloud<pcl::PointXYZ> cloudMap;
//     pcl::fromROSMsg(*msg, cloudMap);
//     std::vector<int> indices;
//     pcl::removeNaNFromPointCloud(cloudMap, cloudMap, indices);
//     otherGlobalMap_ = cloudMap;
//     downsample(otherGlobalMap_, otherGlobalMap_);
// }

void getOtherUAV0GlobalCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::fromROSMsg(*msg, cloudMap);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloudMap, cloudMap, indices);
    other0GlobalMap_ = cloudMap;
    downsample(other0GlobalMap_, other0GlobalMap_);
}

void getOtherUAV1GlobalCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::fromROSMsg(*msg, cloudMap);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloudMap, cloudMap, indices);
    other1GlobalMap_ = cloudMap;
    downsample(other1GlobalMap_, other1GlobalMap_);
}

void getOwnGlobalCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::fromROSMsg(*msg, cloudMap);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloudMap, cloudMap, indices);
    ownGlobalMap_ = cloudMap;
    downsample(ownGlobalMap_, ownGlobalMap_);
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




int main (int argc, char* argv[]){
    ros::init(argc, argv, "map_merger");
    ros::NodeHandle nh;



    selfUAV = nh.getNamespace().c_str();
    nh.getParam(selfUAV+"/mapmerge/otherUAV0", otherUAV0);
    nh.getParam(selfUAV+"/mapmerge/otherUAV1", otherUAV1);
  
    std::cout << "*************************************************************" << std::endl;
    std::cout << "MapMerge Node" << std::endl;
    std::cout << selfUAV << std::endl;
    std::cout << otherUAV0 << std::endl;
    std::cout << otherUAV1 << std::endl;
    std::cout << "*************************************************************" << std::endl;

	ROS_WARN("Trying to subscribe");
    ros::Subscriber map_global_own = nh.subscribe(selfUAV+"/sdf_map/occupancy_all", 10, getOwnGlobalCallback);
    ros::Subscriber map_global_uav1 = nh.subscribe(otherUAV0+"/map_Global", 10, getOtherUAV0GlobalCallback);
    ros::Subscriber map_global_uav2 = nh.subscribe(otherUAV1+"/map_Global", 10, getOtherUAV1GlobalCallback);

    ros::Subscriber within_range = nh.subscribe(selfUAV+"/within_range", 10, getWithinRangeCallback);

    ros::Publisher map_pub_own_global = nh.advertise<sensor_msgs::PointCloud2>(selfUAV+"/map_Global", 10);
    ros::Publisher map_pub_other0_debug = nh.advertise<sensor_msgs::PointCloud2>(selfUAV+"/debugOther0Map", 10);
    ros::Publisher map_pub_other1_debug = nh.advertise<sensor_msgs::PointCloud2>(selfUAV+"/debugOther1Map", 10);
    ros::Publisher pubSize = nh.advertise<std_msgs::Float64>(selfUAV+"/mapSize", 10);

    ros::Rate loop_rate(20);
    Global_Publish.header.frame_id = "/map";


    while(ros::ok()) {

        //ROS_INFO_STREAM_THROTTLE(1.0, "" << selfUAV << " map size: " << ownGlobalMap_.size() << " points: " << ownGlobalMap_.points.size() );
        std_msgs::Float64 mapSize;
        mapSize.data = ownGlobalMap_.size();
        pubSize.publish( mapSize );


        if(otherUAV0InRange_){
            // Publish own
            pcl::toROSMsg(ownGlobalMap_, Global_Publish);
            map_pub_own_global.publish(Global_Publish);
            // Merge
            mergeMaps(ownGlobalMap_, other0GlobalMap_);
            // Reset flag
            otherUAV0InRange_ = false;


            // Publish debug
            pcl::toROSMsg(other0GlobalMap_, Global_Publish);
            map_pub_other0_debug.publish(Global_Publish);
        }



        if(otherUAV1InRange_){
            // Publish own
            pcl::toROSMsg(ownGlobalMap_, Global_Publish);
            map_pub_own_global.publish(Global_Publish);
            // Merge
            mergeMaps(ownGlobalMap_, other1GlobalMap_);
            // Reset flag
            otherUAV1InRange_ = false;

            // Publish debug
            pcl::toROSMsg(other1GlobalMap_, Global_Publish);
            map_pub_other1_debug.publish(Global_Publish);
        }



        ros::spinOnce();
        loop_rate.sleep();
    }
    


    ros::spin();
    return 0;
}