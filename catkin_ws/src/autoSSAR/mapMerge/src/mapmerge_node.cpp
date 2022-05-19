#include "ros/ros.h"
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/features/feature.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/Odometry.h>
#include "mapmerge/coverage.h"
#include <Eigen/SVD>
#include <pcl/features/normal_3d_omp.h>
#include <algorithm>
#include <pcl/impl/point_types.hpp>


pcl::PointCloud<pcl::PointXYZ> own_globalMap_pcd;
// pcl::PointCloud<pcl::PointXYZ> received_map_;
size_t last_point_cloud_size_ = 0;
sensor_msgs::PointCloud2 Global_Publish;
//placeholder for inRange flag, to be set
bool otherUAV0InRange_  = false;
bool otherUAV1InRange_  = false;
bool finishState = false;
bool recentlyMerged_ = false;


std::string selfUAV;
std::string otherUAV0 = "nan";
std::string otherUAV1 = "nan";



//parameters for transform
int max_number_of_iterations_icp_, max_nn_icp_, max_nn_overlap_;
double downsample_leafsize_, epsilon_z_, epsilon_curvature_, epsilon_transformation_, radius_icp_, radius_overlap_;
bool downsample_pointcloud_before_, downsample_pointcloud_after_, filter_outliers_, curvature_check_;
int scan_index_;
pcl::KdTreeFLANN<pcl::PointNormal> kdtree_;

void init(){
    //initialize parameters
    max_number_of_iterations_icp_ = 100;
    max_nn_icp_ = 10;
    max_nn_overlap_ = 10;
    // downsample_leafsize_ = 0.01;
    // epsilon_z_ = 0.01;
    // epsilon_curvature_ = 0.01;
    // epsilon_transformation_ = 0.01;
    // radius_icp_ = 0.1;
    radius_overlap_ = 0.1;
    // downsample_pointcloud_before_ = false;
    // downsample_pointcloud_after_ = false;
    // filter_outliers_ = true;
    // curvature_check_ = true;
    // scan_index_ = 0;
}

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

bool pclSort (pcl::PointNormal i, pcl::PointNormal j)
{
  return (i.x < j.x);
}

bool pclUnique (pcl::PointNormal i, pcl::PointNormal j)
{
  double x_diff = fabs(i.x - j.x);
  double y_diff = fabs(i.y - j.y);
  double z_diff = fabs(i.z - j.z);
  if(x_diff < 0.0001 && y_diff < 0.0001 && z_diff < 0.0001 )
    return 1;
  else
    return 0;
}


void getOverlap(pcl::PointCloud<pcl::PointXYZ> cloud_in, pcl::PointCloud<pcl::PointXYZ> own_cloud, pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp_){
    std::vector<int> nn_indices (max_nn_overlap_);
    std::vector<float> nn_dists (max_nn_overlap_);


    pcl::PointCloud<pcl::PointNormal> cloud_in_copy, own_cloud_copy, overlap_model, overlap_current;
    // Convert to pcl::PointNormal
    copyPointCloud(cloud_in, cloud_in_copy);
    copyPointCloud(own_cloud, own_cloud_copy);
    std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal> >::iterator it;
    kdtree_.setInputCloud (boost::make_shared< pcl::PointCloud < pcl::PointNormal> > (own_cloud_copy));
    for(size_t idx = 0 ; idx < cloud_in.points.size(); idx++ )
    {
      kdtree_.radiusSearch(cloud_in_copy, idx, radius_overlap_, nn_indices, nn_dists, max_nn_overlap_);

      if(nn_indices.size() > 0 )
      {
        overlap_current.points.push_back(cloud_in_copy.points[idx]);
              for(size_t i = 0 ; i < nn_indices.size(); i++)
        {
          overlap_model.points.push_back (kdtree_.getInputCloud()->points[nn_indices[i]]);
        }
      }
    }
    
    // Remove excess overlaps
    std::sort(overlap_model.points.begin(), overlap_model.points.end(), pclSort);
    it = std::unique(overlap_model.points.begin(), overlap_model.points.end(), pclUnique);
    overlap_model.points.resize(it - overlap_model.points.begin());

    // Buffers for alignment
    pcl::PointCloud<pcl::PointNormal>::Ptr overlap_model_copy (new pcl::PointCloud<pcl::PointNormal>(overlap_model));
    pcl::PointCloud<pcl::PointNormal>::Ptr overlap_current_copy (new pcl::PointCloud<pcl::PointNormal>(overlap_current));

    //set source and target for alignment
    icp_.setInputTarget(overlap_model_copy);
    icp_.setInputCloud(overlap_current_copy);
}

void mergeMaps(pcl::PointCloud<pcl::PointXYZ>& map_in, pcl::PointCloud<pcl::PointXYZ>& map_out)
{
    // Buffers for alignment
    pcl::PointCloud<pcl::PointXYZ> Final;
    pcl::PointCloud<pcl::PointNormal> Final_normal;
    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;


    //Find overlap and remove excess
    getOverlap(map_in, map_out, icp);
    icp.align(Final_normal);
    if(icp.hasConverged())
    {
        icp.getFitnessScore();

        // convert to pcl::PointCloud<pcl::PointXYZ>
        copyPointCloud(Final_normal, Final);
        pcl::transformPointCloud(map_in, Final, icp.getFinalTransformation());
        map_out += Final;
    }
    else
    {
        ROS_WARN("ICP has not converged");
    }
}

void downsample(pcl::PointCloud<pcl::PointXYZ>& cloud_in, pcl::PointCloud<pcl::PointXYZ>& cloud_out)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_in.makeShared());
    sor.setLeafSize (0.1, 0.1, 0.1);
    sor.filter (cloud_out);
}


//Callbacks
void getGlobalMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if(!finishState){
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
            pcl::PointCloud<pcl::PointXYZ> cloudMap;
            pcl::fromROSMsg(*msg, cloudMap);
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(cloudMap, cloudMap, indices);
            if(cloudMap.size() > 0 && own_globalMap_pcd.size() > 0)
            {
                // Add new map data
                own_globalMap_pcd += cloudMap;
                downsample(own_globalMap_pcd, own_globalMap_pcd);
            }   
            else
            {
                // If global map empty set local map as global map
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
    if(recentlyMerged)
    {
        recentlyMerged = false;
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

    // Timer to avoid constant merging
    ros::Timer merge_timer = nh.createTimer(ros::Duration(2), mergeTimerCallback);
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
            }

            if(!recentlyMerged_)
            {
                if(otherUAV0InRange_)
                {
                    // ROS_WARN("Other UAV0 is in range");
                    pcl::toROSMsg(own_globalMap_pcd, Global_Publish);
                    other_pub.publish(Global_Publish);
                    otherUAV0InRange_ = false;
                    recentlyMerged_1 = true;
                }

                if(otherUAV1InRange_)
                {
                    // ROS_WARN("Other UAV1 is in range");
                    pcl::toROSMsg(own_globalMap_pcd, Global_Publish);
                    other_pub.publish(Global_Publish);
                    otherUAV1InRange_ = false;
                    recentlyMerged_2 = true;
                }
            }
        }  
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    
    ros::spin();


    return 0;
}