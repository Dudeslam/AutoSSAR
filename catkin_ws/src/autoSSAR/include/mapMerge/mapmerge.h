#ifndef AUTOSSAR_MAPMERGE_H
#define AUTOSSAR_MAPMERGE_H


#include <ros/ros.h>
#include <string>
#include <pcl/common/projection_matrix.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/common/projection_matrix.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>
  

namespace mapMerge
{
    void mergeMaps(pcl::PointCloud<pcl::PointXYZ>& map_in, sensor_msgs::PointCloud2::ConstPtr& map_out);


}

#endif //AUTOSSAR_MAPMERGE_H