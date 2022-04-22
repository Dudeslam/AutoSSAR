#ifndef AUTOSSAR_MAPMERGE_H
#define AUTOSSAR_MAPMERGE_H


#include <ros/ros.h>
#include <string>
#include <pcl/common/projection_matrix.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

namespace mapMerge
{
    void mergeMaps(pcl::PointCloud<pcl::PointXYZ>& map_in, sensor_msgs::PointCloud2& map_out);


}

#endif //AUTOSSAR_MAPMERGE_H