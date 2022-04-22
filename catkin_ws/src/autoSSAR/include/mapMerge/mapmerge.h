#ifndef AUTOSSAR_MAPMERGE_H
#define AUTOSSAR_MAPMERGE_H


#include <ros/ros.h>
#include <string>

namespace mapMerge
{
    void mergeMaps(pcl::PointCloud<pcl::PointXYZ>& cloudMap);


}

#endif //AUTOSSAR_MAPMERGE_H