#include "mapmerge/coverage.h"

void coverage::init(ros::NodeHandle& nh) {

    selfUAV = nh.getNamespace().c_str();

    std::cout << "*************************************************************" << std::endl;
    std::cout << "Self: "<< selfUAV << std::endl;
    std::cout << "*************************************************************" << std::endl;

    Globalmap_size = nh.subscribe("/map_generator/global_cloud", 1, &coverage::mapSize_callback, this);
    UAV0MapSize_sub_ = nh.subscribe(selfUAV + "/MergedMap", 1, &coverage::mergedMapSize_callback, this);
}

void coverage::mapSize_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::fromROSMsg(*msg, cloudMap);
    Globalmap_size = cloudMap.size();
    std::cout << "Map Size: " << Globalmap_size << std::endl;
}

void coverage::MergedMapSize_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::fromROSMsg(*msg, cloudMap);
    SelfMapSize = cloudMap.size();
    std::cout << "Map Size: " << SelfMapSize << std::endl;
}