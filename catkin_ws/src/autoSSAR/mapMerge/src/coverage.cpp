#include "mapmerge/coverage.h"

void coverage::init(ros::NodeHandle& nh) {

    selfUAV = nh.getNamespace().c_str();

    std::cout << "*************************************************************" << std::endl;
    std::cout << "Self: "<< selfUAV << std::endl;
    std::cout << "*************************************************************" << std::endl;

    GlobalmapSize_sub_ = nh.subscribe("/map_generator/global_cloud", 1, &coverage::mapSize_callback, this);
    selfUAVMapSize_sub_ = nh.subscribe(selfUAV + "/MergedMap", 1, &coverage::mergedMapSize_callback, this);
    
    coverage_pub_ = nh.advertise<std_msgs::String>(selfUAV+"/map_generator/coverage", 1);

    timer_ = nh.createTimer(ros::Duration(1), &coverage::mapCoveredCallback, this);
}

void coverage::mapSize_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::fromROSMsg(*msg, cloudMap);
    Globalmap_size = cloudMap.size();
    // std::cout << "Global Map Size: " << Globalmap_size << std::endl;
}

void coverage::mergedMapSize_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::fromROSMsg(*msg, cloudMap);
    SelfMapSize = cloudMap.size();
    // std::cout << selfUAV << " Map Size: " << SelfMapSize << std::endl;
}

void coverage::mapCoveredCallback(const ros::TimerEvent& event) {
    if(Globalmap_size != 0 && SelfMapSize != 0) {
        auto cover = static_cast<double>(Globalmap_size / SelfMapSize)*100;
        std_msgs::String msg;
        msg.data = std::to_string(cover);
        coverage_pub_.publish(msg);
        std::cout << "Global map size: " << Globalmap_size << std::endl;
        std::cout << "Self map size: " << SelfMapSize << std::endl;
        std::cout << "Total cover" << cover << "%" << std::endl;
    }

}