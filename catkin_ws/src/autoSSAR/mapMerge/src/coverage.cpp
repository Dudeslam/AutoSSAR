#include "mapmerge/coverage.h"

void coverage::init(ros::NodeHandle& nh) {

    selfUAV = nh.getNamespace().c_str();

    std::cout << "*************************************************************" << std::endl;
    std::cout << "Self: "<< selfUAV << std::endl;
    std::cout << "*************************************************************" << std::endl;

    GlobalmapSize_sub_ = nh.subscribe("/map_generator/global_cloud", 1, &coverage::mapSize_callback, this);
    selfUAVMapSize_sub_ = nh.subscribe(selfUAV + "/sdf_map/occupancy_all", 1, &coverage::mergedMapSize_callback, this);
    
    GlobalMapSize_pub_ = nh.advertise<std_msgs::String>(selfUAV+"/map_generator/global", 1);
    selfUAVMapSize_pub_ = nh.advertise<std_msgs::String>(selfUAV+"/map_generator/local", 1);

    timer_ = nh.createTimer(ros::Duration(2), &coverage::mapCoveredCallback, this);
}

void coverage::mapSize_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::fromROSMsg(*msg, cloudMap);
    Globalmap_size = cloudMap.points.size();
    // std::cout << "Global Map Size: " << Globalmap_size << std::endl;
}

void coverage::mergedMapSize_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::fromROSMsg(*msg, cloudMap);
    SelfMapSize = cloudMap.points.size();
    // std::cout << selfUAV << " Map Size: " << SelfMapSize << std::endl;
}

void coverage::mapCoveredCallback(const ros::TimerEvent& event) {
    if(Globalmap_size != 0 && SelfMapSize != 0) {
        std_msgs::Float64 msg_global, msg_local;
        msg_global.data = Globalmap_size;
        msg_local.data = SelfMapSize;

        GlobalMapSize_pub_.publish(msg_global);
        selfUAVMapSize_pub_.publish(msg_local);
        std::cout << "Global map size: " << Globalmap_size << std::endl;
        std::cout << selfUAV <<" map size: " << SelfMapSize << std::endl;
    }

}