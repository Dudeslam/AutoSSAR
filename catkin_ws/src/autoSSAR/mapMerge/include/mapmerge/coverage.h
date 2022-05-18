#ifndef _COVERAGE_H_
#define _COVERAGE_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <string.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
class coverage {
    private:
        size_t SelfMapSize;
        size_t Globalmap_size;
        std::string selfUAV;


        //Subscribers
        ros::Subscriber GlobalmapSize_sub_, selfUAVMapSize_sub_;
        
        // Publishers
        ros::Publisher coverage_pub_;


        ros::Timer timer_;
        // Callback functions
        void mapSize_callback(const std_msgs::String::ConstPtr& msg);
        void mergedMapSize_callback(const std_msgs::String::ConstPtr& msg);

        void mapCoveredCallback();

    public:
        coverage(){

        }
        ~coverage(){
                
        }
    void init(ros::NodeHandle& nh);


};



#endif // _COVERAGE_H_