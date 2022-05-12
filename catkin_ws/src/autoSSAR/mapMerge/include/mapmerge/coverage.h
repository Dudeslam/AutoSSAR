#ifndef _COVERAGE_H_
#define _COVERAGE_H_

#include <ros/ros.h>
#include <nav_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <string>

class cooverage {
    private:
        int UAV0_Map_size_, UAV1_Map_size_, UAV2_Map_size_;
        int Globalmap_size;
        string UAV0_name, UAV1_name, UAV2_name;


        //Subscribers
        ros::Subscriber GlobalmapSize_sub_, UAV0MapSize_sub_, UAV1MapSize_sub_, UAV2MapSize_sub_;
        
        // Publishers
        ros::Publisher coverage_pub_;

        // Callback functions
        void mapSize_callback(const std_msgs::String::ConstPtr& msg);
        void mergedMapSize_callback(const std_msgs::String::ConstPtr& msg);

    public:
        coverage(){

        };
        ~coverage(){
                
        };
    void init(ros::NodleHandle& nh);


};



#endif // _COVERAGE_H_