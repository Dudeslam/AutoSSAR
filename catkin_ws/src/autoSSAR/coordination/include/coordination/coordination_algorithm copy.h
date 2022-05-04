#ifndef _COORDINATION_ALGORITHM_
#define _COORDINATION_ALGORITHM_

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


enum COORD_STATE { EXPLORE, MEET, SACRIFICE, RELAY, FINISH, DEAD};

struct otherUAVdata {
    std::string name;
    std::string role;
    nav_msgs::Odometry relayPoint;
    //std::vector<nav_msgs::Odometry> odom_record;    // array for all positions
    int id;
    bool inRange;
};



class coordinationAlgorithm {
private:

    // Ptr to get/set frontiers???
    //shared_ptr<FastPlannerManager> planner_manager_;
    // Var for tmp shared map
    pcl::PointCloud<pcl::PointXYZ> sharedPointCloud_;
    
    // Internal state var
    std::string selfUAV_;

    // ROS utils
    //ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
    ros::Subscriber within_range_sub_, battery_sub_, odom_sub_;


    COORD_STATE state_;
    double batteryCapasity_;
    double distTraversed_;
    double rangeLeft_;
    bool batteryHalfFlag_;
    otherUAVdata nearUAV_;                          // UAV which is within range
    std::vector<otherUAVdata> UAVsEncountered_;     // array for all UAVs encountered
    nav_msgs::Odometry homeOdom_;
    nav_msgs::Odometry currentOdom_;

    ros::Timer meet_timer_;                         // Timer for triggering MEET
    std::vector<std::string> coord_state_str_ = { "EXPLORE", "MEET", "SACRIFICE", "RELAY", "FINISH", "DEAD" };

    // Util callbacks
    //void timerCallback(const ros::TimerEvent& e);
    void batteryCallback(const std_msgs::String& msg);
    void withinRangeCallback(const std_msgs::String& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // helper functions     (new state, whoCalled)
    void triggerMeet(const ros::TimerEvent& e);
    void transitState(COORD_STATE new_state, std::string pos_call);
    void updateRole(void);
    void updateNearUAV(void);

public:
    coordinationAlgorithm(/* args */) {
    }
    ~coordinationAlgorithm() {
    }
    void init(ros::NodeHandle& nh);

};


#endif//_COORDINATION_ALGORITHM_