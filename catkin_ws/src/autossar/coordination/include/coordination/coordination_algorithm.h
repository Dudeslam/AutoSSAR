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


enum COORD_STATE { EXPLORE, MEET, SACRIFICE, RELAY, FINISH, DEAD, DONE};

struct UAVdata {
    std::string name;
    std::string role;
    nav_msgs::Odometry relayPoint;
    //std::vector<nav_msgs::Odometry> odom_record;    // array for all positions
    int id;
    //bool inRange;


    // For compairing
    public :
    bool operator==( const UAVdata& other ) {
        return name == other.name;
            //    && role    == other.role
            //    && relayPoint  == other.relayPoint
            //    && id    == other.id;
    }
    bool operator!=( const UAVdata& other ) {
        return !(name != other.name);
    }
    // Use default for =
    // bool operator=( const UAVdata& other ) {
    //     return name = other.name
    //            && role    = other.role
    //            && relayPoint  = other.relayPoint
    //            && id    = other.id;
    // }
};



class coordinationAlgorithm {
private:

    // Ptr to get/set frontiers???
    //shared_ptr<FastPlannerManager> planner_manager_;
    // Var for tmp shared map
    pcl::PointCloud<pcl::PointXYZ> sharedPointCloud_;
    
    // Internal state var
    UAVdata selfUAV_, nearUAV_, pairedUAV_;

    // ROS utils
    //ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
    ros::Subscriber within_range_sub_, battery_sub_, odom_sub_;
    ros::Publisher cmd_pub_;


    COORD_STATE state_;
    double batteryCapasity_;
    double distTraversed_;
    double rangeLeft_;
    double DEBUG_VAR;

    bool batteryHalfFlag_;
    bool batteryEmptyFlag_;
    bool nearUAVFlag_;
    bool atRelayPointFlag_;
    bool timerExpiredFlag_;
    bool timerRunningFlag_;
    
    nav_msgs::Odometry baseStationOdom_;
    nav_msgs::Odometry currentOdom_;

    ros::Timer run_timer_, timeoutTimer_; // Timer for triggering
    std::vector<std::string> coord_state_str_ = { "EXPLORE", "MEET", "SACRIFICE", "RELAY", "FINISH", "DEAD", "DONE" };

    // Util callbacks
    //void timerCallback(const ros::TimerEvent& e);
    void batteryCallback(const std_msgs::String& msg);
    void withinRangeCallback(const std_msgs::String& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // helper functions     (new state, whoCalled)
    void triggerTimer(const ros::TimerEvent& e);
    void transitState(COORD_STATE new_state, std::string pos_call);
    void evaluateRoles(void);

public:
    coordinationAlgorithm(/* args */) {
    }
    ~coordinationAlgorithm() {
    }
    void init(ros::NodeHandle& nh);
    void runCoordinationAlgorithm(const ros::TimerEvent& e);

};


#endif//_COORDINATION_ALGORITHM_