#ifndef _COORDINATION_ALGORITHM_
#define _COORDINATION_ALGORITHM_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <string>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>



enum COORD_STATE { EXPLORE, MEET, SACRIFICE, RELAY, FINISH };

class coordinationAlgorithm {
private:

    // Ptr to get/set frontiers???
    //shared_ptr<FastPlannerManager> planner_manager_;
    // Var for tmp shared map
    pcl::PointCloud<pcl::PointXYZ> sharedPointCloud_;
    
    // Internal state var
    COORD_STATE state_;
    std::string selfUAV_;


    // ROS utils
    //ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
    ros::Subscriber within_range_, battery_;
    double batteryCapasity_;
    double rangeLeft_;
    bool otherUAV0InRange_;
    bool otherUAV1InRange_;
    //ros::Publisher replan_pub_, new_pub_, bspline_pub_;

    // Util callbacks
    //void timerCallback(const ros::TimerEvent& e);
    void batteryCallback(const nav_msgs::PathConstPtr& msg);
    void withinRangeCallback(const nav_msgs::PathConstPtr& msg);

    // helper functions
    //int callExplorationPlanner();
    void transitState(COORD_STATE new_state, std::string pos_call);


public:
    coordinationAlgorithm(/* args */) {
    }
    ~coordinationAlgorithm() {
    }
    void init(ros::NodeHandle& nh);

};


#endif//_COORDINATION_ALGORITHM_