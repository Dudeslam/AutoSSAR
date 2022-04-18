// #include <iostream>
// using std::cout;
// using std::endl;
// using std::string;

#include <ros/ros.h>
#include <string>


class Distances {
    public:
    Distances(ros::NodeHandle nh;, std::string robotNS...){   //Variadic arguments
        sub_m = nh.subscribe(robotNS+"/odom", 1000, &Distances::subCallback, this);
        //pub_m = nh.advertise;
    }
    void subCallback(const std_msgs::String::ConstPtr& msg)


    protected:
    ros::Subscriber sub_m;
    //ros::Publisher pub_m;
    ros::NodeHandle nh;
}