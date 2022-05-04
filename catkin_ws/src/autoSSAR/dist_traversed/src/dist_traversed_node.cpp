#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>

std::vector<nav_msgs::Odometry> odom_record;    // array for all poses




void getOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  odom_record.push_back(*msg);
  //std::cout << "Seq: " << msg->header.seq << std::endl;
  //std::cout << "Position-> x: [" <<(*msg).pose.pose.position.x<<"], \ty: ["<<(*msg).pose.pose.position.y<<"], \tz: ["<<(*msg).pose.pose.position.z<<"]" << std::endl;
}



// Utility funcion
long double updateTraversedDistance(long double path_length){

    std::vector<nav_msgs::Odometry>::iterator it = odom_record.begin();
    nav_msgs::Odometry last_pose;
    last_pose = *it;
    it++;
    for (; it!=odom_record.end(); ++it) {
        path_length += hypot(  (*it).pose.pose.position.x - last_pose.pose.pose.position.x, (*it).pose.pose.position.y - last_pose.pose.pose.position.y );
        last_pose = *it;
    }

    // reset since we accumulate path_length = to save space
    odom_record.clear();

    return path_length;
}

void printRecord(){
    std::vector<nav_msgs::Odometry>::iterator it = odom_record.begin();
    for (; it!=odom_record.end(); ++it) {
        std::cout << "position-> x: [" <<(*it).pose.pose.position.x<<"], \ty: ["<<(*it).pose.pose.position.y<<"], \tz: ["<<(*it).pose.pose.position.z<<"]" << std::endl;
    }
}


int main(int argc, char **argv){
    ROS_INFO("\ndist_traversed_node started");
    ros::init(argc, argv, "dist_traversed_node");
    ros::NodeHandle nh;

    // For .launch file         // For manual run OBS comment overwrite 2lines below
    std::string selfUAV;        //="/UAV0";
    selfUAV = nh.getNamespace().c_str();

    ros::Subscriber sub = nh.subscribe(selfUAV+"/state_ukf/odom", 100, getOdomCallback);
    ros::Publisher pub = nh.advertise<std_msgs::String>("dist_traversed", 100);

    // For nice printin
    std::cout << std::setfill ('0') << std::setw (6);
    //std::cout << std::setprecision(4) << std::fixed;

    // Set rate
    ros::Rate r(1); // 1 hz

    // accumulated distance
    long double path_length = 0.0;

    while ( ros::ok() ) {
        // If there is anything to publish
        if(!odom_record.empty()){
            // Debug positions:
            // nav_msgs::Odometry odom_current = odom_record.back();
            // std::cout << selfUAV << "'s current position-> x: [" <<odom_current.pose.pose.position.x<<"], \ty: ["<<odom_current.pose.pose.position.y<<"], \tz: ["<<odom_current.pose.pose.position.z<<"]" << std::endl;
            // std::cout << std::endl;
            
            // printRecord();
            // std::cout << std::endl;


            // Update path_length with all data in odom_record
            path_length = updateTraversedDistance(path_length);

            // Convert float to string safetly
            std::ostringstream ss;
            ss << path_length;
            std::string s(ss.str());
            
            // Print distance
            std::cout << "Distance traversed by " << selfUAV << ": " << s << std::endl;

            // Publish traversed distance
            std_msgs::String msg;
            // msg.data = ("Distance traversed by " + selfUAV + ": " + s);
            msg.data = s;
            pub.publish(msg);
        }

        ros::spinOnce();
        r.sleep();
    }




    std::cout << "Closing dist_traversed_node" << std::endl;  
    return 0;
}









