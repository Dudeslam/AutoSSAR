#include "ros/ros.h"


int main(int argc, char **argv)
{
  ROS_INFO("within_range_node started");
  ros::init(argc, argv, "within_range_node");
  ros::NodeHandle nh;

  
  // Set rate
  ros::Rate r(10); // 10 hz

  while (ros::ok()) {

    ros::spinOnce();
    r.sleep();
  }




  std::cout << "Closing within_range_node" << std::endl;
  ros::spin();
  return 0;
}