#include <ros/ros.h>
#include "coordination/coordination_algorithm.h"





int main(int argc, char **argv)
{
  ROS_INFO("coordination_node started");
  ros::init(argc, argv, "coordination_node");
  ros::NodeHandle nh;

  coordinationAlgorithm coordAlgo;
  coordAlgo.init(nh);
  coordAlgo.runCoordinationAlgorithm();

  ros::Duration(1.0).sleep();
  ros::spin();
  std::cout << "Closing coordination_node" << std::endl;
  return 0;
}