// #include <ros/ros.h>
// #include "coordination/coordination_algorithm.h"




// int main(int argc, char **argv)
// {
//   ROS_INFO("coordination_node started");
//   ros::init(argc, argv, "coordination_node");
//   ros::NodeHandle nh;

//   coordinationAlgorithm coordAlgo;
//   coordAlgo.init(nh);

//   ros::Duration(1.0).sleep();

//   while ( ros::ok() ) {
//     ros::spin();
//   }
//   std::cout << "Closing coordination_node" << std::endl;
//   return 0;
// }




















#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"



int main(int argc, char **argv){
  ROS_INFO("\n test started started");
  ros::init(argc, argv, "coordination");
  ros::NodeHandle nh;

  ros::Publisher point_pub = nh.advertise<std_msgs::Float32MultiArray>("pub_man_pos", 50);
  
  std_msgs::Float32MultiArray array;
  array.data.push_back(4.0);
  array.data.push_back(1.60);
  array.data.push_back(1.0);


  // Set rate
  ros::Rate r(0.2); // 1 hz


  while ( ros::ok() ) {
    
    std::cout << "publish" << std::endl;
    point_pub.publish(array);

    ros::spinOnce();
    r.sleep();
  }


  std::cout << "Closing test" << std::endl;  
  return 0;
}



