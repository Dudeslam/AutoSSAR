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
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"



nav_msgs::Odometry odom_;

void setupOdom(){
  std::cout << "setupOdom" << std::endl;

  tf::TransformBroadcaster odom_broadcaster;

  double x = 4.0;
  double y = 1.6;
  double th = 0.0;

  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  current_time = ros::Time::now();

  //compute odometry in a typical way given the velocities of the robot
  double dt = (current_time - last_time).toSec();
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "/simulation";
  odom_trans.child_frame_id = "";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 1.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  odom_.header.stamp = current_time;
  odom_.header.frame_id = "/simulation";

  //set the position
  odom_.pose.pose.position.x = x;
  odom_.pose.pose.position.y = y;
  odom_.pose.pose.position.z = 1.0;
  odom_.pose.pose.orientation = odom_quat;

  //set the velocity
  odom_.child_frame_id = "";
  odom_.twist.twist.linear.x = vx;
  odom_.twist.twist.linear.y = vy;
  odom_.twist.twist.angular.z = vth;

  last_time = current_time;
}


int main(int argc, char **argv){
  ROS_INFO("\n test started started");
  ros::init(argc, argv, "coordination");
  ros::NodeHandle nh;

  ros::Publisher point_pub = nh.advertise<const nav_msgs::Odometry>("exploration_node/pub_man_pos", 50);
  ros::Publisher ManualWaypoint_pub = nh.advertise<nav_msgs:Path>("/waypoint_generator/Manualwaypoints",50);
  setupOdom();

  double x = 4.0;
  double y = 1.6;
  double th = 0.0;

  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;

  double dt = (current_time - last_time).toSec();
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;

 //ss
  // Set rate Hz
  ros::Rate r(10);
  nav_msgs::odometry pubmsg;
  pubmsg.header.frame_id = "map";
  pubmsg.header.stamp = ros::Time::now();
  pubmsg.pose.pose.position.x = odom_.pose.pose.position.x;
  pubmsg.pose.pose.position.y = odom_.pose.pose.position.y;
  pubmsg.pose.pose.position.z = odom_.pose.pose.position.z;
  pubmsg.pose.pose.orientation = odom_.pose.pose.orientation;



  ManualWaypoint_pub.publish(pubmsg);
  while ( ros::ok() ) {
    
    std::cout << "publish" << std::endl;
    point_pub.publish(odom_);

    ros::spinOnce();
    r.sleep();
  }


  std::cout << "Closing test" << std::endl;  
  return 0;
}
//*/











// #include <ros/ros.h>
// #include "coordination/coordination_algorithm.h"
// #include <plan_manage/topo_replan_fsm.h>


// int main(int argc, char **argv)
// {
//   ROS_INFO("coordination_node started");
//   ros::init(argc, argv, "coordination_node");
//   ros::NodeHandle nh;


//   TopoReplanFSM topoFSM_;
//   topoFSM_.init(nh);

//   ros::Duration(1.0).sleep();

//   while ( ros::ok() ) {
//     ros::spin();
//   }


//   return 0;
// }