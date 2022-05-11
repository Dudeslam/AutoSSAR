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
void setupOdom(double x, double y){
  std::cout << "setupOdom" << std::endl;

  tf::TransformBroadcaster odom_broadcaster;
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

geometry_msgs::PoseStamped goal_;
void setupGoal(){
  std::cout << "setupGoal" << std::endl;

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
  goal_.header.stamp = current_time;
  goal_.header.frame_id = "/simulation";

  //set the position
  goal_.pose.position.x = x;
  goal_.pose.position.y = y;
  goal_.pose.position.z = 1.0;
  goal_.pose.orientation = odom_quat;

  last_time = current_time;
}///


int main(int argc, char **argv){
  ROS_INFO("\n test started started");
  ros::init(argc, argv, "coordination");
  ros::NodeHandle nh;
  std::string selfUAV = nh.getNamespace().c_str();

  // traj_server
  // ros::Publisher point_pub = nh.advertise<const nav_msgs::Odometry>("pub_man_pos", 50);
  // FastExplorationFSM
  //ros::Publisher way_pub = nh.advertise<const geometry_msgs::PoseStamped>("/UAV0/waypoint_generator/waypoints", 50);
  ros::Publisher point_pub = nh.advertise<const nav_msgs::Odometry>("exploration_node/pub_manual_pos", 50);
  
  //setupGoal();
  setupOdom(4.0, 1.6);      // Set x,y


  // Set rate Hz
  ros::Rate r(0.2);
  int i = 0;

  while ( ros::ok() ) {
    
    std::cout << "Run: " << i << std::endl;
    for(int j=0; j<10; j++){
      std::cout << "publish: " << j+(i*10) << std::endl;
      //way_pub.publish(goal_);
      point_pub.publish(odom_);
    }
    std::cout << "\n" << std::endl;
    i++;

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