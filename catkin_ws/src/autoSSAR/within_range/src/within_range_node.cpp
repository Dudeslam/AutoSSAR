#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

nav_msgs::Odometry odomSelf;
nav_msgs::Odometry odomOtherUAV0;
nav_msgs::Odometry odomOtherUAV1;
nav_msgs::Odometry odomOtherUAV2;
int maxRange;

/**
 * Callback function executes when new topic data comes.
 * Task of the callback function is to print data to screen.
 */
void getOdomCallback0(const nav_msgs::Odometry::ConstPtr& msg){
  //ROS_INFO("Seq: [%d]", msg->header.seq);
  //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  odomSelf = *msg;
  // std::cout << "Seq: " << msg->header.seq << std::endl;
  // std::cout << "Position-> x: [" <<odomOtherUAV0.pose.pose.position.x<<"], \ty: ["<<odomOtherUAV0.pose.pose.position.y<<"], \tz: ["<<odomOtherUAV0.pose.pose.position.z<<"]" << std::endl;
}

void getOdomCallback1(const nav_msgs::Odometry::ConstPtr& msg){
  odomOtherUAV0 = *msg;
  // std::cout << "Seq: " << msg->header.seq << std::endl;
  // std::cout << "Position-> x: [" <<odomOtherUAV0.pose.pose.position.x<<"], \ty: ["<<odomOtherUAV0.pose.pose.position.y<<"], \tz: ["<<odomOtherUAV0.pose.pose.position.z<<"]" << std::endl;
}

void getOdomCallback2(const nav_msgs::Odometry::ConstPtr& msg){
  odomOtherUAV1 = *msg;
  // std::cout << "Seq: " << msg->header.seq << std::endl;
  // std::cout << "Position-> x: [" <<odomOtherUAV1.pose.pose.position.x<<"], \ty: ["<<odomOtherUAV1.pose.pose.position.y<<"], \tz: ["<<odomOtherUAV1.pose.pose.position.z<<"]" << std::endl;
}

void getOdomCallback3(const nav_msgs::Odometry::ConstPtr& msg){
  odomOtherUAV2 = *msg;
  // std::cout << "Seq: " << msg->header.seq << std::endl;
  // std::cout << "Position-> x: [" <<odomOtherUAV2.pose.pose.position.x<<"], \ty: ["<<odomOtherUAV2.pose.pose.position.y<<"], \tz: ["<<odomOtherUAV2.pose.pose.position.z<<"]" << std::endl;
}



// Utility funcion
long double dist(nav_msgs::Odometry first, nav_msgs::Odometry second){
  double x1 = first.pose.pose.position.x;
  double y1 = first.pose.pose.position.y;
  double z1 = first.pose.pose.position.z;

  double x2 = second.pose.pose.position.x;
  double y2 = second.pose.pose.position.y;
  double z2 = second.pose.pose.position.z;

  long double dist = sqrt(pow(x1-x2,2.0)+pow(y1-y2,2.0)+pow(z1-z2,2.0));
  return dist;
}

bool within_range(nav_msgs::Odometry first, nav_msgs::Odometry second){
  // If an UAV isn't included dist=0, so to avoid publishing in this case it's removed.
  if(dist(first, second) > maxRange || dist(first, second) == 0) return false;
  return true;
}



int main(int argc, char **argv)
{
  ROS_INFO("within_range_node started");
  ros::init(argc, argv, "within_range_node");
  ros::NodeHandle nh;

  std::string selfUAV;
  std::string otherUAV0 = "nan";
  std::string otherUAV1 = "nan";
  std::string otherUAV2 = "nan";
  
  selfUAV = nh.getNamespace().c_str();
  nh.getParam(selfUAV+"/within_range/otherUAV0", otherUAV0);
  nh.getParam(selfUAV+"/within_range/otherUAV1", otherUAV1);
  nh.getParam(selfUAV+"/within_range/otherUAV2", otherUAV2);
  nh.getParam(selfUAV+"/within_range/maxRange", maxRange);

  std::cout << "*************************************************************" << std::endl;
  std::cout << "Within Range Node" << std::endl;
  std::cout << "maxRange: ";
  std::cout << maxRange << std::endl;
  std::cout << selfUAV << std::endl;
  std::cout << otherUAV0 << std::endl;
  std::cout << otherUAV1 << std::endl;
  std::cout << otherUAV2 << std::endl;
  std::cout << "*************************************************************" << std::endl;



  ros::Subscriber sub0 = nh.subscribe(selfUAV+"/state_ukf/odom", 100, getOdomCallback0);
  ros::Subscriber sub1 = nh.subscribe(otherUAV0+"/state_ukf/odom", 100, getOdomCallback1);
  ros::Subscriber sub2 = nh.subscribe(otherUAV1+"/state_ukf/odom", 100, getOdomCallback2);
  ros::Subscriber sub3 = nh.subscribe(otherUAV2+"/state_ukf/odom", 100, getOdomCallback3);
  ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("within_range", 100);

  // For nice printin
  std::cout << std::setfill ('0') << std::setw (6);
  //std::cout << std::setprecision(4) << std::fixed;

  // Set rate
  ros::Rate r(10); // 10 hz



  while (ros::ok()) {
    // Debug positions:
    // std::cout << selfUAV << " Position-> x: [" <<odomSelf.pose.pose.position.x<<"], \ty: ["<<odomSelf.pose.pose.position.y<<"], \tz: ["<<odomSelf.pose.pose.position.z<<"]" << std::endl;
    // std::cout << otherUAV0 << " Position-> x: [" <<odomOtherUAV0.pose.pose.position.x<<"], \ty: ["<<odomOtherUAV0.pose.pose.position.y<<"], \tz: ["<<odomOtherUAV0.pose.pose.position.z<<"]" << std::endl;
    // std::cout << otherUAV1 << " Position-> x: [" <<odomOtherUAV1.pose.pose.position.x<<"], \ty: ["<<odomOtherUAV1.pose.pose.position.y<<"], \tz: ["<<odomOtherUAV1.pose.pose.position.z<<"]" << std::endl;
    // std::cout << std::endl;
    
    // Debug distances
    // std::cout << selfUAV << std::endl;
    // std::cout << "Dist to " << otherUAV0 << ": " << dist(odomSelf, odomOtherUAV0) << " " << within_range(odomSelf, odomOtherUAV0) << std::endl;
    // std::cout << "Dist to " << otherUAV1 << ": " << dist(odomSelf, odomOtherUAV1) << " " << within_range(odomSelf, odomOtherUAV1) << std::endl;
    // std::cout << std::endl;

    if(within_range(odomSelf, odomOtherUAV0) && otherUAV0!="nan"){
      std::cout << selfUAV << std::endl;
      std::cout << "Dist to " << otherUAV0 << ": " << dist(odomSelf, odomOtherUAV0) << " " << within_range(odomSelf, odomOtherUAV0) << std::endl;

<<<<<<< HEAD:catkin_ws/src/autossar/within_range/src/within_range_node.cpp
      odomOtherUAV0.child_frame_id = otherUAV0;
      pub.publish(odomOtherUAV0);
=======
      std_msgs::String msg;
      msg.data = otherUAV0;
      pub.publish(msg);
>>>>>>> main:catkin_ws/src/autoSSAR/within_range/src/within_range_node.cpp
    }

    
    if(within_range(odomSelf, odomOtherUAV1) && otherUAV1!="nan"){
      std::cout << selfUAV << std::endl;
      std::cout << "Dist to " << otherUAV1 << ": " << dist(odomSelf, odomOtherUAV1) << " " << within_range(odomSelf, odomOtherUAV1) << std::endl;

<<<<<<< HEAD:catkin_ws/src/autossar/within_range/src/within_range_node.cpp
      odomOtherUAV1.child_frame_id = otherUAV1;
      pub.publish(odomOtherUAV1);
    }

    
    if(within_range(odomSelf, odomOtherUAV2) && otherUAV2!="nan"){
      std::cout << selfUAV << std::endl;
      std::cout << "Dist to " << otherUAV2 << ": " << dist(odomSelf, odomOtherUAV2) << " " << within_range(odomSelf, odomOtherUAV2) << std::endl;

      odomOtherUAV2.child_frame_id = otherUAV2;
      pub.publish(odomOtherUAV2);
=======
      std_msgs::String msg;
      msg.data = otherUAV1;
      pub.publish(msg);
>>>>>>> main:catkin_ws/src/autoSSAR/within_range/src/within_range_node.cpp
    }

    ros::spinOnce();
    r.sleep();
  }




  std::cout << "Closing within_range_node" << std::endl;  
  return 0;
}



