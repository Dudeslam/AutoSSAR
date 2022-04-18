#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

nav_msgs::Odometry odom0;
nav_msgs::Odometry odom1;
nav_msgs::Odometry odom2;

/**
 * Callback function executes when new topic data comes.
 * Task of the callback function is to print data to screen.
 */
void getOdomCallback0(const nav_msgs::Odometry::ConstPtr& msg){
  //ROS_INFO("Seq: [%d]", msg->header.seq);
  //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  odom0 = *msg;
}

void getOdomCallback1(const nav_msgs::Odometry::ConstPtr& msg){
  odom1 = *msg;
}

void getOdomCallback2(const nav_msgs::Odometry::ConstPtr& msg){
  odom2 = *msg;
}



// Utility funcion
bool withinRange(nav_msgs::Odometry first, nav_msgs::Odometry second){
  double x1 = first.pose.pose.position.x;
  double y1 = first.pose.pose.position.x;
  double z1 = first.pose.pose.position.z;

  double x2 = second.pose.pose.position.x;
  double y2 = second.pose.pose.position.y;
  double z2 = second.pose.pose.position.z;

  long double dist=sqrt(pow(x1-x2,2.0)+pow(y1-y2,2.0)+pow(z1-z2,2.0));
  if(dist > 2) return false;
  return true;
}



int main(int argc, char **argv)
{
  ROS_INFO("Hello, World!");
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "withinRange");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called getOdomCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub0 = nh.subscribe("tb3_0/odom", 100, getOdomCallback0);
  ros::Subscriber sub1 = nh.subscribe("tb3_1/odom", 100, getOdomCallback1);
  ros::Subscriber sub2 = nh.subscribe("tb3_2/odom", 100, getOdomCallback2);
  ros::Publisher withinRange_pub = nh.advertise<std_msgs::String>("withinRange", 100);

  // For nice printin
  std::cout << std::setprecision(10) << std::fixed;

  // Set rate !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  ros::Rate r(0.5); // 10 hz



  while (ros::ok()) {
    std::cout << "tb3_0 Position-> x: [" <<odom0.pose.pose.position.x<<"], \ty: ["<<odom0.pose.pose.position.y<<"], \tz: ["<<odom0.pose.pose.position.z<<"]" << std::endl;
    std::cout << "tb3_1 Position-> x: [" <<odom1.pose.pose.position.x<<"], \ty: ["<<odom1.pose.pose.position.y<<"], \tz: ["<<odom1.pose.pose.position.z<<"]" << std::endl;
    std::cout << "tb3_2 Position-> x: [" <<odom2.pose.pose.position.x<<"], \ty: ["<<odom2.pose.pose.position.y<<"], \tz: ["<<odom2.pose.pose.position.z<<"]" << std::endl;

    if(withinRange(odom0, odom1)){
      std_msgs::String msg;
      msg.data = "withinRange_0_1";
      withinRange_pub.publish(msg);
    }
    if(withinRange(odom0, odom2)){
      std_msgs::String msg;
      msg.data = "withinRange_0_2";
      withinRange_pub.publish(msg);
    }
    if(withinRange(odom1, odom2)){
      std_msgs::String msg;
      msg.data = "withinRange_1_2";
      withinRange_pub.publish(msg);
    }

    ros::spinOnce();
    r.sleep();
  }




  std::cout << "Closing withinRange Node" << std::endl;  
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}



