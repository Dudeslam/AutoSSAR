/**
**  Simple ROS Node
**/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
//#include <rviz/display_context.h>

int main(int argc, char* argv[])
{
	// This must be called before anything else ROS-related
	ros::init(argc, argv, "ros_com_test");

	ROS_INFO("Hello, World!");


	// Create a ROS node handle
	ros::NodeHandle nh_;
	ros::Publisher pub_;

	double x = 2;
	double y = 2;
	double z = 2;
	double theta = 0.2;

	std::string fixed_frame = 0;//context_->getFixedFrame().toStdString();
	tf::Quaternion quat;
	quat.setRPY(0.0, 0.0, theta);
	tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, z)), ros::Time::now(), fixed_frame);
	geometry_msgs::PoseStamped goal;
	tf::poseStampedTFToMsg(p, goal);
	
	ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
		goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
		goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
	
	pub_.publish(goal);

	

/*
	class Localizer
	{
	public:
	  Localizer(ros::NodeHandle& nh)
	  {
		  ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1, 
		  &Localizer::visionCallback, this);
	  }

	  void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)
	  {
		  last_msg_ = msg;
		  ROS_INFO_STREAM(last_msg_->pose.pose);
	  }

	  ros::Subscriber ar_sub_;
	  fake_ar_publisher::ARMarkerConstPtr last_msg_;
	};
*/

	// Don't exit the program.
	ros::spin();
}
