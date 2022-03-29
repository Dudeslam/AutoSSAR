// /**
// **  Attempt 1)
// **/
// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <tf/transform_listener.h>

// int main(int argc, char* argv[])
// {
// 	// This must be called before anything else ROS-related
// 	ros::init(argc, argv, "ros_com_test");

// 	ROS_INFO("Hello, World!");


// 	// Create a ROS node handle
// 	ros::NodeHandle nh_;
// 	ros::Publisher pub_;

// 	double x = 2;
// 	double y = 2;
// 	double z = 2;
// 	double theta = 0.2;

// 	std::string frame_id_ = "world";		//context_->getFixedFrame().toStdString();

// 	tf::Quaternion quat;
// 	quat.setRPY(0.0, 0.0, theta);
// 	tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, z)), ros::Time::now(), frame_id_);
// 	geometry_msgs::PoseStamped goal;
// 	tf::poseStampedTFToMsg(p, goal);
	
// 	ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", frame_id_.c_str(),
// 		goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
// 		goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
	
// 	pub_.publish(goal);

// 	// Don't exit the program.
// 	ros::spin();
// }






/**
**  Attempt 4
**/
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <iostream>
#define M_PI           3.14159265358979323846  /* pi */
 
int main(int argc, char** argv){
   
	// Connect to ROS
  	ros::init(argc, argv, "ros_com_test");
 
	// Create a ROS node handle
	ros::NodeHandle nh_;
	ros::Publisher pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
 
 
	int user_choice = 1;
	char choice_to_continue = 'Y';
	bool run = true;
     
while(run) {
 
    // Ask the user where he wants the robot to go?
    std::cout << "\nWhere do you want the robot to go?" << std::endl;
    std::cout << "\n1 = North" << std::endl;
    std::cout << "2 = East" << std::endl;
    std::cout << "3 = South" << std::endl;
    std::cout << "4 = West" << std::endl;
    std::cout << "\nEnter a number: ";
    std::cin >> user_choice;
 
    // Create a new goal to send to move_base 
	geometry_msgs::PoseStamped goal;
    //move_base_msgs::MoveBaseGoal goal;
 
    // Send a goal to the robot
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
         
    bool valid_selection = true;
 
    // Use map_server to load the map of the environment on the /map topic. 
    // Launch RViz and click the Publish Point button in RViz to
    // display the coordinates to the /clicked_point topic.
    switch (user_choice) {
      case 1:
        std::cout << "\nNorth\n" << std::endl;
        goal.pose.position.x += 10;
    	//goal.target_pose.pose.position.y = 3.7;
        goal.pose.orientation.w = 0;
        break;
      case 2:
        std::cout << "\nEast\n" << std::endl;
        //goal.target_pose.pose.position.x = 8.1;
    	goal.pose.position.y += 10;
        goal.pose.orientation.w = M_PI/2;
        break;
      case 3:
        std::cout << "\nSouth\n" << std::endl;
        goal.pose.position.x -= 10;
    	//goal.target_pose.pose.position.y = 2.0;
        goal.pose.orientation.w = M_PI;
        break;
      case 4:
        std::cout << "\nWest\n" << std::endl;
        //goal.target_pose.pose.position.x = 5.3;
    	goal.pose.position.y -= 10;
        goal.pose.orientation.w = M_PI*1.5;
        break;
      default:
        std::cout << "\nInvalid selection. Please try again.\n" << std::endl;
        valid_selection = false;
    }
         
    // Go back to beginning if the selection is invalid.
    if(!valid_selection) {      continue;		// loop back inside while()
    }
 
	pub_.publish(goal);
	ros::spinOnce();
	ros::Duration d(5);

         
    // Ask the user if he wants to continue giving goals
    do {
      std::cout << "\nWould you like to go to another destination? (Y/N)" << std::endl;
      std::cin >> choice_to_continue;
      choice_to_continue = tolower(choice_to_continue); // Put your letter to its lower case
    } while (choice_to_continue != 'n' && choice_to_continue != 'y'); 
 
    if(choice_to_continue =='n') {
        run = false;
    }  
  }
   
  return 0;
}





// /** WORKS !!!
// **  Attempt 3)		http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
// **					https://answers.ros.org/question/276087/question-about-sending-simple-goal-to-navigation-stack/
// **
// **					Manual Rostopic call:
// **					// rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
// **/
// #include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
// #include <geometry_msgs/PoseStamped.h>

// int main(int argc, char* argv[])
// {
// 	// This must be called before anything else ROS-related
// 	ros::init(argc, argv, "ros_com_test");
// 	ROS_INFO("Hello, World!");


// 	double x = 2;
// 	double y = 2;
// 	double z = 2;
// 	double theta = 0.2;



// 	// Create a ROS node handle
// 	ros::NodeHandle nh_;
// 	ros::Publisher pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

// 	ros::Rate loop_rate(10);

// 	int count = 0;
// 	while (ros::ok())
// 		{
// 		std::string frame_id_ = "map";		//context_->getFixedFrame().toStdString();

// 		tf::Quaternion quat;
// 		quat.setRPY(0.0, 0.0, theta);
// 		tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, z)), ros::Time::now(), frame_id_);
// 		geometry_msgs::PoseStamped goal;
// 		tf::poseStampedTFToMsg(p, goal);

// 		ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", frame_id_.c_str(),
// 			goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
// 			goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);

// 		pub_.publish(goal);
// 		ros::spinOnce();
// 		ros::Duration d(5);
// 		loop_rate.sleep();
// 		++count;
// 	}

// 	return 0;
// }