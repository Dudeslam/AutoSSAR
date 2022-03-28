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

// 	std::string frame_id_;// = "/world";		//context_->getFixedFrame().toStdString();
// 	tf::Stamped() :frame_id_ ("NO_ID_STAMPED_DEFAULT_CONSTRUCTION"){}; //Default constructor used only for preallocation

// 	tf::Quaternion quat;
// 	quat.setRPY(0.0, 0.0, theta);
// 	tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, z)), ros::Time::now(), frame_id_);
// 	geometry_msgs::PoseStamped goal;
// 	tf::poseStampedTFToMsg(p, goal);
	
// 	ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
// 		goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
// 		goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
	
// 	pub_.publish(goal);

// 	// Don't exit the program.
// 	ros::spin();
// }






/**
**  Attempt 2) https://automaticaddison.com/how-to-send-goals-to-the-ros-navigation-stack-using-c/
**/
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <ncurses.h>	// for getch()

//using namespace std;
 
// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 
int main(int argc, char** argv){
   
  // Connect to ROS
  ros::init(argc, argv, "simple_navigation_goals");
 
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
 
  // Wait for the action server to come up so that we can begin processing goals.
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
	if(getch()){return 0;}
  }
 
  int user_choice = 6;
  char choice_to_continue = 'Y';
  bool run = true;
     
  while(run) {
 
    // Ask the user where he wants the robot to go?
    std::cout << "\nWhere do you want the robot to go?" << std::endl;
    std::cout << "\n1 = Bathroom" << std::endl;
    std::cout << "2 = Bedroom" << std::endl;
    std::cout << "3 = Front Door" << std::endl;
    std::cout << "4 = Living Room" << std::endl;
    std::cout << "5 = Home Office" << std::endl;
    std::cout << "6 = Kitchen" << std::endl;
    std::cout << "\nEnter a number: ";
    std::cin >> user_choice;
 
    // Create a new goal to send to move_base 
    move_base_msgs::MoveBaseGoal goal;
 
    // Send a goal to the robot
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
         
    bool valid_selection = true;
 
    // Use map_server to load the map of the environment on the /map topic. 
    // Launch RViz and click the Publish Point button in RViz to 
    // display the coordinates to the /clicked_point topic.
    switch (user_choice) {
      case 1:
        std::cout << "\nGoal Location: Bathroom\n" << std::endl;
        goal.target_pose.pose.position.x = 10.0;
    	goal.target_pose.pose.position.y = 3.7;
        goal.target_pose.pose.orientation.w = 1.0;
        break;
      case 2:
        std::cout << "\nGoal Location: Bedroom\n" << std::endl;
        goal.target_pose.pose.position.x = 8.1;
    	goal.target_pose.pose.position.y = 4.3;
        goal.target_pose.pose.orientation.w = 1.0;
        break;
      case 3:
        std::cout << "\nGoal Location: Front Door\n" << std::endl;
        goal.target_pose.pose.position.x = 10.5;
    	goal.target_pose.pose.position.y = 2.0;
        goal.target_pose.pose.orientation.w = 1.0;
        break;
      case 4:
        std::cout << "\nGoal Location: Living Room\n" << std::endl;
        goal.target_pose.pose.position.x = 5.3;
    	goal.target_pose.pose.position.y = 2.7;
        goal.target_pose.pose.orientation.w = 1.0;
        break;
      case 5:
        std::cout << "\nGoal Location: Home Office\n" << std::endl;
        goal.target_pose.pose.position.x = 2.5;
    	goal.target_pose.pose.position.y = 2.0;
        goal.target_pose.pose.orientation.w = 1.0;
        break;
      case 6:
        std::cout << "\nGoal Location: Kitchen\n" << std::endl;
        goal.target_pose.pose.position.x = 3.0;
    	goal.target_pose.pose.position.y = 6.0;
        goal.target_pose.pose.orientation.w = 1.0;
        break;
      default:
        std::cout << "\nInvalid selection. Please try again.\n" << std::endl;
        valid_selection = false;
    }       
         
    // Go back to beginning if the selection is invalid.
    if(!valid_selection) {
      continue;
    }
 
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
 
    // Wait until the robot reaches the goal
    ac.waitForResult();
 
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The robot has arrived at the goal location");
    else
      ROS_INFO("The robot failed to reach the goal location for some reason");
         
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





// /**
// **  Attempt 3)		http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
// **/
// #include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
// #include <geometry_msgs/PoseStamped.h>

// int main(int argc, char* argv[])
// {
// 	// This must be called before anything else ROS-related
// 	ros::init(argc, argv, "ros_com_test");
// 	ROS_INFO("Hello, World!");





// 	// Create a ROS node handle
// 	ros::NodeHandle nh_;
// 	ros::Publisher pub_ = nh_.advertise<geometry_msgs/PoseStamped>(
// 		{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}
// 	)

// 	ros::Rate loop_rate(10);

// 	int count = 0;
// 	while (ros::ok())
// 	{
// 		std_msgs::String msg;

// 		std::stringstream ss;
// 		ss << "hello world " << count;
// 		msg.data = ss.str();

// 		ROS_INFO("%s", msg.data.c_str());

// 		pub_.publish(msg);

// 		ros::spinOnce();

// 		loop_rate.sleep();
// 		++count;
// 	}

// 	return 0;
// }


// // rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'