#include <ros/ros.h>


int main(int argc, char **argv)
{
    ros::nodehandle nh;
    ros::init(argc, argv, "test_coord_node");

    //publishers waypoint
    ros::Publisher pub_waypoint = nh.advertise<geometry_msgs::PoseStamped>("UAV0/waypoint_generator/waypoints", 1);

    while(ros::ok)
    {
        geometry_msgs::PoseStamped waypoint;
        waypoint.header.frame_id = "map";
        waypoint.pose.position.x = 0.0;
        waypoint.pose.position.y = 0.0;
        waypoint.pose.position.z = 0.0;
        waypoint.pose.orientation.w = 1.0;
        waypoint.pose.orientation.x = 0.0;
        waypoint.pose.orientation.y = 0.0;
        waypoint.pose.orientation.z = 0.0;
        pub_waypoint.publish(waypoint);
        ros::Duration(0.5).sleep();
    }

    ros::spin();

}
