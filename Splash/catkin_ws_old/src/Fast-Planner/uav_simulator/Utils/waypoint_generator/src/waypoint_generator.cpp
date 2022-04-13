#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include "sample_waypoints.h"
#include <vector>
#include <deque>
#include <boost/format.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using bfmt = boost::format;

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
//for coordination
ros::Publisher team_pub;



string waypoint_type = string("manual");
bool is_odom_ready;
nav_msgs::Odometry odom;
nav_msgs::Path waypoints;
//for coordination
bool is_broadcast_link_ready;
geometry_msgs::PoseStamped team_pose;
string team_role = string("leader");


// series waypoint needed
std::deque<nav_msgs::Path> waypointSegments;
ros::Time trigged_time;

void load_seg(ros::NodeHandle& nh, int segid, const ros::Time& time_base) {
    std::string seg_str = boost::str(bfmt("seg%d/") % segid);
    double yaw;
    double time_from_start;
    ROS_INFO("Getting segment %d", segid);
    ROS_ASSERT(nh.getParam(seg_str + "yaw", yaw));
    ROS_ASSERT_MSG((yaw > -3.1499999) && (yaw < 3.14999999), "yaw=%.3f", yaw);
    ROS_ASSERT(nh.getParam(seg_str + "time_from_start", time_from_start));
    ROS_ASSERT(time_from_start >= 0.0);

    std::vector<double> ptx;
    std::vector<double> pty;
    std::vector<double> ptz;

    ROS_ASSERT(nh.getParam(seg_str + "x", ptx));
    ROS_ASSERT(nh.getParam(seg_str + "y", pty));
    ROS_ASSERT(nh.getParam(seg_str + "z", ptz));

    ROS_ASSERT(ptx.size());
    ROS_ASSERT(ptx.size() == pty.size() && ptx.size() == ptz.size());

    nav_msgs::Path path_msg;

    path_msg.header.stamp = time_base + ros::Duration(time_from_start);

    double baseyaw = tf::getYaw(odom.pose.pose.orientation);
    
    for (size_t k = 0; k < ptx.size(); ++k) {
        geometry_msgs::PoseStamped pt;
        pt.pose.orientation = tf::createQuaternionMsgFromYaw(baseyaw + yaw);
        Eigen::Vector2d dp(ptx.at(k), pty.at(k));
        Eigen::Vector2d rdp;
        rdp.x() = std::cos(-baseyaw-yaw)*dp.x() + std::sin(-baseyaw-yaw)*dp.y();
        rdp.y() =-std::sin(-baseyaw-yaw)*dp.x() + std::cos(-baseyaw-yaw)*dp.y();
        pt.pose.position.x = rdp.x() + odom.pose.pose.position.x;
        pt.pose.position.y = rdp.y() + odom.pose.pose.position.y;
        pt.pose.position.z = ptz.at(k) + odom.pose.pose.position.z;
        path_msg.poses.push_back(pt);
    }

    waypointSegments.push_back(path_msg);
}

void load_waypoints(ros::NodeHandle& nh, const ros::Time& time_base) {
    int seg_cnt = 0;
    waypointSegments.clear();
    ROS_ASSERT(nh.getParam("segment_cnt", seg_cnt));
    for (int i = 0; i < seg_cnt; ++i) {
        load_seg(nh, i, time_base);
        if (i > 0) {
            ROS_ASSERT(waypointSegments[i - 1].header.stamp < waypointSegments[i].header.stamp);
        }
    }
    ROS_INFO("Overall load %zu segments", waypointSegments.size());
}

void publish_waypoints() {
    waypoints.header.frame_id = std::string("world");
    waypoints.header.stamp = ros::Time::now();
    pub1.publish(waypoints);
    geometry_msgs::PoseStamped init_pose;
    init_pose.header = odom.header;
    init_pose.pose = odom.pose.pose;
    waypoints.poses.insert(waypoints.poses.begin(), init_pose);
    // pub2.publish(waypoints);
    waypoints.poses.clear();
}

void publish_waypoints_vis() {
    nav_msgs::Path wp_vis = waypoints;
    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = std::string("world");
    poseArray.header.stamp = ros::Time::now();

    {
        geometry_msgs::Pose init_pose;
        init_pose = odom.pose.pose;
        poseArray.poses.push_back(init_pose);
    }

    for (auto it = waypoints.poses.begin(); it != waypoints.poses.end(); ++it) {
        geometry_msgs::Pose p;
        p = it->pose;
        poseArray.poses.push_back(p);
    }
    pub2.publish(poseArray);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    is_odom_ready = true;
    odom = *msg;

    if (waypointSegments.size()) {
        ros::Time expected_time = waypointSegments.front().header.stamp;
        if (odom.header.stamp >= expected_time) {
            waypoints = waypointSegments.front();

            std::stringstream ss;
            ss << bfmt("Series send %.3f from start:\n") % trigged_time.toSec();
            for (auto& pose_stamped : waypoints.poses) {
                ss << bfmt("P[%.2f, %.2f, %.2f] q(%.2f,%.2f,%.2f,%.2f)") %
                          pose_stamped.pose.position.x % pose_stamped.pose.position.y %
                          pose_stamped.pose.position.z % pose_stamped.pose.orientation.w %
                          pose_stamped.pose.orientation.x % pose_stamped.pose.orientation.y %
                          pose_stamped.pose.orientation.z << std::endl;
            }
            ROS_INFO_STREAM(ss.str());

            publish_waypoints_vis();
            publish_waypoints();

            waypointSegments.pop_front();
        }
    }
}

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
/*    if (!is_odom_ready) {
        ROS_ERROR("[waypoint_generator] No odom!");
        return;
    }*/

    trigged_time = ros::Time::now(); //odom.header.stamp;
    //ROS_ASSERT(trigged_time > ros::Time(0));

    ros::NodeHandle n("~");
    n.param("waypoint_type", waypoint_type, string("manual"));
    
    if (waypoint_type == string("circle")) {
        waypoints = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("eight")) {
        waypoints = eight();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("point")) {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("series")) {
        load_waypoints(n, trigged_time);
    } else if (waypoint_type == string("manual-lonely-waypoint")) {
        if (msg->pose.position.z > -0.1) {
            // if height > 0, it's a valid goal;
            geometry_msgs::PoseStamped pt = *msg;
            waypoints.poses.clear();
            waypoints.poses.push_back(pt);
            publish_waypoints_vis();
            publish_waypoints();
            if(team_role == string("leader")){
                //if it is leader, forward the message to relay node
                team_pub.publish(pt);
            }
        } else {
            ROS_WARN("[waypoint_generator] invalid goal in manual-lonely-waypoint mode.");
        }

    } else {
        if (msg->pose.position.z > 0) {
            // if height > 0, it's a normal goal;
            geometry_msgs::PoseStamped pt = *msg;
            if (waypoint_type == string("noyaw")) {
                double yaw = tf::getYaw(odom.pose.pose.orientation);
                pt.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            }
            waypoints.poses.push_back(pt);
            publish_waypoints_vis();
        } else if (msg->pose.position.z > -1.0) {
            // if 0 > height > -1.0, remove last goal;
            if (waypoints.poses.size() >= 1) {
                waypoints.poses.erase(std::prev(waypoints.poses.end()));
            }
            publish_waypoints_vis();
        } else {
            // if -1.0 > height, end of input
            if (waypoints.poses.size() >= 1) {
                publish_waypoints_vis();
                publish_waypoints();
            }
        }
    }
}

void traj_start_trigger_callback(const geometry_msgs::PoseStamped& msg) {
    if (!is_odom_ready) {
        ROS_ERROR("[waypoint_generator] No odom!");
        return;
    }

    ROS_WARN("[waypoint_generator] Trigger!");
    trigged_time = odom.header.stamp;
    ROS_ASSERT(trigged_time > ros::Time(0));

    ros::NodeHandle n("~");
    n.param("waypoint_type", waypoint_type, string("manual"));

    ROS_ERROR_STREAM("Pattern " << waypoint_type << " generated!");
    if (waypoint_type == string("free")) {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("circle")) {
        waypoints = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("eight")) {
        waypoints = eight();
        publish_waypoints_vis();
        publish_waypoints();
   } else if (waypoint_type == string("point")) {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("series")) {
        load_waypoints(n, trigged_time);
    }
}

void team_waypoints_ack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (msg->pose.position.z == -999) {
        // if received leader init message, then send ack back
        team_pose =  ack_team_link();
        team_pub.publish(team_pose);
    }
    // forward the team waypoints to the planner 
    if (waypoint_type == string("manual-lonely-waypoint")) {
        if (msg->pose.position.z > -0.1) {
            // if height > 0, it's a valid goal;
            geometry_msgs::PoseStamped pt = *msg;
            waypoints.poses.clear();
            waypoints.poses.push_back(pt);
            publish_waypoints_vis();
            publish_waypoints();
        } else {
            ROS_WARN("[team_waypoints_ack] invalid goal in manual-lonely-waypoint mode.");
        }
    }
}


void ack_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if(team_role == string("leader")){
        // is_broadcast_link_ready = 1; //ack the link has been established
        if (msg->pose.position.z == -888) {
            // received ack
            is_broadcast_link_ready = 1;
        }
    }else{
        if (msg->pose.position.z == -999) {
            // if received leader init message, then send ack back
            team_pose =  ack_team_link();
            team_pub.publish(team_pose);
        }
        // forward the team waypoints to the planner 
        if (waypoint_type == string("manual-lonely-waypoint")) {
            if (msg->pose.position.z > -0.1) {
                // if height > 0, it's a valid goal;
                geometry_msgs::PoseStamped pt = *msg;
                waypoints.poses.clear();
                waypoints.poses.push_back(pt);
                publish_waypoints_vis();
                publish_waypoints();
            } else {
                ROS_WARN("[team_waypoints_ack] invalid goal in manual-lonely-waypoint mode.");
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_generator");
    ros::NodeHandle n("~");
    n.param("waypoint_type", waypoint_type, string("manual"));
    n.param("team_role", team_role, string("leader"));

    ros::Subscriber sub1 = n.subscribe("odom", 10, odom_callback);
    ros::Subscriber sub2 = n.subscribe("goal", 10, goal_callback);
    ros::Subscriber sub3 = n.subscribe("traj_start_trigger", 10, traj_start_trigger_callback);
    pub1 = n.advertise<nav_msgs::Path>("waypoints", 50);
    pub2 = n.advertise<geometry_msgs::PoseArray>("waypoints_vis", 10);

    string team_pub_topic_str;
    string sub_team_topic_str;

    if(team_role != string("leader")){ //used for team member
        team_pub_topic_str = "team_ack";
        sub_team_topic_str = "/leader/leader_waypoints";
    }else{ //used for team leader
        //for broadcasting the leader waypoints commands
        team_pub_topic_str = "leader_waypoints";
        sub_team_topic_str = "/follower_1/team_ack";
    }

    team_pub = n.advertise<geometry_msgs::PoseStamped>(team_pub_topic_str, 1);
    ros::Subscriber sub_team = n.subscribe(sub_team_topic_str, 1, ack_callback);


    is_broadcast_link_ready = 0;
    
    trigged_time = ros::Time(0);
    //ros::spin();

    ros::Rate loop_rate(5);
    if(team_role == string("leader")){ //used for team member
        team_pose = init_team_link();
    }

    while (ros::ok())
    {
        if(team_role == string("leader") && is_broadcast_link_ready == 0){
            //send some message to help multi-master-fkie to setup the link
            team_pose.header.frame_id = std::string("world");
            team_pose.header.stamp = ros::Time::now();
            team_pub.publish(team_pose);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
