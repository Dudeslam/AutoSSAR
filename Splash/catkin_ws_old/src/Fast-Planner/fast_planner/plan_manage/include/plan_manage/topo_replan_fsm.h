#ifndef _TOPO_REPLAN_FSM_H_
#define _TOPO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

//multi uav coordination
#include <geometry_msgs/PoseStamped.h>

using std::vector;

namespace fast_planner {

class TopoReplanFSM {
private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* planning utils */
  FastPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  int target_type_;  // 1 mannual select, 2 hard code
  double replan_distance_threshold_, replan_time_threshold_;
  double waypoints_[50][3];
  int waypoint_num_;
  bool act_map_;

  /* planning data */
  bool trigger_, have_target_, have_odom_, collide_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d target_point_, end_vel_;                        // target state
  int current_wp_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber waypoint_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  // multi uav coordination
  ros::Subscriber other_glable_pose_sub_;
  ros::Subscriber own_glable_pose_sub_;
  ros::Timer coordination_timer_;
  float uav2uav_dist_;
  int coordination_req_;

  // ros::Subscriber other_traj_sub_;
  Eigen::Vector3d own_pose_;
  Eigen::Vector3d other_pose_1_;
  vector<Eigen::Vector3d> occupied_pts_;

  /* helper functions */
  bool callSearchAndOptimization();    // front-end and back-end method
  bool callTopologicalTraj(int step);  // topo path guided gradient-based
                                       // optimization; 1: new, 2: replan

  //bool callTopologicalTraj(int step, bool coord_enable);  //enable multi-uav coordination
                                     
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();

  /* ROS functions */
  void execFSMCallback(const ros::TimerEvent& e);
  void checkCollisionCallback(const ros::TimerEvent& e);
  void waypointCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

  //for multi-UAV
  // void otherTrajCallback(const plan_manage::BsplineConstPtr& msg);
  void otherPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void ownPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void coordinationCallback(const ros::TimerEvent& e);

  


public:
  TopoReplanFSM(/* args */) {}
  ~TopoReplanFSM() {}

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif