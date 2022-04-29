#include "coordination/coordination_algorithm.h"





void coordinationAlgorithm::init(ros::NodeHandle& nh) {

    // EDIT THIS !!!
    // expl_manager_.reset(new FastExplorationManager);
    // expl_manager_->initialize(nh);
    // planner_manager_ = expl_manager_->planner_manager_;

    // Init state
    state_ = COORD_STATE::EXPLORE;
    selfUAV_ = nh.getNamespace().c_str();

    within_range_ = nh.subscribe(selfUAV_+"/within_range", 1, &coordinationAlgorithm::withinRangeCallback, this);
    battery_      = nh.subscribe(selfUAV_+"/dist_traversed", 1, &coordinationAlgorithm::batteryCallback, this);

}




void coordinationAlgorithm::batteryCallback(const nav_msgs::PathConstPtr& msg){

}


void coordinationAlgorithm::withinRangeCallback(const nav_msgs::PathConstPtr& msg){

}



void coordinationAlgorithm::transitState(COORD_STATE new_state, std::string pos_call) {
  // int pre_s = int(state_);
  // state_ = new_state;
  // cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
}