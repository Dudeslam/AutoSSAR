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
    
    otherUAV0InRange_ = false;
    otherUAV1InRange_ = false;
}




void coordinationAlgorithm::batteryCallback(const std_msgs::String& msg){
  double distTraversed = std::atof(msg->data)
  rangeLeft_ = batteryCapasity_ - distTraversed;
}


void coordinationAlgorithm::withinRangeCallback(const std_msgs::String& msg){
  std::string str = msg.data;

    if( str == otherUAV0 ){
        otherUAV0InRange_ = true;
    }

    if( str == otherUAV1 ){
        otherUAV1InRange_ = true;
    }
}



void coordinationAlgorithm::transitState(COORD_STATE new_state, std::string pos_call) {
  // int pre_s = int(state_);
  // state_ = new_state;
  // cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
}