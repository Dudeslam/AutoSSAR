#include "coordination/coordination_algorithm.h"
#include <typeinfo> // std::cout << " type is: " << typeid(VAR).name() << sdt::endl;

int firstOdom = true;
int getUavNr(std::string uavName){
  int n = uavName.length();
  char str[n+1];
  strcpy(str, uavName.c_str());
  
  int tot, i, j=0, num[100];
  tot = strlen(str);
  for(i=0; i<tot; i++)
  {
    if(str[i]>='0' && str[i]<='9')
    {
        num[j] = str[i];
        num[j] = num[j] - 48;
        j++;
    }
  }
  return num[0];
}

void updateNearUAV(void){
  // Iterate all encountered
  for(int uav_i : UAVsEncountered_){
    // Add UAV if not already encountered
    if(uav_i.name == nearUAV_.name){
      // If relayFlag_
      if(relayFlag_){
        break;
      }
      uav_i.relayPoint = currentOdom_
      break;
    }
    UAVsEncountered_.push_back(nearUAV_);
  }
}


void coordinationAlgorithm::init(ros::NodeHandle& nh) {

    // EDIT THIS !!!
    // expl_manager_.reset(new FastExplorationManager);
    // expl_manager_->initialize(nh);
    // planner_manager_ = expl_manager_->planner_manager_;

    // Init state
    selfUAV_ = nh.getNamespace().c_str();
    nh.getParam(selfUAV_+"/coordination/maxBattery", batteryCapasity_);

    within_range_sub_ = nh.subscribe(selfUAV_+"/within_range", 1, &coordinationAlgorithm::withinRangeCallback, this);
    battery_sub_      = nh.subscribe(selfUAV_+"/dist_traversed", 1, &coordinationAlgorithm::batteryCallback, this);
    odom_sub_         = nh.subscribe(selfUAV_+"/state_ukf/odom", 1, &coordinationAlgorithm::odometryCallback, this);
    
    state_ = COORD_STATE::EXPLORE;

    distTraversed_ = 0;
    rangeLeft_ = 0;
    relayFlag_ = false;

    // Timer for triggering MEET              2min
    meet_timer_ = nh.createTimer(ros::Duration(120), &coordinationAlgorithm::triggerMeet, this);

}




void coordinationAlgorithm::batteryCallback(const std_msgs::String& msg){
  distTraversed_ = std::stod(msg.data);
  rangeLeft_ = batteryCapasity_ - distTraversed_;

  // Raise relayFlag_ if 50% distance left
  if(rangeLeft_ < (batteryCapasity_/2) ){
    relayFlag_ = true;
  }

  // std::cout << selfUAV_ << std::endl;
  // std::cout << "rangeLeft_: " << rangeLeft_ << std::endl;
}

void coordinationAlgorithm::withinRangeCallback(const std_msgs::String& msg){

  // Get info of who is near
  nearUAV_.name = msg.data;
  nearUAV_.inRange = true;
  nearUAV_.id = getUavNr(msg.data);

  transitState(MEET, "withinRangeCallback");
  
  // std::cout << selfUAV_ << std::endl;
  // std::cout << "otherUAVInRange_: " << otherUAV << std::endl;
}

void coordinationAlgorithm::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
  // First call = set Home-position = odom_record[0]
  if( firstOdom ){
    homeOdom_ = (*msg);
    firstOdom = false;
  }
  // Update odom
  currentOdom_ = (*msg);
  
  
  // std::cout << selfUAV_ << std::endl;
  // std::cout << "homeOdom_    Position-> x: ["<< homeOdom_.pose.pose.position.x<<"], \ty: ["<<homeOdom_.pose.pose.position.y<<"], \tz: ["<<homeOdom_.pose.pose.position.z  <<"]" << std::endl;
  // std::cout << "currentOdom_ Position-> x: ["<< currentOdom_.pose.pose.position.x<<"], \ty: ["<<currentOdom_.pose.pose.position.y<<"], \tz: ["<<currentOdom_.pose.pose.position.z  <<"]" << std::endl;
}

void coordinationAlgorithm::triggerMeet(const ros::TimerEvent& e){
  transitState(MEET, "triggerMeet");
}

void coordinationAlgorithm::transitState(COORD_STATE new_state, std::string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
}


void coordinationAlgorithm::updateRole(void){


  switch (state_) {
    case EXPLORE: {
      // Nothing to do
    }

    case MEET: {
      updateNearUAV();  // Adds UAV if new + update odom
      // Nothing to do
    }

    case SACRIFICE: {
      // Nothing to do
    }

    case RELAY: {
      // Nothing to do
    }

    case DEAD: {
      // Nothing to do
    }

    case default:{
      ROS_WARN("DEFAULT REACHED =================================");
    }
  }


  
}