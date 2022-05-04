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




void coordinationAlgorithm::init(ros::NodeHandle& nh) {

    // Init UAVdata state
    selfUAV_.name = nh.getNamespace().c_str();
    nh.getParam(selfUAV_.name+"/coordination/maxBattery", batteryCapasity_);
    selfUAV_.role = coord_state_str_[EXPLORE];
    // Odometry is set in callback
    selfUAV_.id = getUavNr(selfUAV_.name);
    state_ = COORD_STATE::EXPLORE;

    
    // Init callbacks
    within_range_sub_ = nh.subscribe(selfUAV_.name+"/within_range", 1, &coordinationAlgorithm::withinRangeCallback, this);
    battery_sub_      = nh.subscribe(selfUAV_.name+"/dist_traversed", 1, &coordinationAlgorithm::batteryCallback, this);
    odom_sub_         = nh.subscribe(selfUAV_.name+"/state_ukf/odom", 1, &coordinationAlgorithm::odometryCallback, this);
    cmd_pub_          = nh.advertise<nav_msgs::Odometry>(selfUAV_.name+"/commandTheShits", 10);
    // Init state


    // Timer for triggering MEET              1min
    timeoutTimer_ = nh.createTimer(ros::Duration(60), &coordinationAlgorithm::triggerTimer, this);
    timeoutTimer_.stop();   // Starts when

    // Init members
    //batteryCapasity_ is set from param;
    distTraversed_ = 0;
    rangeLeft_ = 0;
    batteryHalfFlag_ = false;
    batteryEmptyFlag_ = false;
    nearUAVFlag_ = false;
    atRelayPointFlag_ = false;
    timerExpiredFlag_ = false;
    
    std::cout << "coordinationAlgorithm::init" << std::endl;
    std::cout << "selfUAV_.name: " << selfUAV_.name << std::endl;
    std::cout << "selfUAV_.role: " << selfUAV_.role << std::endl;
    std::cout << "selfUAV_.id:   " << selfUAV_.id <<std::endl;
    std::cout << "batteryCapasity_: " << batteryCapasity_ << std::endl << std::endl;

}




void coordinationAlgorithm::batteryCallback(const std_msgs::String& msg){
  distTraversed_ = std::stod(msg.data);
  rangeLeft_ = batteryCapasity_ - distTraversed_;

  // Raise batteryHalfFlag_ if 50% distance left
  if(rangeLeft_ < (batteryCapasity_/2) ){
    batteryHalfFlag_ = true;
    //std::cout << "batteryCallback batteryHalfFlag_: " << batteryHalfFlag_ << std::endl;
  }

  if(rangeLeft_ <= 0){
    batteryEmptyFlag_ = true;
    transitState(DEAD, "batteryCallback");
    //std::cout << "batteryCallback batteryEmptyFlag_: " << batteryEmptyFlag_ << std::endl;
  }

  // std::cout << selfUAV_ << std::endl;
  // std::cout << "rangeLeft_: " << rangeLeft_ << std::endl;
}

void coordinationAlgorithm::withinRangeCallback(const std_msgs::String& msg){

  // Get info of who is near
  nearUAV_.name = msg.data;
  nearUAV_.id = getUavNr(msg.data);
  nearUAV_.relayPoint = currentOdom_;

  nearUAVFlag_ = true;
  
  std::cout << selfUAV_.name << std::endl;
  std::cout << "nearUAV_ name: " << nearUAV_.name << std::endl;
  std::cout << "nearUAVFlag_: " << nearUAVFlag_ << std::endl;
}

void coordinationAlgorithm::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
  // First call = set Home-position = odom_record[0]
  if( firstOdom ){
    baseStationOdom_ = (*msg);
    // selfUAV_.relayPoint = currentOdom_;
    firstOdom = false;
  }
  // Update odom
  currentOdom_ = (*msg);
  
  // If at relaypoint set flag    NOTE: maybe impossible to hit?
  if(selfUAV_.relayPoint == currentOdom_){
    atRelayPointFlag_ = true;
    std::cout << "odometryCallback atRelayPointFlag_: " << atRelayPointFlag_ << std::endl;
  } else {
    atRelayPointFlag_ = false;
  }


  // std::cout << selfUAV_ << std::endl;
  // std::cout << "homeOdom_    Position-> x: ["<< homeOdom_.pose.pose.position.x<<"], \ty: ["<<homeOdom_.pose.pose.position.y<<"], \tz: ["<<homeOdom_.pose.pose.position.z  <<"]" << std::endl;
  // std::cout << "currentOdom_ Position-> x: ["<< currentOdom_.pose.pose.position.x<<"], \ty: ["<<currentOdom_.pose.pose.position.y<<"], \tz: ["<<currentOdom_.pose.pose.position.z  <<"]" << std::endl;
}

void coordinationAlgorithm::triggerTimer(const ros::TimerEvent& e){
  timerExpiredFlag_ = true;
  timeoutTimer_.stop(); // not sure if nessesary
  std::cout << "triggerTimer timerExpiredFlag_: " << timerExpiredFlag_ << std::endl;
}

void coordinationAlgorithm::transitState(COORD_STATE new_state, std::string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  std::cout << selfUAV_.name << " [" + pos_call + "]: from " + coord_state_str_[pre_s] + " to " + coord_state_str_[int(new_state)] << std::endl;
}

void coordinationAlgorithm::evaluateRoles(void){
  // If first meeting OR meeting the same UAV
  if ( pairedUAV_.name.empty() || pairedUAV_ == nearUAV_ ){
    std::cout << "evaluateRoles pairedUAV_.name: " <<  pairedUAV_.name << std::endl;
    // Set pairedUAV_
    pairedUAV_ = nearUAV_;         // Note: relayPoint is updated from nearUAV_ set in withinRange

    // Set roles = low id is RELAY
    if(pairedUAV_.id < selfUAV_.id){
      pairedUAV_.role = coord_state_str_[RELAY];
      selfUAV_.role = coord_state_str_[SACRIFICE];
    }
  }
}



void coordinationAlgorithm::runCoordinationAlgorithm(void){

  switch (state_) {
    case EXPLORE: {
      // If below threshold - change to relay or sacrifice
      // BUT if role is set to explore - keep exploring regardless (is if timeout for relay occured)
      if(batteryHalfFlag_ && (selfUAV_.role != coord_state_str_[EXPLORE]) ){
        if(selfUAV_.role == coord_state_str_[SACRIFICE]){
          transitState(SACRIFICE, "State: EXPLORE");
        }
        if(selfUAV_.role == coord_state_str_[RELAY]){
          transitState(RELAY, "State: RELAY");
        }
      }

      // If anyone is near while exploring
      if(nearUAVFlag_){
        nearUAVFlag_ = false;
        transitState(MEET, "State: EXPLORE");
      }

      // Just explore !!!
      break;
    }

    case MEET: {
      // If role is just explorer - do pairing
      if(selfUAV_.role == coord_state_str_[EXPLORE]){
        evaluateRoles();
        transitState(EXPLORE, "State: MEET");
      }

      // If we have paired + going home + met state
      if(nearUAV_ == pairedUAV_ && batteryHalfFlag_){
        // Map will be shared automatically
        cmd_pub_.publish(baseStationOdom_);
        timeoutTimer_.stop();
        transitState(DONE, "State: MEET");
        } else {
          // If another is met - go back to explore
          transitState(EXPLORE, "State: MEET");
        }

      break;
    }

    case SACRIFICE: {
      // first call - go to relay point + start timer
      if(timerExpiredFlag_ == false){
        // Go to relay-point and wait for MEET
        cmd_pub_.publish(selfUAV_.relayPoint);

        if(atRelayPointFlag_){
          timeoutTimer_.start();  // Wait for one minut   LOOP ??? restart???
        }
      }

      if(timerExpiredFlag_ == true){
        // Partner not comming - Go explore = find other UAVs
        selfUAV_.role = coord_state_str_[EXPLORE];

        // Clear pairedUAV_ to allow new mate?
          // Not nessesary as it will relay it's map with anyone, so just searching is fine

        // Set state explorer
        transitState(EXPLORE, "State: SACRIFICE");
      }

      // If anyone is near while exploring
      if(nearUAVFlag_){
        nearUAVFlag_ = false;
        transitState(MEET, "State: SACRIFICE");
      }
      // Nothing to do i.e Exploring
      break;
    }

    case RELAY: {
      // first call - go to relay point + start timer
      if(timerExpiredFlag_ == false){
        // Go to relay-point and wait for MEET
        cmd_pub_.publish(selfUAV_.relayPoint);

        if(atRelayPointFlag_){
          timeoutTimer_.start();  // Wait for one minut
        }
      }

      if(timerExpiredFlag_ == true){
        // Go home, partner not comming
        cmd_pub_.publish(baseStationOdom_);
      }

      // If anyone is near while exploring
      if(nearUAVFlag_){
        nearUAVFlag_ = false;
        transitState(MEET, "State: RELAY");
      }
      // Nothing to do i.e Exploring
      break;
    }

    case DEAD: {
      // Stay here
      while(1){
        cmd_pub_.publish(currentOdom_);
      }
      break;
    }

    case DONE: {
      // Stay here
      while(1){
        cmd_pub_.publish(baseStationOdom_);
      }
      break;
    }

    default:{
      ROS_WARN("\nDEFAULT REACHED =================================\n");
    }
  }


  
}