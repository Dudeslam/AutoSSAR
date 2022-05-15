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

float roundf1(double original){
  return truncf(original * 10) / 10;
}

void setRound1Odom(nav_msgs::Odometry &lhs, const nav_msgs::Odometry rhs){
  lhs.pose.pose.position.x = roundf1(rhs.pose.pose.position.x);
  lhs.pose.pose.position.y = roundf1(rhs.pose.pose.position.y);
  lhs.pose.pose.position.z = 1;
  lhs.child_frame_id = rhs.child_frame_id;

  //ROS_INFO_STREAM_THROTTLE(1.0, "LHS: x: ["<< lhs.pose.pose.position.x <<"], y: ["<<lhs.pose.pose.position.y<<"], \tz: ["<<lhs.pose.pose.position.z  <<"]" );
  //ROS_INFO_STREAM_THROTTLE(1.0, "RHS: x: ["<< rhs.pose.pose.position.x <<"], y: ["<<rhs.pose.pose.position.y<<"], \tz: ["<<rhs.pose.pose.position.z  <<"]" );
}

void findSetMidpoint(nav_msgs::Odometry &lhs, const nav_msgs::Odometry rhs){
  lhs.pose.pose.position.x = ( lhs.pose.pose.position.x + roundf1(rhs.pose.pose.position.x) )/2;
  lhs.pose.pose.position.y = ( lhs.pose.pose.position.y + roundf1(rhs.pose.pose.position.y) )/2;
  lhs.pose.pose.position.z = 1;
}

long double dist(nav_msgs::Odometry first, nav_msgs::Odometry second){
  double x1 = first.pose.pose.position.x;
  double y1 = first.pose.pose.position.y;
  double z1 = first.pose.pose.position.z;

  double x2 = second.pose.pose.position.x;
  double y2 = second.pose.pose.position.y;
  double z2 = second.pose.pose.position.z;

  long double dist = sqrt(pow(x1-x2,2.0)+pow(y1-y2,2.0)+pow(z1-z2,2.0));
  return dist;
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
    cmd_pub_          = nh.advertise<nav_msgs::Odometry>(selfUAV_.name+"/exploration_node/pub_manual_pos", 10);
    
    // Init main callback/loop
    run_timer_ = nh.createTimer(ros::Duration(0.01), &coordinationAlgorithm::runCoordinationAlgorithm, this);


    // Timer for triggering MEET              1min
    timeoutTimer_ = nh.createTimer(ros::Duration(20), &coordinationAlgorithm::triggerTimer, this);
    timeoutTimer_.stop();   // Starts when

    // Init members
    //batteryCapasity_ is set from param;
    distTraversed_ = 0;
    rangeLeft_ = 0;
    DEBUG_VAR = 0;
    batteryHalfFlag_ = false;
    batteryEmptyFlag_ = false;
    nearUAVFlag_ = false;
    atRelayPointFlag_ = false;
    timerExpiredFlag_ = false;
    timerRunningFlag_ = false;
    //pairedUAV_.name.clear();
    nearUAV_.name.clear();
    
    std::cout << "coordinationAlgorithm::init" << std::endl;
    std::cout << "selfUAV_.name: " << selfUAV_.name << std::endl;
    std::cout << "selfUAV_.role: " << selfUAV_.role << std::endl;
    std::cout << "selfUAV_.id:   " << selfUAV_.id <<std::endl;
    std::cout << "batteryCapasity_: " << batteryCapasity_ << std::endl << std::endl;



    // CHEAT !!!
    // pairedUAV_.name = "UAV99"; // OBS cleared below!!!
    // selfUAV_.relayPoint.pose.pose.position.x = 9.0;
    // selfUAV_.relayPoint.pose.pose.position.y = 3.0;
    // selfUAV_.role = coord_state_str_[RELAY];

}




//void coordinationAlgorithm::batteryCallback(const std_msgs::String& msg){
void coordinationAlgorithm::batteryCallback(const std_msgs::Float64& msg){
  //distTraversed_ = std::stod(msg.data);
  distTraversed_ = msg.data;
  rangeLeft_ = batteryCapasity_ - distTraversed_;


  // Raise batteryHalfFlag_ if 50% distance left
  if( (rangeLeft_ < (batteryCapasity_/2))  && selfUAV_.role != coord_state_str_[SACRIFICE] ){
    batteryHalfFlag_ = true;
    //std::cout << "batteryCallback batteryHalfFlag_: " << batteryHalfFlag_ << std::endl;
  }

  // If SACRIFICE simulate longer before return
  if( (rangeLeft_ < ((batteryCapasity_/2)+5)) && selfUAV_.role == coord_state_str_[SACRIFICE] ){
    batteryHalfFlag_ = true;
    //std::cout << "batteryCallback batteryHalfFlag_: " << batteryHalfFlag_ << std::endl;
  }//*/


  if(rangeLeft_ <= 0){
    batteryEmptyFlag_ = true;
    transitState(DEAD, "batteryCallback");
    //std::cout << "batteryCallback batteryEmptyFlag_: " << batteryEmptyFlag_ << std::endl;
  }

  // std::cout << selfUAV_ << std::endl;
  // std::cout << "rangeLeft_: " << rangeLeft_ << std::endl;
}

void coordinationAlgorithm::withinRangeCallback(const nav_msgs::Odometry::ConstPtr& msg){
  // If empty msg, ignore
  if((*msg).child_frame_id == ""){return;}


  // Get info of who is near
  nearUAV_.name = (*msg).child_frame_id;
  nearUAV_.id = getUavNr((*msg).child_frame_id);
  // Set relaypoint to current odom
  nearUAV_.relayPoint = currentOdom_;
  // Update relaypoint to be between the two UAVs
  findSetMidpoint(nearUAV_.relayPoint, (*msg));

  nearUAVFlag_ = true;
  // std::cout << selfUAV_.name << " is within: " << nearUAV_.name << std::endl;
}

void coordinationAlgorithm::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
  // Update odom
  setRound1Odom(currentOdom_, (*msg));
  //std::cout << "currentOdom_: " << currentOdom_.pose.pose.position.x << currentOdom_.pose.pose.position.y << std::endl;

  // First call = set Home-position = odom_record[0]
  if( firstOdom ){
    baseStationOdom_ = currentOdom_;
    // Never enter again
    firstOdom = false;
  }
  


  // If at relaypoint set flag    NOTE: maybe impossible to hit?
  if(dist(selfUAV_.relayPoint, currentOdom_) < 2){
  //if(selfUAV_.relayPoint == currentOdom_){
  //if( (selfUAV_.relayPoint.pose.pose.position.x == currentOdom_.pose.pose.position.x) && (selfUAV_.relayPoint.pose.pose.position.y == currentOdom_.pose.pose.position.y) ){
  //if( round(selfUAV_.relayPoint.pose.pose.position.x) == round(currentOdom_.pose.pose.position.x) && round(selfUAV_.relayPoint.pose.pose.position.y) == round(currentOdom_.pose.pose.position.y) ){
  //if( floor(selfUAV_.relayPoint.pose.pose.position.x) == floor(currentOdom_.pose.pose.position.x) && floor(selfUAV_.relayPoint.pose.pose.position.y) == floor(currentOdom_.pose.pose.position.y) ){
    atRelayPointFlag_ = true;
    //std::cout << "odometryCallback atRelayPointFlag_: " << atRelayPointFlag_ << std::endl;
    //ROS_INFO_STREAM_THROTTLE(1.0, "odometryCallback atRelayPointFlag_: " << atRelayPointFlag_ );
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
  timerRunningFlag_ = false;
  std::cout << "triggerTimer timerExpiredFlag_: " << timerExpiredFlag_ << std::endl;
}

void coordinationAlgorithm::transitState(COORD_STATE new_state, std::string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  //std::cout << selfUAV_.name << " [" + pos_call + "]: from " + coord_state_str_[pre_s] + " to " + coord_state_str_[int(new_state)] << std::endl;
}

void coordinationAlgorithm::evaluateRoles(void){
  // If first meeting OR meeting the same UAV
  if ( pairedUAV_.name.empty() || pairedUAV_ == nearUAV_ ){
    // Set pairedUAV_
    pairedUAV_ = nearUAV_;         // Note: relayPoint is updated from nearUAV_ set in withinRange
    // Update internal relay-point
    selfUAV_.relayPoint = pairedUAV_.relayPoint;


    //ROS_INFO_STREAM_THROTTLE(0.5, ""<< selfUAV_.name << " EvaluateRoles() paired with: " << pairedUAV_.name << "Midtpoint: " << selfUAV_.relayPoint.pose.pose.position.x << " " << selfUAV_.relayPoint.pose.pose.position.y );

    // Set roles = low id is RELAY
    if(pairedUAV_.id < selfUAV_.id){
      pairedUAV_.role = coord_state_str_[RELAY];
      selfUAV_.role = coord_state_str_[SACRIFICE];
    } else {
      pairedUAV_.role = coord_state_str_[SACRIFICE];
      selfUAV_.role = coord_state_str_[RELAY];
    }
  }
}



void coordinationAlgorithm::runCoordinationAlgorithm(const ros::TimerEvent& e){
  //std::cout << selfUAV_.role << coord_state_str_[state_] << nearUAVFlag_ << batteryHalfFlag_ << batteryEmptyFlag_ << std::endl;
  ROS_INFO_STREAM_THROTTLE(0.5, ""<< selfUAV_.name <<" State: " << coord_state_str_[state_] << "\tRole: " << selfUAV_.role
  << "\tFlags: "  << nearUAVFlag_ << batteryHalfFlag_ << batteryEmptyFlag_ << atRelayPointFlag_ << " Dist: " << roundf1(distTraversed_)
  << " Pos: " << currentOdom_.pose.pose.position.x << " " << currentOdom_.pose.pose.position.y
  << "\tRelaypoint: " << selfUAV_.relayPoint.pose.pose.position.x << " " << selfUAV_.relayPoint.pose.pose.position.y );





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
      

      // If anyone is near while exploring
      }else if(nearUAVFlag_){
        nearUAVFlag_ = false;
        transitState(MEET, "State: EXPLORE");
      }

      // Just explore !!!
      break;
    }



    case MEET: {
      // If role is just explorer - do pairing
      // if(selfUAV_.role == coord_state_str_[EXPLORE]){
      //   evaluateRoles();
      //   transitState(EXPLORE, "State: MEET");
      // }

      // If we have paired + going home + met state
      if(nearUAV_ == pairedUAV_ && batteryHalfFlag_){
        // Map will be shared automatically
        timeoutTimer_.stop();
        transitState(DONE, "State: MEET");
        ROS_INFO_STREAM_THROTTLE(1.0, ""<< selfUAV_.name <<" MEET: found my soulmate - continoue!!!" );
        timerExpiredFlag_ = true;
      }
        // } else {
        //   // If another is met - go back to explore
        //   transitState(EXPLORE, "State: MEET");
        // }

      evaluateRoles();
      transitState(EXPLORE, "State: MEET");

      break;
    }



    case SACRIFICE: {
      // 1)   first call - go to relay point
      if(timerExpiredFlag_ == false && atRelayPointFlag_ == false){
        // Go to relay-point and wait for MEET
        selfUAV_.relayPoint.child_frame_id = "GOTO";
        cmd_pub_.publish(selfUAV_.relayPoint);
        //std::cout << "RELAY: cmd_pub_.publish(selfUAV_.relayPoint);" << std::endl;
      }

      // 2)   If arrived at relay-point - start timer
      if(atRelayPointFlag_){
        // STOP UAV - and start timer
        selfUAV_.relayPoint.child_frame_id = "HALT";
        cmd_pub_.publish(selfUAV_.relayPoint);
        // Wait for one minut NOTE doesn't reset timer on re-call so timerRunningFlag_ isn't nessesary
        timeoutTimer_.start();
        //std::cout << "RELAY: timeoutTimer_.start();" << std::endl;
      }

      // 3)   When timer expired - forget pair and go explore
      if(timerExpiredFlag_ == true){
        // Partner not comming - Go explore = find other UAVs
        selfUAV_.role = coord_state_str_[EXPLORE];
        // Reset for next round (if new pair is found)
        timerExpiredFlag_ = false;
        ROS_INFO_STREAM_THROTTLE(1.0, ""<< selfUAV_.name <<"SACRIFICE: no one likes me - restart!!!" );

        // Clear pairedUAV_ to allow new mate?
        // Not nessesary as it will relay it's map with anyone, so just roaming is fine
        // If this isn't done it will Explore untill dead and never reach DONE
        pairedUAV_.name.clear();

        // Release from halt!
        baseStationOdom_.child_frame_id = "RND";
        for(int i=0; i<100; i++){
          cmd_pub_.publish(baseStationOdom_);
        }

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
      
      // 1)   first call - go to relay point
      if(timerExpiredFlag_ == false && atRelayPointFlag_ == false){
        // Go to relay-point and wait for MEET
        selfUAV_.relayPoint.child_frame_id = "GOTO";
        cmd_pub_.publish(selfUAV_.relayPoint);
        //std::cout << "RELAY: cmd_pub_.publish(selfUAV_.relayPoint);" << std::endl;
      }
      
      // 2)   If arrived at relay-point - start timer
      if(atRelayPointFlag_){
        // STOP UAV - and start timer
        selfUAV_.relayPoint.child_frame_id = "HALT";
        cmd_pub_.publish(selfUAV_.relayPoint);
        // Wait for one minut NOTE doesn't reset timer on re-call so timerRunningFlag_ isn't nessesary
        timeoutTimer_.start();
        ROS_INFO_STREAM_THROTTLE(1.0, ""<< selfUAV_.name <<" RELAY: timeoutTimer_.start() at: " << ros::Time::now() );
      }

      // 3)   When timer expired - go home
      if(timerExpiredFlag_ == true){
        // Go home, partner not comming
        baseStationOdom_.child_frame_id = "GOTO";
        cmd_pub_.publish(baseStationOdom_);
        //std::cout << "RELAY: cmd_pub_.publish(baseStationOdom_);" << std::endl;
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
      std::cout << selfUAV_.name << " DEAD: cmd_pub_.publish(baseStationOdom_);" << std::endl;
      currentOdom_.child_frame_id = "HALT";
      while(1){
        cmd_pub_.publish(currentOdom_);
      }
      break;
    }



    case DONE: {
      // Stay here
      std::cout << selfUAV_.name << " DONE: cmd_pub_.publish(baseStationOdom_);" << std::endl;
      baseStationOdom_.child_frame_id = "GOTO";
      while(1){
        if(selfUAV_.role == coord_state_str_[RELAY]){
          cmd_pub_.publish(baseStationOdom_);
        }
        // Else just derp... ie. explore til dead..
      }
      break;
    }



    default:{
      std::cout << "\nDEFAULT REACHED =================================\n);" << std::endl;
      ROS_WARN("\nDEFAULT REACHED =================================\n");
    }
  }


  
}