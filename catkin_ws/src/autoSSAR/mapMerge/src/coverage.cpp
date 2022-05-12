#include "mapmerge/coverage.h"

void coverage::init(ros::NodeHandle& nh) {

    UAV0_name = "UAV0";
    UAV1_name = "UAV1";
    UAV2_name = "UAV2";

    std::cout << "*************************************************************" << std::endl;
    std::cout << "Self: "<< UAV0_name << std::endl;
    std::cout << "Other UAV0: " << UAV1_name << std::endl;
    std::cout << "Other UAV1: " << UAV2_name << std::endl;
    std::cout << "*************************************************************" << std::endl;
  

    //subscribers

}