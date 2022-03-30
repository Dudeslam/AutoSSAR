#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <ros/init.h>


using namespace std;
using namespace octomap;

int main(int /*argc*/, char** /*argv*/){
//get pointcloud from ros
 ros::init(argc, argv, "octomap_test");
 ros::NodeHandle n;
 ros::Subscriber sub = n.subscribe("/sdf_map/depth_cloud", 1000, cloud_cb);
 
 
 ros::spin();



}
