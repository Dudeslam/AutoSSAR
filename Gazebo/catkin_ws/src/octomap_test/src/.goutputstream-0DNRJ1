#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <OctomapROS.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace octomap;

int main(int /*argc*/, char** /*argv*/){
//get pointcloud from ros
 ros::init(argc, argv, "octomap_test");
 ros::NodeHandle n;
 ros::Subscriber sub = n.subscribe("/camera/depth/points", 1000, cloud_cb);
 ros::spin();



}