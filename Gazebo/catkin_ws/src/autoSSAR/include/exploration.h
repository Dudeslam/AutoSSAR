#ifndef JACKALEXPLORATION_EXPLORATION_H_
#define JACKALEXPLORATION_EXPLORATION_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
const double PI = 3.1415926;
const double octo_reso = 0.05;
const int num_of_samples_eva = 15;
const int num_of_bay = 3;


octomap::OcTree* cur_tree;
octomap::OcTree* cur_tree_2d;
octomap_msgs::Octomap msg_octomap;

tf::TransformListener *tf_listener; 

point3d kinect_orig;

ofstream explo_log_file;
std::string octomap_name_3d;

struct sensorModel {
    double horizontal_fov;
    double vertical_fov;
    double angle_inc_hor;
    double angle_inc_vel;
    double width;
    double height;
    double max_range;
    // vector<pair<double, double>> pitch_yaws;
    octomap::Pointcloud SensorRays;
    point3d InitialVector;

    sensorModel(double _width, double _height, double _horizontal_fov, double _vertical_fov, double _max_range)
            : width(_width), height(_height), horizontal_fov(_horizontal_fov), vertical_fov(_vertical_fov), max_range(_max_range) {
        angle_inc_hor = horizontal_fov / width;
        angle_inc_vel = vertical_fov / height;
        for(double j = -height / 2; j < height / 2; ++j) 
            for(double i = -width / 2; i < width / 2; ++i) {
                InitialVector = point3d(1.0, 0.0, 0.0);
                InitialVector.rotate_IP(0.0, j * angle_inc_vel, i * angle_inc_hor);
                SensorRays.push_back(InitialVector);
        }
    }
}; 
sensorModel Kinect_360(128, 96, 2*PI*57/360, 2*PI*43/360, 6);    // Construct sensor model : Kinect

vector<int> sort_MIs(const vector<double> &);

double countFreeVolume(octomap::OcTree *);

octomap::Pointcloud castSensorRays(const octomap::OcTree *octree, const point3d &position,
                                 const point3d &sensor_orientation);

vector<vector<point3d>> extractFrontierPoints(const octomap::OcTree *octree);

vector<pair<point3d, point3d>> extractCandidateViewPoints(vector<vector<point3d>> frontier_groups, point3d sensor_orig, int n );

double calc_MI(const octomap::OcTree *octree, const point3d &sensor_orig, const octomap::Pointcloud &hits, const double before);

void kinectCallbacks( const sensor_msgs::PointCloud2ConstPtr& cloud2_msg );


#endif