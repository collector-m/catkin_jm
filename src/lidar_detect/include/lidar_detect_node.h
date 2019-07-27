#ifndef HEADER_LIDAR_DETECT_NODE_H
#define HEADER_LIDAR_DETECT_NODE_H

#include"lidar_detect/lidarDetect.h"
#include"ros/ros.h"
#include "std_msgs/UInt32.h"

class LidarDetectNode
{
private:
    std::string sub_left_topic;
    std::string sub_right_topic;

    std::string pub_gridmap_topic;
    std::string pub_heart_topic;
    

    ros::Subscriber sub_left;
    ros::Subscriber sub_right;
    ros::Publisher pub_heart;
    ros::Publisher pub_gridmap;
    ros::NodeHandle nh_;

    LidarDectector ld;

    PointCloud::Ptr left_cloud;
    PointCloud::Ptr right_cloud;
    PointCloud::Ptr center_cloud;

    double x_limit_min;
    double x_limit_max;
    double y_limit_min;
    double y_limit_max;
    double z_limit_min; 
    double z_limit_max;
  
    double car_width;
    double car_length;
    double theta_x_lidar;
    double theta_y_lidar;
    double theta_z_lidar;
    double offset_x_lidar;
    double offset_y_lidar;
    double offset_z_lidar;
    bool lidar_transform_switch;
    double Map_xMax;
    double Map_yMax;
    double Map_xMin;
    double Map_yMin;
    double cellResolution;
    int grid_counted;

    // counter clock wise positive rad
    double yaw_left_to_center;
    double yaw_right_to_center;
    // forward positive
    double x_left_to_center;
    double x_right_to_center;
    // left positive
    double y_left_to_center;
    double y_right_to_center;
    
    const static double PI;

public:
    uint32_t left_callback;
    uint32_t right_callback;
    LidarDetectNode();
    void main_loop();
    void subLeftCallback(const sensor_msgs::PointCloud2::ConstPtr& pc);
    void subRightCallback(const sensor_msgs::PointCloud2::ConstPtr& pc);
    // void detectSick(const sensor_msgs::LaserScan::ConstPtr& scan_in);
    void fusion_to_center();
};

#endif