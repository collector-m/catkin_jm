#ifndef HEADER_LIDAR_DETECT_H
#define HEADER_LIDAR_DETECT_H

//ros lib
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
//ros dimatic
// #include <lidar_detect/lidar_detect_modeConfig.h>
//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/extract_clusters.h>  
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class LidarDectector
{
private:

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
  int lidar_transform_switch;
  double Map_xMax;
  double Map_yMax;
  double Map_xMin;
  double Map_yMin;
  double cellResolution;
  int grid_counted;

  PointCloud::Ptr clipperedPoint;
  nav_msgs::OccupancyGridPtr grid;

    void clipperCloudPlane(const PointCloud::Ptr& cloud_filter);
    void transPoint(double &x, double &y, double &z);
    void Grid(const PointCloud::Ptr& cloudtogrid);

public:
    
    LidarDectector();
    void setParam(
        double x_limit_min_,
        double x_limit_max_,
        double y_limit_min_,
        double y_limit_max_,
        double z_limit_min_,
        double z_limit_max_,
        double car_width_,
        double car_length_,
        double theta_x_lidar_,
        double theta_y_lidar_,
        double theta_z_lidar_,
        double offset_x_lidar_,
        double offset_y_lidar_,
        double offset_z_lidar_,
        int lidar_transform_switch_,
        double grid_counted_,
        double Map_xMax_,
        double Map_yMax_,
        double Map_xMin_,
        double Map_yMin_,
        double cellResolution_
    ); 
    nav_msgs::OccupancyGridPtr makeGrid(const PointCloud::Ptr& points);
};


#endif /* HEADER_LIDAR_DETECT_H */