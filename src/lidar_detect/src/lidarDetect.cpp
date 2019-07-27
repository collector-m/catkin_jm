#include "lidar_detect/lidarDetect.h"

void LidarDectector::setParam(
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
) {
  x_limit_min = x_limit_min_;
  x_limit_max = x_limit_max_;
  y_limit_min = y_limit_min_;
  y_limit_max = y_limit_max_;
  z_limit_min = z_limit_min_;
  z_limit_max = z_limit_max_;
  car_width = car_width_;
  car_length = car_length_;
  theta_x_lidar = theta_x_lidar_;
  theta_y_lidar = theta_y_lidar_;
  theta_z_lidar = theta_z_lidar_;
  offset_x_lidar = offset_x_lidar_;
  offset_y_lidar = offset_y_lidar_;
  offset_z_lidar = offset_z_lidar_;
  lidar_transform_switch = lidar_transform_switch_;
  grid_counted = grid_counted_;
  Map_xMax = Map_xMax_;
  Map_yMax = Map_yMax_;
  Map_xMin = Map_xMin_;
  Map_yMin = Map_yMin_;
  cellResolution = cellResolution_;
}

LidarDectector::LidarDectector() {
  grid = boost::make_shared<nav_msgs::OccupancyGrid>();
  clipperedPoint = boost::make_shared<PointCloud>();
}

void LidarDectector::Grid(const PointCloud::Ptr& cloudtogrid)
{
  grid->data.clear();

  size_t numb_of_grid;

  grid->header.seq =1;
  grid->header.frame_id = "base_link";
  grid->info.origin.position.z =0;
  grid->info.origin.position.x =0;
  grid->info.origin.position.y =0;
  grid->info.origin.orientation.w =1;
  grid->info.origin.orientation.x =0;
  grid->info.origin.orientation.y =0;
  grid->info.origin.orientation.z =0;
  
  int xCells = ((int) ((Map_xMax - Map_xMin) / cellResolution));
  int yCells = ((int) ((Map_yMax - Map_yMin) / cellResolution));
  int size = xCells * yCells;

  grid->header.seq++;
  grid->header.stamp.sec =ros::Time::now().sec;
  grid->header.stamp.nsec =ros::Time::now().nsec;
  grid->info.map_load_time =ros::Time::now();
  grid->info.resolution = cellResolution;//cellRes=cellResolution=cellResolution;
  grid->info.width = xCells;//xCells = ((int) ((xMax - xMin) / cellResolution));
  grid->info.height = yCells;
  grid->info.origin.position.x = Map_xMin;//xMin
  grid->info.origin.position.y = Map_yMin;
  
  grid->data.resize(size,0);

    int xCell=0, yCell=0;
    if(0 != cloudtogrid->size())
    {
        for(size_t i = 0;i < cloudtogrid->points.size();i++ )
      {

	//convert to local frame(original x:up,y:left; local x:right,y:up)
        double x = -cloudtogrid->points[i].y;
        double y = cloudtogrid->points[i].x;
        double z = cloudtogrid->points[i].z;	

        //double x = cloudtogrid->points[i].x;
        //double y = cloudtogrid->points[i].y;
        //double z = cloudtogrid->points[i].z;

        if(lidar_transform_switch){
          transPoint(x,y,z);
        }

        if(x < Map_xMin || x > Map_xMax || y < Map_yMin || y > Map_yMax) //task01 size of gridmap
        {
          continue;
          // ROS_INFO(" size grip");
        }
        else if (x<=car_width/2 && x>=-car_width/2 && y<=car_length/2 && y>=-car_length/2)
        {
          continue;
        }
        else
        {
          xCell = (int) ((x - Map_xMin) / cellResolution);
          yCell = (int) ((y - Map_yMin) / cellResolution);

          // gen Occupancy Grid param 
          numb_of_grid = yCell * xCells + xCell;
          if (numb_of_grid > size)
          {
            continue;
          }
          grid->data[yCell * xCells + xCell] += 1;

          // ROS_INFO(" callback grip");
        }
      }
      for(size_t j = 0;j < size;j++ )
      {
        if(grid->data[j] >= grid_counted ) 
        {
          grid->data[j] = 100;          
        }
        else
        {
          grid->data[j] = 0;
        }
      }
    }
}

void LidarDectector::clipperCloudPlane(const PointCloud::Ptr& cloud_filter)
{
  double z_min=0;
  double z_max=0;
  double layer=0;
  PointCloud::Ptr fixpoint(new PointCloud());
  //set param
  float zc_max=z_limit_max,zc_min =z_limit_min,x_max=x_limit_max,x_min=x_limit_min,y_max=y_limit_max,y_min=y_limit_min;
  if(0 != cloud_filter->size())
  {
    // ROS_INFO("velodyne point size %ld ",cloud_filter->size() );
    // ROS_INFO("----------------------------------------------");
    //build condition
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
                                new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, x_max)) //threshold
                              );
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
                                new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, x_min)) //threshold
                              );

    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
                                new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, y_max)) //threshold
                              );
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
                                new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, y_min)) //threshold
                              );

    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
                                new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, zc_max)) //threshold
                              );
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
                                new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, zc_min)) //threshold
                              );
    // Build the filter using condition 
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud (cloud_filter);
    // Apply filter
    condrem.filter (*fixpoint); 
    // ROS_INFO("fixpoint point size %ld ",fixpoint->size() );

    //projection the shadows on the coefficient ax+by+cz+d=0
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;
    //project 
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (fixpoint);
    proj.setModelCoefficients (coefficients);
    proj.filter (*clipperedPoint);
  }
}

void LidarDectector::transPoint(double &x, double &y, double &z)
{
    Eigen::Vector3f pt0,pt1;
    pt0 << x, y, z;
    Eigen::Matrix3f rotX, rotY, rotZ;
    rotX << 1,0,0, 0,cos(theta_x_lidar),-sin(theta_x_lidar), 0,sin(theta_x_lidar),cos(theta_x_lidar);
    rotY << cos(theta_y_lidar),0,sin(theta_y_lidar), 0,1,0, -sin(theta_y_lidar),0,cos(theta_y_lidar);
    rotZ << cos(theta_z_lidar),-sin(theta_z_lidar),0, sin(theta_z_lidar),cos(theta_z_lidar),0, 0,0,1;

    pt1 = rotZ*rotY*rotX*pt0; //H P B
    x = pt1(0)+offset_x_lidar;
    y = pt1(1)+offset_y_lidar;
    z = pt1(2)+offset_z_lidar;
}

nav_msgs::OccupancyGridPtr LidarDectector::makeGrid(const PointCloud::Ptr& points) {
  clipperCloudPlane(points);
  Grid(clipperedPoint);
  return grid;
}
