#include "lidar_detect_node.h"
#include <std_msgs/String.h>
#include <cmath>

const double LidarDetectNode::PI = 3.1415926;

LidarDetectNode::LidarDetectNode() : left_cloud(new PointCloud()), 
right_cloud(new PointCloud()),
center_cloud(new PointCloud())
{
    ros::param::get("~z_limit_min",z_limit_min);
    ros::param::get("~z_limit_max",z_limit_max);
    ros::param::get("~y_limit_min",y_limit_min);
    ros::param::get("~y_limit_max",y_limit_max);
    ros::param::get("~x_limit_min",x_limit_min);
    ros::param::get("~x_limit_max",x_limit_max);
    ros::param::get("lidar_transform_switch",lidar_transform_switch);

    ros::param::get("~car_width",car_width);
    ros::param::get("~car_length",car_length);

    ros::param::get("~Map_xMax",Map_xMax);
    ros::param::get("~Map_yMax",Map_yMax);
    ros::param::get("~Map_xMin",Map_xMin);
    ros::param::get("~Map_yMin",Map_yMin);
    ros::param::get("~cellResolution",cellResolution);
    ros::param::get("~grid_counted",grid_counted);

    ros::param::get("~sub_left_topic", sub_left_topic);
    ros::param::get("~sub_right_topic", sub_right_topic);
    ros::param::get("~pub_heart_topic", pub_heart_topic);
    ros::param::get("~pub_gridmap_topic", pub_gridmap_topic);

    ros::param::get("~yaw_left_to_center", yaw_left_to_center);
    yaw_left_to_center = yaw_left_to_center * PI / 180.0;
    ros::param::get("~yaw_right_to_center", yaw_right_to_center);
    yaw_right_to_center = yaw_right_to_center * PI / 180;
    ros::param::get("~x_left_to_center", x_left_to_center);
    ros::param::get("~x_right_to_center", x_right_to_center);
    ros::param::get("~y_left_to_center", y_left_to_center);
    ros::param::get("~y_right_to_center", y_right_to_center);
    
    left_callback = 0;
    right_callback = 0;
    sub_left = nh_.subscribe(sub_left_topic,1,&LidarDetectNode::subLeftCallback,this);
    sub_right = nh_.subscribe(sub_right_topic,1,&LidarDetectNode::subRightCallback,this);
   
    pub_heart = nh_.advertise<std_msgs::UInt32>(pub_heart_topic, 1, this);
    pub_gridmap = nh_.advertise<nav_msgs::OccupancyGrid>(pub_gridmap_topic, 1, this);

    ld.setParam(
        x_limit_min,
        x_limit_max,
        y_limit_min,
        y_limit_max,
        z_limit_min,
        z_limit_max,
        car_width,
        car_length,
        theta_x_lidar,
        theta_y_lidar,
        theta_z_lidar,
        offset_x_lidar,
        offset_y_lidar,
        offset_z_lidar,
        lidar_transform_switch,
        grid_counted,
        Map_xMax,
        Map_yMax,
        Map_xMin,
        Map_yMin,
        cellResolution
    );  
}

void LidarDetectNode::subLeftCallback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{ 
    ++left_callback;
    ROS_INFO(" left_callback %d : ", left_callback);
    pcl::fromROSMsg(*pc, *left_cloud);
    ROS_INFO("left point size %ld ",left_cloud->size());
}

void LidarDetectNode::subRightCallback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
    ++right_callback;
    ROS_INFO(" right_callback %d : ", right_callback);
    pcl::fromROSMsg(*pc, *right_cloud);
    ROS_INFO("right point size %ld ",right_cloud->size());
}

void LidarDetectNode::fusion_to_center() {
    center_cloud->points.resize(left_cloud->points.size() + right_cloud->points.size());
    for (size_t i=0; i<left_cloud->points.size(); ++i) {
        center_cloud->points[i].x = left_cloud->points[i].x * cos(yaw_left_to_center)
                                        + left_cloud->points[i].y * sin(yaw_left_to_center)
                                        - x_left_to_center;
        center_cloud->points[i].y = left_cloud->points[i].y * cos(yaw_left_to_center)
                                        - left_cloud->points[i].x * sin(yaw_left_to_center)
                                        - y_left_to_center;
        center_cloud->points[i].z = left_cloud->points[i].z;
    }

    size_t j = left_cloud->points.size();

    for (size_t i=0; i<right_cloud->points.size(); ++i) {
        
        center_cloud->points[j+i].x = right_cloud->points[i].x * cos(yaw_right_to_center)
                                        + right_cloud->points[i].y * sin(yaw_right_to_center)
                                        - x_right_to_center;
        center_cloud->points[j+i].y = right_cloud->points[i].y * cos(yaw_right_to_center)
                                        - right_cloud->points[i].x * sin(yaw_right_to_center)
                                        - y_right_to_center; 
        center_cloud->points[j+i].z = right_cloud->points[i].z;
    }

}


void LidarDetectNode::main_loop() {
    std_msgs::UInt32 heart;
    ros::Rate loop_rate(20);
    while(nh_.ok()) {
        ros::spinOnce();
        fusion_to_center();
        pub_gridmap.publish(ld.makeGrid(center_cloud));
        heart.data = left_callback + right_callback;
        pub_heart.publish(heart);
        loop_rate.sleep();
    }
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "point_process");
    LidarDetectNode ldn;
    ldn.main_loop();
	return 0;
}