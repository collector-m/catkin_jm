#include "decision/functions.hpp"


int main(int argc, char *argv[])
{

	ros::init(argc, argv, "map_builder");

	ros::NodeHandle n;
	ros::Time odomReceivedTime;

	// define package path
	std::string path = ros::package::getPath("decision");
	ROS_INFO("Package path: %s\n",path.c_str());
	std::string rawMap_path = path+"/maps/rawMap.txt";

	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&odomCallback, _1, &postureData, &odomReceivedTime));

	ros::Rate loop_rate(10);

	while (ros::ok())
	{

		ros::spinOnce();

    	static std::ofstream fsRoad(rawMap_path, std::ofstream::trunc);

		fsRoad << std::setprecision(15) << 1 << " " << postureData.imuData.longitude << " " << postureData.imuData.latitude << " " << postureData.imuData.yaw << " " << 4 << " " << 0 << std::endl;

		loop_rate.sleep();

	}

	return 0;
}