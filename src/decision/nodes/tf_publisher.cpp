#include "decision/functions.hpp"


int main(int argc, char *argv[])
{

	// define package path
	std::string path = ros::package::getPath("decision");
	ROS_INFO("Package path: %s\n",path.c_str());
	std::string UIMap_params_path = path+"/maps/UIMap_params.txt";

	// UIMap params load
	read_ui(UIMap_params_path, uiData);
	std::cout << "UIMap params loading done." << std::endl;

	// UiData uiData;
	ros::init(argc, argv, "tf_publisher");

	ros::NodeHandle n;
	ros::Time odomReceivedTime;
	tf::TransformBroadcaster broadcaster0; 
	tf::TransformBroadcaster broadcaster1; 
	tf::TransformBroadcaster broadcaster2; 

	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&odomCallback, _1, &postureData, &odomReceivedTime));

	ros::Rate loop_rate(60);

	int count = 0;
	while (ros::ok())
	{

		ROS_INFO_ONCE("Sending TF ...\n");

		geometry_msgs::TransformStamped map2odom;
		map2odom.header.frame_id = "map";
		map2odom.child_frame_id = "odom";
		// update transform
		map2odom.header.stamp = ros::Time::now();
		map2odom.transform.translation.x = uiData.GlobalOffset_X;
		map2odom.transform.translation.y = uiData.GlobalOffset_Y;
		map2odom.transform.translation.z = 0;
		map2odom.transform.rotation = tf::createQuaternionMsgFromYaw(0);
		broadcaster0.sendTransform(map2odom);

		geometry_msgs::TransformStamped odom2baselink;
		odom2baselink.header.frame_id = "odom";
		odom2baselink.child_frame_id = "base_link";
		// update transform
		odom2baselink.header.stamp = ros::Time::now();
		odom2baselink.transform.translation.x = postureData.imuData.longitude-uiData.GlobalOffset_X;
		odom2baselink.transform.translation.y = postureData.imuData.latitude-uiData.GlobalOffset_Y;
		odom2baselink.transform.translation.z = 0;
		odom2baselink.transform.rotation = tf::createQuaternionMsgFromYaw(postureData.imuData.yaw/180*M_PI * (-1) );
		broadcaster1.sendTransform(odom2baselink);

		geometry_msgs::TransformStamped baselink2velodyne;
		baselink2velodyne.header.frame_id = "base_link";
		baselink2velodyne.child_frame_id = "rslidar";
		// update transform
		baselink2velodyne.header.stamp = ros::Time::now();
		baselink2velodyne.transform.translation.x = 0;
		baselink2velodyne.transform.translation.y = 0;
		baselink2velodyne.transform.translation.z = 0;
		baselink2velodyne.transform.rotation = tf::createQuaternionMsgFromYaw(0);
		broadcaster2.sendTransform(baselink2velodyne);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}