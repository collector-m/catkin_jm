#include "decision/functions.hpp"

// obstacle distance (m)
#define OBSTACLE_DIS_MAX 20
#define OBSTACLE_DIS_MID 10
#define OBSTACLE_DIS_MIN 5

// speed definition (m/s)
#define SPEED_ULTRA 20
#define SPEED_HIGH 4.5
#define SPEED_MID 4.0
#define SPEED_LOW 2.0
#define SPEED_ZERO 0
#define SPEED_TEST 1.5
#define SPEED_ACC_MAX 40/3.6//8.5

// acc definitions
#define ACC_ULTRA 4
#define ACC_HIGH 3 
#define ACC_MID 2
#define ACC_LOW 1

#define FOLLOW_ROAD_MAP_FLAG 1
#define FOLLOW_VISION_AND_LIDAR_FLAG 1
#define FOLLOW_VISION_FLAG 1
#define FOLLOW_LIDAR_FLAG 1

#define POSTURE_SENSOR_FLAG 1 // 0: both, 1: IMU, 2: GPS

#define LIDAR_INSTALLED_FLAG true 


#define OVERTAKE_DIST_THR 40.0

#define LOOP_RATE 20


// reconfigure callback
void callbackconfig(decision::decision_paramConfig &config, uint32_t level)
{
  // ros::Rate r(1);
	params_modify_switch = config.params_modify_switch;
	control_modify_switch = config.control_modify_switch;
	path_record_switch = config.path_record_switch;

	if(params_modify_switch == 1){

		//preview params
		previewInner_param = config.previewInner_param;
		previewCurvature0_param = config.previewCurvature0_param;
		previewCurvature5_param = config.previewCurvature5_param;
		uiData.pathNumber = config.pathNumber;

		//PID params
		kp_large = config.kp_large;
		kd_large = config.kd_large;
		kp_small = config.kp_small;
		kd_small = config.kd_small;
		FILTER_RATE = config.FILTER_RATE;

		//curve params
		extend_length = config.extend_length;

		if(control_modify_switch == 1){
			decisionData.targetSpeedTest = config.targetSpeed/3.6;
			decisionData.targetSteeringAngle = config.targetSteeringAngle;
			decisionData.targetSteeringVelocity = config.targetSteeringVelocity;
		}
	}
	else 
		control_modify_switch = 0;
  // r.sleep();
}

//-------------------------------------------------main------------------------------------------------

int main(int argc, char *argv[])
{
	// define package path
	std::string path = ros::package::getPath("decision");
	ROS_INFO("Package path: %s\n",path.c_str());
	std::string roadMap_path = path+"/maps/roadMap.txt";
	std::string stopPoint_path = path+"/maps/stopPoint.txt";
	std::string pathNumber_path = path+"/maps/pathNumber.txt";
	std::string UIMap_params_path = path+"/maps/UIMap_params.txt";

	//--------------------------------------------- load road net ---------------------------------------------
	//for test:
	std::vector<std::vector<RoadPoint>> rawRoadPointss;

	rawRoadPointss.reserve(10000);
	uint32_t rawRoadPointssSize = loadRoadMapSize(roadMap_path);
	rawRoadPointss.resize(rawRoadPointssSize+1);

	for (uint32_t i = 0; i < rawRoadPointssSize + 1; i++) {
		rawRoadPointss[i].reserve(1000);
	}
	
	loadRoadMapConverted(roadMap_path, rawRoadPointss, OFFSET_X, OFFSET_Y);
	
	std::cout << "-------Map loading done." << std::endl;
	
	std::vector<RoadPoint> rawStopPoints;
	uint32_t rawStopPointsSize = loadStopPointsSize(stopPoint_path);
	rawStopPoints.reserve(rawStopPointsSize + 1);
	
	loadStopPoints(stopPoint_path, rawStopPoints, OFFSET_X, OFFSET_Y);

	std::cout << "-----uiData.pathNumberStop Points loading done." << std::endl;
	buildFinalRoadMap(rawRoadPointss, rawStopPoints);

	std::cout << "--------Final map building done." << std::endl;
	
	std::vector<std::vector<uint32_t>> pathNumberLists;
	pathNumberLists.reserve(MAX_PATH);
	pathNumberLists.resize(MAX_PATH);
	for ( int i = 0; i < MAX_PATH; i ++){
		pathNumberLists.reserve(MAX_PATH_NUMBER);
	}
	loadPathNumber(pathNumber_path, pathNumberLists);

	std::cout << "Path Number loading done." << std::endl;

	// UIMap params load
	read_ui(UIMap_params_path, uiData);

	std::cout << "UIMap params loading done." << std::endl;

  //--------------------------------------------- road net load done ---------------------------------------------

  decisionData.postureSensorMode = POSTURE_SENSOR_FLAG;

	// ROS initiate
	ros::init(argc, argv, "decision");

	// ROS node create
	ros::NodeHandle n;

	// reconfigure callback ----------------------------------
  dynamic_reconfigure::Server<decision::decision_paramConfig> dynamic_reconfigure_server_;
  dynamic_reconfigure::Server<decision::decision_paramConfig>::CallbackType f;
  f = boost::bind(&callbackconfig, _1, _2);
	dynamic_reconfigure_server_.setCallback(f);

	// timer for receivers
	ros::Time odomReceivedTime;
	ros::Time lidarReceivedTime;

	// publisher establish -----------------------------------
	ros::Publisher roadPoint_pub_ = n.advertise<geometry_msgs::PoseArray>("route", 1, true);
	ros::Publisher past_path_pub_ = n.advertise<geometry_msgs::PoseArray>("past_path", 1, true);
	ros::Publisher curve_pub_ = n.advertise<geometry_msgs::PoseArray>("curve", 1, true);
	ros::Publisher virtual_curve_pub_ = n.advertise<geometry_msgs::PoseArray>("virtual_curve", 1, true);
	
	ros::Publisher previewPoint_pub = n.advertise<geometry_msgs::PoseArray>("preview_point", 1, true);
	
	// main command publisher
	ros::Publisher decision_pub = n.advertise<std_msgs::Float64MultiArray>("decision_pub", 1);

	// test publisher
	ros::Publisher steering_angle_pub = n.advertise<std_msgs::Float64>("steering_angle", 1);
	ros::Publisher steering_speed_pub = n.advertise<std_msgs::Float64>("steering_speed", 1);
	ros::Publisher angle_diff_pub = n.advertise<std_msgs::Float64>("angle_diff", 1);
	ros::Publisher PID_value_pub = n.advertise<std_msgs::Float64>("pid_value", 1);
	ros::Publisher preview_dist_pub = n.advertise<std_msgs::Float64>("preview_dist", 1);
	ros::Publisher converted_yaw_pub = n.advertise<std_msgs::Float64>("converted_yaw", 1);
	ros::Publisher converted_preview_angle_pub = n.advertise<std_msgs::Float64>("converted_preview_angle", 1);
	std_msgs::Float64 steering_test; 

	// ros::Publisher target_speed_pub = n.advertise<std_msgs::Float64>("target_speed", 1);
	// ros::Publisher current_speed_pub = n.advertise<std_msgs::Float64>("current_speed", 1);
	ros::Publisher map_speed_pub = n.advertise<std_msgs::Float64>("map_target_speed", 1);
	std_msgs::Float64 speed_test; 

	// decision pub message establish
	std_msgs::Float64MultiArray decision_pub_array;
	decision_pub_array.data.reserve(7);

	// define global loop rate
	ros::Rate loop_rate(LOOP_RATE);
  double reconfig_counter = 0;

	// global map establish
	map_t* global_map = map_alloc();
  ROS_ASSERT(global_map);

	// subscriber establish ----------------------------------
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&odomCallback, _1, &postureData, &odomReceivedTime));
	ros::Subscriber grid_sub = n.subscribe<nav_msgs::OccupancyGrid>("gridmap", 1, boost::bind(&gridCallback, _1, &lidarData, &lidarDecisionData, &lidarReceivedTime, global_map));
	ros::Subscriber decision_sub = n.subscribe<std_msgs::Float64MultiArray>("decision_sub", 1, boost::bind(&decisionSubCallback, _1, &decisionData, &uiData));

	while (ros::ok())
	{
		// ROS_INFO("RECONFIG: params_modify_switch: %d control_modify_switch: %d", params_modify_switch, control_modify_switch);
		// ROS_INFO("RECONFIG: pathNumber: %d ",uiData.pathNumber);
    // ROS_INFO("RECONFIG: previewInner: %.3f, previewCurvature0: %.3f, previewCurvature5: %.3f ",previewInner_param, previewCurvature0_param, previewCurvature5_param);
		// ROS_INFO("RECONFIG: targetSpeed: %.3f, targetSteeringAngle: %.3f, targetSteeringVelocity: %.3f ",decisionData.targetSpeedTest, decisionData.targetSteeringAngle, decisionData.targetSteeringVelocity);

    // main processor ----------------------------------------
    decisionData.currentState = STATE_INITIAL;

    pathSelection(decisionData, uiData, pathNumberLists.size()-1);

		roadPointPublisher(roadPoint_pub_,rawRoadPointss,pathNumberLists, decisionData.pathNumber);

    ROS_INFO("Listening <<<<<<<<<<<<<<<<");

		ros::spinOnce();

		pastPathPublisher(past_path_pub_, postureData);

		ROS_INFO("now: %.3f, Odom time: %.3f",ros::Time::now().toSec(), odomReceivedTime.toSec());
    if((ros::Time::now().toSec()-odomReceivedTime.toSec())>1){
      postureData.b_isAlive = 0;
      postureData.imuData.b_isValid = 0;
      postureData.imuData.b_gpsValid = 0;
			ROS_ERROR("GPS DIED!!!");
    }

		ROS_INFO("now: %.3f, lidar time: %.3f",ros::Time::now().toSec(), lidarReceivedTime.toSec());
		if((ros::Time::now().toSec()-lidarReceivedTime.toSec())>1){
      lidarData.b_isAlive = 0;
			lidarData.b_isValid = 0;
			ROS_ERROR("LIDAR DIED!!!");
    }

    if (1 == postureData.b_isAlive &&
     		1 == postureData.imuData.b_isValid &&
//      1 == postureData.gpsData.b_isDifferential &&
//      1 == lidarData.b_isAlive &&
//      1 == lidarData.b_isValid &&
        1 == FOLLOW_ROAD_MAP_FLAG){

		  ROS_INFO("RECEIVED GPS DATA <<<<<<<<<<<<<<<");
			ROS_INFO("GPS X: %.3f Y: %.3f YAW: %.3f SPD: %.3f",postureData.imuData.longitude, postureData.imuData.latitude,postureData.imuData.yaw, postureData.imuData.velocity);

			//UI coords calculate
			calculate_ui(uiData, postureData);

      //get current velocity
      decisionData.velocity = postureData.imuData.velocity;
			if(decisionData.velocity < 0)
				decisionData.velocity = 0;

      // steering angle & speed
      followRoadMap(postureData.imuData.longitude, postureData.imuData.latitude, postureData.imuData.yaw, postureData.imuData.velocity, rawRoadPointss, decisionData, pathNumberLists);
			steering_test.data = decisionData.targetSteeringAngle;
			previewPointPublisher(previewPoint_pub, decisionData);
			std::cout << "---decisionData.currentState:" << decisionData.currentState << std::endl;
      if (decisionData.currentState == STATE_FOLLOW_ROADMAP) {
        std::cout << GREEN << "following gps" << RESET << std::endl;
				decisionData.targetWorkMode = 1;
      }
      else {
        std::cout << RED << "no road point!" << RESET << std::endl;
				decisionData.targetWorkMode = 0;
      }

    }
    else{
      ROS_WARN("NO GPS DATA RECEIVED OR VALID GPS DATA!!!!");
			decisionData.targetWorkMode = 0;
			// decisionData.targetSpeed = 0;
    }

    // target speed decision for intersection
		ROS_INFO("Speed After Follow: %.3f",decisionData.targetSpeed*3.6);
    // trafficLightJudge (postureData, cvData, rawStopPoints, decisionData);
		// ROS_INFO("Speed After Light : %.3f",decisionData.targetSpeed*3.6);
    mapJudge(decisionData, cvData, visionData);
		ROS_INFO("Speed After Judge : %.3f",decisionData.targetSpeed*3.6);

    reconfig_counter++;

    if(true == LIDAR_INSTALLED_FLAG && true == lidarData.b_isAlive && decisionData.targetWorkMode >= 1){
			ROS_INFO("RECEIVED LIDAR DATA <<<<<<<<<<<<<<<");
      if(true == decisionData.b_onRoadPoint){

				ROS_INFO("ON ROAD, JUDGING LIDAR *************");
        lidarJudgeLongerPreview(decisionData, lidarDecisionData, postureData, lidarData, rawRoadPointss, curve_pub_, virtual_curve_pub_);

				ROS_INFO("[LL] Target Speed ->");
				ROS_INFO("To Preview Point:       %.3f km/h",decisionData.targetSpeed*3.6);
				ROS_INFO("To Longer Lidar Range:  %.3f km/h",lidarDecisionData.targetSpeed*3.6);

				decisionData.targetWorkMode = 2;

        // std::cout << "[LL] Target Speed ->" << std::endl;
        // std::cout << "To Preview Point:                               " << decisionData.targetSpeed*3.6 <<" km/h"<< std::endl;
        // std::cout << "To Longer Lidar Range:                          " << lidarDecisionData.targetSpeed*3.6 <<" km/h"<< std::endl;

    //    decisionData.targetSpeed = decisionData.targetSpeed < lidarDecisionData.targetSpeed? decisionData.targetSpeed: lidarDecisionData.targetSpeed;
      }
			else{
				ROS_WARN("OFF ROAD, NO LIDAR JUDGE!!!!");
			}
    }
		else{
			ROS_WARN("NO LIDAR DATA RECEIVED OR VALID GPS DATA!!!!");
		}

		clearCurveList(decisionData.curveList);

		//ui
		static double lastLongitude = 0;
		static double lastLatitude = 0;
		if (uiData.b_stopFlag == false) {
			lastLongitude = postureData.gpsData.longitude;
			lastLatitude = postureData.gpsData.latitude;
		}
		else{
			decisionData.targetSpeed = 0.0;
			double errorX = lastLongitude - postureData.gpsData.longitude;
			double errorY = lastLatitude - postureData.gpsData.latitude;
			//decisionData.targetAccLevel = ACC_UISTOP;
			std::cout << "Last Posture: " << errorX << ", " << errorY << std::endl;
			std::cout << "Stopped because of UI." << std::endl;
		}

// 		if (postureData.imuData.velocity < 0.7 && decisionData.targetSpeed > ALMOST_ZERO){ // avoid jerk at launch !
//     	decisionData.targetSpeed = 0.5;
// //    std::cout << "targetSpeed Limited at low vehicleSpeed!" << std::endl;
//   	}


    // decision publisher --------------------------------------------

    // ROS_INFO("Sending   >>>>>>>>>>>>>>>>\n");

		decision_pub_array.data.clear();
		decision_pub_array.data.push_back(decisionData.targetWorkMode); //0: no GPS 1: followRoadmap 2: lidarAlive

		if(control_modify_switch == 1)
    	decision_pub_array.data.push_back(decisionData.targetSpeedTest); // m/s
		else
			decision_pub_array.data.push_back(decisionData.targetSpeed);

			// decisionData.targetSteeringVelocity = 10;
		// ROS_INFO("STEERING-ANGLE-OUT: %.3f",decisionData.targetSteeringAngle);
    decision_pub_array.data.push_back(decisionData.targetSteeringAngle); // degree [-540,540]
    decision_pub_array.data.push_back(decisionData.targetSteeringVelocity); // degree/s
		decision_pub_array.data.push_back(uiData.b_pos_isValid); // 0: no GPS 1: hmi coord valid
		decision_pub_array.data.push_back(uiData.x_ui); // hmi coord
		decision_pub_array.data.push_back(uiData.y_ui); // hmi coord
		decision_pub_array.data.push_back(0); // brake pressure
		decision_pub_array.data.push_back(0); // tempWorkMode

    decision_pub.publish(decision_pub_array);


		steering_angle_pub.publish(steering_test);
		steering_test.data = decisionData.targetSteeringVelocity;
		steering_speed_pub.publish(steering_test);
		steering_test.data = angleDiffTest;
		angle_diff_pub.publish(steering_test);
		// steering_test.data = PID_value;
		// PID_value_pub.publish(steering_test);
		steering_test.data = preview_dist_test;
		preview_dist_pub.publish(steering_test);
		steering_test.data = converted_yaw;
		converted_yaw_pub.publish(steering_test);
		steering_test.data = converted_preview_angle;
		converted_preview_angle_pub.publish(steering_test);

		speed_test.data = decisionData.targetSpeed*3.6;
		// target_speed_pub.publish(speed_test);
		speed_test.data = decisionData.velocity*3.6;
		// current_speed_pub.publish(speed_test);
		speed_test.data = mapTargetSpeed*3.6;
		map_speed_pub.publish(speed_test);

		ROS_INFO("Spinning  --------------------------------\n");

		loop_rate.sleep();

	}

	free(global_map->cells);
	free(global_map);

	return 0;
}