#ifndef __decision__functions__
#define __decision__functions__

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <cstdlib>
// #include <decision_paramConfig.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/timer.hpp>

#include "qingColor.h"

//ROS include
//ros map
#include <map>
#include "ros/ros.h"
#include "ros/assert.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

#include <ros/package.h>
// reconfigure callback 
#include <dynamic_reconfigure/server.h>
#include <decision/decision_paramConfig.h>

// Original hpp below

//#define MAX_ID 1
#define MINIMAL_ID 1

#define MIN(a,b) ((a)<=(b)?:(a),(b))

//#define OFFSET_X -57291.78 - 2
//#define OFFSET_X -57309.491 - 2
//#define OFFSET_X -57291.027872297
//#define OFFSET_X -57291.72787
#define OFFSET_X 0
//#define OFFSET_Y 4453577.394 + 3
//#define OFFSET_Y 4453587.50552676
#define OFFSET_Y 0

// #define offset_tempX 29401.225
// #define offset_tempY 4359900.766

//#define STATE_READY 0
//#define STATE_START 1
//#define STATE_ROADMAP 2
//#define STATE_VISION 3
//#define STATE_LIDAR 4 

#define X_SIZE_MIN -20.0 // lateral
#define X_SIZE_MAX 20.0 // X-symmetric
#define Y_SIZE_MIN 0.0
#define Y_SIZE_MAX 80.0 // longitudinal
#define GRID_X_RESOLUTION 0.2
#define GRID_Y_RESOLUTION 0.2
#define GRID_INDEX_SIZE ((X_SIZE_MAX-X_SIZE_MIN)/GRID_X_RESOLUTION)*((Y_SIZE_MAX-Y_SIZE_MIN)/GRID_Y_RESOLUTION)
#define GRID_INDEX_X_SIZE (X_SIZE_MAX-X_SIZE_MIN)/GRID_X_RESOLUTION

// #define LIDAR_SAFE_ZONE_MIN 1.7
extern double LIDAR_SAFE_ZONE_MIN;
#define LIDAR_SAFE_ZONE_MAX 4.6
// #define LIDAR_SAFE_DISTANCE_MIN 4.0 // +5 bias 
extern double LIDAR_SAFE_DISTANCE_MIN;
// #define LIDAR_SAFE_DISTANCE_MAX 25.0 // +5 bias
extern double LIDAR_SAFE_DISTANCE_MAX;
#define DEFAULT_LIDAR_PREVIEW_DISTANCE 20 

// curves vertical check
#define CURVE_POINT_RESOLUTION 0.2
#define GRID_X_RESOLUTION_CHECK GRID_X_RESOLUTION/2
#define GRID_Y_RESOLUTION_CHECK GRID_Y_RESOLUTION/2
#define LIDAR_SAFE_ZONE_MAX_R_NUM LIDAR_SAFE_ZONE_MAX/2/MIN(GRID_X_RESOLUTION,GRID_Y_RESOLUTION)
#define LIDAR_SAFE_ZONE_MIN_R_NUM LIDAR_SAFE_ZONE_MIN/2/MIN(GRID_X_RESOLUTION,GRID_Y_RESOLUTION)



#define LIDAR_MAX_SPEED 60 / 3.6 // m/s
#define LIDAR_MIN_SPEED 5 / 3.6 // m/s
#define PREVIEW_DISTANCE_RATIO 0.5//0.75
//take over
#define TAKE_OVER_PERIOD 200 //ms
#define OVER_TAKE_SPEED_THRESHOLD 5 / 3.6
#define TAKE_OVER_MAX_SPEED 5 / 3.6 //old = 2.8 // m/s

//#define CURVES_SIZE 7
//#define CURVE_LENGTH 100

extern double MAX_ABS_STEERING_RATE;
// #define MAX_ABS_STEERING_RATE 25
extern double MAX_ABS_STEERING_RATE_TAKE_OVER;
//#define MAX_ABS_STEERING_RATE 2.5
#define MAX_ABS_STEERING_ANGLE_HIGH_SPEED 45
#define MAX_ABS_STEERING_ANGLE_LOW_SPEED 450


#define STATE_INITIAL 0
#define STATE_FOLLOW_ROADMAP 1
#define STATE_PARKING 2

//curve related.
#define CP_DISTANCE 3
#define CURVE_POINT_NUM 150
#define CURVE_NUM 2
#define CURVE_DISTANCE 3.5 //huandaojuli


//path number
#define MAX_PATH 10
#define MAX_PATH_NUMBER 100

#define ALMOST_ZERO 0.01

#define OFFEST_LIDARMAP 64


//curve control point dist
extern double global_preview_dist;
extern double CP_DISTANCE_BACK;
extern double CP_DISTANCE_FRONT;
extern double global_preview_curvature;
extern double global_current_curvature;
extern double cp_dist_back_factor;
extern double cp_dist_front_factor;
//curve adjust
extern double extend_length;
//object avoid
extern double actual_curve_dist;
extern double curve_index;

extern double mapTargetSpeed;

//OFFSET
extern double offset_tempX;
extern double offset_tempY;

//Display Lidar
#define DISPLAY_GRID_ON 0 //1: only index 2: index + curve 3: origin
const int GRID_X_NUM = floor(X_SIZE_MAX/GRID_X_RESOLUTION*2);
const int GRID_Y_NUM = floor((Y_SIZE_MAX-Y_SIZE_MIN)/GRID_Y_RESOLUTION);
const int GRID_NUM = GRID_Y_NUM * GRID_X_NUM;

// reconfigure params
extern double previewInner_param; // old 2
extern double previewInner1_param;
extern double previewCurvature0_param; // old 15
extern double previewCurvature5_param;
extern bool params_modify_switch;
extern bool control_modify_switch;

//steering test
extern double angleDiffTest;
extern double PID_value;
extern double preview_dist_test;

//steering angle PID params
extern double kp_large;
extern double kd_large;
extern double kp_small;
extern double kd_small;
extern double ki_test;
extern double FILTER_RATE;

extern double converted_yaw;
extern double converted_preview_angle;

//past path record
extern bool path_record_switch;
extern bool path_record_time;

// yaw weight
extern double YAW_WEIGHT;

struct Zone{
  double xLeft = 0;
  double xRight = 0;
  double yTop = 0;
  double yTopWider = 0.0;
  double yTopEvenWider = 0.0;
  double angleLeft = 0;
  double angleRight = 0;
  double pathAngle = 0;
  bool b_isValid = false;
  bool b_leftCandidate = false;
  bool b_rightCandidate = false;
};

struct LidarData{
  std::vector<uint32_t> blockedAreaIndex;
  bool b_isAlive = false;
  bool b_isValid = false;
  Zone zone;
};

struct LaneParameter{
  bool b_isSolid;
  uint8_t laneType;
  int32_t realParameter1;
  int32_t realParameter2;
  int32_t realParameter3;
  int32_t realLineK;
  int32_t realLineB;
  int32_t latOffset;
  int32_t yawAngle;
  int32_t latOffsetFiltered;
  int32_t yawAngleFiltered;
};

struct LaneData{
  uint8_t laneCount;
  uint8_t laneStatus[4];
//  int32_t offsetRight;
//  int32_t offsetLeft;
//  int32_t angleError;
  double offsetRight;
  double offsetLeft;
  double angleError;
  LaneParameter leftLaneParameter;
  LaneParameter rightLaneParameter;
};

struct TrafficLightData{
  bool b_leftIsValid;
  uint8_t leftPassable;
  bool b_straightIsValid;
  uint8_t straightPassable;
  bool b_rightIsValid;
  uint8_t rightPassable;
  uint8_t start;
};

struct TrafficSignData{
  bool b_isValid;
  uint16_t pattern;
};

struct VisionData{
  LaneData laneData;
  TrafficLightData trafficLightData;
  TrafficSignData trafficSignData;
  
  bool b_isAlive;
};


struct GpsData{
  double longitude;
  double latitude;
  bool b_isValid;
  bool b_isDifferential;
};

struct ImuData{
  double velocity;
  double roll;
  double yaw;
  double pitch;
  bool b_isValid;
  
  double yawRate = 0.0;

  bool b_gpsValid = false;
  double longitude = 0.0;
  double latitude = 0.0;
};

struct PostureData{
  GpsData gpsData;
  ImuData imuData;
  
  std::vector<double> longitude_array;
  std::vector<double> latitude_array;
  std::vector<double> yaw_array;

  bool b_isAlive;
};

struct ActuatorData{
  uint8_t aebStatus;
  uint8_t epsStatus;
  uint8_t torqueStatus;
  uint8_t decStatus;
  uint8_t systemStatus;
  uint8_t gearControlStatus;
  uint8_t breakPedalStatus;
  uint8_t cruiseStatus;
  uint8_t gearPositionStatus;
  double vehicleSpeedStatus = 0;
  double vehicleSteerStatus = 0;
  double vehicleSpeed = 0;
  uint8_t wsuSceneStatus = 0;
  uint8_t wsuAlarmLevelStatus = 0;
  
  bool b_isAlive;
};

struct UiData{
//  uint16_t nextId = 0;
  bool b_stopFlag = false;
  bool b_speedOff = true;
  bool b_steeringOff = true;
  bool b_config1 = false;
  uint16_t pathNumber = 1; // old 0
//  bool b_smallCircleSelected = true;
  bool b_isValid = false;
  bool b_pos_isValid = false;
  
//  bool b_readyToGo = false;
//  uint16_t lastId = 0;
  
  double x_ui = 0;
  double y_ui = 0;

  bool b_isAlive;

  double GlobalOffset_X;
  double GlobalOffset_Y;
  double UIMap_Xoffset0;
  double UIMap_Yoffset0;
  double UIMap_Scale;
  double UIMap_Margin;
  double UIMap_rotateDeg;
  double UIMap_Xoffset; 
  double UIMap_Yoffset;
  double UIMap_Ymax;
};

/*
struct IovData{
  bool b_isValid = false;
  uint32_t phase = 0;
  uint32_t time = 0;
  double advSpd = 0;
  
  bool b_isAlive;
};
*/

struct IovData{
  bool b_isValid = false;
  bool b_isAlive = false;
  boost::posix_time::ptime lastPacketTime;

  bool b_vehicleValid = false;
  uint32_t vehicleId = 0;
  uint32_t vehicleType = 0;
  double vehicleGaussX = 0.0;
  double vehicleGaussY = 0.0;
  double vehicleSpeed = 0.0;
  double vehicleTime = 0.0;
  double vehicleAdvisedSpeed = 0.0;

  bool b_lightValid = false;
  uint32_t lightPhase = 0;
  double lightTime = 0.0;
  double lightAdvisedSpeed = 0.0;
};

struct CvData{
  bool b_isValid = false;
  bool b_isAlive = false;
  boost::posix_time::ptime lastPacketTime;

  bool b_vehicleValid = false;
  uint32_t vehicleId = 0;
  uint32_t vehicleType = 0;
  double vehicleGaussX = 0.0;
  double vehicleGaussY = 0.0;
  double vehicleSpeed = 0.0;
  double vehicleTime = 0.0;
  double vehicleAdvisedSpeed = 0.0;

  bool b_lightValid = false;
  uint32_t lightPhase = 0;
  double lightTime = 0.0;
  double lightAdvisedSpeed = 0.0;
};

struct StartSignal{
  uint8_t data = 0;
  bool b_isValid = 0;
};

struct Point{
  double x;
  double y;
  double angle;
};

// parameters for road point: 1 road type, 2 curvature, 3 maxSpeed
// parameters for stop point: 1 point type, 2 effective area, 3 maxSpeed
struct RoadPoint{
  uint32_t index = 0;
  uint32_t roadId = 0;
  uint32_t totalIndex = 0;
  double longitude = 0.0;
  double latitude = 0.0;
  double courseAngle = 0.0;
  int16_t roadType = 0;
  double height = 0.0;
  int32_t parameter1 = 0;
  double parameter2 = 0.0;
  double parameter3 = 0.0;
  double distance = 0.0;
  bool b_isValid = false;
  double preview_dist = 0;

  double lateral_dist = 0;
  double a_b = 0;

};



struct Curve{
  int32_t index = 0;
  Point points[CURVE_POINT_NUM];
  std::vector<Point> pointList;
};

class DecisionData{
public:
  std::vector<Curve> curveList;
  RoadPoint currentPosture; // real point
  RoadPoint currentPoint; // point in roadmap
  RoadPoint previewPoint;
  RoadPoint previewPointTakeOver;
  double targetSteeringAngle = 0.0;
  double targetSteeringVelocity = 0.0;
  double targetSpeed = 0.0;
  double targetThrottle = 0.0;
  double throttleIntegral = 0.0;
  double targetBrakePressure = 0.0;
  double previewDistance = 0.0;
  uint8_t targetGearPosition = 0;
  int16_t targetAccLevel = 2;
  int32_t currentState = 0;
  int32_t currentId = 0;
  int32_t currentIndex = 0;
  int32_t currentTotalIndex = 0;
  int32_t roadType = 0;
  int32_t nextId = 0;
  int32_t nextNextId = 0;

  uint32_t targetWorkMode = 0;
  uint16_t postureSensorMode = 0;
  bool b_takeOverEnable = false;
  bool b_takingOver = false;
  bool b_comingBack = false;
  bool b_redLightStopped = false;
  bool b_onRoadPoint = false;
  
  uint16_t pathNumber = 0;
  boost::posix_time::ptime takingOverTimeCounter;
  
  bool b_isValid = false;
  bool b_isAlive;

  double velocity = 0;
  double last_advisedSpeed = 0; //last speed for lidar judge
  double last_advisedSpeedTakeOver = 0; //last speed for lidar judge
  bool stationStopFlag = 0; //station stop

  double targetSpeedTest = 0;

  double current_steering_angle = 0;
 
public:
  uint32_t updateStateMachine();
};

//struct RoadMap{
//  uint32_t roadId = 0;
//  double longitude = 0.0;
//  double latitude = 0.0;
//  double courseAngle = 0.0;
//  double distance = 0.0;
//};

class PidController{
private:
  bool b_hasStarted, b_threshIsOn;
  double kp, ki, kd, interval, frequency, lastError, errorThresh, integral, fractionValue;
  
public:
  explicit PidController (double kpInput, double kiInput, double kdInput, double errorThreshInput, bool b_threshInOnInput, double intervalInput);
  ~PidController ();
  double update (double error);
  void modifyGains(double kp, double ki, double kd);
  void resetI ();
};

//-------------------------------------------- ROS vars ------------------------------------------------

typedef struct
{
  // Occupancy state (-1 = free, 0 = unknown, +1 = occ)
  int occ_state;

  // Distance to the nearest occupied cell
  double occ_dist;

} map_cell_t;

// occupancy grid map
// Description for a map
typedef struct
{
  // Map origin; the map is a viewport onto a conceptual larger map.
  double origin_x, origin_y;
  
  // Map scale (m/cell)
  double scale;

  // Map dimensions (number of cells)
  int size_x, size_y;
  
  // The map data, stored as a grid
  map_cell_t *cells;

  // Max distance at which we care about obstacles, for constructing
  // likelihood field
  double max_occ_dist;
  
} map_t;

typedef struct{
  double v[3];
} pose_vector;

//state variables
extern LidarData lidarData;
extern VisionData visionData;
extern PostureData postureData;
extern ActuatorData actuatorData;
extern UiData uiData;
extern DecisionData decisionData;
extern DecisionData lidarDecisionData;
extern IovData iovData;
extern CvData cvData;

//-----------------------------------------------------------------------------------------------------

bool angleDiffLessThan(double angle1, double angle2, double diff);
//RoadPoint movePreviewPointForLaneChange (RoadPoint originalRoadPoint, double yaw);
void gaussConvert(double longitude1,double latitude1,double& x1,double& y1);
uint32_t loadRoadMapSize(const std::string fileName);
void loadRoadMap(const std::string fileName, std::vector<std::vector<RoadPoint>>& rawRoadPointss);
void loadRoadMapConverted(const std::string fileName, std::vector<std::vector<RoadPoint>>& rawRoadPointss, double offsetX, double offsetY);
uint32_t loadStopPointsSize(const std::string fileName);
void loadStopPoints(const std::string fileName, std::vector<RoadPoint>& rawStopPoints, double offsetX, double offsetY);
void loadTerminalPoint(const std::string fileName, RoadPoint& terminalPoint);
//void loadPathNumber(const std::string fileName, std::vector<std::vector<uint32_t>>& pathNumberss);
void loadPathNumber(const std::string fileName, std::vector<std::vector<uint32_t>>& pathNumberLists);

void buildFinalRoadMap(std::vector<std::vector<RoadPoint>>& roadPointss, const std::vector<RoadPoint>& stopPoints);

void pathSelection(DecisionData& decisionData, const UiData& uiData, const uint16_t maxPathNumber);
uint32_t getNextRoadId (uint32_t currentId, uint32_t size, const DecisionData& decisionData, std::vector<std::vector<uint32_t>>& pathNumberLists);
uint32_t getLastRoadId (uint32_t currentId, uint32_t size);

void getCoordinate(std::vector<RoadPoint>& rawRoadPoints, std::vector<RoadPoint>& roadPoints);
double pointDistance(double x1, double x2, double y1, double y2);
//void generateCandidates(RoadPoint previewPoint, uint16_t leftCandidateNumbers, uint16_t rightCandidateNumbers, double candidateDistance);

// velocity
void velocityDecision(const LidarData& lidarData, bool b_stopFromUI, const PostureData& postureData, DecisionData& decisionData);
// end of velocity


RoadPoint getCurrentPosture(double longitude, double latitude, double yaw, std::vector<std::vector<RoadPoint>>& roadPoints, int32_t currentId);
RoadPoint getCurrentPoint (double longitude, double latitude, double yaw, std::vector<std::vector<RoadPoint>>& roadPointss, int32_t lastId, int32_t nextId, std::vector<std::vector<uint32_t>>& pathNumberLists, uint16_t pathNumber);
double getAnglebtwn(double x1, double y1, double x2, double y2, double c_a);
//RoadPoint getPreviewPoint (double velocity, double curvature, RoadPoint& currentPosture, int32_t nextId, int32_t nextNextId, std::vector<std::vector<RoadPoint>>& roadPointss);
double getPreviewDistance(double velocity, double curvature, RoadPoint& currentPoint, DecisionData& decisionData);
//RoadPoint getPreviewPoint (double distance, double velocity, double curvature, RoadPoint& currentPosture, double yaw, int32_t nextId, int32_t nextNextId, std::vector<std::vector<RoadPoint>>& roadPointss);
RoadPoint getPreviewPoint (double distance, double velocity, double curvature, RoadPoint& currentPosture, double yaw, int32_t nextId, int32_t nextNextId, std::vector<std::vector<RoadPoint>>& roadPointss);
double limitSpeed(DecisionData& decisionData, double lastTargetSpeed);
double limitTargetSpeed(DecisionData& decisionData, double targetSpeed);
double limitSteeringAngle(double steeringAngle, double currentSteeringAngle, double velocity, double yaw, bool overTakeEnable);
double limitPreviewDistance(double targetDistance, double lastTargetDistance, double velocity);
  
RoadPoint getTargetPoint (RoadPoint currentPosture, std::vector<RoadPoint>& roadPoints);
double getAngle (double x0, double y0, double x1, double y1);
double getAngleDiff(double ang0, double ang1);
double getAngleDiff1(double ang0, double ang1);
double getYawDiff (double yawVehicle, double yawRoad);
//double followVision();
void followLidar(double refAngle, const PostureData& postureData, DecisionData& decisionData);
void followLane(const VisionData& visionData, const PostureData& postureData, DecisionData& decisionData);
void followRoadMap (double longitude, double latitude, double yaw, double velocity, std::vector<std::vector<RoadPoint>>& roadPointss, DecisionData& decisionData, std::vector<std::vector<uint32_t>>& pathNumberLists);
void followLidarAndLane (const LidarData& lidarData, const VisionData& visionData, const PostureData& postureData,DecisionData& decisionData);

void trafficLightJudge(const PostureData& postureData, const CvData& cvData, const std::vector<RoadPoint>& stopPoints, DecisionData& decisionData);
void terminalPointJudge(const PostureData& postureData, const RoadPoint& terminalPoint, DecisionData& decisionData);
//void stopPointJudge(const PostureData& postureData, const RoadPoint& terminalPoint, DecisionData& decisionData);
void stopPointJudge(const PostureData& postureData, const CvData& cvData, const std::vector<RoadPoint>& stopPoints,  DecisionData& decisionData);
void mapJudge(DecisionData& decisionData, const CvData& cvData, const VisionData& visionData);

//bool checkGrids (LidarData lidarData);
//int32_t curveSelection (std::vector<Curve>& curves, LidarData lidarData);
double curvesLidarCheck (Curve& curve, std::vector<uint32_t>& blockedAreaIndex, double previewDistance, double targetSpeed, double judgeSpeed, bool isVirtual=false);
bool gridOccupied(Point point, uint32_t gridIndex, double previewDistance);
double curvePointOccupied(Point& point, bool*& b_obstacle, std::vector<int32_t>& IndexLateralList, std::vector<int32_t>& IndexVerticalList, std::vector<int32_t>& IndexCurveList, double judgeSpeed, bool isVirtual=false);
Curve createVirtualCurve();

Point pointOnCubicBezier (std::vector<Point> cp, double t);
void generateCurve (const RoadPoint& startPoint, const RoadPoint& endPoint, Curve& curve);
void generateCurveList (const RoadPoint& startPoint, const std::vector<RoadPoint>& endPointList, std::vector<Curve>& curveList);
void clearCurveList(std::vector<Curve>& curveList);

void generateCurvePreviewPoint(DecisionData& decisionData, const PostureData& postureData);
void generateLidarCurvePreviewPoint (DecisionData& decisionData, const std::vector<std::vector<RoadPoint>>& roadPointss, const PostureData& postureData, DecisionData& lidarDecisionData);
void generateLidarCurvePreviewPointForLine (DecisionData& decisionData, std::vector<std::vector<RoadPoint>>& roadPointss, const PostureData& postureData, DecisionData& lidarDecisionData);

double cruiseController(const PostureData& postureData, const IovData& iovData, const ActuatorData actuatorData);

void lidarJudge(DecisionData& decisionData, PostureData& postureData, LidarData& lidarData);
//void lidarJudgeLongerPreview(DecisionData& lidarDecisionData, PostureData& postureData, LidarData& lidarData);
void lidarJudgeLongerPreview(DecisionData& decisionData, DecisionData& lidarDecisionData, PostureData& postureData, LidarData& lidarData, std::vector<std::vector<RoadPoint>>& roadPointss, ros::Publisher curve_pub_, ros::Publisher virtual_curve_pub_);
//Display Lidar
void lidarJudgeDisplay(bool* b_obstacles,std::vector<int32_t>& IndexLateralList, std::vector<int32_t>& IndexVerticalList, std::vector<int32_t>& IndexCurveList);

void lidarDistJudge(DecisionData& decisionData );

void accLimiter(double& advisedSpeed, double& advisedSpeedTakeOver, DecisionData& decisionData);

//CXY
/*
void speedPlanWithLidar(DecisionData& decisionData, double advisedSpeed);
void lidarJudge(LidarData& lidarData, DecisionData& decisionData, PostureData& postureData);*/

double stanley(DecisionData& decisionData, RoadPoint& currentPoint, double& yawDiff, double& velocity);


//-------------------------------------------- ROS funs ------------------------------------------------


map_t *map_alloc(void);

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom, PostureData* postureDataPtr, ros::Time* odomReceivedTimePtr);
void gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid, LidarData* lidarDataPtr, DecisionData* lidarDecisionDataPtr, ros::Time* lidarReceivedTimePtr, map_t* map);
void decisionSubCallback(const std_msgs::Float64MultiArray::ConstPtr& decision_sub, DecisionData* decisionDataPtr, UiData* uiDataPtr);

void previewPointPublisher(ros::Publisher previewPoint_pub, DecisionData decisionData);
void roadPointPublisher(ros::Publisher roadPoint_pub_,std::vector<std::vector<RoadPoint>>& RoadPointss, std::vector<std::vector<uint32_t>>& pathNumberLists, uint16_t pathNumber);
void pastPathPublisher(ros::Publisher past_path_pub_, PostureData& postureData);
void curvePublisher(ros::Publisher curve_pub_, DecisionData& decisionData);
void curvePublisher(ros::Publisher curve_pub_, const Curve& curve); 
void read_ui(std::string fileName, UiData& uiData);
void calculate_ui(UiData& uiData, PostureData& postureData);


//-----------------------------------------------------------------------------------------------------


#endif /* defined(__decision__functions__) */
