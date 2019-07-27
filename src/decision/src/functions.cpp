#include "decision/functions.hpp"

#define LATITUDE_THRESH 10
#define LONGITUDE_THRESH 10
#define COURSE_THRESH 60
#define MIN_DISTANCE 5 // in meters
#define PREVIEW_VELOCITY_MULTIPLIER 2.9 // old=2.2, 1.2 as GUO Konghui suggested.
//#define PREVIEW_VELOCITY_MULTIPLIER 3.0 // 1.2 as GUO Konghui suggested.
//#define MAX_PREVIEW_DISTANCE 20
//#define MAX_PREVIEW_DISTANCE 20
//#define DEFAULT_PREVIEW_DISTANCE 18
#define DEFAULT_PREVIEW_DISTANCE 6 
#define MAX_PREVIEW_DISTANCE 35 //
// #define MAX_PREVIEW_DISTANCE 8 
//#define MIN_PREVIEW_DISTANCE 5
//#define MIN_PREVIEW_DISTANCE 10
//#define MIN_PREVIEW_DISTANCE 6 //
#define MIN_PREVIEW_DISTANCE 3
#define TAKE_OVER_PREVIEW_DISTANCE 10 // caution
#define DISTANCE_RANGE 3

#define LAST_ROAD_ID 5
#define MIN_ROAD_ID 100

// lane following
#define LANE_PREVIEW_DISTANCE 18
#define TURN_PREVIEW_DISTANCE 12
#define LANE_WIDTH 3.5
#define LANE_FILTER 0.8
#define VELOCITY_FOLLOW_LANE 8
#define MID_LANE_WEIGHT 5.0

// lidar following
#define LIDAR_FILTER 0.8
#define VELOCITY_FOLLOW_LIDAR 5

// lidar and lane following
#define VELOCITY_FOLLOW_LANE_AND_LIDAR 5

// obstacle distance (m)
#define OBSTACLE_DIS_MAX 75
#define OBSTACLE_DIS_MIDMAX 60
#define OBSTACLE_DIS_MID 40
#define OBSTACLE_DIS_MIN 20
#define OBSTACLE_DIS_MINIMAL 10 

// speed definition (m/s)
#define SPEED_ULTRA 20
#define SPEED_HIGH  15//old 10
#define SPEED_MIDHIGH 7.5
#define SPEED_MID 9  //old 5
#define SPEED_LOW 2
#define SPEED_ZERO 0
#define SPEED_TEST 2

// acc definitions   // +1 all 20160604pm added
#define ACC_ULTRA 4
#define ACC_HIGH 3
#define ACC_MID 2 
#define ACC_LOW 1

#define LIMIT_COURSE 1
#define MAX_COURSE 300
#define MIN_COURSE 240

//#define TRAFFIC_LIGHT_DISTANCE_LARGE 30
#define TRAFFIC_LIGHT_DISTANCE_LARGE 10
//#define TRAFFIC_LIGHT_DISTANCE_SMALL 7
#define TRAFFIC_LIGHT_DISTANCE_SMALL 5
#define TERMINAL_DISTANCE_LARGE 15
#define TERMINAL_DISTANCE_SMALL 3

// #define YAW_WEIGHT 0.3

#define ACC_STANDARD_DISTANCE 10
#define ACC_MINIMAL_DISTANCE 7

// for qys
//#define ACC_STANDARD_DISTANCE 5 // old 10
//#define ACC_MINIMAL_DISTANCE 4

//speed_smooth
//#define SPEED_SMOOTH true 
#define SPEED_SMOOTH false 

// acc new
#define ACC_NORMAL 2
#define ACC_OBSTACLE 3
#define ACC_UISTOP 2

//steering adjust
// #define STEERING_ANGLE_ADJUST 4 //e400 074
// #define STEERING_ANGLE_ADJUST -6 //e400 111
// #define STEERING_ANGLE_ADJUST 6 //e200 858
// #define STEERING_ANGLE_ADJUST 25 //e200 866
#define STEERING_ANGLE_ADJUST 3//e400 111

// past path record
#define PAST_PATH_SIZE 100

//speed params
#define SPEED_WEIGHT 0.9
#define TOP_SPEED 10
#define SPEED_PID_SWITCH false
#define ROADMAP_LIMIT_SWITCH true
#define TURN_SPEED_LIMIT 15
#define INITIAL_SPEED 12 //km/h
#define CURV_WEIGHT 0.8

// target speed
//const double spdcrv1 = 16 / 3.6; // speed at curvature 0.1
const double spdcrv1 = 60 / 3.6; // speed at curvature 0.1 //origin 15
const double spdcrv2 = 15 / 3.6; // speed at curvature 0.5 //origin 8
const double spdcrv3 = 8 / 3.6; // speed at curvature 5 ( radius 11m )//old 10.5
const double spdK1 = (spdcrv1-spdcrv2)/0.4; //0.4
const double spdK2 = (spdcrv2-spdcrv3)/4.5;


#define doNothing()

//Display Lidar
#define DISPLAY_MAP_ON 0 //2 lidar map; 1 numbers only

//double temp1 = 0.0;
//double temp2 = 0.0;
//double temp3 = 0.0;

// reconfigure params
double previewInner_param=0.3; // old 2
double previewInner1_param = 0.5;
double previewCurvature0_param=15; // old 15
double previewCurvature5_param=0;
bool params_modify_switch = 0;
bool control_modify_switch = 0;

//curve params
double global_preview_dist = DEFAULT_LIDAR_PREVIEW_DISTANCE;
double global_preview_curvature = 0.5;
double global_current_curvature = 0.5;
double cp_dist_back_factor = 0.5;
double cp_dist_front_factor = 0.4;
double CP_DISTANCE_BACK = DEFAULT_LIDAR_PREVIEW_DISTANCE * 0.5;
double CP_DISTANCE_FRONT = DEFAULT_LIDAR_PREVIEW_DISTANCE * 0.4;
//curve adjust
double extend_length = 0;
//object avoid
double actual_curve_dist = CURVE_DISTANCE;
double curve_index = 0;

double mapTargetSpeed = 0;

//OFFSET
double offset_tempX = 1.0/0.0;
double offset_tempY = 1.0/0.0;

//state variables
LidarData lidarData;
VisionData visionData;
PostureData postureData;
ActuatorData actuatorData;
UiData uiData;
DecisionData decisionData;
DecisionData lidarDecisionData;
IovData iovData;
CvData cvData;

//steering test
double angleDiffTest = 0;
double PID_value = 0;
double preview_dist_test = 0;

//steering angle PID params
double kp_large = 550;
double kd_large = 40;
double kp_small = 300;
double kd_small = 40;
double ki_test = 0;
double FILTER_RATE0 = 0.5; //for angleDiff
double FILTER_RATE = 0.9; //for targetSteeringAngle
double FILTER_RATE1 = 0.9; //for previewCourseAngle
double FILTER_RATE11 = 0.5; //for previewCourseAngle
double FILTER_RATE2 = 0.9; //for yawDiff
double FILTER_RATE3 = 0.8; // for current point curvature

// stanley
#define FILTER_RATE4 0.8

double MAX_ABS_STEERING_RATE = 150;
double MAX_ABS_STEERING_RATE_TAKE_OVER = 150;

double converted_yaw = 0;
double converted_preview_angle = 0;

// yaw weight
double YAW_WEIGHT = 0.0;

//record past path
bool path_record_switch = false;
bool path_record_time = 0;

double LIDAR_SAFE_ZONE_MIN = 2.5;
double LIDAR_SAFE_DISTANCE_MIN = 3;

double LIDAR_SAFE_DISTANCE_MAX = 25;

// stanley
#define KDStanley 4      //old10
#define Tire2Wheel 21
#define StanleyPDist 1    //old2
#define StanleyYKD 2  //old1
#define P_dist_stanley 1.5 

PidController::PidController(double kpInput, double kiInput, double kdInput, double errorThreshInput, bool b_threshInOnInput, double intervalInput){
  kp = kpInput;
  ki = kiInput;
  kd = kdInput;
  errorThresh = errorThreshInput;
  b_threshIsOn = b_threshInOnInput;
  interval = intervalInput;
  frequency = 1 / intervalInput;
  fractionValue = 0.95;
  
//  integral = integralValue;
  b_hasStarted = false;
}

PidController::~PidController(){
  doNothing();
}

double PidController::update (double error){
  //ki
  double q = 0;
  if (false == b_threshIsOn) {
    q = 1;
  }
  else if (fabs (integral) < errorThresh && true == b_threshIsOn ) {
    q = 1;
  }
  else {
    q = 0;
    if (fabs(integral) > fractionValue * errorThresh) {
      integral = integral * fractionValue;
    }
    else;
  }
  integral += interval * q * error;
  
  //kd
  double derivateive = 0;
  if (!b_hasStarted) {
    b_hasStarted = true;
    derivateive = 0;
  }
  else {
    derivateive = (error - lastError) * frequency;
  }
  lastError = error;
  
//  std::cout << "I: " << integral << std::endl;
//  std::cout << "D: " << derivateive << std::endl;
  std::cout << "PID error: " << error << std::endl;
  std::cout << "PID value: " << kp *error + ki * integral + kd * derivateive << std::endl;
  PID_value = kp *error + ki * integral + kd * derivateive;
  //pid return value
  return (kp *error + ki * integral + kd * derivateive);
}

void PidController::modifyGains(double newKp, double newKi, double newKd){
  kp = newKp;
//  if (kp > newKp) {
//    kp = (9*kp + 1*newKp)/10;
//  }
//  else{
//    kp = newKp;
//  }
//  kp = kp>400?400:kp;
  
  ki = newKi;
  kd = newKd;
}

void PidController::resetI(){
  integral = 0;
}

bool angleDiffLessThan(double angle1, double angle2, double diff){
  bool ret = false;
  if (std::cos((angle1 - angle2)/180*M_PI) < std::cos(diff/180*M_PI)) {
    ret = true;
  }
  else{
    ret = false;
  }
  return ret;
}


uint32_t DecisionData::updateStateMachine(){
  currentState = currentState + 1;
  return currentState;
}

//RoadPoint movePreviewPointForLaneChange (RoadPoint originalRoadPoint, double yaw){
//  RoadPoint ret;
//  
//  
//  return ret;
//}

void gaussConvert(double longitude1,double latitude1,double& x1,double& y1)
{
  double lon;
  double lat;
  int a=6378137;
  double lon0;
  double e=0.0066943799013;
  double e2=0.006739496742270;
  double deltL;
  double t;
  double n;
  double C1;
  double C2;
  double C3;
  double C4;
  double C5;
  double N;
  double X;
  double A;
  lon=longitude1*2*M_PI/360;
  lat=latitude1*2*M_PI/360;
  lon0=117*2*M_PI/360;
  deltL=lon-lon0;
  t=tan(lat);
  n=e2*cos(lat);
  C1=1+3*pow(e,2)/4+45*pow(e,4)/64+175*pow(e,6)/256+11025*pow(e,8)/16384;
  C2=3*pow(e,2)/4+15*pow(e,4)/16+525*pow(e,6)/512+2205*pow(e,8)/2048;
  C3=15*pow(e,4)/64+105*pow(e,6)/256+2205*pow(e,8)/4096;
  C4=35*pow(e,6)/512+315*pow(e,8)/2048;
  C5=315*pow(e,8)/131072;
  N=a/sqrt(1-pow(e,2)*pow(sin(lat),2));
  X=a*(1-pow(e,2))*(C1*lat-0.5*C2*sin(2*lat)+C3*sin(4*lat)/4-C4*sin(6*lat)/6+C5*sin(8*lat));
  A=1+pow(deltL,2)/12*pow(cos(lat),2)*(5-pow(t,2)+9*pow(n,2)+4*pow(n,4))+pow(deltL,4)/360*pow(cos(lat),4)*(61-58*pow(t,2)+pow(t,4));
  y1=X+0.5*N*sin(lat)*cos(lat)*pow(deltL,2)*A;//正北方向
  x1=N*cos(lat)*deltL*(1+pow(deltL,2)/6*pow(cos(lat),2)*(1-pow(t,2)+pow(n,2))+pow(deltL,4)/120*pow(cos(lat),4)*(5-18*pow(t,2)+pow(t,4)-14*pow(n,2)-58*pow(n,2)*pow(t,2)));//正东方向
}

bool inArea(double longitudeX, double latitudeY, double targetX, double targetY, double distance){
  bool ret = false;
//  double longitudeX = 0.0, latitudeY = 0.0;
//  gaussConvert (longitude, latitude, longitudeX, latitudeY);
  
  //  std::cout << std::setprecision(15) << "lon: " << longitudeX << ", lat: " << latitudeY << std::endl;
  //  std::cout << std::setprecision(15) << "tx : " << targetX << ", ty : " << targetY << std::endl;
  
  
  if (std::abs(longitudeX - targetX) < distance && std::abs(latitudeY - targetY) < distance){
    ret = true;
  }
  else{
    ret = false;
  }
  
  //  std::cout << "inarea?: " << ret << std::endl;
  return ret;
}

double pointDistance(double x1, double x2, double y1, double y2){
  double distance;
  double dX;
  double dY;
  dX = std::abs(x1 - x2);
  dY = std::abs(y1 - y2);
  distance = sqrt( dX * dX + dY * dY);
  return distance;
}

double gpsDistance(double longitude, double latitude, RoadPoint roadPoint){
  double xRM = 0, yRM = 0;
  double x = 0, y = 0;
  gaussConvert(roadPoint.longitude, roadPoint.latitude, xRM, yRM);
  gaussConvert(longitude, latitude, x, y);
  double distance = pointDistance(x, xRM, y, yRM);
  return distance;
}

// super stupid!!!
uint32_t loadRoadMapSize(const std::string fileName){
  std::string line;
  std::ifstream fs;
  uint32_t lastRoadId = 0;
  fs.open(fileName, std::ios::in);
  if (fs.fail()){
    std::cout << RED << "!!!!!!FATAL!!!!!!LOAD ROAD MAP FAILED!!!!!!" << RESET << std::endl;
  }else;
  while(getline(fs, line))
  {
    if (line.length() > 0){
      RoadPoint roadPoint;
      std::stringstream ss(line);
      //      ss >> roadPoint.roadId >> roadPoint.longitude >> roadPoint.latitude >> roadPoint.height >> roadPoint.parameter1 >> roadPoint.parameter2;
      ss >> roadPoint.roadId >> roadPoint.longitude >> roadPoint.latitude >> roadPoint.courseAngle >> roadPoint.roadType >> roadPoint.parameter2;
      
      lastRoadId = roadPoint.roadId;
    }else;
  }
  fs.close();
  return lastRoadId;
}

void loadRoadMap(const std::string fileName, std::vector<std::vector<RoadPoint>>& rawRoadPointss){
  std::string line;
  std::ifstream fs;
  uint32_t index = 0;
  uint32_t lastRoadId = 0;
  fs.open(fileName, std::ios::in);
  if (fs.fail()){
    std::cout << RED << "!!!!!!FATAL!!!!!!LOAD ROAD MAP FAIL!!!!!!" << RESET << std::endl;
  }else;
  while(getline(fs, line))
  {
    if (line.length() > 0){
      RoadPoint roadPoint;
      std::stringstream ss(line);
//      ss >> roadPoint.roadId >> roadPoint.longitude >> roadPoint.latitude >> roadPoint.height >> roadPoint.parameter1 >> roadPoint.parameter2;
      ss >> roadPoint.roadId >> roadPoint.longitude >> roadPoint.latitude >> roadPoint.courseAngle >> roadPoint.roadType >> roadPoint.parameter2 >> roadPoint.parameter3;
      if (roadPoint.roadId != lastRoadId) {
        index = 0;
      }
      else;
      
      roadPoint.index = index;
      rawRoadPointss[roadPoint.roadId].push_back(roadPoint);
      lastRoadId = roadPoint.roadId;
      
//      std::cout << "length: " << rawRoadPointss[lastRoadId].size() << std::endl;
//      std::cout << "Index: " << index - 1
//      std::cout << std::setprecision(15) << roadPoint.roadId << ", " << roadPoint.longitude << ", " << roadPoint.latitude << ", " << roadPoint.courseAngle << std::endl;
//      std::cout << std::setprecision(15) << rawRoadPointss[lastRoadId][index].roadId << ", " << rawRoadPointss[lastRoadId][index].longitude << ", " << rawRoadPointss[lastRoadId][index].latitude << ", " << rawRoadPointss[lastRoadId][index].courseAngle << std::endl;
      
      index ++;
    }else;
  }
  fs.close();
}

void loadRoadMapConverted(const std::string fileName, std::vector<std::vector<RoadPoint>>& rawRoadPointss, double offsetX, double offsetY){
  std::string line;
  std::ifstream fs;
  uint32_t index = 0;
  uint32_t lastRoadId = 0;
  uint32_t totalIndex = 0;
  fs.open(fileName, std::ios::in);
  if (fs.fail()){
    std::cout << RED << "!!!!!!FATAL!!!!!!LOAD ROAD MAP FAIL!!!!!!" << RESET << std::endl;
    return;
  }else;
  while(getline(fs, line))
  {
    if (line.length() > 0){
      RoadPoint roadPoint;
      std::stringstream ss(line);
      //      ss >> roadPoint.roadId >> roadPoint.longitude >> roadPoint.latitude >> roadPoint.height >> roadPoint.parameter1 >> roadPoint.parameter2;
      ss >> roadPoint.roadId >> roadPoint.longitude >> roadPoint.latitude >> roadPoint.courseAngle >> roadPoint.roadType >> roadPoint.parameter2 >> roadPoint.parameter3;
      roadPoint.longitude += offsetX;
      roadPoint.latitude += offsetY;
      if (roadPoint.roadId != lastRoadId) {
        index = 0;
      }
      else;
      
      roadPoint.totalIndex = totalIndex;
      roadPoint.index = index;
      rawRoadPointss[roadPoint.roadId].push_back(roadPoint);
      lastRoadId = roadPoint.roadId;
      
      //      std::cout << "length: " << rawRoadPointss[lastRoadId].size() << std::endl;
      //      std::cout << "Index: " << index - 1
      //      std::cout << std::setprecision(15) << roadPoint.roadId << ", " << roadPoint.longitude << ", " << roadPoint.latitude << ", " << roadPoint.courseAngle << std::endl;
      //      std::cout << std::setprecision(15) << rawRoadPointss[lastRoadId][index].roadId << ", " << rawRoadPointss[lastRoadId][index].longitude << ", " << rawRoadPointss[lastRoadId][index].latitude << ", " << rawRoadPointss[lastRoadId][index].courseAngle << std::endl;
      
      index ++;
      totalIndex ++;

      if(offset_tempX > roadPoint.longitude)
        offset_tempX = roadPoint.longitude;
      if(offset_tempY > roadPoint.latitude)
        offset_tempY = roadPoint.latitude;

    }else;
  }
  fs.close();
}

uint32_t loadStopPointsSize(const std::string fileName){
  std::string line;
  std::ifstream fs;
  fs.open(fileName, std::ios::in);
  if (fs.fail()){
    std::cout << RED << "!!!!!!FATAL!!!!!!LOAD STOP POINT FAILED!!!!!!" << RESET << std::endl;
  }else;
  uint32_t stopPointsNumbers = 0;
  while(getline(fs, line))
  {
    if (line.length() > 0){
      
      stopPointsNumbers ++;
    }else;
  }
  fs.close();
  return stopPointsNumbers;
}

void loadStopPoints(const std::string fileName, std::vector<RoadPoint>& rawStopPoints, double offsetX, double offsetY){
  rawStopPoints.clear();
  std::string line;
  std::ifstream fs;
  uint32_t index = 0;
  fs.open(fileName, std::ios::in);
  if (fs.fail()){
    std::cout << RED << "!!!!!!FATAL!!!!!!LOAD STOP POINT FAIL!!!!!!" << RESET << std::endl;
    return;
  }else;
  RoadPoint roadPoint;
  rawStopPoints.push_back(roadPoint); // push an empty one so that index starting from 1;
  while(getline(fs, line))
  {
    if (line.length() > 0){
//      RoadPoint roadPoint;
      std::stringstream ss(line);
      ss >> roadPoint.roadId >> roadPoint.longitude >> roadPoint.latitude >> roadPoint.courseAngle >> roadPoint.parameter1 >> roadPoint.parameter2 >> roadPoint.parameter3;
      roadPoint.longitude += offsetX;
      roadPoint.latitude += offsetY;
      roadPoint.index = index;
      rawStopPoints.push_back(roadPoint);
      index++;
      
      //      std::cout << "length: " << rawRoadPointss[lastRoadId].size() << std::endl;
      //      std::cout << "Index: " << index - 1
      //      std::cout << std::setprecision(15) << roadPoint.roadId << ", " << roadPoint.longitude << ", " << roadPoint.latitude << ", " << roadPoint.courseAngle << std::endl;
      //      std::cout << std::setprecision(15) << rawRoadPointss[lastRoadId][index].roadId << ", " << rawRoadPointss[lastRoadId][index].longitude << ", " << rawRoadPointss[lastRoadId][index].latitude << ", " << rawRoadPointss[lastRoadId][index].courseAngle << std::endl;
      
    }else;
  }
  fs.close();
}

void loadTerminalPoint(const std::string fileName, RoadPoint& roadPoint){
  std::string line;
  std::ifstream fs;
  fs.open(fileName, std::ios::in);
  if (fs.fail()){
    std::cout << RED << "!!!!!!FATAL!!!!!!LOAD TERMINAL POINT FAIL!!!!!!" << RESET << std::endl;
  }else;

  while(getline(fs, line))
  {
    if (line.length() > 0){
      std::stringstream ss(line);
      //      ss >> roadPoint.roadId >> roadPoint.longitude >> roadPoint.latitude >> roadPoint.height >> roadPoint.parameter1 >> roadPoint.parameter2;
      ss >> roadPoint.roadId >> roadPoint.longitude >> roadPoint.latitude >> roadPoint.courseAngle >> roadPoint.roadType >> roadPoint.parameter2;
      
      roadPoint.index = 0;
      
      //      std::cout << "length: " << rawRoadPointss[lastRoadId].size() << std::endl;
      //      std::cout << "Index: " << index - 1
      //      std::cout << std::setprecision(15) << roadPoint.roadId << ", " << roadPoint.longitude << ", " << roadPoint.latitude << ", " << roadPoint.courseAngle << std::endl;
      //      std::cout << std::setprecision(15) << rawRoadPointss[lastRoadId][index].roadId << ", " << rawRoadPointss[lastRoadId][index].longitude << ", " << rawRoadPointss[lastRoadId][index].latitude << ", " << rawRoadPointss[lastRoadId][index].courseAngle << std::endl;
      
    }else;
  }
  fs.close();
}

void loadPathNumber(const std::string fileName, std::vector<std::vector<uint32_t>>& pathNumberLists){
  std::string line;
  std::ifstream fs;
  uint32_t totalIndex = 1;
  uint32_t pathNumbers = 0;
  fs.open(fileName, std::ios::in);
  if (fs.fail()){
    std::cout << RED << "!!!!!!FATAL!!!!!!LOAD Path number fail!!!!!!" << RESET << std::endl;
    return;
  }else;
  while(getline(fs, line))
  {
    if (line.length() > 0){
      pathNumbers = 0;
      pathNumberLists[totalIndex].push_back(pathNumbers);
      std::stringstream ss(line);
      while (ss) {
        if(ss >> pathNumbers){
          pathNumberLists[totalIndex].push_back(pathNumbers);
          std::cout << pathNumbers << std::endl;
        }
      }
    }else;
    totalIndex++;
  }
  fs.close();
}


//uint32_t getNextRoadId (uint32_t currentId, uint32_t maxId){
//  uint32_t nextId = 0;
////  if (61 == currentId) {
//  if (maxId <= currentId) {
//    nextId = MINIMAL_ID;
//  }
//  else {
//    nextId = currentId + 1;
//  }
//  
//  return nextId;
////  return 0;
//}

void pathSelection(DecisionData& decisionData, const UiData& uiData, const uint16_t maxPathNumber){
  if(uiData.pathNumber < maxPathNumber){
    decisionData.pathNumber = uiData.pathNumber;
  }
  else{
    decisionData.pathNumber = 1; // old 0
  }
  //stupid
//  decisionData.pathNumber = 1;
}

uint32_t getNextRoadId (uint32_t currentId, uint32_t maxId, const DecisionData& decisionData, std::vector<std::vector<uint32_t>>& pathNumberLists){
  uint32_t nextId = 1;
  if (decisionData.pathNumber == 0) {
    if (maxId <= currentId) {
      nextId = MINIMAL_ID;
    }
    else {
      nextId = currentId + 1;
    }
  }
  else{
    static uint32_t currentIdIndex = 1;
    uint32_t pathMaxIdIndex = static_cast<uint32_t>(pathNumberLists[(decisionData.pathNumber)].size()) - 1;
    
    if (currentIdIndex < pathMaxIdIndex ) {
      if (currentId == pathNumberLists[decisionData.pathNumber][currentIdIndex]) {
        nextId = pathNumberLists[decisionData.pathNumber][currentIdIndex + 1];
      }
      else if (currentId == pathNumberLists[decisionData.pathNumber][currentIdIndex + 1]){
        currentIdIndex = currentIdIndex + 1;
        if (currentIdIndex >= pathMaxIdIndex) {
          nextId = pathNumberLists[decisionData.pathNumber][MINIMAL_ID];
        }
        else{
          nextId = pathNumberLists[decisionData.pathNumber][currentIdIndex + 1];
 
        }
      }
      else{
        for (int i = 1; i <= pathMaxIdIndex; i++) {
          if ( currentId == pathNumberLists[decisionData.pathNumber][i]) {
            currentIdIndex = i;
            if (currentIdIndex == pathMaxIdIndex) {
              nextId = pathNumberLists[decisionData.pathNumber][MINIMAL_ID];
            }
            else{
              nextId = pathNumberLists[decisionData.pathNumber][currentIdIndex + 1];
            }
            break;
          }else;
        }
      }
    }
    else{
      if (currentId == pathNumberLists[decisionData.pathNumber][currentIdIndex]) {
        doNothing();
      }
      else{
        for (int i = 1; i <= pathMaxIdIndex; i++) {
          if ( currentId == pathNumberLists[decisionData.pathNumber][i]) {
            currentIdIndex = i;
            break;
          }
        }
      }
      nextId = pathNumberLists[decisionData.pathNumber][MINIMAL_ID];
    }
    //to eliminate "0" or "max" bug  !! tbd.
//    if (nextId < pathNumberLists[decisionData.pathNumber][MINIMAL_ID]) {
//      nextId = pathNumberLists[decisionData.pathNumber][MINIMAL_ID];
//    }
//    else if(nextId > pathNumberLists[decisionData.pathNumber][pathMaxIdIndex]){
//      nextId = pathNumberLists[decisionData.pathNumber][pathMaxIdIndex];
//    }
//    else;

  }
  
  
  return nextId;
}

uint32_t getLastRoadId (uint32_t currentId, uint32_t maxId){
  uint32_t nextId = 0;
//  if (61 == currentId) {
  if (currentId <= MINIMAL_ID) {
    nextId = maxId;
  }
  else {
    nextId = currentId - 1;
  }
  
  return nextId;
//  return 0;
}

void getCoordinate(std::vector<RoadPoint>& rawRoadPoints, std::vector<RoadPoint>& roadPoints){
  for ( int i = 0; i < rawRoadPoints.size(); i ++){
    RoadPoint roadPoint;
    gaussConvert(rawRoadPoints[i].longitude, rawRoadPoints[i].latitude, roadPoint.longitude, roadPoint.latitude);
    roadPoint.index = rawRoadPoints[i].index;
    roadPoint.roadId = rawRoadPoints[i].roadId;
    roadPoint.height = rawRoadPoints[i].height;
    roadPoint.parameter1 = rawRoadPoints[i].parameter1;
    roadPoint.parameter2 = rawRoadPoints[i].parameter2;
    roadPoints.push_back(roadPoint);
//    std::cout << std::setprecision(15) << roadPoint.latitude << ", " << roadPoint.longitude << ", " << roadPoint.height<< std::endl;
  }
}

// stanley
double getAnglebtwn(double x1, double y1, double x2, double y2, double c_a)
{
  // >0 left <0 right
  double a1 = getAngle(x2,y2,x1,y1);
  a1 = 90-a1;
  if(a1<0) a1+=360;
  std::cout << "[getAnglebtwn] a1: " << a1 << " c_a :" << c_a << " delta: " << c_a-a1 << std::endl;
  return c_a-a1;
}

RoadPoint getCurrentPoint (double longitude, double latitude, double yaw, std::vector<std::vector<RoadPoint>>& roadPointss, int32_t lastId, int32_t nextId, DecisionData& decisionData, std::vector<std::vector<uint32_t>>& pathNumberLists, uint16_t pathNumber) {
//  std::cout << "Paras: " << longitude << ", " << latitude << ", " << yaw << std::endl;
  uint32_t roadPointsSize = static_cast<uint32_t>(roadPointss.size());
//  std::cout << "size: " << roadPointsSize << std::endl;
  std::vector<RoadPoint> roadPointCandidates;
  roadPointCandidates.reserve(1000);
  double minimalX = 100, minimalY = 100;
//  roadPointCandidates.clear();

  double first_id = pathNumberLists[pathNumber][1];
  double last_id = pathNumberLists[pathNumber].size()-2 + first_id;

  for (uint32_t i = first_id; i <= last_id; i++){
    uint32_t subRoadPointsSize = static_cast<uint32_t>(roadPointss[i].size());
//    std::cout << "sub road point size: " << subRoadPointsSize << std::endl;
    for (uint32_t j = 0; j < subRoadPointsSize; j++) {
      minimalX = std::abs(roadPointss[i][j].longitude - longitude) < minimalX? std::abs(roadPointss[i][j].longitude - longitude) : minimalX;
      minimalY = std::abs(roadPointss[i][j].latitude - latitude) < minimalY? std::abs(roadPointss[i][j].latitude - latitude) : minimalY;
      if ((std::abs(roadPointss[i][j].latitude - latitude) < LATITUDE_THRESH)
          && (std::abs(roadPointss[i][j].longitude - longitude) < LONGITUDE_THRESH)
//          && (true == angleDiffLessThan(roadPointss[i][j].courseAngle, yaw, COURSE_THRESH))
          && (std::fabs((360 + 180 + (int)roadPointss[i][j].courseAngle - (int)yaw)%360 - 180) < COURSE_THRESH)
            // && (std::fabs(roadPointss[i][j].courseAngle - yaw) < COURSE_THRESH)
          ){
//      if ((std::abs(roadPointss[i][j].latitude - latitude) < LATITUDE_THRESH) && (std::abs(roadPointss[i][j].longitude - longitude) < LONGITUDE_THRESH) && (std::abs(roadPointss[i][j].courseAngle - yaw) < COURSE_THRESH)){
//      if ((std::abs(roadPointss[i][j].latitude - latitude) < LATITUDE_THRESH) && (std::abs(roadPointss[i][j].longitude - longitude) < LONGITUDE_THRESH)){
//        std::cout << "got one" << std::endl;
        roadPointCandidates.push_back(roadPointss[i][j]);
//        std::cout << std::fabs(((360 + (int)roadPointss[i][j].courseAngle - (int)yaw)%360 - 360)) << std::endl;
//        std::cout << std::fabs(((360 + (int)roadPointss[i][j].courseAngle - (int)yaw)%360 - 360)) -COURSE_THRESH << std::endl;
//        std::cout << "Found one!" << std::endl;
      }
      else{
//        std::cout << "got none!" << std::endl;
      };
    }
  }
  uint32_t candidateSize = static_cast<uint32_t>(roadPointCandidates.size());
  if (0 == candidateSize) {
    decisionData.b_onRoadPoint = false;
    std::cout << "Current Point Not Found. Min X: " << minimalX << ", Min Y: " << minimalY << std::endl;
  }

  
  RoadPoint ret;
  if (0 < candidateSize) {
    decisionData.b_onRoadPoint = true;
    std::cout << "candi size: " << candidateSize << std::endl;
    uint32_t minPointId = 0;
//    uint32_t minRoadId = MIN_ROAD_ID;
    double minDistance = MIN_DISTANCE;
    
//    for (uint32_t i = 0; i < candidateSize; i ++) {
//      if (roadPointCandidates[i].roadId  < minRoadId) {
//        minRoadId = roadPointCandidates[i].roadId;
//      }
//      else;
//    }
//    uint32_t roadPointssSize = static_cast<uint32_t>(roadPointss.size()) - 1;
//    int32_t nextId = getNextRoadId(lastId, roadPointssSize, decisionData);
    for ( uint32_t i = 0; i < candidateSize; i ++) {
//      if (roadPointCandidates[i].roadId == minRoadId || roadPointCandidates[i].roadId == minRoadId + 1) {
      if (roadPointCandidates[i].roadId == lastId || roadPointCandidates[i].roadId == nextId) {
        double distTemp = pointDistance(longitude, roadPointCandidates[i].longitude, latitude, roadPointCandidates[i].latitude);
    //    std::cout << "distTemp: " << distTemp << std::endl;
        if ( distTemp <= minDistance){
          minDistance = distTemp;
          minPointId = i;
        }
        else;
      }
      else;
    }
    
    ret = roadPointCandidates[minPointId];
    ret.b_isValid = true;
    
    std::cout << "Min distance: " << minDistance << std::endl;

    // stanley
    std::cout << "longitude " << longitude << " latitude " << latitude << std::endl;
    std::cout << "ret.longitude " << ret.longitude << " ret.latitude " << ret.latitude << " ret.cAngle " << ret.courseAngle << std::endl;
    double a_b = getAnglebtwn(longitude, latitude, ret.longitude, ret.latitude, ret.courseAngle);

    if(a_b >= 0)
      ret.lateral_dist = minDistance;
    else
      ret.lateral_dist = -minDistance;
  }
  else {
    ret.b_isValid = false;
  }
//  std::cout << "min id: " << minId << std::endl;
  return ret;
}

double getPreviewDistance(double velocity, double curvature, RoadPoint& currentPoint, DecisionData& decisionData){
  double distance = DEFAULT_PREVIEW_DISTANCE;
  static double lastPreviewDistance = DEFAULT_PREVIEW_DISTANCE;
  if (decisionData.b_takingOver == true) {
    distance = TAKE_OVER_PREVIEW_DISTANCE;
    lastPreviewDistance = distance;
  }
  else{
    double distanceCurvature = 0;
    curvature = std::fabs(curvature);
    double previewInner=previewInner_param; // old 2
    double previewInner1 = previewInner1_param;
    double previewCurvature0=previewCurvature0_param; // old 15
    double previewCurvature5=previewCurvature5_param;

    if (curvature > 1){
//      distanceCurvature = -1 * curvature + 4;
//      distanceCurvature = -0.94 * curvature + 5.7;
        distanceCurvature = (previewInner-previewCurvature5)*(5-curvature)/4+previewCurvature5; 
    }
    else if ((curvature <= 1)&&(curvature > 0.6)){
        distanceCurvature = (previewInner1-previewInner)*(1-curvature)/0.4+previewInner; 
    }
    else if (curvature <= 0.6){  
//      distanceCurvature = -23 * curvature + 15;
        distanceCurvature = -(previewCurvature0-previewInner)*curvature/0.6 + previewCurvature0;
    }

    if (distanceCurvature < 0){
      distanceCurvature = 0;
    }else;
    
    double distanceVelocity = PREVIEW_VELOCITY_MULTIPLIER * velocity;
    if (distanceVelocity > MAX_PREVIEW_DISTANCE) {
      distanceVelocity = MAX_PREVIEW_DISTANCE;
    }
    else if (distanceVelocity < MIN_PREVIEW_DISTANCE) {
      distanceVelocity = MIN_PREVIEW_DISTANCE;
    }
    else;
    distance = distanceCurvature + distanceVelocity;
    distance = limitPreviewDistance (distance, lastPreviewDistance, velocity);
    lastPreviewDistance = distance;
  }
//  distance = 5.0;
  decisionData.previewDistance = distance;
  return distance;
}

RoadPoint getPreviewPoint (double distance, double velocity, double curvature, RoadPoint& currentPosture, double yaw, int32_t nextId, int32_t nextNextId, std::vector<std::vector<RoadPoint>>& roadPointss){
  
//  if(currentPosture.roadType == 4){
//    distance = TURN_PREVIEW_DISTANCE;
//  }
//  std::cout << "Preview Distance: " << distance << std::endl;
  std::cout << "roadType: " << currentPosture.roadType << std::endl;
  bool b_findPreviewPoint = false;
  RoadPoint previewPointCandidate;
//  int32_t index = currentPosture.index;
  for (int32_t index = currentPosture.index; index < roadPointss[currentPosture.roadId][0].index + roadPointss[currentPosture.roadId].size(); index ++ ) {
//  for (int32_t index = roadPointss[currentPosture.roadId][0].index; index < roadPointss[currentPosture.roadId][0].index + roadPointss[currentPosture.roadId].size(); index ++ ) {
    double distanceFound = pointDistance(currentPosture.longitude, roadPointss[currentPosture.roadId][index].longitude, currentPosture.latitude, roadPointss[currentPosture.roadId][index].latitude);
    if ( distanceFound > distance - DISTANCE_RANGE && distanceFound < distance + DISTANCE_RANGE){
      double previewAngle = getAngle(currentPosture.longitude, currentPosture.latitude, roadPointss[currentPosture.roadId][index].longitude, roadPointss[currentPosture.roadId][index].latitude);
//      double previewAngle = getAngle(currentPosture.longitude, currentPosture.latitude, roadPointss[nextId][index].longitude, roadPointss[nextId][index].latitude);
      double angleDiff = getAngleDiff(yaw, previewAngle);
//      double angleDiff = 0;
//      170412 Qing
      if (std::abs(angleDiff) < COURSE_THRESH){
        previewPointCandidate = roadPointss[currentPosture.roadId][index];
        b_findPreviewPoint = true;
      }else;
    }
    else;
  }
//  if (false == b_findPreviewPoint) {
//  always check next segment
  if (true) {
   for (int32_t index = roadPointss[nextId][0].index; index < roadPointss[nextId][0].index + roadPointss[nextId].size(); index ++ ) {
//   for (int32_t index = roadPointss[nextId][0].index; index < roadPointss[nextId][0].index + roadPointss[nextId].size(); index ++ ) {
      double distanceFound = pointDistance(currentPosture.longitude, roadPointss[nextId][index].longitude, currentPosture.latitude, roadPointss[nextId][index].latitude);
      if ( distanceFound > distance - DISTANCE_RANGE && distanceFound < distance + DISTANCE_RANGE){
        double previewAngle = getAngle(currentPosture.longitude, currentPosture.latitude, roadPointss[nextId][index].longitude, roadPointss[nextId][index].latitude);
//        double angleDiff = getAngleDiff(currentPosture.courseAngle, previewAngle);
        double angleDiff = getAngleDiff(yaw, previewAngle);
//        double angleDiff = 0;
//      170412 Qing
        if (std::abs(angleDiff) < COURSE_THRESH+30){
          previewPointCandidate = roadPointss[nextId][index];
          b_findPreviewPoint = true;
        }
      }
      else;
    }
  }else;

  /*
//  always check next and next segment
  if (true) {
   for (int32_t index = roadPointss[nextNextId][0].index; index < roadPointss[nextNextId][0].index + roadPointss[nextId].size(); index ++ ) {
//   for (int32_t index = roadPointss[nextId][0].index; index < roadPointss[nextId][0].index + roadPointss[nextId].size(); index ++ ) {
      double distanceFound = pointDistance(currentPosture.longitude, roadPointss[nextNextId][index].longitude, currentPosture.latitude, roadPointss[nextNextId][index].latitude);
      if ( distanceFound > distance - DISTANCE_RANGE && distanceFound < distance + DISTANCE_RANGE){
        double previewAngle = getAngle(currentPosture.longitude, currentPosture.latitude, roadPointss[nextNextId][index].longitude, roadPointss[nextNextId][index].latitude);
//        double angleDiff = getAngleDiff(currentPosture.courseAngle, previewAngle);
        double angleDiff = getAngleDiff(yaw, previewAngle);
//        double angleDiff = 0;
        if (std::abs(angleDiff) < 120){
          previewPointCandidate = roadPointss[nextNextId][index];
          b_findPreviewPoint = true;
        }
      }
      else;
    }
  }else;
   */
  
  
  RoadPoint ret;
  if (true == b_findPreviewPoint) {
    ret = previewPointCandidate;
    std::cout << "Current Point ID   : " << currentPosture.roadId << std::endl;
    std::cout << "Current Point Index: " << currentPosture.index << std::endl << "Preview Point Index: " << previewPointCandidate.index << std::endl;
    std::cout << "Current Point Total Index: " << currentPosture.totalIndex << std::endl << "Preview Point Total Index: " << previewPointCandidate.totalIndex << std::endl;
    double distanceFound = pointDistance(currentPosture.longitude, previewPointCandidate.longitude, currentPosture.latitude, previewPointCandidate.latitude);
    std::cout << CYAN << "Actual Preview Distance: " << RESET << distanceFound << std::endl;
    ret.b_isValid = true;
    ret.preview_dist = distanceFound;

    // stanley
    // double a_b = getAnglebtwn(longitude, latitude, ret.longitude, ret.latitude, ret.courseAngle);
    // if(a_b >= 0)
    //   ret.lateral_dist = minDistance;
    // else
    //   ret.lateral_dist = -minDistance;

  }
  else {
    ret = currentPosture;
    ret.b_isValid = false;
    std::cout << "NO PREVIEW POINT FOUND!" << std::endl;
  }
  return ret;
}

RoadPoint movePreviewPoint(RoadPoint previewPoint, double curveDistance){
  RoadPoint previewPointTakeOver = previewPoint;
  previewPointTakeOver.longitude = previewPoint.longitude - std::cos(previewPoint.courseAngle/180*M_PI) * curveDistance;
  previewPointTakeOver.latitude = previewPoint.latitude + std::sin(previewPoint.courseAngle/180*M_PI) * curveDistance;
  previewPointTakeOver.parameter2 = 4.0;
  previewPointTakeOver.parameter3 = SPEED_LOW;
  return previewPointTakeOver;
}

//void limitSpeed(DecisionData& decisionData, double lastTargetSpeed){
//  std::cout << "decision: " << decisionData.targetSpeed << std::endl;
//  std::cout << "last: " << lastTargetSpeed << std::endl;
//  if (std::abs(decisionData.targetSpeed -lastTargetSpeed) > 0.1 && decisionData.targetSpeed >SPEED_LOW) {
//  if (std::abs(decisionData.targetSpeed -lastTargetSpeed) > 0.2) {
//    if (decisionData.targetSpeed > lastTargetSpeed) {
//      decisionData.targetSpeed = lastTargetSpeed + 0.2;
//    }
//    else if (decisionData.targetSpeed < lastTargetSpeed){
//      decisionData.targetSpeed = lastTargetSpeed - 0.1;
//      ;
//    }
//    else;
//  }
//}

#define MAX_SPEED_INCREMENT 0.005

double limitSpeed(double targetSpeed, double lastTargetSpeed){
  
  std::cout << "decision Speed: " << std::fixed << std::setprecision(1) << targetSpeed*3.6 << " km/h\n";
  std::cout << "last     Speed: " << std::fixed << std::setprecision(1) << lastTargetSpeed*3.6 << " km/h\n";
//  if (std::abs(decisionData.targetSpeed -lastTargetSpeed) > 0.1 && decisionData.targetSpeed >SPEED_LOW) {
  if (std::abs(targetSpeed -lastTargetSpeed) > MAX_SPEED_INCREMENT ) {
    if (targetSpeed > lastTargetSpeed) {
      targetSpeed = lastTargetSpeed + MAX_SPEED_INCREMENT;
    }
    else if (targetSpeed < lastTargetSpeed){
//      decisionData.targetSpeed = lastTargetSpeed - 0.1;
      ;
    }
    else;
  }
  return targetSpeed;
}

double limitTargetSpeed(DecisionData& decisionData, double targetSpeed){
  double speedUpperBound = -0.5*std::fabs(decisionData.targetSteeringAngle) + 30;
  speedUpperBound = speedUpperBound > 5? speedUpperBound : 5;
  
//  if (targetSpeed > speedUpperBound) {
//    targetSpeed = speedUpperBound;
//  }
//  else;
  
  return targetSpeed;
}

double limitSteeringAngle (double targetSteeringAngle, double currentSteeringAngle, double velocity, double yaw, bool overTakeEnable) {
if (overTakeEnable==false){  
  if (std::abs(targetSteeringAngle - currentSteeringAngle) > MAX_ABS_STEERING_RATE) {
    if (targetSteeringAngle > currentSteeringAngle) {
      targetSteeringAngle = currentSteeringAngle + MAX_ABS_STEERING_RATE;
    }
    else if (targetSteeringAngle < currentSteeringAngle){
      targetSteeringAngle = currentSteeringAngle - MAX_ABS_STEERING_RATE;
    }
    else;
  }
  else;
}
else{
   if (std::abs(targetSteeringAngle - currentSteeringAngle) > MAX_ABS_STEERING_RATE_TAKE_OVER) {
    if (targetSteeringAngle > currentSteeringAngle) {
      targetSteeringAngle = currentSteeringAngle + MAX_ABS_STEERING_RATE_TAKE_OVER;
    }
    else if (targetSteeringAngle < currentSteeringAngle){
      targetSteeringAngle = currentSteeringAngle - MAX_ABS_STEERING_RATE_TAKE_OVER;
    }
    else;
  }
  else;

}
//  std::cout << "Limited Steering: " << targetSteeringAngle << std::endl;
  
  if (std::abs(targetSteeringAngle) >= MAX_ABS_STEERING_ANGLE_LOW_SPEED) {
    if (targetSteeringAngle > 0) {
      targetSteeringAngle = MAX_ABS_STEERING_ANGLE_LOW_SPEED;
    }
    else if ( targetSteeringAngle < 0){
      targetSteeringAngle = -MAX_ABS_STEERING_ANGLE_LOW_SPEED;
    }
    else;
  }
  else; 
//  // just simple code!! 360 not considered!! // tbd
//  if (true == LIMIT_COURSE) {
//    if ( yaw > MAX_COURSE ) {
//      if (targetSteeringAngle > 0) {
//        targetSteeringAngle = 0;
//      }
//      else;
//    }
//    else if (yaw < MIN_COURSE){
//      if (targetSteeringAngle < 0) {
//        targetSteeringAngle = 0;
//      }
//      else;
//    }
//    else;
//  }
//  else;
  
  return targetSteeringAngle;
}

double limitPreviewDistance(double targetDistance, double lastTargetDistance, double velocity){
  std::cout << "Target Preview Distan: " << targetDistance << std::endl;
  std::cout << "Last Preview Distance: " << lastTargetDistance << std::endl;
//  if (std::abs(decisionData.targetSpeed -lastTargetSpeed) > 0.1 && decisionData.targetSpeed >SPEED_LOW) {
    if(velocity*3.6 > 20){
      if (targetDistance > lastTargetDistance + 1 ) {
        targetDistance = lastTargetDistance + 1; // previous 3
      }
    }
    else if(velocity*3.6 > 10 ){
      if (targetDistance > lastTargetDistance + 0.015) {
        targetDistance = lastTargetDistance + 0.015; // previous 0.5
      }
    }
    if (targetDistance < lastTargetDistance - 4 ){
      targetDistance = lastTargetDistance - 4;
    }
  return targetDistance;
}

double getAngle (double x0, double y0, double x1, double y1){
  double deltaY = y1 - y0;
  double deltaX = x1 - x0;
  //  double theta = 0;
  //  if ( 0 == deltaX) {
  //    if (0 > deltaY) {
  //      theta = -M_PI / 2;
  //    }
  //    else {
  //      theta = M_PI / 2;
  //    }
  //  }
  //  else{
  //    theta = atan
  //  }
  return std::atan2(deltaY, deltaX) / M_PI * 180;
}

// range is : ang0: Yaw : 0 ~ 360; ang1: Preview Angle zhijiaozuobiao : -180 ~ 180
double getAngleDiff (double ang0, double ang1){
  if (ang0 > 180) {
    ang0 = ang0 - 360;
  }
  else;
  ang0 = -ang0;
//  double temp1 = ang0;
  // converted_yaw = ang0;
  
  ang1 = ang1 - 90;
  if (ang1 < -180) {
    ang1 = ang1 + 360;
  }
  else;
  // converted_preview_angle = ang1;
//  double temp2 = ang1;
//  std::cout << "Converted Preview: " << ang1 << std::endl;
  
  double ret = ang1 - ang0;
  if (ret > 180) {
    ret = ret - 360;
  }
  else if (ret < -180){
    ret = ret + 360;
  }
  else;
  
//  double temp3 = ret;
  
//  std::cout << "Converted Yaw: " << temp1 << std::endl;
//  std::cout << "Converted Preview: " << temp2 << std::endl;
//  std::cout << "Angle Error: " << temp3 << std::endl;

  return ret;
}

// range is : ang0: Yaw : 0 ~ 360; ang1: Preview Angle zhijiaozuobiao : -180 ~ 180
double getAngleDiff1 (double ang0, double ang1){
  if (ang0 > 180) {
    ang0 = ang0 - 360;
  }
  else;
  // ang0 = -ang0
  ang0 = ang0 + 90;
//  double temp1 = ang0;
  converted_yaw = ang0;
  
  if(ang0 > 180){
    ang0 -= 360;
  }
  else if(ang0 < -180){
    ang0 += 360;
  }
  // ang1 = ang1 - 90;
  if (ang1 < -180) {
    ang1 = ang1 + 360;
  }
  else if(ang1 > 180){
    ang1 -= 360;
  }
  converted_preview_angle = ang1;
//  double temp2 = ang1;
//  std::cout << "Converted Preview: " << ang1 << std::endl;

  // double ret = ang1 - ang0;
  double ret = ang0 - ang1;
  if (ret > 180) {
    ret = ret - 360;
  }
  else if (ret < -180){
    ret = ret + 360;
  }
  else;
  
//  double temp3 = ret;
  
//  std::cout << "Converted Yaw: " << temp1 << std::endl;
//  std::cout << "Converted Preview: " << temp2 << std::endl;
//  std::cout << "Angle Error: " << temp3 << std::endl;

  return ret;
}

//yawVehicle: 0 ~ 360; yawRoad: 0 ~ 360
double getYawDiff (double yawVehicle, double yawRoad){
  if (yawVehicle > 180) {
    yawVehicle = yawVehicle - 360;
  }
  else;
  yawVehicle = -yawVehicle;
  
  if (yawRoad > 180) {
    yawRoad = yawRoad - 360;
  }
  else;
  yawRoad = -yawRoad;
  
  double ret = yawRoad - yawVehicle;
  
  if (ret > 180) {
    ret = ret - 360;
  }
  else if (ret < -180){
    ret = ret + 360;
  }
  else;
  
  std::cout << "Yaw Angle Vehi: " << yawVehicle << std::endl;
  std::cout << "Yaw Angle Road: " << yawRoad << std::endl;
  std::cout << "Yaw Angle Diff: " << ret << std::endl;
  
  return ret;
}

void followLidarAndLane (const LidarData& lidarData, const VisionData& visionData, const PostureData& postureData, DecisionData& decisionData){
  static PidController lidarPidController (5, 0, 0, 0, false, 0.1);
  static PidController lanePidController (1, 0, 0, 0, false, 0.1);
  static double pathAngleLidar;
  static double errorLookAhead;
  static double laneWeight;
  static double offsetMid;
  static double angleError;
  
  
  pathAngleLidar = LIDAR_FILTER * lidarData.zone.pathAngle + (1 - LIDAR_FILTER) * pathAngleLidar;
  double steeringAngleLidar = lidarPidController.update(pathAngleLidar);
  
  
  if (visionData.laneData.laneStatus[1] ==2 && visionData.laneData.laneStatus[2] == 2) {
    offsetMid = LANE_FILTER * (visionData.laneData.offsetRight - visionData.laneData.offsetLeft)/2 + (1 - LANE_FILTER)* offsetMid;
    angleError = LANE_FILTER * visionData.laneData.angleError + (1 - LANE_FILTER) * visionData.laneData.angleError;
  }
  else if (visionData.laneData.laneStatus[1] == 2) {
    offsetMid = LANE_FILTER * (LANE_WIDTH/2.0 - visionData.laneData.offsetRight) + ( 1 - LANE_FILTER) * offsetMid;
    angleError = LANE_FILTER * visionData.laneData.angleError + (1 - LANE_FILTER) * visionData.laneData.angleError;
  }
  else if (visionData.laneData.laneStatus[2] ==2){
    offsetMid = LANE_FILTER * (visionData.laneData.offsetLeft - LANE_WIDTH/2.0) + (1 - LANE_FILTER) * offsetMid;
    angleError = LANE_FILTER * visionData.laneData.angleError + (1 - LANE_FILTER) * visionData.laneData.angleError;
  }
  else {
    offsetMid = LANE_FILTER * 0 + (1 - LANE_FILTER) * offsetMid;
    angleError = LANE_FILTER * 0 + (1 - LANE_FILTER) * visionData.laneData.angleError;
  }
  
  errorLookAhead = LANE_FILTER * (-std::sin(angleError)*LANE_PREVIEW_DISTANCE + MID_LANE_WEIGHT * offsetMid) + (1 - LANE_FILTER) * errorLookAhead;
  std::cout << "offsetMid: " << offsetMid << std::endl;
  std::cout << "angleError: " << angleError << std::endl;
  
  double steeringAngleLane = lanePidController.update(errorLookAhead);
  
  if (visionData.laneData.laneStatus[1] == 2 || visionData.laneData.laneStatus[2] == 2) {
    laneWeight = laneWeight < 0.8? laneWeight + 0.01 : laneWeight;
  }
  else{
    laneWeight = laneWeight > 0? laneWeight - 0.01: 0;
  }
  if (laneWeight > 0.8) {
    laneWeight = 0.8;
  }
  else if (laneWeight < 0){
    laneWeight = 0.0;
  }
  else;
  double steeringAngle = laneWeight * steeringAngleLane + (1 - laneWeight) * steeringAngleLidar;
  steeringAngle = limitSteeringAngle(steeringAngle, decisionData.targetSteeringAngle, VELOCITY_FOLLOW_LANE_AND_LIDAR, postureData.imuData.yaw, false);
  decisionData.targetSteeringAngle = steeringAngle;
  
}

void followLidar (double refAngle, const PostureData& postureData, DecisionData& decisionData){
  static PidController lidarPidController (5, 0, 0, 0, false, 0.1);
//  double steeringAngle = lidarPidController.update(refAngle);
  static double pathAngleLidar;
  pathAngleLidar = LIDAR_FILTER * refAngle + (1 - LIDAR_FILTER) * pathAngleLidar;
  double steeringAngleLidar = lidarPidController.update(pathAngleLidar);
  steeringAngleLidar = limitSteeringAngle(steeringAngleLidar, decisionData.targetSteeringAngle, VELOCITY_FOLLOW_LIDAR, postureData.imuData.yaw, false);
  decisionData.targetSteeringAngle = steeringAngleLidar;
}

void followLane (const VisionData& visionData, const PostureData& postureData, DecisionData& decisionData){
  static PidController lanePidController (1, 0, 0, 0, false, 0.1);
  static double offsetMid;
  static double angleError;
  static double errorLookAhead;
  if (visionData.laneData.laneStatus[1] ==2 && visionData.laneData.laneStatus[2] == 2) {
    offsetMid = LANE_FILTER * (visionData.laneData.offsetRight - visionData.laneData.offsetLeft)/2 + (1 - LANE_FILTER)* offsetMid;
    angleError = LANE_FILTER * visionData.laneData.angleError + (1 - LANE_FILTER) * visionData.laneData.angleError;
  }
  else if (visionData.laneData.laneStatus[1] == 2) {
    offsetMid = LANE_FILTER * (LANE_WIDTH/2.0 - visionData.laneData.offsetRight) + ( 1 - LANE_FILTER) * offsetMid;
    angleError = LANE_FILTER * visionData.laneData.angleError + (1 - LANE_FILTER) * visionData.laneData.angleError;
  }
  else if (visionData.laneData.laneStatus[2] ==2){
    offsetMid = LANE_FILTER * (visionData.laneData.offsetLeft - LANE_WIDTH/2.0) + (1 - LANE_FILTER) * offsetMid;
    angleError = LANE_FILTER * visionData.laneData.angleError + (1 - LANE_FILTER) * visionData.laneData.angleError;
  }
  else {
    offsetMid = LANE_FILTER * 0 + (1 - LANE_FILTER) * offsetMid;
    angleError = LANE_FILTER * 0 + (1 - LANE_FILTER) * visionData.laneData.angleError;
  }

  errorLookAhead = LANE_FILTER * (-std::sin(angleError)*LANE_PREVIEW_DISTANCE + MID_LANE_WEIGHT * offsetMid) + (1 - LANE_FILTER) * errorLookAhead;
  std::cout << "offsetMid: " << offsetMid << std::endl;
  std::cout << "angleError: " << angleError << std::endl;
  
  double steeringAngle = lanePidController.update(errorLookAhead);
  steeringAngle = limitSteeringAngle(steeringAngle, decisionData.targetSteeringAngle, VELOCITY_FOLLOW_LANE, postureData.imuData.yaw, false);
  decisionData.targetSteeringAngle = steeringAngle;
}


void generateLidarCurvePreviewPoint (DecisionData& decisionData, std::vector<std::vector<RoadPoint>>& roadPointss, const PostureData& postureData, DecisionData& lidarDecisionData){

	lidarDecisionData = decisionData;
  lidarDecisionData.curveList.clear();

  static double last_curvature_diff = 0;
  double curvature_diff = fabs(global_current_curvature) + fabs(global_preview_curvature);
  curvature_diff = FILTER_RATE0 * last_curvature_diff + (1-FILTER_RATE0) * curvature_diff;

//  std::cout << lidarDecisionData.currentId << std::endl;
	double lidarPreviewDistance = DEFAULT_LIDAR_PREVIEW_DISTANCE;
  static double last_lidarPreviewDistance = DEFAULT_LIDAR_PREVIEW_DISTANCE;
  // if(curvature_diff > 0.5)
  //   lidarPreviewDistance /= 1.5;
  // lidarPreviewDistance -= fabs(curvature_diff)*2.5;

  lidarPreviewDistance = decisionData.previewDistance;

  if(lidarPreviewDistance < 13) lidarPreviewDistance = 13;

  lidarPreviewDistance = FILTER_RATE * last_lidarPreviewDistance + (1-FILTER_RATE) * lidarPreviewDistance;

  LIDAR_SAFE_DISTANCE_MAX = lidarPreviewDistance + 3;

  std::cout << "currentPreviewDistance: " << decisionData.previewDistance << std::endl;
  // if(decisionData.b_takingOver == true)
  //   lidarPreviewDistance = 3;
  lidarDecisionData.previewDistance = lidarPreviewDistance;
  last_lidarPreviewDistance = lidarPreviewDistance;
  std::cout << "lidarPreviewDistance:   " << lidarDecisionData.previewDistance << std::endl;
  lidarDecisionData.targetSpeed = SPEED_MID;
  double lidarVelocity = 0.0;
  double lidarCurvature = 0.0;

	RoadPoint lidarPreviewPoint = getPreviewPoint(lidarPreviewDistance, lidarVelocity, lidarCurvature, decisionData.currentPoint, postureData.imuData.yaw, decisionData.nextId, decisionData.nextNextId, roadPointss);
	lidarDecisionData.previewPoint = lidarPreviewPoint;
  global_preview_dist = lidarPreviewPoint.preview_dist;
  global_preview_curvature = lidarPreviewPoint.parameter2;


  //judge curve control point dist factor according to preview point curvature
  cp_dist_front_factor = fabs(global_current_curvature) / 4;
  cp_dist_back_factor = fabs(global_preview_curvature) / 4; 

  static double last_cp_front = 0;
  static double last_cp_back = 0;

  // cp_dist_front_factor = fabs(global_current_curvature) / (fabs(global_current_curvature) + fabs(global_preview_curvature));
  // cp_dist_back_factor = fabs(global_preview_curvature) / (fabs(global_current_curvature) + fabs(global_preview_curvature));

  cp_dist_front_factor = FILTER_RATE0 * last_cp_front + (1-FILTER_RATE0) * cp_dist_front_factor;
  cp_dist_back_factor = FILTER_RATE0 * last_cp_back + (1-FILTER_RATE0) * cp_dist_back_factor;

  if(cp_dist_front_factor < 0.05) cp_dist_front_factor = 0.05;
  else if(cp_dist_front_factor > 0.95) cp_dist_front_factor = 0.95;
  if(cp_dist_back_factor < 0.05) cp_dist_back_factor = 0.05;
  else if(cp_dist_back_factor > 0.95) cp_dist_back_factor = 0.95;

  // extend_length = 23 - fabs(curvature_diff) * 3.5;
  // if(extend_length < 0) extend_length = 0;
  extend_length = 0;

  CP_DISTANCE_BACK = (lidarPreviewDistance + extend_length) * cp_dist_back_factor;
  CP_DISTANCE_FRONT = (lidarPreviewDistance + extend_length) * cp_dist_front_factor;

  std::cout << "extend_length:             " << extend_length << std::endl;
  std::cout << "GLOBAL_PREVIEW_DIST:       " << global_preview_dist << std::endl;
  std::cout << "global_preview_curvature:  " << global_preview_curvature << std::endl;
  std::cout << "global_current_curvature:  " << global_current_curvature << std::endl;
  std::cout << "CP_DISTANCE_BACK:          " << CP_DISTANCE_BACK << std::endl;
  std::cout << "CP_DISTANCE_FRONT:         " << CP_DISTANCE_FRONT << std::endl;

	generateCurvePreviewPoint(lidarDecisionData, postureData);
//  std::cout << "size: " << lidarDecisionData.curveList.size()<< std::endl;
}

void generateLidarCurvePreviewPointForLine (DecisionData& decisionData, std::vector<std::vector<RoadPoint>>& roadPointss, const PostureData& postureData, DecisionData& lidarDecisionData){

}

void followRoadMap (double longitude, double latitude, double yaw, double velocity, std::vector<std::vector<RoadPoint>>& roadPointss, DecisionData& decisionData, std::vector<std::vector<uint32_t>>& pathNumberLists){
//  std::cout << roadPointss[1][0].latitude << std::endl;
//  std::cout << roadPointss[1][0].longitude << std::endl;
  
//  static double lastIntegralValue;
  static uint16_t lastRoadType;
  // need consider Ki tbd:
//  static PidController roadMapPidController (200, 0, 40, 0, false, 0.1);
//  150 for YBY
//  static PidController roadMapPidController (150, 0, 80, 0, false, 0.1);
  static PidController roadMapPidController (150, ki_test, 80, 0, false, 0.1);
//  static PidController roadMapTurnPidController (450, 0, 0, 0, false, 0.1);
//  static PidController roadMapTurnPidController (650, 0, 0, 0, false, 0.1);
//  static PidController roadMapTurnPidController (700, 0, 0, 0, false, 0.1);
//  double distance = PREVIEW_VELOCITY_MULTIPLIER * velocity;
  
  uint32_t roadPointssSize = static_cast<uint32_t>(roadPointss.size()) - 1;
  uint32_t lastId = decisionData.currentId;
  uint32_t nextIdFromLast = getNextRoadId(lastId, roadPointssSize, decisionData, pathNumberLists);
  RoadPoint currentPoint = getCurrentPoint(longitude, latitude, yaw, roadPointss, lastId, nextIdFromLast, decisionData, pathNumberLists, decisionData.pathNumber);
  
  global_current_curvature = currentPoint.parameter2;
  std::cout << "-----------------global_current_curvature:" << global_current_curvature << std::endl;

  if (decisionData.currentId > roadPointss.size()) {
    decisionData.currentId = MINIMAL_ID;
  }
  std::cout<<" CurrentPoint Yaw = "<<currentPoint.courseAngle<<"  VehicleYaw = "<<yaw<<std::endl;
  std::cout << "CurrentPoint Yaw - VehicleYaw: " << std::fabs((360 + 180 + (int)currentPoint.courseAngle - (int)yaw)%360 - 180) << std::endl;
  std::cout << "Current ID: " << decisionData.currentId << std::endl;
  std::cout << "Next    ID: " << decisionData.nextId << std::endl;
  if (true == currentPoint.b_isValid) {
    decisionData.currentId = currentPoint.roadId;
    decisionData.roadType = currentPoint.roadType;
    decisionData.currentIndex = currentPoint.index;
    decisionData.currentTotalIndex = currentPoint.totalIndex;
    decisionData.currentPoint = currentPoint;
    
    std::cout << "currentPoint.parameter2: "<<currentPoint.parameter2 << std::endl;


    
    int32_t nextId = getNextRoadId(currentPoint.roadId, roadPointssSize, decisionData, pathNumberLists);
    decisionData.nextId = nextId;
//    int32_t nextNextId = getNextRoadId(nextId, roadPointssSize, decisionData, pathNumberLists);
    int32_t nextNextId = nextId;
    decisionData.nextNextId = nextNextId;
//    if (nextId == 0) {
//      std::cout << "!!!" << std::endl;
//    }
    
    static double lastPreviewCurvature = 0.0;
    static double lastMaxmaxPreviewCurvature = 0.0;
    double maxPreviewCurvature = std::fmax(std::fabs(lastPreviewCurvature), std::fabs(currentPoint.parameter2));
    maxPreviewCurvature = (1 - FILTER_RATE) * maxPreviewCurvature + FILTER_RATE * lastMaxmaxPreviewCurvature;
    lastMaxmaxPreviewCurvature = maxPreviewCurvature;
    std::cout << "maxPreviewCurvature: " << maxPreviewCurvature << std::endl;
//    int32_t nextId = getLastRoadId(currentPoint.roadId, roadPointssSize);

    double previewDistance = getPreviewDistance(velocity, maxPreviewCurvature, currentPoint, decisionData);

//    RoadPoint previewPoint = getPreviewPoint(velocity, maxPreviewCurvature, currentPoint, yaw, nextId, nextNextId, roadPointss);

    //modify preview distance for taking over
    // if (true == decisionData.b_takingOver)
    //   previewDistance = 3;

    static double last_previewDistance = 5;
    previewDistance = last_previewDistance * FILTER_RATE + previewDistance * (1 - FILTER_RATE);
    // if(previewDistance - last_previewDistance > 0.01) previewDistance = last_previewDistance + 0.01;
    last_previewDistance = previewDistance;

    //fix previewDistance for stanley
    // previewDistance = P_dist_stanley;

    preview_dist_test = previewDistance;  

    RoadPoint previewPoint = getPreviewPoint(previewDistance, velocity, maxPreviewCurvature, currentPoint, yaw, nextId, nextNextId, roadPointss);
    lastPreviewCurvature = previewPoint.parameter2;
    decisionData.previewPoint = previewPoint;
    decisionData.previewPointTakeOver = previewPoint;
    std::cout << "--------------previewPoint.b_isValid:" << previewPoint.b_isValid << std::endl;
    if (true == previewPoint.b_isValid) {
      decisionData.currentState = STATE_FOLLOW_ROADMAP; // 1 - FOLLOW_ROAD_MAP
      double previewAngle = 0.0;
      if (false == decisionData.b_takingOver){
        previewAngle = getAngle(longitude, latitude, previewPoint.longitude, previewPoint.latitude);
        std::cout << "Following normal lane." << std::endl;
      }
      else if (true == decisionData.b_takingOver){ // during taking over
        RoadPoint previewPointTakeOver = movePreviewPoint(previewPoint, actual_curve_dist);
        decisionData.previewPointTakeOver = previewPointTakeOver;
        previewAngle = getAngle(longitude, latitude, previewPointTakeOver.longitude, previewPointTakeOver.latitude);
        std::cout << YELLOW << "Following left lane." << RESET << std::endl;
      }
      
      static double last_angleDiff = 0;
      static double last_yawDiff = 0;
      static double last_courseAngle = 0;
      static double llast_courseAngle = 0;
      double previewCourseAngle = 0;
      previewCourseAngle = last_courseAngle * FILTER_RATE1 + previewPoint.courseAngle * (1-FILTER_RATE1);
      last_courseAngle = previewCourseAngle;
      previewCourseAngle = llast_courseAngle * FILTER_RATE11 + previewCourseAngle * (1-FILTER_RATE11);
      llast_courseAngle = previewCourseAngle;
      double angleDiff = getAngleDiff(yaw, previewAngle);
      double yawDiff = getYawDiff(yaw, previewCourseAngle);

      //1st degree filter
      // angleDiff = FILTER_RATE0 * last_angleDiff + (1 - FILTER_RATE0) * angleDiff + decisionData.current_steering_angle/100;
      // angleDiff = converted_preview_angle - converted_yaw;
      angleDiff = FILTER_RATE0 * last_angleDiff + (1 - FILTER_RATE0) * angleDiff;
      last_angleDiff = angleDiff;

      yawDiff = FILTER_RATE2 * last_yawDiff + (1 - FILTER_RATE2) * yawDiff;
      last_yawDiff = yawDiff;
      
//      std::cout << "Converted Yaw: " << temp1 << std::endl;
//      std::cout << "Converted Preview: " << temp2 << std::endl;
      std::cout << "Yaw:                                                        " << converted_yaw << ", Preview Angle:                   " << converted_preview_angle << std::endl;
      std::cout << "Angle Error:                                                " << angleDiff << std::endl;
    //  std::cout << "Yaw Diff: " << angleDiff << std::endl;
      angleDiffTest = angleDiff;
      
      // double steeringAngle = decisionData.targetSteeringAngle;
//      if (previewPoint.roadType == 0 || previewPoint.roadType == 1) {
//      if (previewPoint.roadType == 0 ) {
//        if (previewPoint.roadType != lastRoadType) {
//          roadMapPidController.resetI();
//        }
//        else;
      
    //    lastIntegralValue = roadMapPidController.integral;
      
      //kp
      static double lastKp = 100;
      double newKp = lastKp;
      static double lastKd = 50;
      double newKd = lastKd;
      //    if (std::fabs(currentPoint.parameter2) < 0.5) {
      ////      newKp = 400*std::fabs(currentPoint.parameter2) + 150;
      //      newKp = 200;
      //    }
      //    else if(std::fabs(currentPoint.parameter2) >= 0.5){
      ////      newKp = 40*std::fabs(currentPoint.parameter2) + 330;
      //      newKp = 850;
      //    }
      //    else;
      std::cout << "Curvature: " << std::fabs(maxPreviewCurvature) <<std::endl;
      static double last_c_cur = 0;
      double c_cur = last_c_cur * FILTER_RATE3 + maxPreviewCurvature * (1-FILTER_RATE3);
      last_c_cur = c_cur;

      //  PID gain
      if (std::fabs(c_cur) >= 0.5) {
        newKp = kp_large; // previous 500 
        newKd = kd_large; //previous 15
        YAW_WEIGHT = 0.05;
      }
      else{
        newKp = lastKp - 1; // previous 10
        if (newKp < kp_small) {
          newKp = kp_small;
          newKd = kd_small;
        }else;
        YAW_WEIGHT += 0.001;
        if(YAW_WEIGHT > 0.5)
          YAW_WEIGHT = 0.5;
      }
      
      if (decisionData.b_takingOver == true) {
        newKp = kp_large;
        newKd = kd_large;
      }
      
      lastKp = newKp;
      lastKd = newKd;
      
      double targetSpeed = 0;
      if(SPEED_PID_SWITCH == true)
      {
        //speed 5~25
        // target speed
        // calc targetspeed
        static double last_targetSpeed = 0;

        // printf("curv<0.5,  target speed = - %.3f * curvvature + %.3f\n",spdK1,spdcrv1+spdK1*0.1);
        // printf("curv>=0.5, target speed = - %.3f * curvvature + %.3f\n",spdK2,spdcrv2+spdK2*0.5);
        static double last_curvature_temp = 0;
        double curvature_temp = (previewPoint.parameter2 > global_current_curvature) ? previewPoint.parameter2 : global_current_curvature; 
        curvature_temp = (1-CURV_WEIGHT) * curvature_temp + CURV_WEIGHT * last_curvature_temp;
        printf("curvature = %.3f\n",std::fabs(curvature_temp));
        last_curvature_temp = curvature_temp;

        if (std::fabs(curvature_temp) < 0.5) {
          targetSpeed = - spdK1 * std::fabs(curvature_temp) + spdcrv1+spdK1*0.1; //0.1
        }
        else if (std::fabs(curvature_temp) >= 0.5){
          targetSpeed = - spdK2 * std::fabs(curvature_temp) + spdcrv2+spdK2*0.5;
        }
        else;
        
        targetSpeed = (1 - SPEED_WEIGHT) * targetSpeed + SPEED_WEIGHT * last_targetSpeed;

        if (targetSpeed > (TOP_SPEED/3.6)) {//old 10
          targetSpeed = (TOP_SPEED/3.6);
          std::cout << "Top Speed Limited" << std::endl;
        }

        else if (targetSpeed < 1.5){
          targetSpeed = 1.5;
        }
        else;

        if(curvature_temp >= 0.5)
        {
          if (targetSpeed - last_targetSpeed > 0.01){
            targetSpeed = last_targetSpeed + 0.01;
          }
        }

        last_targetSpeed = targetSpeed;
        std::cout << CYAN << "targetSpeed after PID:                        " << targetSpeed*3.6 << RESET << " km/h\n";
      }

      roadMapPidController.modifyGains(newKp, ki_test, newKd);
      std::cout << "Gain - Kp: " << newKp << std::endl;
      std::cout << "Gain - Kd: " << newKd << std::endl;
//      steeringAngle = roadMapPidController.update(angleDiff/180*M_PI);
      
      double error = (angleDiff * (1-YAW_WEIGHT) +yawDiff * YAW_WEIGHT)/180*M_PI;
      double error_a =   (angleDiff * (1-YAW_WEIGHT) +yawDiff * YAW_WEIGHT);
      std::cout << "error: " << error_a << std::endl; 

      // stanley
      // currentPoint.lateral_dist -= P_dist_stanley * yawDiff;
      double s_angle = stanley(decisionData, currentPoint, error_a, velocity);
      // double s_angle = stanley(decisionData, previewPoint, error_a, velocity);
      s_angle *= Tire2Wheel;
      static double last_s_angle = 0;
      s_angle = FILTER_RATE4 * last_s_angle + (1-FILTER_RATE4) * s_angle;
      last_s_angle = s_angle;
      std::cout << "[stanley] s_angle: " << s_angle << std::endl;

      double steeringAngle = 0;

      steeringAngle = roadMapPidController.update(error);
//        std::cout << "lane driving. " << std::endl;
//      }
//      else {
//        steeringAngle = roadMapTurnPidController.update(angleDiff/180*M_PI);
//    //    lastIntegralValue = roadMapTurnPidController.integral;
//        std::cout << "turning. " << std::endl;
//      }

      //stanley 
      if (false == decisionData.b_takingOver)
        steeringAngle = s_angle;
      
      lastRoadType = previewPoint.roadType;
      static double last_steeringAngle = 0;
      if (decisionData.b_takeOverEnable==false){
        if (last_steeringAngle*steeringAngle>0 && 
          fabs(last_steeringAngle)<fabs(steeringAngle))
        steeringAngle = limitSteeringAngle(steeringAngle, last_steeringAngle, decisionData.targetSpeed, yaw, false);
      }
      else{
        steeringAngle = limitSteeringAngle(steeringAngle, last_steeringAngle, decisionData.targetSpeed, yaw, true);
      }
    //  if(velocity < 0.5){
 		// 	steeringAngle = 0;
 		// 	std::cout<< "--steering angle limited to zero!\n";
 		// }

     //1st degree filter
      steeringAngle = steeringAngle * (1-FILTER_RATE) + FILTER_RATE * last_steeringAngle;
  		decisionData.targetSteeringAngle = steeringAngle + STEERING_ANGLE_ADJUST;

      std::cout << GREEN << "  steeringAngle OUT = " << RESET << std::fixed << std::setprecision(3) << decisionData.targetSteeringAngle <<'\n';
      // decisionData.targetSteeringVelocity = fabs((steeringAngle - last_steeringAngle)*10);
      // if(fabs(last_steeringAngle)<fabs(steeringAngle))
      //   decisionData.targetSteeringVelocity = 15;
      // else
      //   decisionData.targetSteeringVelocity = 30;

      decisionData.targetSteeringVelocity = fabs(angleDiff)*140/90+300;
      if (last_steeringAngle*steeringAngle>0 && 
          fabs(last_steeringAngle)>fabs(steeringAngle))
          {
            decisionData.targetSteeringVelocity = fabs(angleDiff)*140/90+300;
          }
      // decisionData.targetSteeringVelocity = decisionData.targetSteeringVelocity - velocity*3.6;
      // decisionData.targetSteeringVelocity = (160 - velocity*3.6)/160 * decisionData.targetSteeringVelocity;
      if(decisionData.targetSteeringVelocity < 5)
        decisionData.targetSteeringVelocity  = 5;

      std::cout << GREEN << "  targetSteeringVelocity OUT = " << RESET << std::fixed << std::setprecision(1) << decisionData.targetSteeringVelocity <<'\n';
      last_steeringAngle = steeringAngle;
      //Limit speed when close to start point
//      if (inArea(longitude, latitude, roadPointss[1][0].longitude, roadPointss[1][0].latitude, 20.0)) {
//        targetSpeed = (targetSpeed > INITIAL_SPEED) ? INITIAL_SPEED : targetSpeed;
//      }
//      else;
      //Limit speed when close to a particular point
//      if (inArea(longitude, latitude, -1004200, 3357570, 17.0)) {
//        targetSpeed = (targetSpeed > LIDAR_MAX_SPEED) ? LIDAR_MAX_SPEED : targetSpeed;
//      }
//      else;
//      
      static double lastTargetSpeed = 0;
      
      lastTargetSpeed = decisionData.targetSpeed;
//      decisionData.targetSpeed = limitTargetSpeed(decisionData, targetSpeed)/3.6;
//      targetSpeed = limitTargetSpeed(decisionData, targetSpeed)/3.6;
      //targetSpeed = limitSpeed(targetSpeed, lastTargetSpeed);
      
      std::cout << YELLOW <<  " roadmap speed: "<< currentPoint.parameter3*3.6 << RESET << std::endl;
      if (targetSpeed > currentPoint.parameter3 && ROADMAP_LIMIT_SWITCH == true) {
        targetSpeed = currentPoint.parameter3;
        //set small speed to 0
        if(targetSpeed*36 < 5) targetSpeed = 0;
        std::cout << YELLOW <<  " Speed limited by roadmap!\n" << RESET << std::endl;
      }
      else if(SPEED_PID_SWITCH == false){
        targetSpeed = currentPoint.parameter3;
      }

      mapTargetSpeed = currentPoint.parameter3;

      if (true == SPEED_SMOOTH){
        if ( std::fabs(currentPoint.parameter2) < 0.5 && velocity < (8/3.6) ){
          if (targetSpeed - lastTargetSpeed > 0.0125){
            targetSpeed = lastTargetSpeed + 0.0125;
          }
          else{
          //wrong
//            if ( targetSpeed - velocity > 0 ){
//              targetSpeed = velocity;
//            }
//            else;
          }
          //smooth decrease
          if ( lastTargetSpeed - targetSpeed > 0.1){
            targetSpeed = lastTargetSpeed - 0.1;
          }

        }
        else if(std::fabs(currentPoint.parameter2) < 0.5 && velocity < (15/3.6) ){
          if (targetSpeed - lastTargetSpeed > 0.015){
            targetSpeed = lastTargetSpeed + 0.015;
          }
          else;
          //smooth decrease
          if ( lastTargetSpeed - targetSpeed > 0.2){
            targetSpeed = lastTargetSpeed - 0.2;
          }
        }

        else{
          if (targetSpeed > (TURN_SPEED_LIMIT/3.6) && std::fabs(currentPoint.parameter2) >= 0.5){
            targetSpeed = (TURN_SPEED_LIMIT/3.6);
            std::cout << MAGENTA << "TURN DETECTED, LIMIT SPEED" << RESET << std::endl;
          }
          else;

          if (targetSpeed - lastTargetSpeed > 0.0125){
            targetSpeed = lastTargetSpeed + 0.0125;
          }
          else;

          //smooth decrease
          if ( lastTargetSpeed - targetSpeed > 0.2){
            targetSpeed = lastTargetSpeed - 0.2;
          }
        }
      }
      else;

      if(targetSpeed > (8/3.6) && velocity < (5/3.6))
        targetSpeed = 8/3.6; // avoid jerking at start

//      if (true) {
//        if (targetSpeed - velocity > 1){
//          targetSpeed = velocity + 1;// old limit=1
//          std::cout << " targetSpeed limited by smooth acceleration\n";
//        }
//      }

      if (targetSpeed > (TOP_SPEED/3.6)) {//old 10
          targetSpeed = (TOP_SPEED/3.6);
          std::cout << "Top Speed Limited" << std::endl;
        }

      std::cout << GREEN << "  targetSpeed OUT =                          "  << std::fixed << std::setprecision(1) << targetSpeed*3.6 << RESET << " km/h\n";
      
      decisionData.targetSpeed = targetSpeed;
//      decisionData.targetSpeed = targetSpeed/3.6;
//      std::cout << "Decision Steering: " << decisionData.targetSteeringAngle << std::endl;
    }
    else {
      // tbd end?
      decisionData.currentState = STATE_INITIAL;
    }
  }
  
  else{
//    decisionData.updateStateMachine();
    //tbd
    decisionData.currentId += 1;
    doNothing();
  }
}

//legacy function, use curvePointOccupied instead.
bool gridOccupied(Point point, uint32_t gridIndex){
  
  //tbd. 0 and 200 not considered yet.
  bool b_occupied = false;
  uint16_t indexExpandSize = static_cast<uint16_t>(LIDAR_SAFE_ZONE_MIN/2/GRID_X_RESOLUTION);
 
  double x = point.x;
  double y = point.y;
//  std::cout << "x: " << x << ", y: " << y << std::endl;
 
//  if (y>2){
//    std::cout << "here" << std::endl;
//  }
  if( x < X_SIZE_MIN){
    x = X_SIZE_MIN;
  }
  else if (x > X_SIZE_MAX){
    x = X_SIZE_MAX - GRID_X_RESOLUTION;
  }
  else{
    ;
  }
  if (y < Y_SIZE_MIN){
    y = Y_SIZE_MIN;
  }
  else if (y > Y_SIZE_MAX){
    y = Y_SIZE_MAX - GRID_Y_RESOLUTION;
  }
  else{
    ;
  }
  
  int16_t xIndex = static_cast<int16_t>((x - X_SIZE_MIN) / GRID_X_RESOLUTION);
  xIndex = (xIndex < 0)? 0: xIndex;
  xIndex = (xIndex > (X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION - 1) ? (X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION - 1 : xIndex;
  
  int16_t yIndex = static_cast<int16_t>((y - Y_SIZE_MIN) / GRID_Y_RESOLUTION);
  yIndex = (yIndex < 0)? 0: yIndex;
  yIndex = (yIndex > (Y_SIZE_MAX - Y_SIZE_MIN)/GRID_Y_RESOLUTION - 1) ? (Y_SIZE_MAX - Y_SIZE_MIN)/GRID_Y_RESOLUTION -1 : yIndex;
  
  uint32_t index = yIndex * (X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION + xIndex;
  index = (index <= 0)? 0: index;
  index = (index > (X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION * (Y_SIZE_MAX - Y_SIZE_MIN)/GRID_Y_RESOLUTION - 1) ? (X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION * (Y_SIZE_MAX - Y_SIZE_MIN)/GRID_Y_RESOLUTION - 1: index;
  
//  if(index > 2000){
//    std::cout << "3";
//    
//  }else;
  
  for (uint32_t i = index - indexExpandSize; i < index + indexExpandSize; i++) {
    if (i == gridIndex) {
      b_occupied = true;
    }
    else;
  }

  return b_occupied;
}

Curve createVirtualCurve() {
  Curve virtualCurve;
  double xMin = 0.0;
  double xMax = 0.0;
  double yMin = 0.0;
  double yMax = 7.5;
  double yStep = (yMax - yMin) / CURVE_POINT_NUM;
  for (size_t i=0; i<CURVE_POINT_NUM; ++i) {
    Point p;
    p.x = 0.0;
    p.y = yMin + i*yStep;
    virtualCurve.points[i] = p;
  }
  return virtualCurve;
}

double curvePointOccupied(Point& point, bool* b_obstacle, double previewDistance, std::vector<int32_t>& IndexLateralList, std::vector<int32_t>& IndexVerticalList,std::vector<int32_t>& IndexCurveList, double judgeSpeed, bool isVirtual=false){
  bool b_occupied = false;
  double advisedSpeed = judgeSpeed;
  double advisedSpeedLateral = judgeSpeed;
  double advisedSpeedLongitudinal = judgeSpeed;
  
  int32_t indexExpandSizeMin = static_cast<uint16_t>(LIDAR_SAFE_ZONE_MIN/2/GRID_X_RESOLUTION);
  //std::cout << "___________LIDAR_SAFE_ZONE_MIN________: " << LIDAR_SAFE_ZONE_MIN << std::endl; 
  //std::cout << "___________indexExpandSizeMin________: " << indexExpandSizeMin << std::endl; 
  int32_t indexExpandSizeMax = static_cast<uint16_t>(LIDAR_SAFE_ZONE_MAX/2/GRID_X_RESOLUTION);
//  int32_t indexDistanceSizeMin = static_cast<uint16_t>(LIDAR_SAFE_DISTANCE_MIN/GRID_Y_RESOLUTION);
  if (previewDistance*PREVIEW_DISTANCE_RATIO < LIDAR_SAFE_DISTANCE_MIN) {
    previewDistance = LIDAR_SAFE_DISTANCE_MIN/PREVIEW_DISTANCE_RATIO;
  }
  else;

  //int32_t indexDistanceSizeMin = static_cast<uint16_t>(previewDistance*PREVIEW_DISTANCE_RATIO/GRID_Y_RESOLUTION);
  if (isVirtual) { // vertical virtual curve
    //std::cout << "___________Virtual Curve_____________" << std::endl;
    LIDAR_SAFE_DISTANCE_MIN = 5.0;
  }
  int32_t indexDistanceSizeMin = static_cast<uint16_t>(LIDAR_SAFE_DISTANCE_MIN/GRID_Y_RESOLUTION);
  int32_t indexDistanceSizeMax = static_cast<uint16_t>(LIDAR_SAFE_DISTANCE_MAX/GRID_Y_RESOLUTION);
  
//  int32_t indexDistanceSizeMax = static_cast<uint16_t>(LIDAR_SAFE_DISTANCE_MAX/GRID_Y_RESOLUTION);
  
//  std::cout << "'Check': " << previewDistance << ", " << indexDistanceSizeMin<< std::endl; 


  std::ofstream fs("zoneData.txt", std::ofstream::trunc);
  fs << std::setprecision(15) << LIDAR_SAFE_ZONE_MIN << " " << LIDAR_SAFE_ZONE_MAX << " " << previewDistance*PREVIEW_DISTANCE_RATIO << " " << LIDAR_SAFE_DISTANCE_MAX << std::endl;

  const uint16_t xIndexMax = (X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION - 1;
  const uint16_t yIndexMax = (Y_SIZE_MAX - Y_SIZE_MIN)/GRID_Y_RESOLUTION - 1;
  const uint32_t indexMax = (X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION * (Y_SIZE_MAX - Y_SIZE_MIN)/GRID_Y_RESOLUTION - 1;
  
  double x = point.x;
  double y = point.y;
  std::cout << "x: " << x << ", y: " << y << std::endl;
  
  //  if (y>2){
  //    std::cout << "here" << std::endl;
  //  }
  if( x < X_SIZE_MIN){
    x = X_SIZE_MIN;
  }
  else if (x > X_SIZE_MAX){
    x = X_SIZE_MAX - GRID_X_RESOLUTION;
  }
  else{
    ;
  }
  if (y < Y_SIZE_MIN){
    y = Y_SIZE_MIN;
  }
  else if (y > Y_SIZE_MAX){
    y = Y_SIZE_MAX - GRID_Y_RESOLUTION;
  }
  else;
  
  int16_t xIndex = static_cast<int16_t>((x - X_SIZE_MIN) / GRID_X_RESOLUTION);
  xIndex = (xIndex < 0)? 0: xIndex;
  xIndex = (xIndex > xIndexMax) ? xIndexMax : xIndex;
  
  int16_t yIndex = static_cast<int16_t>((y - Y_SIZE_MIN) / GRID_Y_RESOLUTION);
  yIndex = (yIndex < 0)? 0: yIndex;
  yIndex = (yIndex > yIndexMax) ? yIndexMax : yIndex;
  
  int32_t index = yIndex * (X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION + xIndex;
  index = (index <= 0)? 0: index;
  index = (index > indexMax) ? indexMax: index;
  //Display Lidar
  IndexCurveList.push_back(index);

//横向距离小于最大值则开始进入判断，横向距离在最大值于最小值之间且纵向距离小于最小值则开始减速，纵向距离大于最小值不做动作，其他情况即横纵向距离均小于最小值则速度为零
//  std::cout << "<check1>: " << index << ", " << indexExpandSizeMax << std::endl;
  for (int32_t i = index - indexExpandSizeMax; i < index + indexExpandSizeMax; i++) {
    if (i <= indexMax) {
//    std::cout << "<check1>: " << i << ", " << index << std::endl;
      if (true == b_obstacle[i]) {
      //  std::cout << "_____________<check3>_____________: " << i << std::endl;
      //  std::cout << "_____________index_______________: " << index << std::endl;
      //  std::cout << "_____________xIndex_______________ = " << xIndex << std::endl;
      //  std::cout << "_____________yIndex_______________ = " << yIndex << std::endl;
        b_occupied = true;
//         double currentMaxLateral = advisedSpeed;
// //        std::cout << std::fabs(i-index)-indexExpandSizeMin << std::endl;
//         if (std::fabs(i-index)-indexExpandSizeMin > 0 && yIndex < indexDistanceSizeMin*2) {
          
// //          std::cout << std::fabs(((double)i-index)) << std::endl;
//           currentMaxLateral = advisedSpeed/static_cast<double>(indexExpandSizeMax - indexExpandSizeMin) * (std::fabs((double)(i - index)) - indexExpandSizeMin);
//         }
//         else if (yIndex >= indexDistanceSizeMin*2){
//           doNothing();
//         }
//         else if(yIndex < indexDistanceSizeMin){
//           currentMaxLateral = 0;
//           //Display Lidar
//           IndexLateralList.push_back(index);
//         }
//         advisedSpeedLateral = advisedSpeedLateral > currentMaxLateral? currentMaxLateral:advisedSpeedLateral;
        
        // logitudinal //纵向距离大于最小值且横向距离小于最小值则开始减速，纵向距离小于最小值且横向距离小于最小值则速度为零
        double currentMaxLongitudinal = advisedSpeed;
        if (yIndex > indexDistanceSizeMin && i > index - indexExpandSizeMin && i < index + indexExpandSizeMin) {
          currentMaxLongitudinal = advisedSpeed/(double)(indexDistanceSizeMax - indexDistanceSizeMin) * std::fabs(yIndex - indexDistanceSizeMin);
          std::cout << "__________Jiansu_______________ =" << currentMaxLongitudinal << std::endl;
        }
        else if (yIndex <= indexDistanceSizeMin && i > index - indexExpandSizeMin && i < index + indexExpandSizeMin)
        {
          currentMaxLongitudinal = 0;
          std::cout << "_________TINGCHE_______________ = " << std::endl;
          //Display Lidar
          IndexVerticalList.push_back(index);
        }
        else;
        
        advisedSpeedLongitudinal = advisedSpeedLongitudinal > currentMaxLongitudinal? currentMaxLongitudinal: advisedSpeedLongitudinal;
//        std::cout << "<check2>: " << advisedSpeedLongitudinal << std::endl;
      }
      else;
    }
    else;
  }
  advisedSpeed = std::fmin(advisedSpeedLateral, advisedSpeedLongitudinal);
  
 //std::cout << "___________________advised Speed_____________: " << advisedSpeed <<std::endl;
//  return advisedSpeed/5;
  return advisedSpeed;
}

void interpolate(Curve& curve){
  curve.pointList.clear();
  curve.pointList.reserve(CURVE_POINT_NUM);
  curve.pointList.push_back(curve.points[0]);
  //std::cout << "curve list number: " << curve.pointList.size();
  for (uint32_t i = 1; i < CURVE_POINT_NUM; i++){
    Point pointDistance;
    pointDistance.x = curve.points[i].x - curve.points[i-1].x;
    pointDistance.y = curve.points[i].y - curve.points[i-1].y;
    pointDistance.angle = std::atan2(pointDistance.y, pointDistance.x);
    double distance = std::sqrt(std::pow(pointDistance.x, 2) + std::pow(pointDistance.y, 2));
    double curvePointResolution = CURVE_POINT_RESOLUTION;
    uint32_t dividingNumber = std::ceil(distance/curvePointResolution);
    //std::cout<< "dividing number: " << dividingNumber << std::endl;
    for(uint32_t j = 0; j < dividingNumber; j++){
      Point finerPoint;
      finerPoint.x = curve.points[i-1].x + static_cast<double>(pointDistance.x/dividingNumber);
      finerPoint.y = curve.points[i-1].y + static_cast<double>(pointDistance.y/dividingNumber);
      finerPoint.angle = pointDistance.angle;
      curve.pointList.push_back(finerPoint);
    }
  }
  
}

double curvesLidarVerticalCheck (Curve& curve, std::vector<uint32_t>& blockedAreaIndex){
// return percentage [0,1] of advised speed
  double headingVertical, cosVertical, sinVertical;
  double shiftX, shiftY, indexX, indexY, cellIndex;
  double advisedValue;
  double pointNum;
  
  interpolate(curve);
  pointNum = curve.pointList.size();
  std::cout<<"[LidarCheck] curve pointNum = "<<pointNum<<'\n';
  std::cout<<"[LidarCheck] R_NUM = "<<LIDAR_SAFE_ZONE_MAX_R_NUM<<std::endl;
//  for(int i=1; i<pointNum; i++){
//    std::cout<<"  ptHdg = "<<curve.pointList[i].angle<<std::endl;
//  }
  for(int i=1; i<pointNum; i++){
    //std::cout<<"[VC] i = "<<i<<'\n';
    headingVertical = curve.pointList[i].angle + M_PI/2; // heading angle of perpendicular
    if( headingVertical > M_PI ) headingVertical -=2*M_PI;
    //std::cout<<"  heading = "<<headingVertical<<std::endl;
    cosVertical=std::cos(headingVertical);
    sinVertical=std::sin(headingVertical);
    for(int t=-LIDAR_SAFE_ZONE_MAX_R_NUM; t<=LIDAR_SAFE_ZONE_MAX_R_NUM; t++){
    	//std::cout<<"[LidarCheck]"<<t<<std::endl;
      shiftX=t*GRID_X_RESOLUTION_CHECK*cosVertical;
      shiftY=t*GRID_Y_RESOLUTION_CHECK*sinVertical;
      
      // calc index in grid-cell
      indexX=std::floor((curve.pointList[i].x+shiftX)/GRID_X_RESOLUTION);
      indexY=std::floor((curve.pointList[i].y+shiftY)/GRID_Y_RESOLUTION);
      // begin cell-check
      cellIndex = (indexY-1)*GRID_INDEX_X_SIZE+indexX;
      if(i==11){
      //std::cout<<"[LidarCheck] shift XY = "<<shiftX<<' '<<shiftY<<std::endl;
      //std::cout<<"[LidarCheck] cell XY = "<<curve.pointList[i].x+shiftX<<' '<<curve.pointList[i].y+shiftY<<std::endl;
      //std::cout<<"[LidarCheck] cellIndex = "<<cellIndex<<std::endl;
      }
      if( blockedAreaIndex[cellIndex]) 
      if( std::abs(t)<=LIDAR_SAFE_ZONE_MIN_R_NUM ){
      	advisedValue = 0;
        //std::cout<<"[LidarCheck] set zero"<<std::endl;
      	//return advisedValue;
      }
      else{
      	advisedValue = std::min( advisedValue, (t-LIDAR_SAFE_ZONE_MIN_R_NUM)/(LIDAR_SAFE_ZONE_MAX_R_NUM-LIDAR_SAFE_ZONE_MIN_R_NUM) );
      	advisedValue *= 1;
      }
    }
  }

  curve.pointList.clear();
  pointNum = curve.pointList.size();
  std::cout<<"[VC] pointnum cleared = "<<pointNum<<'\n';

  return advisedValue; 
}

double curvesLidarCheck (Curve& curve, std::vector<uint32_t>& blockedAreaIndex, double previewDistance, double targetSpeed, double judgeSpeed, bool isVirtual){
  bool b_occupied = false;
  double advisedSpeed = judgeSpeed;
  
//  const uint16_t xIndexMax = (X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION - 1;
//  const uint16_t yIndexMax = (Y_SIZE_MAX - Y_SIZE_MIN)/GRID_Y_RESOLUTION - 1;
  const uint32_t indexMax = (X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION * (Y_SIZE_MAX - Y_SIZE_MIN)/GRID_Y_RESOLUTION - 1;
  
  //stupid 80000
  bool b_obstacles[indexMax+1] = {false};
  std::cout << "Obstacle Size: " << blockedAreaIndex.size()<< std::endl;
  if (blockedAreaIndex.size() > 0) {
    for (int i = 0; i < blockedAreaIndex.size(); i++) {
      if (blockedAreaIndex[i] < indexMax + 1) {
        b_obstacles[blockedAreaIndex[i]] = true;
        //Test
        // std::cout << "ObstacleIndex " << blockedAreaIndex[i] << std::endl;
      }
      else;
    }
  }
  else;
  
  
  if (blockedAreaIndex.size() == 0) {
    b_occupied = false;
    advisedSpeed = judgeSpeed;
  }
  
  else{
    std::cout << "Checking If Curve Points Occupied. " << std::endl;
    //Display Lidar
    std::vector<int32_t> IndexLateralList;
    IndexLateralList.reserve(10000);
    std::vector<int32_t> IndexVerticalList;
    IndexVerticalList.reserve(10000);
    std::vector<int32_t> IndexCurveList;
    IndexVerticalList.reserve(10000);
    int32_t blockedAreaIndexNum = blockedAreaIndex.size();
    std::cout << "blockedAreaIndexNum "<< blockedAreaIndexNum << std::endl;

    for (uint32_t i = 0; i < CURVE_POINT_NUM; i++) {
      // std::cout << i << std::endl;
      double currentAdvisedSpeed = curvePointOccupied(curve.points[i], b_obstacles, previewDistance, IndexLateralList, IndexVerticalList, IndexCurveList, judgeSpeed, isVirtual); //Display Lidar
      if (currentAdvisedSpeed < judgeSpeed) {
        advisedSpeed = advisedSpeed > currentAdvisedSpeed? currentAdvisedSpeed: advisedSpeed;
      }
      else;
      
      if (advisedSpeed < 0.01)
      {
        advisedSpeed = 0.0;
        break;
      }else;
//      std::cout << "X: " <<curve.points[i].x << ",Y: "<<curve.points[i].y << std::endl;
//      for (uint32_t j = 0; j < blockedAreaIndex.size(); j++) {
//        if (gridOccupied (curve.points[i], blockedAreaIndex[j])) {
//          b_occupied = true;
//        }
//        else;
//        
//        if (true == b_occupied) {
//          break;
//        }
//        else;
//      }
      
//      if (true == b_occupied) {
//        break;
//      }
//      else;
    }

    //Display Lidar
    lidarJudgeDisplay(b_obstacles,IndexLateralList,IndexVerticalList,IndexCurveList);
    IndexLateralList.clear();
    IndexVerticalList.clear();
    IndexCurveList.clear();

  }
//  return b_occupied;
  return advisedSpeed;
}

//Display Lidar
void lidarJudgeDisplay(bool* b_obstacles,std::vector<int32_t>& IndexLateralList,std::vector<int32_t>& IndexVerticalList,std::vector<int32_t>& IndexCurveList){
  int32_t indexExpandSizeMin = static_cast<uint16_t>(LIDAR_SAFE_ZONE_MIN/2/GRID_X_RESOLUTION);
  int32_t indexExpandSizeMax = static_cast<uint16_t>(LIDAR_SAFE_ZONE_MAX/2/GRID_X_RESOLUTION);
  int32_t indexDistanceSizeMin = static_cast<uint16_t>(LIDAR_SAFE_DISTANCE_MIN/GRID_Y_RESOLUTION);
  int32_t indexDistanceSizeMax = static_cast<uint16_t>(LIDAR_SAFE_DISTANCE_MAX/GRID_Y_RESOLUTION/2);
  const uint16_t xIndexMax = (X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION - 1;
  const uint16_t yIndexMax = (Y_SIZE_MAX - Y_SIZE_MIN)/GRID_Y_RESOLUTION - 1;
  const uint32_t indexMax = (X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION * (Y_SIZE_MAX - Y_SIZE_MIN)/GRID_Y_RESOLUTION - 1;
  int32_t idx,index;
  int32_t i0,i,j,x_temp,y_temp;
  //std::cout << "xIndexMax " << xIndexMax << std::endl;
  //std::cout << "yIndexMax " << yIndexMax << std::endl;
  // std::cout << "IndexCurveList SIZE "<< IndexCurveList.size() << std::endl;
  //Sort
  int32_t temp_curve = 0;
  for(i = 1;i < IndexCurveList.size();i++){
    for(j = i;j < IndexCurveList.size();j++)
      if(IndexCurveList[j-1]>IndexCurveList[j]){
        temp_curve = IndexCurveList[j-1];
        IndexCurveList[j-1] = IndexCurveList[j];
        IndexCurveList[j] = temp_curve;
      }
  }
  //delete multiple
  std::vector<int32_t> IndexCurveList1;
  IndexCurveList1.reserve(1000);
  i = 0;
  while(i < IndexCurveList.size()){
    IndexCurveList1.push_back(IndexCurveList[i]);
    //std::cout<<i<<std::endl;
    if(i == IndexCurveList.size()-1) break;
    i++;
    while((IndexCurveList[i]==IndexCurveList[i-1])){
      i++;
      if(i == IndexCurveList.size()-1) break;
    }
  }
  //generate curve points
  std::vector<int32_t> IndexCurvePoints;
  IndexCurvePoints.reserve(1000);
  for(i = 0;i < IndexCurveList1.size();i++){
    index = IndexCurveList1[i];
    for(j = index-indexExpandSizeMin;j <= index+indexExpandSizeMin;j++){
      IndexCurvePoints.push_back(j);
    }
  }
  std::vector<int32_t> IndexCurvePoints1;
  IndexCurvePoints1.reserve(1000);
  i = 0;
  int32_t multiple_flag = 0;
  IndexCurvePoints1.push_back(IndexCurvePoints[i]);
  while(i < IndexCurvePoints.size()){
    i++;
    //std::cout<<i<<std::endl;
    for(j = 0;j < IndexCurvePoints1.size();j++){
      if(IndexCurvePoints[i] == IndexCurvePoints1[j]) multiple_flag = 1;
    }
    if(multiple_flag == 0) IndexCurvePoints1.push_back(IndexCurvePoints[i]);
    multiple_flag = 0;
  }
  // std::cout<< "IndexCurvePoints ";
  // for(i = 0;i < IndexCurvePoints1.size();i++)
  //   std::cout<< IndexCurvePoints1[i] <<" ";
  // std::cout<<std::endl;
  //Display Lidar
  int32_t move_dist = -80;
  int32_t toggle_flag = DISPLAY_MAP_ON;
  double DispDist = LIDAR_SAFE_DISTANCE_MIN;
/*  char sIn = getchar();
  if(sIn == '+'){
    toggle_flag = 1;
  }
  else if(sIn == '-'){
    toggle_flag = 0;
  }*/
  if(toggle_flag == 2){
    int32_t flag_curve = 0;
    for(j = indexDistanceSizeMax;j >= 0;j--){
      std::cout <<"|";
      for(i = 0;i <= xIndexMax;i++){
        idx = j * xIndexMax + i + j;
        // idx = j * xIndexMax + i;
        for(i0 = 0;i0 < IndexCurvePoints1.size();i0++){
          index = IndexCurvePoints1[i0];
          if(idx == index){
            if((true == b_obstacles[idx])&&(i>(X_SIZE_MAX-DispDist)/GRID_X_RESOLUTION)&&(i<(X_SIZE_MAX+DispDist)/GRID_X_RESOLUTION)) std::cout << "+";
            else if((i>(X_SIZE_MAX-DispDist)/GRID_X_RESOLUTION)&&(i<(X_SIZE_MAX+DispDist)/GRID_X_RESOLUTION)) std::cout << "0";
            flag_curve = 1;
            break;
          }
        }
        if((flag_curve == 0)&&(true == b_obstacles[idx])&&(i>(X_SIZE_MAX-DispDist)/GRID_X_RESOLUTION)&&(i<(X_SIZE_MAX+DispDist)/GRID_X_RESOLUTION)) std::cout << "*";
        else if((flag_curve == 0)&&(i>(X_SIZE_MAX-DispDist)/GRID_X_RESOLUTION)&&(i<(X_SIZE_MAX+DispDist)/GRID_X_RESOLUTION)) std::cout << "o";
        flag_curve = 0;
      }
      //std::cout << i << " "<< j << " " << i0 << " "<< y_temp;
      std::cout <<"|"<< std::endl;
      move_dist--;
    }
  }
  //Display Lidar
  if((toggle_flag == 1)||(toggle_flag == 2)){
    std::cout<< CYAN <<"IndexLateralList " << RESET;
    for(i = 0;i < IndexLateralList.size();i++)
      std::cout<< IndexLateralList[i] <<" ";
    std::cout<<std::endl;
    std::cout<< CYAN <<"IndexVerticalList "<< RESET;
    for(i = 0;i < IndexVerticalList.size();i++)
      std::cout<< IndexVerticalList[i] <<" ";
    std::cout<<std::endl;
/*    std::cout<< "IndexCurveList ";
    for(i = 0;i < IndexCurveList1.size();i++)
      std::cout<< IndexCurveList1[i] <<" ";
    std::cout<<std::endl;*/
  }
}

/*
int32_t curveSelection (std::vector<Curve>& curves, LidarData lidarData, uint32_t currentRoadIrome){
//  uint32_t curvesSize = static_cast<uint32_t>(curves.size());
  bool b_curvePassable[CURVE_NUM] = {false};
  for (uint32_t i = 0; i < CURVE_NUM; i++) {
    if (false == curvesLidarCheck(curves[i], lidarData.blockedAreaIndex)) {
      b_curvePassable[i] = false;
    }
    else{
      b_curvePassable[i] = true;
    }
  }
  
  int32_t selectedCurve = 0;
  for (int32_t i = 0; i <static_cast<int>(CURVE_NUM/2.0); i++) {
    if (true == b_curvePassable[i]) {
      selectedCurve = i;
    }
    else;
  }
  return selectedCurve;
}
*/

// velocity
void velocityDecision(const LidarData& lidarData, bool b_stopFromUI, const PostureData& postureData, DecisionData& decisionData){

  std::cout << "yTop: " << lidarData.zone.yTop << std::endl;
  std::cout << "yTopWider: " << lidarData.zone.yTopWider << std::endl;
  std::cout << "yTopEvenWider: " << lidarData.zone.yTopEvenWider << std::endl;
  std::cout << "Current ID: " << decisionData.currentId << std::endl;
  static double lastTargetSpeed;
  if (decisionData.currentState == STATE_FOLLOW_ROADMAP) {
    std::cout << "Speed Roadmap." << std::endl;
    if (static_cast<double>(postureData.imuData.velocity)/3.6 > SPEED_MID) {
      std::cout << "speed > mid. " << std::endl;
      if (lidarData.zone.yTop > OBSTACLE_DIS_MAX &&
          lidarData.zone.yTopWider > OBSTACLE_DIS_MIDMAX &&
          lidarData.zone.yTopEvenWider > OBSTACLE_DIS_MID
          ){
        decisionData.targetSpeed = SPEED_HIGH;
        decisionData.targetAccLevel = ACC_MID;
        std::cout << "high speed" <<std::endl;
      }
      else if ((lidarData.zone.yTop > OBSTACLE_DIS_MIDMAX && lidarData.zone.yTop <= OBSTACLE_DIS_MAX) &&
          lidarData.zone.yTopWider > OBSTACLE_DIS_MID &&
          lidarData.zone.yTopEvenWider > OBSTACLE_DIS_MIN
          ){
        decisionData.targetSpeed = SPEED_MIDHIGH;
        decisionData.targetAccLevel = ACC_MID;
        std::cout << "mid high speed" <<std::endl;
      }
      else if ( (lidarData.zone.yTop > OBSTACLE_DIS_MID && lidarData.zone.yTop <= OBSTACLE_DIS_MIDMAX) &&
               (lidarData.zone.yTopWider > OBSTACLE_DIS_MIN ) &&
               (lidarData.zone.yTopEvenWider > OBSTACLE_DIS_MINIMAL)
               ){
        decisionData.targetSpeed = SPEED_MID;
        decisionData.targetAccLevel = ACC_MID;
        std::cout << "mid speed" <<std::endl;
      }
      else if ( (lidarData.zone.yTop > OBSTACLE_DIS_MIN && lidarData.zone.yTop <= OBSTACLE_DIS_MID) &&
               lidarData.zone.yTopWider > OBSTACLE_DIS_MINIMAL
               ){
        decisionData.targetSpeed = SPEED_LOW;
        decisionData.targetAccLevel = ACC_MID;
        std::cout << "low speed" <<std::endl;
      }
      else {
        //tbd
        decisionData.targetSpeed = SPEED_ZERO;
        decisionData.targetAccLevel = ACC_MID;
        std::cout << "0 speed" <<std::endl;
      }
    }
    else{
      std::cout << "speed < mid. " << std::endl;
      if (lidarData.zone.yTop > OBSTACLE_DIS_MID &&
          lidarData.zone.yTopWider > OBSTACLE_DIS_MIN
          ){
        decisionData.targetSpeed = SPEED_HIGH;
        decisionData.targetAccLevel = ACC_MID;
        std::cout << "high speed" <<std::endl;
      }
      else if ( (lidarData.zone.yTop > OBSTACLE_DIS_MIN) &&
               (lidarData.zone.yTopWider > OBSTACLE_DIS_MINIMAL )
               ){
        decisionData.targetSpeed = SPEED_MID;
        decisionData.targetAccLevel = ACC_MID;
        std::cout << "mid speed" <<std::endl;
      }
      //modified.
      else if ( (lidarData.zone.yTop > OBSTACLE_DIS_MINIMAL)
               ){
        decisionData.targetSpeed = SPEED_LOW;
        decisionData.targetAccLevel = ACC_MID;
        std::cout << "low speed" <<std::endl;
      }
      else {
        //tbd
        decisionData.targetSpeed = SPEED_ZERO;
        decisionData.targetAccLevel = ACC_MID;
        std::cout << "0 speed" <<std::endl;
      }
    }
    
    //steering
    if (std::abs(decisionData.targetSteeringAngle) > 20 && decisionData.targetSpeed >= SPEED_MID) {
      decisionData.targetSpeed = SPEED_MID;
      std::cout << "mid speed because of steering." <<std::endl;
    }
    else;
    if (std::abs(decisionData.targetSteeringAngle) > 30 && decisionData.targetSpeed >= SPEED_LOW){
      decisionData.targetSpeed = SPEED_LOW;
      std::cout << "low speed because of steering." <<std::endl;
    }
    else;
    
    //terminating
    if (decisionData.currentId >= 63 && decisionData.targetSpeed >= SPEED_MIDHIGH) {
      decisionData.targetSpeed = SPEED_MIDHIGH;
    }
    else;
    if (decisionData.currentId >= 64 && decisionData.targetSpeed >= SPEED_MID) {
      decisionData.targetSpeed = SPEED_MID;
    }
    else;
    if (decisionData.currentId >= 65 && decisionData.targetSpeed >= SPEED_LOW) {
      decisionData.targetSpeed = SPEED_LOW;
    }
    else;
    if (lastTargetSpeed >= decisionData.targetSpeed && decisionData.targetSpeed >= SPEED_MID) {
      decisionData.targetAccLevel = 0x02;
    }
    else;
  }
  
  else if (decisionData.currentState == STATE_INITIAL){
    std::cout << "Speed Initial." << std::endl;
    if (lidarData.zone.yTop > OBSTACLE_DIS_MAX &&
        lidarData.zone.yTopWider > OBSTACLE_DIS_MID &&
        lidarData.zone.yTopEvenWider > OBSTACLE_DIS_MIN
        ){
//      decisionData.targetSpeed = SPEED_HIGH;
      decisionData.targetSpeed = SPEED_MID;
      decisionData.targetAccLevel = ACC_MID;
      std::cout << "mid speed" <<std::endl;
    }
    else if ( (lidarData.zone.yTop > OBSTACLE_DIS_MID && lidarData.zone.yTop <= OBSTACLE_DIS_MAX) ||
             (lidarData.zone.yTopWider > OBSTACLE_DIS_MIN && lidarData.zone.yTopWider <= OBSTACLE_DIS_MID) ||
             lidarData.zone.yTopEvenWider <= OBSTACLE_DIS_MIN
             ){
      decisionData.targetSpeed = SPEED_MID;
      decisionData.targetAccLevel = ACC_MID;
      std::cout << "mid speed" <<std::endl;
    }
    else if ( (lidarData.zone.yTop > OBSTACLE_DIS_MIN && lidarData.zone.yTop <= OBSTACLE_DIS_MID) ||
             lidarData.zone.yTopWider <= OBSTACLE_DIS_MIN
             ){
      decisionData.targetSpeed = SPEED_LOW;
      decisionData.targetAccLevel = ACC_MID;
      std::cout << "low speed" <<std::endl;
    }
    else {
      //tbd
      decisionData.targetSpeed = SPEED_ZERO;
      decisionData.targetAccLevel = ACC_MID;
      std::cout << "0 speed" <<std::endl;
    }
  }
  
  else if (decisionData.currentState == STATE_PARKING){
    std::cout << "Speed Parking." << std::endl;
    if ( lidarData.zone.yTop > OBSTACLE_DIS_MIN){
      decisionData.targetSpeed = SPEED_LOW;
      decisionData.targetAccLevel = ACC_MID;
      std::cout << "low speed" <<std::endl;
    }
    else {
      //tbd
      decisionData.targetSpeed = SPEED_ZERO;
      decisionData.targetAccLevel = ACC_MID;
      std::cout << "0 speed" <<std::endl;
    }
  }
  
  if (true == b_stopFromUI) {
    decisionData.targetSpeed = SPEED_ZERO;
    decisionData.targetAccLevel = ACC_LOW;
    std::cout << "Stopped because of UI. " << std::endl;
  }
  else{
    std::cout << "driving. " << std::endl;
  }
//  limitSpeed(decisionData, lastTargetSpeed);
  lastTargetSpeed = decisionData.targetSpeed;
}
// end of velocity

void trafficLightJudge(const PostureData& postureData, const CvData& cvData, const std::vector<RoadPoint>& stopPoints,  DecisionData& decisionData){
  bool intersectionFlagLarge = false;
  bool intersectionFlagSmall = false;
//  std::cout << " Stop Point Size: " << stopPoints.size() << std::endl;
//  std::cout << std::setprecision(10)  << stopPoints[1].longitude << ", " <<stopPoints[1].latitude << std::endl;
  static double lastTargetSpeed;
  // start from 1 for 0 is empty
  for (uint32_t i = 1; i < stopPoints.size(); i++) {
    if(1 == stopPoints[i].parameter1){

    // ting zai hou mian.
  //    bool checkFlagLarge = inArea(postureData.gpsData.longitude, postureData.gpsData.latitude, stopPoints[i].longitude - TRAFFIC_LIGHT_DISTANCE_LARGE, stopPoints[i].latitude, TRAFFIC_LIGHT_DISTANCE_LARGE);
      bool checkFlagLarge = inArea(postureData.gpsData.longitude, postureData.gpsData.latitude, stopPoints[i].longitude, stopPoints[i].latitude, TRAFFIC_LIGHT_DISTANCE_LARGE);
//      printf("[CV] check flag %d\n",checkFlagLarge);
      //tingzai houmian
  //    bool checkFlagSmall = inArea(postureData.gpsData.longitude, postureData.gpsData.latitude, stopPoints[i].longitude - TRAFFIC_LIGHT_DISTANCE_SMALL, stopPoints[i].latitude, TRAFFIC_LIGHT_DISTANCE_SMALL);
      bool checkFlagSmall = inArea(postureData.gpsData.longitude, postureData.gpsData.latitude, stopPoints[i].longitude, stopPoints[i].latitude, TRAFFIC_LIGHT_DISTANCE_SMALL);
      if ( true == checkFlagLarge) {
        intersectionFlagLarge = true;
        std::cout << "[CV] Close to intersection. Index:  " << i << "." << std::endl;
      }else;
      
      if (true == checkFlagSmall) {
        intersectionFlagSmall = true;
        std::cout << "[CV] In intersection. Index:  " << i << "." << std::endl;
      }else;
    }
  }
  if (cvData.b_isAlive == false){
    std::cout << "[CV] cv dead, no traffic light info!" << std::endl;
  }
  if (cvData.b_isAlive == true && cvData.lightPhase == 0 && intersectionFlagSmall == true && decisionData.targetSpeed > SPEED_ZERO) {
    decisionData.targetSpeed = SPEED_ZERO;
    std::cout << "[CV] Stopped because of traffic light. " << std::endl;
//    limitSpeed(decisionData, lastTargetSpeed);
    lastTargetSpeed = decisionData.targetSpeed;
  }
  else if (cvData.lightPhase == 0 && intersectionFlagLarge == true && decisionData.targetSpeed > SPEED_LOW){
    decisionData.targetSpeed = SPEED_LOW;
    std::cout << "[CV] Low speed because of traffic light. " << std::endl;
//    limitSpeed(decisionData, lastTargetSpeed);
    lastTargetSpeed = decisionData.targetSpeed;
  }
  else;
  
}

void buildFinalRoadMap(std::vector<std::vector<RoadPoint>>& roadPointss, const std::vector<RoadPoint>& stopPoints){
  std::cout << "Building Final Roadmap." << std::endl;
  for (int i = 0; i < roadPointss.size(); i++) {
    for (int j = 0; j < roadPointss[i].size(); j++) {
      bool b_foundStopPoint = false;
      for (int k = 0; k < stopPoints.size(); k++){
        bool checkFlag = inArea(roadPointss[i][j].longitude, roadPointss[i][j].latitude, stopPoints[k].longitude, stopPoints[k].latitude, stopPoints[k].parameter2);
        if (true == checkFlag) {
          b_foundStopPoint = true;
//          roadPointss[i][j].parameter1 = stopPoints[k].parameter1;
          if (stopPoints[k].parameter1 == 2) {
            roadPointss[i][j].parameter3 = std::fmin(roadPointss[i][j].parameter3, stopPoints[k].parameter3);
            roadPointss[i][j].parameter1 = 2; //station stop
            // std::cout << "current ijk: "<< i <<", " << j<<", " <<k << std::endl;
            // std::cout << "RoadPoint Parameter: " <<  roadPointss[i][j].parameter3 << std::endl;
            // std::cout << "StopPoint Parameter: " <<  stopPoints[k].parameter3 << std::endl;
          }
          else;
          if (stopPoints[k].parameter1 == 3){
            roadPointss[i][j].parameter1 = 3;
//            std::cout << "current ijk: "<< i <<", " << j<<", " <<k << std::endl;
          }
        }
        else{
          if (false == b_foundStopPoint){
            roadPointss[i][j].parameter1 = 0;
          }
        }
      }
    }
  }
}

void stopPointJudge(const PostureData& postureData, const CvData& cvData, const std::vector<RoadPoint>& stopPoints,  DecisionData& decisionData){
  //  std::cout << "Stop Point Size: " << stopPoints.size() << std::endl;
  //  std::cout << std::setprecision(10)  << stopPoints[1].longitude << ", " <<stopPoints[1].latitude << std::endl;
//  static double lastTargetSpeed;
  // start from 1 for 0 is empty
  for (uint32_t i = 1; i < stopPoints.size(); i++) {
//    std::cout << stopPoints[i].parameter2 << std::endl;
    // ting zai hou mian.
    //    bool checkFlagLarge = inArea(postureData.gpsData.longitude, postureData.gpsData.latitude, stopPoints[i].longitude - TRAFFIC_LIGHT_DISTANCE_LARGE, stopPoints[i].latitude, TRAFFIC_LIGHT_DISTANCE_LARGE);
    bool checkFlag = inArea(postureData.gpsData.longitude, postureData.gpsData.latitude, stopPoints[i].longitude, stopPoints[i].latitude, stopPoints[i].parameter2);
    //tingzai houmian
    //    bool checkFlagSmall = inArea(postureData.gpsData.longitude, postureData.gpsData.latitude, stopPoints[i].longitude - TRAFFIC_LIGHT_DISTANCE_SMALL, stopPoints[i].latitude, TRAFFIC_LIGHT_DISTANCE_SMALL);
//    bool checkFlagSmall = inArea(postureData.gpsData.longitude, postureData.gpsData.latitude, stopPoints[i].longitude, stopPoints[i].latitude, TRAFFIC_LIGHT_DISTANCE_SMALL);
//    if ( true == checkFlagLarge) {
//      intersectionFlagLarge = true;
//      std::cout << "Close to intersection. Index:  " << i << "." << std::endl;
//    }else;
//    
//    if (true == checkFlagSmall) {
//      intersectionFlagSmall = true;
//      std::cout << "In intersection. Index:  " << i << "." << std::endl;
//    }else;
    
    
    decisionData.b_takeOverEnable = false;;
    if (checkFlag == true) {
      if (stopPoints[i].parameter1 == 1) {
        if (cvData.lightPhase == 0) {
          decisionData.targetSpeed = decisionData.targetSpeed > stopPoints[i].parameter3? stopPoints[i].parameter3 : decisionData.targetSpeed;
          std::cout << "Stopped because of traffic light index: " << i << std::endl;
        }else;
      }
      else if(stopPoints[i].parameter1 == 2){
//        std::cout << stopPoints[i].parameter3 << std::endl;
        decisionData.targetSpeed = decisionData.targetSpeed > stopPoints[i].parameter3? stopPoints[i].parameter3 : decisionData.targetSpeed;
        std::cout << "Speed limited because of stop point: " << i << std::endl;
      }
      else if(stopPoints[i].parameter1 == 3){
//        decisionData.targetSpeed = decisionData.targetSpeed > stopPoints[i].parameter3? stopPoints[i].parameter3 : decisionData.targetSpeed;
        decisionData.b_takeOverEnable = true;
        std::cout << "Take over enabled." << std::endl;
      }
      else;
    }
    else;
  }
//  if (iovData.phase == 0 && intersectionFlagSmall == true && decisionData.targetSpeed > SPEED_ZERO) {
//    decisionData.targetSpeed = SPEED_ZERO;
//    std::cout << "Stopped because of traffic light. " << std::endl;
//    //    limitSpeed(decisionData, lastTargetSpeed);
////    lastTargetSpeed = decisionData.targetSpeed;
//  }
//  else if (iovData.phase == 0 && intersectionFlagLarge == true && decisionData.targetSpeed > SPEED_LOW){
//    decisionData.targetSpeed = SPEED_LOW;
//    std::cout << "Low speed because of traffic light. " << std::endl;
//    //    limitSpeed(decisionData, lastTargetSpeed);
////    lastTargetSpeed = decisionData.targetSpeed;
//  }
//  else;
  
}

void mapJudge(DecisionData& decisionData, const CvData& cvData, const VisionData& visionData){
  std::cout << "[MJ] judging Map. " << std::endl;
  decisionData.b_takeOverEnable = false;
  decisionData.b_redLightStopped = false;

  if(ROADMAP_LIMIT_SWITCH == true)
    decisionData.targetSpeed = std::fmin(decisionData.targetSpeed, decisionData.currentPoint.parameter3);
  
  decisionData.stationStopFlag = 0; //station stop

  if (decisionData.currentPoint.parameter1 == 1) {
//    if (iovData.phase == 0 || (visionData.trafficLightData.leftPassable == false && visionData.trafficLightData.b_leftIsValid == true)) {
//    if (iovData.phase == 0 ) {
//    if (visionData.trafficLightData.leftPassable == 0 && visionData.trafficLightData.b_leftIsValid == true) {
//      decisionData.targetSpeed = SPEED_ZERO;
//      decisionData.b_redLightStopped = true;
//      std::cout << "Stopped because of traffic light. "  << std::endl;
//    }else;
    ;
  }
  else if(decisionData.currentPoint.parameter1 == 2){
    //        std::cout << stopPoints[i].parameter3 << std::endl;
    std::cout << "[MJ] Speed limited because of stop point. "<< std::endl;
    decisionData.stationStopFlag = 1; //station stop
  }
  else if(decisionData.currentPoint.parameter1 == 3){
    decisionData.b_takeOverEnable = true;
    std::cout << "[MJ] Overtaking  enabled." << std::endl;
  }
  else;
}


void terminalPointJudge(const PostureData& postureData, const RoadPoint& terminalPoint, DecisionData& decisionData){
  bool terminalFlagLarge = false;
  static bool terminalFlagSmall = false;
  static double lastTargetSpeed;
  if (inArea(postureData.gpsData.longitude, postureData.gpsData.latitude, terminalPoint.longitude, terminalPoint.latitude, TERMINAL_DISTANCE_LARGE)) {
    terminalFlagLarge = true;
    std::cout << "In large terminal area. " << std::endl;
  }else;
  
  if (inArea(postureData.gpsData.longitude, postureData.gpsData.latitude, terminalPoint.longitude, terminalPoint.latitude, TERMINAL_DISTANCE_SMALL)) {
    terminalFlagSmall = true;
    std::cout << "In small terminal area. " << std::endl;
  }else;
  
  if (true == terminalFlagSmall && decisionData.targetSpeed > SPEED_ZERO) {
    decisionData.targetSpeed = SPEED_ZERO;
    std::cout << "Stopped because of terminal." << std::endl;
//    limitSpeed(decisionData, lastTargetSpeed);
    lastTargetSpeed = decisionData.targetSpeed;
  }
  else if (true == terminalFlagLarge && decisionData.targetSpeed > SPEED_LOW){
    decisionData.targetSpeed = SPEED_LOW;
    std::cout << "low speed because of close to terminal." << std::endl;
//    limitSpeed(decisionData, lastTargetSpeed);
    lastTargetSpeed = decisionData.targetSpeed;
  }
  else;
}

//void stopPointJudge(const PostureData& postureData, const RoadPoint& terminalPoint, DecisionData& decisionData){
//  bool terminalFlagLarge = false;
////  static bool terminalFlagSmall = false;
//  bool terminalFlagSmall = false;
//  static double lastTargetSpeed;
//  if (inArea(postureData.gpsData.longitude, postureData.gpsData.latitude, terminalPoint.longitude, terminalPoint.latitude, TERMINAL_DISTANCE_LARGE)) {
//    terminalFlagLarge = true;
//    std::cout << "In large terminal area. " << std::endl;
//  }else;
//  
//  if (inArea(postureData.gpsData.longitude, postureData.gpsData.latitude, terminalPoint.longitude, terminalPoint.latitude, TERMINAL_DISTANCE_SMALL)) {
//    terminalFlagSmall = true;
//    std::cout << "In small terminal area. " << std::endl;
//  }else;
//  
//  if (true == terminalFlagSmall && decisionData.targetSpeed > SPEED_ZERO) {
//    decisionData.targetSpeed = SPEED_ZERO;
//    std::cout << "Stopped because of terminal." << std::endl;
////    limitSpeed(decisionData, lastTargetSpeed);
//    lastTargetSpeed = decisionData.targetSpeed;
//  }
//  else if (true == terminalFlagLarge && decisionData.targetSpeed > SPEED_LOW){
//    decisionData.targetSpeed = SPEED_LOW;
//    std::cout << "low speed because of close to terminal." << std::endl;
////    limitSpeed(decisionData, lastTargetSpeed);
//    lastTargetSpeed = decisionData.targetSpeed;
//  }
//  else;
//}

Point pointOnCubicBezier (std::vector<Point> cp, double t){
  /*
   cp在此是四個元素的陣列:
   cp[0]為起始點，或上圖中的P0
   cp[1]為第一個控制點，或上圖中的P1
   cp[2]為第二個控制點，或上圖中的P2
   cp[3]為結束點，或上圖中的P3
   t為參數值，0 <= t <= 1
   */
  
  double   ax, bx, cx;
  double   ay, by, cy;
  double   tSquared, tCubed;
  Point result;
  
  /*計算多項式係數*/
  
  cx = 3.0 * (cp[1].x - cp[0].x);
  bx = 3.0 * (cp[2].x - cp[1].x) - cx;
  ax = cp[3].x - cp[0].x - cx - bx;
  
  cy = 3.0 * (cp[1].y - cp[0].y);
  by = 3.0 * (cp[2].y - cp[1].y) - cy;
  ay = cp[3].y - cp[0].y - cy - by;
  
  /*計算位於參數值t的曲線點*/
  
  tSquared = t * t;
  tCubed = tSquared * t;
  
  result.x = (ax * tCubed) + (bx * tSquared) + (cx * t) + cp[0].x;
  result.y = (ay * tCubed) + (by * tSquared) + (cy * t) + cp[0].y;
  
  return result;
}

void generateCurve (const RoadPoint& startPoint, const RoadPoint& endPoint, Curve& curve){
  
//  std::cout << "endpoint!" << endPointList[0].longitude << ", " << endPointList[0].latitude<< std::endl;
  double dx = endPoint.longitude - startPoint.longitude;
  double dy = endPoint.latitude - startPoint.latitude;
//  double theta = startPoint.courseAngle;
  
//  std::cout << "endpoint!" << dx << ", " << dy << std::endl;
  Point controlPoint[4] = { 0 };
  
  controlPoint[0].x = 0.0;
  controlPoint[0].y = 0.0;
  controlPoint[0].angle = 0.0;
  
//  controlPoint[3].x = dx * std::cos(startPoint.courseAngle) + dy * std::sin(startPoint.courseAngle);
//  controlPoint[3].y = -dx * std::sin(startPoint.courseAngle) + dy * std::cos(startPoint.courseAngle); 
  controlPoint[3].y = dy * std::cos(startPoint.courseAngle/180*M_PI) + dx * std::sin(startPoint.courseAngle/180*M_PI);
  controlPoint[3].x = -dy * std::sin(startPoint.courseAngle/180*M_PI) + dx * std::cos(startPoint.courseAngle/180*M_PI);
//  controlPoint[3].angle = static_cast<int32_t>(360 + endPoint.courseAngle - startPoint.courseAngle) % 360;
  controlPoint[3].angle = static_cast<int32_t>(360 + endPoint.courseAngle) % 360;
  
  controlPoint[1].x = 0.0;
  controlPoint[1].y = CP_DISTANCE_BACK;
  controlPoint[1].angle = 0;
  
  controlPoint[2].x = controlPoint[3].x - CP_DISTANCE_FRONT * std::sin(controlPoint[3].angle/180*M_PI);
  controlPoint[2].y = controlPoint[3].y - CP_DISTANCE_FRONT * std::cos(controlPoint[3].angle/180*M_PI);
  controlPoint[2].angle = controlPoint[3].angle;
  
  std::vector<Point> controlPointList;
  controlPointList.clear();
  for (uint32_t i = 0; i < 4; i++) {
    controlPointList.push_back(controlPoint[i]);
  }
  
//  RoadPoint controlPoint1, controlPoint2;
//  
//  controlPoint1.longitude = startPoint.longitude + CP_DISTANCE * std::sin(startPoint.courseAngle/180*M_PI);
//  controlPoint1.latitude = startPoint.latitude + CP_DISTANCE * std::cos(startPoint.courseAngle/180*M_PI);
//  controlPoint1.courseAngle = startPoint.courseAngle;
//  
//  controlPoint2.longitude = endPoint.longitude - CP_DISTANCE * std::sin(endPoint.courseAngle/180*M_PI);
//  controlPoint2.latitude = endPoint.latitude - CP_DISTANCE * std::cos(endPoint.courseAngle/180*M_PI);
//  controlPoint2.courseAngle = endPoint.courseAngle;
//  
//  std::vector<RoadPoint> controlPointList;
//  controlPointList.clear();
//  controlPointList.push_back(startPoint);
//  controlPointList.push_back(controlPoint1);
//  controlPointList.push_back(controlPoint2);
//  controlPointList.push_back(endPoint);
  
//  curve.clear();
  double dt = 1.0/(CURVE_POINT_NUM - 1);
  for (uint32_t i = 0; i < CURVE_POINT_NUM; i++) {
    curve.points[i] = pointOnCubicBezier(controlPointList, i * dt);
  }
  
  //test

//  std::ofstream fs("testData.txt", std::ofstream::app);
  std::ofstream fs("curveData.txt", std::ofstream::trunc);
//  fs << std::setprecision(15) << startPoint.longitude << ", " << startPoint.latitude << ", " << startPoint.courseAngle << std::endl;
//  std::cout << endPoint.longitude << ", " << endPoint.latitude << ", " << endPoint.courseAngle << std::endl;
//  fs << std::setprecision(15) << endPoint.longitude << ", " << endPoint.latitude << ", " << endPoint.courseAngle << std::endl;
//  fs << std::setprecision(15) << "saveing data" << std::endl;

//    fs << std::setprecision(15) << LIDAR_SAFE_ZONE_MIN << " " << LIDAR_SAFE_ZONE_MAX << std::endl;
//    fs << std::setprecision(15) << LIDAR_SAFE_DISTANCE_MIN << " " << LIDAR_SAFE_DISTANCE_MAX << std::endl;
  for (uint32_t i = 0; i < 4; i++) {
    fs << std::setprecision(15) << controlPointList[i].x << " " << controlPointList[i].y << std::endl;
  }
  
//  std::cout << "ControlPoint!" << controlPointList[3].x << ", " << controlPointList[3].y << std::endl;
  for (uint32_t i = 0; i < CURVE_POINT_NUM; i++) {
    fs << std::setprecision(15) << curve.points[i].x << " " << curve.points[i].y << std::endl;
//    std::cout << curve.points[i].x << " " << curve.points[i].y << std::endl;
  }
  fs.close();
  

}

void generateCurveList (const RoadPoint& startPoint, const std::vector<RoadPoint>& endPointList, std::vector<Curve>& curveList){
  curveList.reserve(CURVE_NUM);
//  std::vector<Curve> curveList2;
//  curveList2.reserve(1);
  Curve curve;
  for (int i = 0; i < CURVE_NUM; i++) {
    generateCurve(startPoint, endPointList[i], curve);
    curveList.push_back(curve);
//    curveList2.push_back(curve);
//    for (int j = 0 ; j< CURVE_POINT_NUM; j++) {
//      std::cout << "x: " << curve.points[j].x << ",y: " << curve.points[j].y << std::endl;
//      std::cout << "x: " << curveList[0].points[j].x << ",y: " << curveList[0].points[j].y << std::endl;
//    }
  }
}

void generateCurvePreviewPoint(DecisionData& decisionData, const PostureData& postureData){
  RoadPoint startPoint, endPoint;
  startPoint.longitude = 0.0;
  startPoint.latitude = 0.0;
  startPoint.courseAngle = 0.0;
  // global coordinate
  double tempX = decisionData.previewPoint.longitude - postureData.gpsData.longitude;
  double tempY = decisionData.previewPoint.latitude - postureData.gpsData.latitude;
  // rotate to local coordinate
  endPoint.longitude = tempX * cos(postureData.imuData.yaw/180*M_PI) - tempY * sin(postureData.imuData.yaw/180*M_PI);
  endPoint.latitude = tempY * cos(postureData.imuData.yaw/180*M_PI) + tempX * sin(postureData.imuData.yaw/180*M_PI);
  endPoint.courseAngle = decisionData.previewPoint.courseAngle - postureData.imuData.yaw;
  if (endPoint.courseAngle < 0) {
    endPoint.courseAngle += 360;
  }
  else if (endPoint.courseAngle > 360){
    endPoint.courseAngle -= 360;
  }
  else;
  
  double theta = endPoint.courseAngle + 90;
  if (theta < 0) {
    theta += 360;
  }
  else if (theta > 360){
    theta -= 360;
  }
  else;
  
  std::vector<RoadPoint> endPointList;
  endPointList.clear();
  endPointList.reserve(CURVE_NUM);
  for (int i = 0; i < CURVE_NUM; i++) {
    if (i % 2 == 0) {
      endPointList[i].longitude = endPoint.longitude + (i/2)*CURVE_DISTANCE*sin(theta/180*M_PI);
      endPointList[i].latitude = endPoint.latitude + (i/2)*CURVE_DISTANCE*cos(theta/180*M_PI);
      endPointList[i].courseAngle = endPoint.courseAngle;
    }
    else{
      endPointList[i].longitude = endPoint.longitude - ((i+1)/2)*CURVE_DISTANCE*sin(theta/180*M_PI);
      endPointList[i].latitude = endPoint.latitude - ((i+1)/2)*CURVE_DISTANCE*cos(theta/180*M_PI);
      endPointList[i].courseAngle = endPoint.courseAngle;
    }
    endPointList.push_back(endPointList[i]);
  }
//  std::cout << "endpoint!" << endPointList[0].longitude << ", " << endPointList[0].latitude<< std::endl;
  
  
  
  std::cout << std::setprecision(1)<< "PreviewPoint: " << decisionData.previewPoint.longitude << ", " << decisionData.previewPoint.latitude << std::endl;
  std::cout << std::setprecision(1)<< "CurrentPoint: " << decisionData.currentPoint.longitude << ", " << decisionData.currentPoint.latitude << std::endl;
  std::cout << std::setprecision(1)<< "Posture Data: " << postureData.gpsData.longitude << ", " << postureData.gpsData.latitude << std::endl;

//  std::cout << tempX << ", " << tempY << ", " << postureData.imuData.yaw << std::endl;
//  std::cout << endPoint.longitude << ", " << endPoint.latitude << std::endl;
  std::ofstream fs("locationData.txt", std::ofstream::trunc);
  fs << std::setprecision(15) << postureData.gpsData.longitude << " " << postureData.gpsData.latitude << " " << postureData.imuData.yaw << std::endl;
  fs.close();
  
  decisionData.curveList.clear();
  generateCurveList(startPoint, endPointList, decisionData.curveList);
  
//  std::cout << "x: " << decisionData.curveList.
  
}

void clearCurveList(std::vector<Curve>& curveList){
  curveList.clear();
}

//void generateCandidates(RoadPoint previewPoint, uint16_t leftCandidateNumbers, uint16_t rightCandidateNumbers, double candidateDistance){
//  
//}

double cruiseController(const PostureData& postureData, const IovData& iovData, const ActuatorData actuatorData){
  double distance = std::sqrt((postureData.gpsData.longitude-iovData.vehicleGaussX)*(postureData.gpsData.longitude-iovData.vehicleGaussX)+ 
  		(postureData.gpsData.latitude-iovData.vehicleGaussY)*(postureData.gpsData.latitude-iovData.vehicleGaussY)) - 3.5;
  double distanceDiff = distance - ACC_STANDARD_DISTANCE;
  double speedDiff = iovData.vehicleSpeed - postureData.imuData.velocity;
  //double speedDiff = iovData.vehicleSpeed - actuatorData.vehicleSpeed;
  double kDistance = 1, kSpeed = 0.5; // old 1 0.5 
  static bool b_hasStopped = false;

  double targetSpeed = kDistance * distanceDiff + kSpeed * speedDiff + iovData.vehicleSpeed; 

  std::cout << "[CV] inter dist.: " << distance << std::endl;
  std::cout << "[CV] dist. error: " << distanceDiff << std::endl;
  std::cout << "[CV] speed error: " << speedDiff << std::endl;
  std::cout << "[CV] 1st  target: " << targetSpeed << std::endl;
  if (targetSpeed < 0.5){
    targetSpeed = 0;
    b_hasStopped = true;
  }
  else if (targetSpeed >= 0.5 && targetSpeed < 1.8){
    if(true == b_hasStopped){
      targetSpeed = 0.0;
    }
    else{
//  	  targetSpeed = 1.0;
    }
  }
  else{
    b_hasStopped = false;
  }
  if ( targetSpeed > 40/3.6 ){
    targetSpeed = 40/3.6;
    std::cout << "! Top Cruise Speed Limited" << std::endl;
  }
  if (distance < ACC_MINIMAL_DISTANCE){
    targetSpeed = 0;
    b_hasStopped = true;
  }else;
  std::cout << "[CV] targetSpeed = " << targetSpeed << " m/s\n";
  return targetSpeed;
}

void lidarJudge(DecisionData& decisionData, PostureData& postureData, LidarData& lidarData){
   double advisedSpeedTakeOver = LIDAR_MAX_SPEED;
   double judgeSpeed = LIDAR_MAX_SPEED;
//  double advisedSpeedComeBack = LIDAR_MAX_SPEED;
  generateCurvePreviewPoint(decisionData, postureData);
  double advisedSpeed = curvesLidarCheck(decisionData.curveList[0], lidarData.blockedAreaIndex, decisionData.previewDistance+5, decisionData.targetSpeed, judgeSpeed);
//  std::cout << "!LidarAdvSpd: " << advisedSpeed <<std::endl;
  if (true == decisionData.b_takeOverEnable){
    advisedSpeedTakeOver = curvesLidarCheck(decisionData.curveList[1], lidarData.blockedAreaIndex, decisionData.previewDistance+5, decisionData.targetSpeed, judgeSpeed);
//    advisedSpeedComeBack = curvesLidarCheck(decisionData.curveList[2], lidarData.blockedAreaIndex);
  }
  std::cout << RED << "[OTC] Advised Speed in Lane: "  << RESET  << std::fixed << std::setprecision(1) << advisedSpeed*3.6<<" km/h"<< std::endl;
  std::cout << RED << "[OTC] Advised Speed in Left Lane: "  << RESET  << std::fixed << std::setprecision(1) << advisedSpeedTakeOver*3.6<<" km/h"<< std::endl;
  
  decisionData.targetAccLevel = ACC_NORMAL;
  // normal one curve
  if (decisionData.b_takeOverEnable == false) {
    std::cout<<"[OTC] Overtaking not enabled\n";
    decisionData.b_takingOver = false;
    if(advisedSpeed < LIDAR_MAX_SPEED ){
      if (decisionData.targetSpeed > advisedSpeed) {
        decisionData.targetSpeed = advisedSpeed;
        //decisionData.targetAccLevel = ACC_HIGH;                 //20160604pm added,
        decisionData.targetAccLevel = ACC_OBSTACLE;
        std::cout << RED << "[OTC] Speed limited because of obstacle." << RESET << std::endl;
      }
      else{
        decisionData.targetAccLevel = ACC_NORMAL;
        std::cout << GREEN << "[OTC] No Obstacles!" << RESET << std::endl;
      }
    }
  }
  else if(decisionData.b_takeOverEnable == true){
    std::cout<<"[OTC] Overtaking Enabled\n";
    if(decisionData.b_takingOver == false){ // not in overtaking
          std::cout << "[OTC] not in overtaking\n";
      if (advisedSpeed < LIDAR_MAX_SPEED && advisedSpeedTakeOver >= ALMOST_ZERO) {  // left lane passable
            std::cout << "[OTC] left lane passible\n";
        if (decisionData.targetSpeed > advisedSpeed && advisedSpeed >= OVER_TAKE_SPEED_THRESHOLD ) {
          decisionData.targetSpeed = advisedSpeed;
          decisionData.targetAccLevel = ACC_OBSTACLE;
          std::cout << MAGENTA << "[OTC] Speed limited because of obstacle in lane." << RESET << std::endl;
        }
        else if(decisionData.targetSpeed > advisedSpeed && advisedSpeed < OVER_TAKE_SPEED_THRESHOLD ){
          std::cout << "[OTC] Begin overtaking! " << std::endl;
          decisionData.b_takingOver = true;
          decisionData.targetAccLevel = ACC_NORMAL;
          decisionData.takingOverTimeCounter = boost::posix_time::microsec_clock::local_time();
        }
        else{
          decisionData.targetAccLevel = ACC_NORMAL;
          std::cout << "[OTC] No Obstacles in lane!" << std::endl;
        }
      }
      else if(advisedSpeed < LIDAR_MAX_SPEED){
        if (decisionData.targetSpeed > advisedSpeed) {
          decisionData.targetSpeed = advisedSpeed;
          decisionData.targetAccLevel = ACC_OBSTACLE;
          std::cout << MAGENTA << "[OTC] Speed limited because of obstacle in lane." << RESET << std::endl;
          std::cout << "[OTC] Not Overtaking because of obstacle." << std::endl;
        }
        else{
          decisionData.targetAccLevel = ACC_NORMAL;
          std::cout << "[OTC] No Obstacles in lane!" << std::endl;
          std::cout << "[OTC] Not Overtaking because of obstacle." << std::endl;
        }
      }
    }
    else if (decisionData.b_takingOver == true){
      std::cout<<"[OTC] Overtaking now!\n";
//      if (decisionData.targetSpeed > TAKE_OVER_MAX_SPEED) {
//        decisionData.targetSpeed = TAKE_OVER_MAX_SPEED;
//      }
//      else;
      
      if (advisedSpeed == LIDAR_MAX_SPEED){

        decisionData.targetAccLevel = ACC_NORMAL;
        boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration diff;

        diff = now - decisionData.takingOverTimeCounter;
        std::cout << "[OTC] Overtake Time Counter: " << diff.total_milliseconds() << std::endl;

        if (diff.total_milliseconds() > TAKE_OVER_PERIOD) {
          if (decisionData.targetSpeed > advisedSpeed) {
            decisionData.targetSpeed = advisedSpeed;
          }else;
          decisionData.b_takingOver = false;
          std::cout << "[OTC] Coming Back after Overtake!" << std::endl;
        }
        else if (advisedSpeedTakeOver < LIDAR_MAX_SPEED ){
          if (decisionData.targetSpeed > advisedSpeedTakeOver) {
            decisionData.targetSpeed = advisedSpeedTakeOver;
            decisionData.targetAccLevel = ACC_OBSTACLE;
            std::cout << MAGENTA << "[OTC] Speed limited because of obstacle in Left lane." << RESET << std::endl;
            std::cout << "[OTC] in Overtaking period." << std::endl;
          }
          else{
            std::cout << "[OTC] No Obstacles in left lane!" << std::endl;
            std::cout << "[OTC] in Overtaking period." << std::endl;
          }
        }
        else;
      }
      else if (advisedSpeedTakeOver < LIDAR_MAX_SPEED ){
        decisionData.takingOverTimeCounter = boost::posix_time::microsec_clock::local_time();
        if (decisionData.targetSpeed > advisedSpeedTakeOver) {
          decisionData.targetSpeed = advisedSpeedTakeOver;
          decisionData.targetAccLevel = ACC_OBSTACLE;
          std::cout << MAGENTA << "[OTC] Speed limited because of obstacle in Left lane." << RESET << std::endl;
          std::cout << "[OTC] Not Coming back." << std::endl;
        }
        else{
          std::cout << "[OTC] No Obstacles in left lane!" << std::endl;
        }
      }
      else{
        decisionData.targetSpeed = advisedSpeedTakeOver;
        decisionData.targetAccLevel = ACC_NORMAL;
        decisionData.takingOverTimeCounter = boost::posix_time::microsec_clock::local_time();
      };
    }
    else;
  }
  else{
    ;
  }
//  std::cout << "!LidarAdvSpd: " << decisionData.targetSpeed <<std::endl;
//  std::cout << "!LidarAdvSpd: " << advisedSpeed <<std::endl;
}

/*
void lidarJudgeLongerPreview(DecisionData& decisionData, DecisionData& lidarDecisionData, PostureData& postureData, LidarData& lidarData, std::vector<std::vector<RoadPoint>>& roadPointss){
  double advisedSpeedTakeOver = LIDAR_MAX_SPEED;
//  double advisedSpeedComeBack = LIDAR_MAX_SPEED;
//  generateLidarCurvePreviewPoint(lidarDecisionData, postureData);
  generateLidarCurvePreviewPoint(decisionData, roadPointss, postureData, lidarDecisionData);
  double advisedSpeed = curvesLidarCheck(lidarDecisionData.curveList[0], lidarData.blockedAreaIndex, lidarDecisionData.previewDistance);
  
//  std::cout << "!!LidarAdvSpd: " << advisedSpeed <<std::endl;

  lidarDecisionData.targetAccLevel = ACC_NORMAL;
  
  if(advisedSpeed < LIDAR_MAX_SPEED ){
    if (lidarDecisionData.targetSpeed > advisedSpeed) {
      lidarDecisionData.targetSpeed = advisedSpeed;
      //decisionData.targetAccLevel = ACC_HIGH;                 //20160604pm added,
      lidarDecisionData.targetAccLevel = ACC_OBSTACLE;
      std::cout << "[LLidar] Speed limited because of obstacle." << std::endl;
    }
    else{
      lidarDecisionData.targetAccLevel = ACC_NORMAL;
      std::cout << "[LLidar] No Obstacles!" << std::endl;
    }
  }
}
*/
//170907
void lidarJudgeLongerPreview(DecisionData& decisionData, DecisionData& lidarDecisionData, PostureData& postureData, LidarData& lidarData, std::vector<std::vector<RoadPoint>>& roadPointss, ros::Publisher curve_pub_, ros::Publisher virtual_curve_pub_){
  double judgeSpeed = (decisionData.targetSpeed>LIDAR_MAX_SPEED) ? LIDAR_MAX_SPEED:decisionData.targetSpeed;
  judgeSpeed = (decisionData.targetSpeed<LIDAR_MIN_SPEED) ? LIDAR_MIN_SPEED:decisionData.targetSpeed;
  double advisedSpeedTakeOver = judgeSpeed;
  generateLidarCurvePreviewPoint(decisionData, roadPointss, postureData, lidarDecisionData);
//  std::cout << "  lidarDecisionData.curveList.size " << lidarDecisionData.curveList.size() << std::endl;

  lidarDistJudge(decisionData);

  double advisedSpeed = curvesLidarCheck(lidarDecisionData.curveList[0], lidarData.blockedAreaIndex, lidarDecisionData.previewDistance, decisionData.targetSpeed, judgeSpeed);
//  std::cout << "!LidarAdvSpd: " << advisedSpeed <<std::endl;
  Curve virtualCurve = createVirtualCurve();
  // check virtual curve in low curvature road
  std::cout << "-----decisionData.currentPoint.parameter2: " << decisionData.currentPoint.parameter2 << std::endl;
  if (decisionData.currentPoint.parameter2 < 0.1 && decisionData.currentPoint.parameter2 > (-0.1)) {
    std::cout << "+++++++++++++++CHECK VIRTUAL+++++++++++++++++++" <<std::endl;
    double advisedSpeedVirtualCurve = curvesLidarCheck(virtualCurve, lidarData.blockedAreaIndex, lidarDecisionData.previewDistance, decisionData.targetSpeed, judgeSpeed, true);
    advisedSpeed = std::min(advisedSpeedVirtualCurve, advisedSpeed);
    curvePublisher(virtual_curve_pub_, virtualCurve);
    
  }
  if (true == decisionData.b_takeOverEnable){
    advisedSpeedTakeOver = curvesLidarCheck(lidarDecisionData.curveList[1], lidarData.blockedAreaIndex, lidarDecisionData.previewDistance, decisionData.targetSpeed, judgeSpeed);
//    advisedSpeedComeBack = curvesLidarCheck(decisionData.curveList[2], lidarData.blockedAreaIndex);
  }
  // curvePublisher(virtual_curve_pub_, virtualCurve);
  curvePublisher(curve_pub_, lidarDecisionData);

  //limit acceleration
  accLimiter(advisedSpeed, advisedSpeedTakeOver, decisionData);

  

  //set small speed to 0
  if(advisedSpeed*36 < 5) advisedSpeed = 0;
  if(advisedSpeedTakeOver*36 < 5) advisedSpeedTakeOver = 0;

  std::cout << "[OTC] CURRENT SPEED:                          " << decisionData.velocity*3.6 <<" km/h"<< std::endl;
  std::cout << "[OTC] Advised Speed in Lane:                  " << std::fixed<< advisedSpeed *3.6<<" km/h"<< std::endl;
  std::cout << "[OTC] Advised Speed in Left Lane:             " << std::fixed<< advisedSpeedTakeOver *3.6<<" km/h"<< std::endl;
  
//  double advisedSpeed2 = curvesLidarVerticalCheck(lidarDecisionData.curveList[0], lidarData.blockedAreaIndex);
//  double advisedSpeedOvertake2 = curvesLidarVerticalCheck(lidarDecisionData.curveList[1], lidarData.blockedAreaIndex);
//  std::cout << "[OTC2] Advised Speed in Lane: " << advisedSpeed2 << std::endl;
//  std::cout << "[OTC2] Advised Speed in Left Lane: " << advisedSpeedOvertake2 << std::endl;
  
  decisionData.targetAccLevel = ACC_NORMAL;
  // normal one curve
  if (decisionData.b_takeOverEnable == false) {
    std::cout<<"[OTC] Overtaking not enabled\n";
    decisionData.b_takingOver = false;
    if(advisedSpeed < judgeSpeed ){
      if (decisionData.targetSpeed > advisedSpeed) {
        decisionData.targetSpeed = advisedSpeed;
        //decisionData.targetAccLevel = ACC_HIGH;                 //20160604pm added,
        decisionData.targetAccLevel = ACC_OBSTACLE;
        std::cout << RED << "[OTC] Speed limited because of obstacle." << RESET << std::endl;
      }
      else{
        decisionData.targetAccLevel = ACC_NORMAL;
        std::cout << GREEN << "[OTC] No Obstacles!" << RESET << std::endl;
      }
    }
  }
  else if(decisionData.b_takeOverEnable == true){
    std::cout<<"[OTC] Overtaking Enabled\n";
    if(decisionData.b_takingOver == false){ // not in overtaking
          std::cout << "[OTC] not in overtaking\n";
      if (advisedSpeed < judgeSpeed && advisedSpeedTakeOver >= ALMOST_ZERO) {  // left lane passable
            std::cout << "[OTC] left lane passible\n";
        if (decisionData.targetSpeed > advisedSpeed && advisedSpeed >= OVER_TAKE_SPEED_THRESHOLD ) {
          decisionData.targetSpeed = advisedSpeed;
          decisionData.targetAccLevel = ACC_OBSTACLE;
          std::cout << "[OTC] Speed limited because of obstacle in lane." << std::endl;
        }
        else if(decisionData.targetSpeed > advisedSpeed && advisedSpeed < OVER_TAKE_SPEED_THRESHOLD ){
          std::cout << "[OTC] Begin overtaking! " << std::endl;
          decisionData.b_takingOver = true;
          decisionData.targetAccLevel = ACC_NORMAL;
          decisionData.takingOverTimeCounter = boost::posix_time::microsec_clock::local_time();
        }
        else{
          decisionData.targetAccLevel = ACC_NORMAL;
          std::cout << "[OTC] No Obstacles in lane!" << std::endl;
        }
      }
      else if(advisedSpeed < judgeSpeed){
        if (decisionData.targetSpeed > advisedSpeed) {
          decisionData.targetSpeed = advisedSpeed;
          decisionData.targetAccLevel = ACC_OBSTACLE;
          std::cout << "[OTC] Speed limited because of obstacle in lane." << std::endl;
          std::cout << "[OTC] Not Overtaking because of obstacle." << std::endl;
        }
        else{
          decisionData.targetAccLevel = ACC_NORMAL;
          std::cout << "[OTC] No Obstacles in lane!" << std::endl;
          std::cout << "[OTC] Not Overtaking because of obstacle." << std::endl;
        }
      }
    }
    else if (decisionData.b_takingOver == true){
      std::cout<<"[OTC] Overtaking now!\n";
      if (decisionData.targetSpeed > TAKE_OVER_MAX_SPEED) {
        decisionData.targetSpeed = TAKE_OVER_MAX_SPEED;
      }
      else;
      
      if (advisedSpeed == judgeSpeed){

        decisionData.targetAccLevel = ACC_NORMAL;
        boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration diff;

        diff = now - decisionData.takingOverTimeCounter;
        std::cout << "[OTC] Overtake Time Counter: " << diff.total_milliseconds() << std::endl;

        if (diff.total_milliseconds() > TAKE_OVER_PERIOD) {
          if (decisionData.targetSpeed > advisedSpeed) {
            decisionData.targetSpeed = advisedSpeed;
          }else;
          decisionData.b_takingOver = false;
          std::cout << "[OTC] Coming Back after Overtake!" << std::endl;
        }
        else if (advisedSpeedTakeOver < judgeSpeed ){
          if (decisionData.targetSpeed > advisedSpeedTakeOver) {
            decisionData.targetSpeed = advisedSpeedTakeOver;
            decisionData.targetAccLevel = ACC_OBSTACLE;
            std::cout << "[OTC] Speed limited because of obstacle in Left lane." << std::endl;
            std::cout << "[OTC] in Overtaking period." << std::endl;
          }
          else{
            std::cout << "[OTC] No Obstacles in left lane!" << std::endl;
            std::cout << "[OTC] in Overtaking period." << std::endl;
          }
        }
        else;
      }
      // as long as there is obstacles in right line, car will not quit overtaking.
      else if (advisedSpeedTakeOver < judgeSpeed ){
        decisionData.takingOverTimeCounter = boost::posix_time::microsec_clock::local_time();
        if (decisionData.targetSpeed > advisedSpeedTakeOver) {
          decisionData.targetSpeed = advisedSpeedTakeOver;
          decisionData.targetAccLevel = ACC_OBSTACLE;
          std::cout << "[OTC] Speed limited because of obstacle in Left lane." << std::endl;
          std::cout << "[OTC] Not Coming back." << std::endl;
        }
        else{
          std::cout << "[OTC] No Obstacles in left lane!" << std::endl;
        }
      }
      else{
        decisionData.targetSpeed = advisedSpeedTakeOver;
        decisionData.targetAccLevel = ACC_NORMAL;
        decisionData.takingOverTimeCounter = boost::posix_time::microsec_clock::local_time();
      };
      decisionData.targetSpeed = (decisionData.targetSpeed>TAKE_OVER_MAX_SPEED)?TAKE_OVER_MAX_SPEED:decisionData.targetSpeed;
    }
    else;
  }
  else{
    ;
  }
}

void accLimiter(double& advisedSpeed, double& advisedSpeedTakeOver, DecisionData& decisionData)
{
  if(decisionData.velocity*3.6 > 20 && (advisedSpeed-decisionData.last_advisedSpeed) > (0.5/3.6)){
    advisedSpeed = decisionData.last_advisedSpeed + (0.5/3.6);
  }
  else if(decisionData.velocity*3.6 > 15 && (advisedSpeed-decisionData.last_advisedSpeed) > (0.4/3.6)){
    advisedSpeed = decisionData.last_advisedSpeed + (0.4/3.6);
  }
  else if(decisionData.velocity*3.6 > 10 && (advisedSpeed-decisionData.last_advisedSpeed) > (0.3/3.6)){
    advisedSpeed = decisionData.last_advisedSpeed + (0.3/3.6);
  }
  else if((advisedSpeed-decisionData.last_advisedSpeed) > (0.2/3.6)){
    advisedSpeed = decisionData.last_advisedSpeed + (0.2/3.6);
  }

  if(decisionData.velocity*3.6 > 20 && (advisedSpeed-decisionData.last_advisedSpeedTakeOver) > (0.5/3.6)){
    advisedSpeedTakeOver = decisionData.last_advisedSpeedTakeOver + (0.5/3.6);
  }
  else if(decisionData.velocity*3.6 > 15 && (advisedSpeed-decisionData.last_advisedSpeedTakeOver) > (0.4/3.6)){
    advisedSpeedTakeOver = decisionData.last_advisedSpeedTakeOver + (0.4/3.6);
  }
  else if(decisionData.velocity*3.6 > 10 && (advisedSpeed-decisionData.last_advisedSpeedTakeOver) > (0.3/3.6)){
    advisedSpeedTakeOver = decisionData.last_advisedSpeedTakeOver + (0.3/3.6);
  }
  else if((advisedSpeed-decisionData.last_advisedSpeedTakeOver) > (0.2/3.6)){
    advisedSpeedTakeOver = decisionData.last_advisedSpeedTakeOver + (0.2/3.6);
  }

  decisionData.last_advisedSpeed = advisedSpeed;
  decisionData.last_advisedSpeedTakeOver = advisedSpeedTakeOver;

}

void lidarDistJudge(DecisionData& decisionData )
{
  if(decisionData.velocity*3.6 > 35)
  {
    LIDAR_SAFE_ZONE_MIN = 7;
  }
  else if(decisionData.velocity*3.6 > 30)
  {
    LIDAR_SAFE_ZONE_MIN = 6.8;
  }
  else if(decisionData.velocity*3.6 > 25)
  {
    LIDAR_SAFE_ZONE_MIN = 6.6;
  }
  else if(decisionData.velocity*3.6 > 20)
  {
    LIDAR_SAFE_ZONE_MIN = 6.4;
  }
  else if(decisionData.velocity*3.6 > 17.5)
  {
    LIDAR_SAFE_ZONE_MIN = 6.2;
  }
  else if(decisionData.velocity*3.6 > 15)
  {
    LIDAR_SAFE_ZONE_MIN = 5.8;
  }
  else if(decisionData.velocity*3.6 > 12.5)
  {
    LIDAR_SAFE_ZONE_MIN = 5.6;
    if(decisionData.b_takingOver == true)
    {
      LIDAR_SAFE_ZONE_MIN = 2.3;
    }
  }
  else if(decisionData.velocity*3.6 > 10)
  {
    LIDAR_SAFE_ZONE_MIN = 3.8;
    if(decisionData.b_takingOver == true)
    {
      LIDAR_SAFE_ZONE_MIN = 2.3;
    }
  }
  else if(decisionData.velocity*3.6 > 8)
  {
    LIDAR_SAFE_ZONE_MIN = 3.6;      // old 2.7
    if(decisionData.b_takingOver == true)
    {
      LIDAR_SAFE_ZONE_MIN = 2.5;
    }
  }
  else if(decisionData.velocity*3.6 > 6)
  {
    LIDAR_SAFE_ZONE_MIN = 3.4;       // old 2.5
    if(decisionData.b_takingOver == true)
    {
      LIDAR_SAFE_ZONE_MIN = 2.5;
    }
  }
  else if(decisionData.velocity*3.6 > 4)
  {
    LIDAR_SAFE_ZONE_MIN = 3.2;        // old 2.5
    if(decisionData.b_takingOver == true)
    {
      LIDAR_SAFE_ZONE_MIN = 2.5;       
    }
  }
  else
  {
    LIDAR_SAFE_ZONE_MIN = 3.2;
    if(decisionData.b_takingOver == true)
    {
      LIDAR_SAFE_ZONE_MIN = 2.5;
    }
  }

  if(decisionData.velocity*3.6 > 15)
  {
    LIDAR_SAFE_DISTANCE_MIN = 3;
  }
  else if(decisionData.velocity*3.6 > 6)
  {
    // LIDAR_SAFE_DISTANCE_MIN = (decisionData.velocity*3.6-6)/9*1 + 2;
    LIDAR_SAFE_DISTANCE_MIN = 5;
  }
  else 
  {
    LIDAR_SAFE_DISTANCE_MIN = 3.2;
  }
}

//CXY
/*
void speedPlanWithLidar(DecisionData& decisionData, double advisedSpeed){
  advisedSpeed = MIN(advisedSpeed,LIDAR_MAX_SPEED);
  if (advisedSpeed < decisionData.targetSpeed) {
    decisionData.targetSpeed = advisedSpeed;
    decisionData.targetAccLevel = ACC_OBSTACLE;
    std::cout << "[OTC] Speed limited by obstacle!" << std::endl;
  }
  else{
    decisionData.targetAccLevel = ACC_NORMAL;
    std::cout << "[OTC] Speed Normal." << std::endl;
  }
}

void lidarJudge(LidarData& lidarData, DecisionData& decisionData, PostureData& postureData){
  if (true == lidarData.b_isAlive){  
    double advisedSpeedOvertake = LIDAR_MAX_SPEED;
    generateCurvePreviewPoint(decisionData, postureData);
    double advisedSpeed = curvesLidarCheck(decisionData.curveList[0], lidarData.blockedAreaIndex, decisionData.previewDistance);
    if (true == decisionData.b_takeOverEnable){
      advisedSpeedOvertake = curvesLidarCheck(decisionData.curveList[1], lidarData.blockedAreaIndex, decisionData.previewDistance);
    }

    std::cout << "[OTC] Advised Speed in Lane: " << std::setprecision(1) << advisedSpeed << " m/s\n";
    std::cout << "[OTC] Advised Speed in Left Lane: " << std::setprecision(1) << advisedSpeedOvertake << " m/s\n";
    
    bool b_currentLaneObstacle = advisedSpeed < LIDAR_MAX_SPEED ? true:false;
    bool b_overtakeLaneObstacle = advisedSpeedOvertake < LIDAR_MAX_SPEED ? true:false;
    bool b_currentLaneBlocked = advisedSpeed <= ALMOST_ZERO ? true:false;
    bool b_overtakeLaneBlocked = advisedSpeedOvertake <= ALMOST_ZERO ? true:false;

    std::cout << "[OTC] Current Lane: ";
    if (b_currentLaneBlocked == true) std::cout << "Blocked!\n";
    else if (b_currentLaneObstacle == true) std::cout << "Obstacle!\n";
    else std::cout << "Normal.\n";
    std::cout << "[OTC] Left Lane: ";
    if (b_overtakeLaneBlocked == true) std::cout << "Blocked!\n";
    else if (b_overtakeLaneObstacle == true) std::cout << "Obstacle!\n";
    else std::cout << "Normal.\n";

    if (decisionData.b_takeOverEnable == false){
      std::cout << "[OTC] Overtaking OFF\n";
      decisionData.b_takingOver = false;
      speedPlanWithLidar(decisionData,advisedSpeed);
    }
    else if (decisionData.b_takeOverEnable == true){
      std::cout << "[OTC] Overtaking ON\n";
      if (decisionData.b_takingOver == false){
        std::cout << "[OTC] not in overtake\n";
        if (b_currentLaneBlocked == false || b_overtakeLaneBlocked == true){
          speedPlanWithLidar(decisionData,advisedSpeed);
        }
        else if (b_currentLaneBlocked == true && b_overtakeLaneBlocked == false){
          std::cout << "[OTC] transfer to overtake!\n";
          decisionData.b_takingOver = true;
          decisionData.targetSpeed = MIN(decisionData.targetSpeed,MIN(advisedSpeedOvertake,LIDAR_MAX_SPEED));
          decisionData.targetAccLevel = ACC_NORMAL;
          overtakeDistance = 0;
        } 
      }
      else if(decisionData.b_takingOver == true){ 
        std::cout << "[OTC] Overtaking!\n";
        if (overtakeDistance < OVERTAKE_DIST_THR){ // defined in this file
          overtakeDistance += actuatorData.vehicleSpeed * TIMER4_PERIOD/1000;
          std::cout << "[OTC] overtake distance = " << std::setprecision(1) << overtakeDistance;
          speedPlanWithLidar(decisionData,advisedSpeedOvertake);
        }
        else if (overtakeDistance >= OVERTAKE_DIST_THR){
          std::cout << "[OTC] return from overtake!\n";
          decisionData.b_takingOver = false;
          decisionData.targetSpeed = MIN(decisionData.targetSpeed,MIN(advisedSpeed,LIDAR_MAX_SPEED));
          decisionData.targetAccLevel = ACC_NORMAL;
        }
      }
    }
  }  
}
*/


double stanley(DecisionData& decisionData, RoadPoint& currentPoint, double& yawDiff, double& velocity)
{
  double ksoft = 1;
  static double l_y = 0;
  double inner = KDStanley * (-currentPoint.lateral_dist) / (ksoft + velocity);
  // std::cout << "[stanley] atan inside " << inner << " atan " << atan(inner) << std::endl;
  double s_angle = yawDiff + StanleyYKD * (yawDiff - l_y)/0.1 + StanleyPDist * atan(inner);
  l_y = yawDiff;
  std::cout<<"[stanley] lateral_dist "<<currentPoint.lateral_dist<<std::endl;
  return s_angle;
 
}

//-------------------------------------------- ROS funs ------------------------------------------------

// Create a new map
map_t *map_alloc(void)
{
  map_t *map;

  map = (map_t*) malloc(sizeof(map_t));

  // Assume we start at (0, 0)
  map->origin_x = 0;
  map->origin_y = 0;
  
  // Make the size odd
  map->size_x = 0;
  map->size_y = 0;
  map->scale = 0;
  
  // Allocate storage for main map
  map->cells = (map_cell_t*) NULL;
  
  return map;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom, PostureData* postureDataPtr, ros::Time* odomReceivedTimePtr){

  int gps_state;

  postureDataPtr->imuData.longitude = odom->pose.pose.position.x;
  postureDataPtr->imuData.latitude = odom->pose.pose.position.y;
  postureDataPtr->gpsData.longitude = odom->pose.pose.position.x;
  postureDataPtr->gpsData.latitude = odom->pose.pose.position.y;
  gps_state = odom->pose.pose.position.z;
  postureDataPtr->imuData.yaw = tf::getYaw(odom->pose.pose.orientation)*180/M_PI;
  if(postureDataPtr->imuData.yaw < 0) postureDataPtr->imuData.yaw += 360;
  postureDataPtr->imuData.velocity = odom->twist.twist.linear.x; 
  
  postureDataPtr->b_isAlive = 1;

  if(gps_state == 4)
  {
    postureDataPtr->imuData.b_isValid = 1;
    postureDataPtr->imuData.b_gpsValid = 1;

    //path record
    if(path_record_switch == true)
    {
      if(path_record_time == 0)
      {
        postureDataPtr->longitude_array.clear();
        postureDataPtr->latitude_array.clear();
        postureDataPtr->yaw_array.clear();
        path_record_time = 1;
      }
      postureDataPtr->longitude_array.insert(postureDataPtr->longitude_array.begin(), odom->pose.pose.position.x);
      postureDataPtr->latitude_array.insert(postureDataPtr->latitude_array.begin(), odom->pose.pose.position.y);
      postureDataPtr->yaw_array.insert(postureDataPtr->yaw_array.begin(), 90-postureDataPtr->imuData.yaw);
      if(postureDataPtr->longitude_array.size() > PAST_PATH_SIZE)
      {
        postureDataPtr->longitude_array.pop_back();
        postureDataPtr->latitude_array.pop_back();
        postureDataPtr->yaw_array.pop_back();
      }
    }
    else
    {
      path_record_time = 0;
    }
  }
  else
  {
    postureDataPtr->imuData.b_isValid = 0;
    postureDataPtr->imuData.b_gpsValid = 0;
  }

  *odomReceivedTimePtr = odom->header.stamp;
  // ROS_INFO("Odom time: %.3f",odomReceivedTimePtr->toSec());

}

void gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid, LidarData* lidarDataPtr, DecisionData* lidarDecisionDataPtr, ros::Time* lidarReceivedTimePtr, map_t* map){
  
    ROS_INFO("Received a %d X %d map @ %.3f m/pix, Origin x: %.3f, y: %.3f ",
            grid->info.width,
            grid->info.height,
            grid->info.resolution,
            grid->info.origin.position.x,
            grid->info.origin.position.y);

  	map->size_x = grid->info.width;
  	map->size_y = grid->info.height;
  	map->scale = grid->info.resolution;
  	map->origin_x = grid->info.origin.position.x;
  	map->origin_y = grid->info.origin.position.y;
  	// Convert to player format
    //  The map data, in row-major order, starting with (0,0).  Occupancy
    //  probabilities are in the range [0,100].  Free is -1.
  	map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  	ROS_ASSERT(map->cells);
    // ROS_INFO("Created cells!\n");
  	for(int i=0;i<map->size_x * map->size_y;i++)
  	{
    	if(grid->data[i] == 0)
      		map->cells[i].occ_state = -1;
    	else if(grid->data[i] == 100)
      		map->cells[i].occ_state = +1;
    	else
      		map->cells[i].occ_state = 0;
  	}
    // ROS_INFO("Cells Defined!\n");

    int totalNumber = map->size_x * map->size_y;
    // std::vector<bool> grids;
    // grids.reserve(GRID_INDEX_SIZE);
    // grids.resize(GRID_INDEX_SIZE,1);
    // ROS_INFO("grids created");

    lidarDataPtr->blockedAreaIndex.clear();
    for(int i=0;i<totalNumber;i++){
      if((i < GRID_INDEX_SIZE)&&(map->cells[i].occ_state == 1)){
          lidarDataPtr->blockedAreaIndex.push_back(i);
          // grids[i] = 0;
      }
    }
    // ROS_INFO("grids defined!\n");

    if(DISPLAY_GRID_ON == 3){
      for(int32_t i=GRID_Y_NUM/2-1; i>=0; i--){//0
        std::cout<<"||";
        for(int32_t j=0; j<map->size_x; j++){
            int32_t idx = i*map->size_x + j;
            if( map->cells[idx].occ_state == 1 &&(j>(X_SIZE_MAX-7.5)/map->scale)&&(j<(X_SIZE_MAX+7.5)/map->scale)) std::cout<<"o";
            else if((j>(X_SIZE_MAX-1.5)/map->scale)&&(j<(X_SIZE_MAX+1.5)/map->scale)&&(i<8/map->scale)) std::cout<<"+"; 
            else if((j>(X_SIZE_MAX-7.5)/map->scale)&&(j<(X_SIZE_MAX+7.5)/map->scale)) std::cout<<"*";
        }
        std::cout<<"||"<<std::endl;
      }
    }
    

    ROS_INFO("blockedAreaIndex SIZE: %ld",lidarDataPtr->blockedAreaIndex.size());

    lidarDataPtr->b_isAlive = 1;
		lidarDataPtr->b_isValid = 1;
    *lidarReceivedTimePtr = ros::Time::now();
    ROS_INFO("lidar time: %.3f",lidarReceivedTimePtr->toSec());
    free(map->cells);

}

void decisionSubCallback(const std_msgs::Float64MultiArray::ConstPtr& decision_sub, DecisionData* decisionDataPtr, UiData* uiDataPtr){
  
  uiDataPtr->pathNumber = decision_sub->data[0];
  uiDataPtr->b_stopFlag = decision_sub->data[1];
  decisionDataPtr->current_steering_angle = decision_sub->data[2];
  ROS_INFO("Received Path Number: %d",decisionDataPtr->pathNumber);
  ROS_INFO("Received current_steering_angle: %.1f",decisionDataPtr->current_steering_angle);

}

void previewPointPublisher(ros::Publisher previewPoint_pub, DecisionData decisionData){

	geometry_msgs::PoseArray cloud_msg;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = "base_link";

  RoadPoint previewPoint;

  if(decisionData.b_takingOver == true)
  {
    previewPoint = decisionData.previewPointTakeOver;
  }
  else
  {
    previewPoint = decisionData.previewPoint;
  }

  cloud_msg.poses.resize(1);
  tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(-(previewPoint.courseAngle-90)/180*M_PI),
                                   tf::Vector3(previewPoint.longitude-offset_tempX,
                                               previewPoint.latitude-offset_tempY, 0)),
                          cloud_msg.poses[0]);

  previewPoint_pub.publish(cloud_msg);

}

void roadPointPublisher(ros::Publisher roadPoint_pub_,std::vector<std::vector<RoadPoint>>& RoadPointss, std::vector<std::vector<uint32_t>>& pathNumberLists, uint16_t pathNumber){

	// ros::Rate r(1/LOOP_DURATION);
	// ROS_INFO("Pointcloud publish Begin >>>>>>>>>>>>> ");
	geometry_msgs::PoseArray cloud_msg;

  // ROS_INFO("OFFSETX: %lf, OFFSETY: %lf, pathNumber: %d",offset_tempX,offset_tempY,pathNumber);

  int i,j;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = "base_link";
  double first_id = pathNumberLists[pathNumber][1];
  double last_id = pathNumberLists[pathNumber].size()-2 + first_id;
  // double size = (last_id - first_id) * RoadPointss[first_id].size() + RoadPointss[last_id].size();
  double size = 0;
  for(i = first_id; i <= last_id; i++){
    size += RoadPointss[i].size();
  }
  // ROS_INFO("first_id: %.0f, last_id: %.0f, Total index: %lf",first_id,last_id,size);
  cloud_msg.poses.resize(size);
  int index = 0;
  for(i = first_id; i <= last_id; ++i)
  {
      for(j = 0; j < RoadPointss[i].size(); ++j){
          tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(-(RoadPointss[i][j].courseAngle-90)/180*M_PI),
                                   tf::Vector3(RoadPointss[i][j].longitude-offset_tempX,
                                               RoadPointss[i][j].latitude-offset_tempY, 0)),
                          cloud_msg.poses[index]);
          index++;
      }

  }
  roadPoint_pub_.publish(cloud_msg);
  
	 ROS_INFO("-------------------------roadPoint publish Done----------------------------");
	// r.sleep();

}

void curvePublisher(ros::Publisher curve_pub_, const Curve& curve) {
  geometry_msgs::PoseArray cloud_msg;
  int i,j;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = "base_link";
  ROS_INFO("decisionData.curveList.size() %ld",decisionData.curveList.size());
  int size = CURVE_POINT_NUM;
  cloud_msg.poses.resize(size);
  int index = 0;
  for(j = 0; j < size; ++j){
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(curve.points[j].angle),
                                  tf::Vector3(curve.points[j].x,
                                              curve.points[j].y, M_PI/2)),
                        cloud_msg.poses[index]);
        index++;
    }
  curve_pub_.publish(cloud_msg);
  ROS_INFO("---I publish a curve!");
}

void curvePublisher(ros::Publisher curve_pub_, DecisionData& decisionData){
  
  geometry_msgs::PoseArray cloud_msg;
  int i,j;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = "base_link";
  // ROS_INFO("decisionData.curveList.size() %ld",decisionData.curveList.size());
  int size = decisionData.curveList.size() * CURVE_POINT_NUM;
  cloud_msg.poses.resize(size);
  int index = 0;
  // for(i = 0; i < decisionData.curveList.size(); ++i)
  for(i = 0; i < 1; ++i) //self lane curve only
  {
      for(j = 0; j < CURVE_POINT_NUM; ++j){
          tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(decisionData.curveList[i].points[j].angle),
                                   tf::Vector3(decisionData.curveList[i].points[j].x,
                                               decisionData.curveList[i].points[j].y, M_PI/2)),
                          cloud_msg.poses[index]);
          index++;
      }

  }
  curve_pub_.publish(cloud_msg);
  ROS_INFO("===I publish a  decisionData curve!");
}

void pastPathPublisher(ros::Publisher past_path_pub_, PostureData& postureData){
  
  geometry_msgs::PoseArray cloud_msg;
  int j;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = "base_link";
  // ROS_INFO("decisionData.curveList.size() %ld",decisionData.curveList.size());
  int size = postureData.longitude_array.size();
  cloud_msg.poses.resize(size);
  int index = 0;

  for(j = 0; j < size; ++j)
  {
      tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(postureData.yaw_array[j]/180*M_PI),
                               tf::Vector3(postureData.longitude_array[j]-offset_tempX,
                                           postureData.latitude_array[j]-offset_tempY, 0)),
                      cloud_msg.poses[index]);
      index++;
  }

  past_path_pub_.publish(cloud_msg);

}

void read_ui(std::string fileName, UiData& uiData){

	std::string line;
	std::ifstream fs;

  fs.open(fileName, std::ios::in);
  if (fs.fail()){
    std::cout << RED << "!!!!!!FATAL!!!!!!LOAD UI MAP FAIL!!!!!!" << RESET << std::endl;
    return;
  }else;
  while(getline(fs, line))
  {
    if (line.length() > 0){
      std::stringstream ss(line);
      ss >> uiData.GlobalOffset_X >> uiData.GlobalOffset_Y >> uiData.UIMap_Xoffset0 >> uiData.UIMap_Yoffset0 >> uiData.UIMap_Scale >> uiData.UIMap_Margin >> uiData.UIMap_Ymax >> uiData.UIMap_rotateDeg >> uiData.UIMap_Xoffset >> uiData.UIMap_Yoffset;
      uiData.b_pos_isValid = true;
      break;
    }
    else{
      uiData.b_pos_isValid = false;
    }
  }

  ROS_INFO("OFFSET_X: %.3f, OFFSET_Y: %.3f\n", uiData.GlobalOffset_X, uiData.GlobalOffset_Y);
}

void calculate_ui(UiData& uiData, PostureData& postureData){
  
  double x,y,temp_x,temp_y;

  x = postureData.imuData.longitude; 
  y = postureData.imuData.latitude; 

  std::string line;
  std::ifstream fs;

  x = x - uiData.GlobalOffset_X; 
  y = y - uiData.GlobalOffset_Y; 
  temp_x = x * cos(uiData.UIMap_rotateDeg) - y * sin(uiData.UIMap_rotateDeg);
  temp_y = x * sin(uiData.UIMap_rotateDeg) + y * cos(uiData.UIMap_rotateDeg);
  x = temp_x - uiData.UIMap_Xoffset0;
  y = temp_y - uiData.UIMap_Yoffset0;
  x = round(x * uiData.UIMap_Scale) + uiData.UIMap_Margin + uiData.UIMap_Xoffset;
  y = round(y * uiData.UIMap_Scale) + uiData.UIMap_Margin;
  y = uiData.UIMap_Ymax - y + uiData.UIMap_Margin + uiData.UIMap_Yoffset;
  uiData.x_ui = x;
  uiData.y_ui = y;

}


//-----------------------------------------------------------------------------------------------------