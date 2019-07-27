#ifndef C2E_H_
#define C2E_H_


#include <string>
#include <stdint.h>
#include <vector>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "ros/ros.h"
#include <math.h>
#include <cstdlib>

typedef enum
{
  undefined = 0,
  initialisation = 1,
  configuration = 2,
  idle = 3,
  rotated = 4,
  in_preparation = 5,
  ready = 6,
  ready_for_measurement = 7
} status_t;

typedef struct
{
  unsigned char can_data[13];
}CAN_MESSAGE;

/*!
* @class LMS1xx
* @brief Class responsible for communicating with LMS1xx device.
*
* @author Konrad Banachowicz
*/

class CAN_ETH
{
public:
  CAN_ETH();
  virtual ~CAN_ETH();

  /*!
  * @brief Connect to LMS1xx.
  * @param host LMS1xx host name or ip address.
  * @param port LMS1xx port number.
  */
  void connect(std::string host, int port = 2111);

  /*!
  * @brief Disconnect from LMS1xx device.
  */
  void disconnect();

  /*!
  * @brief Get status of connection.
  * @returns connected or not.
  */
  bool isConnected();
  
 
  /*!
  * @brief Get current status of LMS1xx device.
  * @returns status of LMS1xx device.
  */
 // status_t queryStatus();

  /*!
  * @brief Log into LMS1xx unit.
  * Increase privilege level, giving ability to change device configuration.
  */
 // void login();
  void sent_to_can(void);
  void receive_from_can(void);
std::vector<CAN_MESSAGE>  can_Tmessage;
std::vector<CAN_MESSAGE>  can_Rmessage;

protected:
  

  bool connected_;
  int socket_fd_;
};


class ODOM_PUB
{
public:
  ODOM_PUB();
  virtual ~ODOM_PUB();
 // void login();
  void receive_from_can(void);
  void PraseData( CAN_MESSAGE &canmessge);
  void gaussConvert(double& x1,double& y1)
  {
    // Earth Ellipsoid WGS84
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
    lon=(double)vel_position_y*2*M_PI/360/10000000;
    lat=(double)vel_position_x*2*M_PI/360/10000000;
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

  void set_params(ros::NodeHandle nh);

  void PID_params_change();

  void pub_cmd_down(void);
  void pub_cmd_down1(void);
  void pub_cmd_down2(void);
  void pub_set_up(void);
  void pub_odom(void);
  std::vector<CAN_MESSAGE>  can_Rmessage;
  ros::Publisher cmd_pub;
  ros::Publisher decision_sub ;
  ros::Publisher odom_pub;
  ros::Publisher  can_pub;
  ros::Publisher  can_pub1;
  ros::Publisher  can_pub2;
  ros::Publisher steering_pub;
  ros::Publisher target_speed_pub;
	ros::Publisher current_speed_pub;
  /*
  std_msgs::Float64  vel_position_x;
  std_msgs::Float64  vel_position_y;
  std_msgs::Float64  vel_speed;
  std_msgs::Float64  vel_heading;
  */

  long vel_position_x = 0;
  long vel_position_y = 0;
  double  vel_speed = 0;
  double  vel_heading = 0;
  double current_steering_angle = 0;
  int gps_state = 0;

  ros::Time cmdReceivedTime;

  short targetWorkMode = 0;
  short targetSteeringAngle = 0;
  short targetSteeringVelocity = 0;
  double targetSpeed = 0;
  short  b_pos_isValid = 0;
  short  x_ui = 0;
  short  y_ui = 0;
  double targetBrakePressure = 0;
  short tempWorkMode = 0;

  float path_number = 1;
  float stop_flag = 0;

  ros::Time lidarReceivedTime;
  ros::Time lidarAliveTime;
  ros::Time GPSAliveTime;
  int heart_count = 0;
  bool lidar_status = 0;

  ros::Time carReceivedTime;
  ros::Time carAliveTime;
  int car_counter = 0;
  bool car_status = 0;
  bool car_flag = 0;
  bool car_init = 0;
  double filter_rate0 = 0.9;
  double filter_rate1 = 0.5;

  const uint16_t SteeringIDArray[3] = 
  {
    	0x20,	//AS_Control
    	0x394,	//AS_Feedback
    	0x403,	//AS_Feed
  };
  const uint16_t EHBIDArray[3] = 
  {
    	0x501,	//EHB_Control_R
      0x511,  //EHB_Control_L
    	0x502,	//EHB_Feedback
  };
  const uint32_t VCUIDArray[1] =
  {
      0xCFF0811,
  };

  double pos_test_x = 1;
  double pos_test_y = 1;

  double Kp_th = 225;
  double Ki_th = 10;
  double Kd_th = 0;

  double Kp_b = 25;
  double Ki_b = 0;
  double Kd_b = 10;

  double Kp_th_1;
  double Ki_th_1;

  double Kp_b_1;

  double Kp_th_2;
  double Ki_th_2;

  double Kp_b_2;

  double Kp_th_3;
  double Ki_th_3;

  double Kp_b_3;

  double Kp_th_4;
  double Ki_th_4;

  double Kp_th_f;
  double Ki_th_f;

  double Kp_b_f;

};



typedef struct
{
  unsigned char can_info;
  unsigned long can_id;
  unsigned char can_data[8];
}CAN_STRUCT;



#endif /* LMS1XX_H_ */

