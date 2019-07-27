//#include <zmqapi.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <sstream>
#include <stdio.h>
#include <iomanip>
#include <stdlib.h>
#include <errno.h>
// for zmq:
#include <zmq.h>
// for json:
#include <json/json.h>
#include <queue>

// for thread and timer:
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/timer.hpp>
// system TCP
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <netinet/in.h>

// #include <Modbus.h>
// #include <ErrorCode.h>

#define X_SIZE_MIN -20.0 // lateral
#define X_SIZE_MAX 20.0 // X-symmetric
#define Y_SIZE_MIN 0.0
#define Y_SIZE_MAX 80.0 // longitudinal
#define GRID_X_RESOLUTION 0.2
#define GRID_Y_RESOLUTION 0.2
#define GRID_INDEX_SIZE ((X_SIZE_MAX-X_SIZE_MIN)/GRID_X_RESOLUTION)*((Y_SIZE_MAX-Y_SIZE_MIN)/GRID_Y_RESOLUTION)
#define GRID_INDEX_X_SIZE (X_SIZE_MAX-X_SIZE_MIN)/GRID_X_RESOLUTION

boost::posix_time::ptime startTime;
boost::posix_time::time_duration duration;

class Jobs{
  public:
    explicit Jobs (boost::asio::io_service& io, std::vector<int>& receivingSocketList, std::vector<void*>& sendingSocketList);
    virtual ~Jobs();

    virtual void data_subscriber();
    virtual void can_subscriber();
    virtual void handler();
    virtual void publisher();

    void v_regulator(double& v0, double& v1);

    double v0,v1;
    double v0_ad,v1_ad;
    double v0_judge,v1_judge;
    uint8_t msg_data0[256];	//ad
    uint8_t buf_data0[2048];
	uint8_t msg_data1[256];	//da
    uint8_t buf_data1[2048];
    uint8_t msg_data2[256];	//switches
    uint8_t buf_data2[2048];

    uint8_t buf_can1[2048];
    uint8_t buf_can2[2048];

	double counter = 0;
    double soTargetSteeringAngle = 0;
    double soTargetSteeringVelocity = 100;
    double soTargetSpeed = 0;
    double soWorkMode = 0;
    double soSteeringControlMode;
    double soSpeedControlMode;
    double soTargetThrottle = 0;
    double soTargetBrakePressure = 0;
    double currentSpeed = 0;
    // switches
    uint16_t mode_steering = 0;
    uint16_t mode_pedal = 0;
    uint16_t mode_EHB = 0;
    uint16_t heart = 0; //indicate current speed receive status
    uint16_t ehb_footpress = 0;
    uint16_t mobileyeFlag = 0; //indicate mobileye receive status
    //speed control
    double throttle = 0;
    double last_throttle = 0;
    double brake_pressure = 0;
    double last_brake_pressure = 0;
    double speed_error = 0;
    double throttle_integral = 0;
    const double kp_t = 0.5;
    const double ki_t = 0;
    const double kp_b = 0.4;
    const double throttle_threshold = 40;
    const double integral_threshold = 10;
    const double speed_error_threshold = 2;

    //LKA parameters
    int lane_type[10] = {6,6,6,6,6,6,6,6,6,6,};
    int quality[10] = {0,0,0,0,0,0,0,0,0,0,};
    int model_degree[10] = {0,0,0,0,0,0,0,0,0,0,};
    double c0[10] = {0,0,0,0,0,0,0,0,0,0,};
    double c1[10] = {0,0,0,0,0,0,0,0,0,0,};
    double c2[10] = {0,0,0,0,0,0,0,0,0,0,};
    double c3[10] = {0,0,0,0,0,0,0,0,0,0,};
    double lateral_dist[10] = {0,0,0,0,0,0,0,0,0,0,}; //C0 Range:[-127,128], Unit:meter
    double curvature[10] = {0,0,0,0,0,0,0,0,0,0,}; //C2 Range:[-0.02,0.02], Unit:N/A
    double curvature_derivative[10] = {0,0,0,0,0,0,0,0,0,0,}; //C3 Range:[-0.00012,0.00012], Unit:N/A
    double slope[10] = {0,0,0,0,0,0,0,0,0,0,}; //C1 Range:[-0.357,0.357], Unit:radians, positive means steering towards the right
    //reference point defines the lateral position of the lane center at reference point distance
    double Ref_Point_Lateral_Pos = 0; //physical distance between camera and reference point 1 on lateral axis, Range:[-127.996,127.996], Unit:meters.
    double Ref_Point_dist = 0; //physical distance between reference point and camera, Range:[0,127.996], Unit:meters
    bool Ref_Point_validity = 0; //0:valid, 1:invalid.
    int next_markers_num = 0;

    //lane curve
    struct curve_point{
        double x;
        double y;
    };
    std::vector<curve_point> lane_curve;
    std::vector<uint16_t> lane_curve_num;
    void curve_calculator(std::vector<curve_point>& lane_curve, std::vector<uint16_t>& lane_curve_num, double* c0, double* c1, double* c2, double* c3);

    //can IDs
    const uint16_t IDArray[3] = 
    {
    	0x43B,	//ToPEDAL
    	0x44B,	//PEDAL_Fbk
    	0x45B,	//PADEL_AD
    };
    const uint16_t SteeringIDArray[3] = 
    {
    	0x20,	//AS_Control
    	0x394,	//AS_Feedback
    	0x403,	//AS_Feed
    };
    const uint16_t EHBIDArray[2] = 
    {
    	0x501,	//EHB_Control
    	0x502,	//EHB_Feedback
    };
    const uint16_t SpeedIDArray[1] = 
    {
    	0x320,	//Current_Speed
    };
    const uint16_t LKAIDArray[29] = 
    {
    	0x766,	//Left lane type validity and position; Left lane Curvature and Curvature Derivative
    	0x767,	//Left lane heading, view range signal
    	0x768,	//Right lane type validity and position; Right lane Curvature and Curvature Derivative
    	0x769,	//Right lane heading, view range signal
    	0x76a,	//Reference points position and distance
    	0x76b,	//Number of next lane markers reported
    	// 0x76c ~ 0x781: next 4 pairs of lane markers 
    	0x76c,	//next lane left a
    	0x76d,	//next lane left b
    	0x76e,	//next lane right a
    	0x76f,	//next lane right b
    	0x770,	
    	0x771,
    	0x772,
    	0x773,
    	0x774,
    	0x775,
    	0x776,
    	0x777,
    	0x778,
    	0x779,
    	0x77a,
    	0x77b,
    	0x77c,
    	0x77d,
    	0x77e,
    	0x77f,
    	0x780,
    	0x781,
    };
    //can send fcn
    void TransPedalToCan(uint8_t heart, const uint16_t* IDArray, double soTargetSpeed, double currentSpeed, double soTargetBrakePressure , double soTargetThrottle, uint16_t mode_pedal);
    void TransSteeringToCan(const uint16_t* IDArray, double soTargetSteeringAngle, double soTargetSteeringVelocity, uint8_t mode_steering);
    void TransADToCan(const uint16_t* IDArray, double v0, double v1, uint8_t heart);
    void TransEHBToCan(const uint16_t* IDArray, double soTargetBrakePressure, uint8_t mode_EHB);

  private:
    boost::asio::strand strand1;
    // boost::asio::strand strand2;
    // boost::asio::strand strand3;
    // boost::asio::strand strand4;

    boost::asio::deadline_timer timer1;
    // boost::asio::deadline_timer timer2;
    // boost::asio::deadline_timer timer3;
    // boost::asio::deadline_timer timer4;

    std::vector<int> rSocketList;
    std::vector<void*> cSocketList;
    std::vector<void*> sSocketList;

    std::vector<std::string> rStringList;
    std::vector<std::string> sStringList;
//    std::stringstream rSS; // receving string stream
//    std::string sString; // sending string

    boost::mutex mutex;

};

void Jobs::TransPedalToCan(uint8_t heart, const uint16_t* IDArray, double soTargetSpeed,double currentSpeed, double soTargetBrakePressure , double soTargetThrottle, uint16_t mode_pedal){
	int s_can1 = rSocketList[0];
	int s_can2 = rSocketList[1];
	int len1,len2;

	uint8_t can_data[256];
	can_data[0] = 0x08;
	can_data[1] = 0x00;
	can_data[2] = 0x00;
	can_data[3] = (IDArray[1] >> 8) & 0xFF;
	can_data[4] = IDArray[1] & 0xFF;
	can_data[5] = heart & 0xFF;
	can_data[6] = mode_pedal & 0x01;	
	can_data[7] = static_cast<uint8_t>(static_cast<uint16_t>(currentSpeed/0.58) & 0xFF);
	can_data[8] = static_cast<uint8_t>((static_cast<uint16_t>(currentSpeed/0.58) >> 8) & 0xFF);
	can_data[9] = static_cast<uint8_t>(static_cast<uint32_t>(soTargetSpeed) & 0xff);
	can_data[10] = static_cast<uint8_t>((static_cast<uint32_t>(soTargetSpeed) >> 8) & 0xff );
	can_data[11] = static_cast<uint8_t>(soTargetThrottle);
	can_data[12] = static_cast<uint8_t>(soTargetBrakePressure);
	len1 = send(s_can1,can_data,13,MSG_DONTWAIT);
	std::cout <<"sending to can1 (PEDAL_Fbk): " << len1 << std::endl;

}

void Jobs::TransSteeringToCan(const uint16_t* IDArray, double soTargetSteeringAngle, double soTargetSteeringVelocity, uint8_t mode_steering){
	int s_can1 = rSocketList[0];
	int s_can2 = rSocketList[1];
	int len1,len2;

	uint8_t can_data[256];
	can_data[0] = 0x08;
	can_data[1] = 0x00;
	can_data[2] = 0x00;
	can_data[3] = (IDArray[2] >> 8) & 0xFF;
	can_data[4] = IDArray[2] & 0xFF;
	can_data[5] = 0x00;
	can_data[6] = 0x10;
	can_data[7] = 0x00;
	can_data[8] = 0x00;
	can_data[9] = 0x00;
	can_data[10] = 0x00;
	can_data[11] = 0x00;
	can_data[12] = 0x00;
	len1 = send(s_can1,can_data,13,MSG_DONTWAIT);
	std::cout <<"sending to can1 (AS_Feed): " << len1 << std::endl;

	can_data[0] = 0x08;
	can_data[1] = 0x00;
	can_data[2] = 0x00;
	can_data[3] = (IDArray[0] >> 8) & 0xFF;
	can_data[4] = IDArray[0] & 0xFF;
	can_data[5] = 0x00;
	// mode_steering = 1;
	can_data[6] = (mode_steering & 0x01) << 3;
	can_data[7] = 0x00;
	can_data[8] = 0x00;
	can_data[9] = static_cast<uint8_t>((static_cast<int16_t>((-soTargetSteeringAngle+2048)*16) >> 8) & 0xff);
	can_data[10] = static_cast<uint8_t>((static_cast<int16_t>((-soTargetSteeringAngle+2048)*16)) & 0xff);
	soTargetSteeringVelocity = 250;
	can_data[11] = static_cast<uint8_t>(soTargetSteeringVelocity/4.0);
	can_data[12] = 0x00;
	len1 = send(s_can1,can_data,13,MSG_DONTWAIT);
	std::cout <<"sending to can1 (AS_Control): " << len1 << std::endl;

}

void Jobs::TransADToCan(const uint16_t* IDArray, double v0, double v1, uint8_t heart){

	int s_can1 = rSocketList[0];
	int s_can2 = rSocketList[1];
	int len1,len2;

	v_regulator(v0,v1);
	uint8_t can_data[256];
	can_data[0] = 0x08;
	can_data[1] = 0x00;
	can_data[2] = 0x00;
	can_data[3] = (IDArray[2] >> 8) & 0xFF;
	can_data[4] = IDArray[2] & 0xFF;
	can_data[5] = heart & 0xFF;
	can_data[6] = 0x00;
	can_data[7] = static_cast<uint8_t>(static_cast<int16_t>(v1/3.3*4095) & 0xFF);
	can_data[8] = static_cast<uint8_t>((static_cast<int16_t>(v1/3.3*4095) >> 8) & 0xFF);
	can_data[9] = static_cast<uint8_t>(static_cast<int16_t>(v0/3.3*4095) & 0xFF);
	can_data[10] = static_cast<uint8_t>((static_cast<int16_t>(v0/3.3*4095) >> 8) & 0xFF);
	can_data[11] = 0x00;
	can_data[12] = 0x00;
	len1 = send(s_can1,can_data,13,MSG_DONTWAIT);
	std::cout <<"sending to can1 (PADEL_AD): " << len1 << std::endl;

}

void Jobs::TransEHBToCan(const uint16_t* IDArray, double soTargetBrakePressure, uint8_t mode_EHB){
	if (soTargetBrakePressure > 50)
		soTargetBrakePressure = 50;

	int s_can1 = rSocketList[0];
	int s_can2 = rSocketList[1];
	int len1,len2;

	uint8_t can_data[256];
	can_data[0] = 0x08;
	can_data[1] = 0x00;
	can_data[2] = 0x00;
	can_data[3] = (IDArray[0] >> 8) & 0xFF;
	can_data[4] = IDArray[0] & 0xFF;
	can_data[5] = mode_EHB & 0x01;
	can_data[6] = static_cast<uint8_t>(static_cast<int16_t>(soTargetBrakePressure) & 0xff );
	can_data[7] = 0x00;
	can_data[8] = 0x00;
	can_data[9] = 0x00;
	can_data[10] = 0x00;
	can_data[11] = 0x00;
	can_data[12] = 0x00;
	len1 = send(s_can1,can_data,13,MSG_DONTWAIT);
	std::cout <<"sending to can1 (EHB_Control): " << len1 << std::endl;
}

void Jobs::v_regulator(double& v0, double& v1){
	if(v0 > 3) v0 = 3;
	if(v0 < 0.6) v0 = 0.6;
	if(v1 > 1.5) v1 = 1.5;
	if(v1 < 0.3) v1 = 0.3;
}

void Jobs::curve_calculator(std::vector<curve_point>& lane_curve, std::vector<uint16_t>& lane_curve_num, double* c0, double* c1, double* c2, double* c3){

    const uint16_t xIndexMax = (X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION - 1;
    const uint16_t yIndexMax = (Y_SIZE_MAX - Y_SIZE_MIN)/GRID_Y_RESOLUTION - 1;
    const uint32_t indexMax = (X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION * (Y_SIZE_MAX - Y_SIZE_MIN)/GRID_Y_RESOLUTION - 1;
    int32_t idx,index;
    int32_t i0,i,j,x_temp,y_temp;
    std::cout << "xIndexMax " << xIndexMax << std::endl;
    std::cout << "yIndexMax " << yIndexMax << std::endl;

    for(int32_t i=yIndexMax/4-1; i>=0; i--){//0
//            cout<<i<<": ";
        std::cout<<"||";
        for(int32_t j=0; j<xIndexMax; j++){

            int32_t idx = i*xIndexMax + j;
            double y = i*GRID_Y_RESOLUTION;

            //Formula: x = c3*y.^3 + c2*y.^2 + c1*y + c0
            double x0 = c3[0]*y*y*y + c2[0]*y*y + c1[0]*y + c0[0] + 20;
            int32_t j0 = floor(x0/GRID_X_RESOLUTION);

            double x1 = c3[1]*y*y*y + c2[1]*y*y + c1[1]*y + c0[1] + 20;
            int32_t j1 = floor(x1/GRID_X_RESOLUTION);

//                cout<<idx<<' ';
            if( (j0 == j || j1 == j) &&(j>(X_SIZE_MAX-7.5)/GRID_X_RESOLUTION)&&(j<(X_SIZE_MAX+7.5)/GRID_X_RESOLUTION)) std::cout<<"o";
            else if((j>(X_SIZE_MAX-7.5)/GRID_X_RESOLUTION)&&(j<(X_SIZE_MAX+7.5)/GRID_X_RESOLUTION)) std::cout<<"*";
        }
        std::cout<<"||"<<std::endl;
    }

}