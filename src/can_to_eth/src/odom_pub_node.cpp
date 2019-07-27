/*
 * LMS1xx.cpp
 *
 *  Created on: 09-08-2010
 *  Author: Konrad Banachowicz
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include <csignal>
#include <cstdio>
#include <can_to_eth/c2e.h>
#include "ros/ros.h"
#include <can_to_eth/can_data.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <stdint.h>
#include <math.h>
#include <string>

#include "std_msgs/UInt32.h"

//#include <Float64MultiArray.h>
//#include <Float64.h>

//#include "sensor_msgs/LaserScan.h"
/*
#define DEG2RAD M_PI/180.0


char TCPdataIP[100] = "192.168.0.30";
int TCPdataPort = 502;
char CANIP[100] = "192.168.0.8";
int CAN1Port = 20002;
int CAN2Port = 20006;
char InnerIP[100] = "127.0.0.1";
int CommPort = 6970;
*/

ODOM_PUB::ODOM_PUB() 
{
}

ODOM_PUB::~ODOM_PUB()
{
}

 int gate00,gate01,gate02,gate03;


void ODOM_PUB::PraseData( CAN_MESSAGE &canmessge)
{
  CAN_STRUCT Temp_can;
int t_state,t_heading,t_temp;
long t_latitude,t_logitude;
Temp_can.can_info = canmessge.can_data[0];

Temp_can.can_id = 0;
for(int i=1;i<5;i++)
{
  Temp_can.can_id <<= 8;
  Temp_can.can_id |= canmessge.can_data[i];

}


for(int i=0;i<8;i++)
{
  Temp_can.can_data[i] = canmessge.can_data[i+5];
}
 
/*   for debug*/
#ifdef  DEBUG_info

 ROS_INFO("PraseData: %d",Temp_can.can_info);

 ROS_INFO("PraseData: %d",Temp_can.can_id);


for(int i=0;i<8;i++)
{
  ROS_INFO("canmessge: %d",Temp_can.can_data[i]);

}


#endif

  
  switch(Temp_can.can_id)
  {
    case 0x40d:
    t_heading = 0;
     gps_state = Temp_can.can_data[3];
    t_heading = (((int)Temp_can.can_data[2])<<8) | Temp_can.can_data[1] ;
    vel_heading = (double)t_heading/100 ;
    // ROS_INFO("vel_heading: %.3f",vel_heading);
    GPSAliveTime = ros::Time::now();
    break;

    case 0x30d:
     t_latitude = 0;
    for(int i=3;i>=0;i--)
    {
        t_latitude <<= 8;
        t_latitude |= Temp_can.can_data[i];

    }
    vel_position_x = t_latitude;
    t_logitude = 0;
    for(int i=7;i>3;i--)
    {
        t_logitude <<= 8;
        t_logitude |= Temp_can.can_data[i];

    }
    vel_position_y = t_logitude;
    // ROS_INFO("t_latitude: %ld, t_logitude: %ld",t_latitude,t_logitude);
    GPSAliveTime = ros::Time::now();

    // low prass filter
    static double last_x = 0;
    static double last_y = 0;
    vel_position_x = filter_rate0 * last_x + (1-filter_rate0) * vel_position_x;
    vel_position_y = filter_rate0 * last_y + (1-filter_rate0) * vel_position_y;
    last_x = vel_position_x;
    last_y = vel_position_y;

    static double llast_x = 0;
    static double llast_y = 0;
    vel_position_x = filter_rate1 * llast_x + (1-filter_rate1) * vel_position_x;
    vel_position_y = filter_rate1 * llast_y + (1-filter_rate1) * vel_position_y;
    llast_x = vel_position_x;
    llast_y = vel_position_y;
  
    break;
    
    case 0x91:
    // ROS_INFO("received velocity!!!");
    // if(Temp_can.can_data[3]&0x80)
    // {
    gate00 = int(Temp_can.can_data[0]); // Driving Mode
    gate01 = int(Temp_can.can_data[1]); // acc
    gate02 = int(Temp_can.can_data[2]); // dec
    gate03 = int(Temp_can.can_data[7]); // SpeedCounter
    car_counter += gate03;
    if(car_counter > 255)
      car_counter = 0;

     t_temp = (((int)Temp_can.can_data[3]<<8) | Temp_can.can_data[4]) ;
      //  vel_speed =  ((double)t_temp)/10/3.6;

       vel_speed =  ((double)t_temp)/256/3.6;
      // ROS_INFO("vel_speed: %f",vel_speed*3.6);
    // }
    current_steering_angle = (double)((Temp_can.can_data[5]<<8) | Temp_can.can_data[6]);
    break;

    case 0x601:
      path_number = Temp_can.can_data[0]+1;
      ROS_INFO("path_number: %.0f",path_number);
      break;

    case 0x701:
      stop_flag = Temp_can.can_data[0];
      break;
  }
  
    
}

void ODOM_PUB::PID_params_change()
{
  float v = vel_speed*3.6;
  if(v > 20)
  {
    Kp_th_f = Kp_th_1;
    Ki_th_f = Ki_th_1;
    Kp_b_f = Kp_b_2;
  }
  else if((v>10&&v<16))
  {
    Kp_th_f = Kp_th_2;
    Ki_th_f = Ki_th_2;
    Kp_b_f = Kp_b_3;
  }
  else if((v>17&&v<18))
  {
    Kp_th_f = Kp_th_2;
    Ki_th_f = Ki_th_2;
    Kp_b_f = Kp_b_3;
  }
  else if(v>7&&v<10)
  {
    Kp_th_f = Kp_th_3;
    Ki_th_f = Ki_th_3;
    Kp_b_f = Kp_b_3;
  }
  else if(v>18&&v<20)
  {
    Kp_b_f = Kp_b_3;
  }
  else if(v>25)
  {
    Kp_b_f = Kp_b_1;
  }
  else
  {
    Kp_th_f = Kp_th;
    Ki_th_f = Ki_th;
    Kp_b_f = Kp_b-0.2;
  }

}

void ODOM_PUB::pub_cmd_down(void)//(ros::Publisher &Temp_pub,ODOM_PUB *cmd_rev)
{
  static unsigned char counter=0;
  can_to_eth::can_data pub_data;

  // for direct control
  if(tempWorkMode == 1)
    targetWorkMode = 2;
  
  counter++;  

  PID_params_change();

  memset(&pub_data.can_data[0],0,13);
  // targetSpeeed & targetWorkMode
  pub_data.can_data[0] = 8;
  pub_data.can_data[1] = 0;
  pub_data.can_data[2] = 0;
  pub_data.can_data[3] = 0x05;
  pub_data.can_data[4] = 0x00;
  pub_data.can_data[5] = targetWorkMode;
  pub_data.can_data[6] = 2; //low level filter
  pub_data.can_data[7] = static_cast<uint8_t>((static_cast<uint16_t>(targetSpeed*10+5000)>>8) & 0xff);
  pub_data.can_data[8] = static_cast<uint8_t>(static_cast<uint16_t>(targetSpeed*10+5000)&0xff);
  pub_data.can_data[9] = Ki_th_f; //Ki_th
  pub_data.can_data[10] = Kp_th_f; //Kp_th
  pub_data.can_data[11] = Ki_b; //Ki_b
  pub_data.can_data[12] = Kp_b_f; //Kp_b
  can_pub.publish(pub_data);

  memset(&pub_data.can_data[0],0,13);
  // targetSteeringAngle
  pub_data.can_data[0] = 8;
  pub_data.can_data[1] = 0;
  pub_data.can_data[2] = 0;
  pub_data.can_data[3] = 0x04;
  pub_data.can_data[4] = 0xff;

  ROS_INFO("current_steering_angle: %.3f",current_steering_angle);
  ROS_INFO("targetSteeringAngle: %d",targetSteeringAngle/10);
  ROS_INFO("gps_state: %d\n",gps_state);

  pub_data.can_data[6]=static_cast<uint8_t>((static_cast<int16_t>((targetSteeringAngle/10+2048)*16) >> 8) & 0xff);;
  pub_data.can_data[7]=static_cast<uint8_t>((static_cast<int16_t>((targetSteeringAngle/10+2048)*16)) & 0xff);
  pub_data.can_data[8]=targetSteeringVelocity&0xff;
  pub_data.can_data[9] = Kd_th; //Kd_th
  pub_data.can_data[10] = Kd_b; //Kd_b
  pub_data.can_data[12] = counter;

  can_pub.publish(pub_data);


  // memset(&pub_data.can_data[0],0,13);
  // // hmi
  // pub_data.can_data[0] = 8;
  // pub_data.can_data[1] = 0;
  // pub_data.can_data[2] = 0;
  // pub_data.can_data[3] = 0x05;
  // pub_data.can_data[4] = 0x01;
  // pub_data.can_data[5] = b_pos_isValid;
  // pub_data.can_data[7] = x_ui>>8;
  // pub_data.can_data[6] = x_ui&0xff;
  // pub_data.can_data[9] = y_ui>>8;
  // pub_data.can_data[8] = y_ui&0xff;

  // pub_data.can_data[12] = counter;

  // can_pub.publish(pub_data);
  ROS_INFO("gate state **00: %d **01: %d **02: %d **03: %d",gate00,gate01,gate02,gate03);   
  // ROS_INFO("vel_speed: %f",vel_speed*3.6);
  ROS_INFO("vel_speed: %.1f km/h",vel_speed*3.6);
  ROS_INFO("targetWorkMode: %d, tempWorkMode: %d, ",targetWorkMode,tempWorkMode);
  ROS_INFO("targetSpeed: %.1f km/h", targetSpeed);
  ROS_INFO("targetBrakePressure: %.1f",targetBrakePressure/10);

  /*
  targetWorkMode
  targetSpeed
  targetSteeringAngle
  targetSteeringVelocity
  b_pos_isValid
  x_ui
  y_ui
  */

  counter %= 255;

}

void ODOM_PUB::pub_cmd_down1(void)//(ros::Publisher &Temp_pub,ODOM_PUB *cmd_rev)
{
  static unsigned char counter=0;
  can_to_eth::can_data pub_data;
  ROS_INFO("Publishing direct control .....");
  counter++;
  double limit_upper = 2000;
  double limit_lower = -2000;

  targetSpeed = (targetSpeed>limit_lower) ? targetSpeed:limit_lower;
  targetSpeed = (targetSpeed<limit_upper) ? targetSpeed:limit_upper;

  memset(&pub_data.can_data[0],0,13);
  // AS_FEED
  pub_data.can_data[0] = 8;
  pub_data.can_data[1] = 0;
  pub_data.can_data[2] = 0;
  pub_data.can_data[3] = (SteeringIDArray[2] >> 8) & 0xFF;
  pub_data.can_data[4] = SteeringIDArray[2] & 0xFF;
  pub_data.can_data[6] = 0x10;
  can_pub2.publish(pub_data);

  memset(&pub_data.can_data[0],0,13);
  //AS_CONTROL
  pub_data.can_data[0] = 8;
  pub_data.can_data[1] = 0;
  pub_data.can_data[2] = 0;
  pub_data.can_data[3] = (SteeringIDArray[0] >> 8) & 0xFF;
  pub_data.can_data[4] = SteeringIDArray[0] & 0xFF;
  pub_data.can_data[6] = (tempWorkMode & 0x01) << 3;
	pub_data.can_data[9] = static_cast<uint8_t>((static_cast<int16_t>((-targetSteeringAngle/10+2048)*16) >> 8) & 0xff);
	pub_data.can_data[10] = static_cast<uint8_t>((static_cast<int16_t>((-targetSteeringAngle/10+2048)*16)) & 0xff);
	pub_data.can_data[11] = static_cast<uint8_t>(targetSteeringVelocity/4.0);
  can_pub2.publish(pub_data);

  memset(&pub_data.can_data[0],0,13);
  //EHB_Control_R
  pub_data.can_data[0] = 8;
  pub_data.can_data[1] = 0;
  pub_data.can_data[2] = 0;
  pub_data.can_data[3] = (EHBIDArray[0] >> 8) & 0xFF;
  pub_data.can_data[4] = EHBIDArray[0] & 0xFF;
  pub_data.can_data[5] = tempWorkMode & 0x01;
	pub_data.can_data[6] = static_cast<uint8_t>(static_cast<int16_t>(targetBrakePressure) & 0xff );
  can_pub2.publish(pub_data);

  memset(&pub_data.can_data[0],0,13);
  //EHB_Control_L
  pub_data.can_data[0] = 8; // data length 8
  pub_data.can_data[1] = 0;
  pub_data.can_data[2] = 0;
  pub_data.can_data[3] = (EHBIDArray[1] >> 8) & 0xFF;
  pub_data.can_data[4] = EHBIDArray[1] & 0xFF;
  pub_data.can_data[5] = tempWorkMode & 0x01;
	pub_data.can_data[6] = static_cast<uint8_t>(static_cast<int16_t>(targetBrakePressure) & 0xff );
  can_pub2.publish(pub_data);

  // memset(&pub_data.can_data[0],0,13);
  // //VCU
  // pub_data.can_data[0] = 0x88; // extended, data length 8
  // pub_data.can_data[1] = (VCUIDArray[0] >> 24) & 0xFF;;
  // pub_data.can_data[2] = (VCUIDArray[0] >> 16) & 0xFF;;
  // pub_data.can_data[3] = (VCUIDArray[0] >> 8) & 0xFF;
  // pub_data.can_data[4] = VCUIDArray[0] & 0xFF;
  // pub_data.can_data[5] = static_cast<uint8_t>(tempWorkMode & 0x01 +  (tempWorkMode << 2) & 0x1100);
	// pub_data.can_data[6] = static_cast<uint8_t>(static_cast<int16_t>(limit_lower/4) & 0xff );
	// pub_data.can_data[7] = static_cast<uint8_t>((static_cast<int16_t>(limit_lower/4)>>8) & 0x0f + static_cast<int16_t>(limit_upper/4) & 0x0f );
	// pub_data.can_data[8] = static_cast<uint8_t>(((static_cast<int16_t>(limit_upper/4) & 0x111111110000)>>4) & 0xff );
	// pub_data.can_data[8] = static_cast<uint8_t>(static_cast<int16_t>(targetSpeed+5000) & 0xff );
	// pub_data.can_data[9] = static_cast<uint8_t>((static_cast<int16_t>(targetSpeed+5000)>>8) & 0xff );
  // can_pub1.publish(pub_data);

  /*
  targetWorkMode
  targetSpeed
  targetSteeringAngle
  targetSteeringVelocity
  b_pos_isValid
  x_ui
  y_ui
  */

  counter %= 255;

}

void ODOM_PUB::pub_set_up(void)
{
  std_msgs::Float64MultiArray decision_array;
	decision_array.data.reserve(3);
  decision_array.data.clear();
  decision_array.data.push_back(path_number);
  decision_array.data.push_back(stop_flag);
  decision_array.data.push_back(current_steering_angle);
  decision_sub.publish(decision_array);
}

void ODOM_PUB::pub_odom(void)
{

	    tf::TransformBroadcaster broadcaster;
    	double x = 0; 
      double y = 0;
      double th = 0;

	// velocity
       double vx =vel_speed; //e400
      //  double vx =vel_speed/4; //e200
       double vy = 0;
       double vth =0;

       //!!!!!!!!! TEST !!!!!!!!!!
      //  vel_position_x = pos_test_x;
      //  vel_position_y = pos_test_y;

       gaussConvert(x,y);
      //  x = vel_position_x/10000000;
      //  y = vel_position_y/10000000;
 //      x = ((double)vel_ctrl.vel_position_x)/10000000;
  //    y=  (double)vel_ctrl.vel_position_y/10000000;
        th = vel_heading;

        ros::Time current_time = ros::Time::now();
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th/180*M_PI);
	      geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.frame_id = "odom";
	      odom_trans.child_frame_id = "base_link";


	    	odom_trans.header.stamp = current_time; 
	    	odom_trans.transform.translation.x = x; 
	    	odom_trans.transform.translation.y = y; 
	    	odom_trans.transform.translation.z = 0.0;
      	odom_trans.transform.rotation = odom_quat;
        broadcaster.sendTransform(odom_trans);
	//	odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);
  
        ROS_INFO("GPS X: %.3f Y: %.3f, vel_heading: %.3f",x,y,vel_heading);

		//filling the odometry
	    	nav_msgs::Odometry odom;
	    	odom.header.stamp = current_time;
	    	odom.header.frame_id = "odom";
	    	odom.child_frame_id = "base_link";

		// position
	    	odom.pose.pose.position.x = x;
	    	odom.pose.pose.position.y = y;
	    	odom.pose.pose.position.z = gps_state;
        odom.pose.pose.orientation = odom_quat;



		//velocity
	    	odom.twist.twist.linear.x = vx;
	    	odom.twist.twist.linear.y = vy;
	    	odom.twist.twist.linear.z = 0.0;
	    	odom.twist.twist.angular.x = 0.0;
	    	odom.twist.twist.angular.y = 0.0;
	    	odom.twist.twist.angular.z = vth;

		

		// publishing the odometry and the new tf
	//	broadcaster.sendTransform(odom_trans);
	    	odom_pub.publish(odom);


}


void velcmd_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg, ODOM_PUB *cmd_rev)
{
 //can_to_eth::can_data temp_tx_canmessge=*msg;
 
       float Temp_cmd[9];

       for(int i=0;i<9;i++)      // 0   spped  1 head    2turn speed  
       Temp_cmd[i] = msg->data[i];
     // to can data

         // ROS_INFO("TargetSteeringANGLE: %.3f\n",Temp_cmd[2]);

       cmd_rev->targetWorkMode =(unsigned int)Temp_cmd[0];
       cmd_rev->targetSpeed =(double)(Temp_cmd[1]*3.6);
       cmd_rev->targetSteeringAngle = (double)Temp_cmd[2]*10; //actual steering angle * 10
       cmd_rev->targetSteeringVelocity = ( short)Temp_cmd[3]/4;
       cmd_rev->b_pos_isValid = (unsigned int)Temp_cmd[4];
       cmd_rev->x_ui = (unsigned int)Temp_cmd[5];
       cmd_rev->y_ui = (unsigned int)Temp_cmd[6];
       cmd_rev->targetBrakePressure = (double)Temp_cmd[7];
       cmd_rev->tempWorkMode = (unsigned int)Temp_cmd[8];

       cmd_rev->cmdReceivedTime = ros::Time::now();

// ROS_WARN("velcmd_Callback  velcmd");

}




void CanRec_Callback(const can_to_eth::can_data::ConstPtr& msg,ODOM_PUB *port)
{
     can_to_eth::can_data temp_tx_canmessge=*msg;
    //ROS_WARN("CanRec_Callback  can_r_0");
     CAN_MESSAGE temp_canmessge;
     memcpy(temp_canmessge.can_data,(unsigned char*)&temp_tx_canmessge,13);
     port->can_Rmessage.push_back(temp_canmessge);

}

void lidarHeartCallback(const std_msgs::UInt32::ConstPtr& msg_ptr, ODOM_PUB* cmd_rev_ptr)
{
  
  static int last_heart_beat = 0;
  cmd_rev_ptr->lidar_status = 1;
  cmd_rev_ptr->heart_count = msg_ptr->data;
  // ROS_INFO("heart beat: %d\n",cmd_rev_ptr->heart_count);
  if(cmd_rev_ptr->heart_count == last_heart_beat)
  {
    cmd_rev_ptr->lidar_status = 0;
    
    // ROS_ERROR("Lidar DIED!!!");
  }
  else
  {
    cmd_rev_ptr->lidarAliveTime = ros::Time::now();
  }
  cmd_rev_ptr->lidarReceivedTime = ros::Time::now();
  last_heart_beat = cmd_rev_ptr->heart_count;

}

void carStatusCheck(ODOM_PUB* cmd_rev_ptr)
{
  ROS_INFO("car counter: %d\n",cmd_rev_ptr->car_counter);
  static int last_car_counter = 0;
  cmd_rev_ptr->car_status = 1;
  if(cmd_rev_ptr->car_init == 0)
  {
    cmd_rev_ptr->carAliveTime = ros::Time::now();
    cmd_rev_ptr->car_init = 1;
  }

  if(cmd_rev_ptr->car_counter == last_car_counter)
  {
    cmd_rev_ptr->car_status = 0;
    
    // ROS_ERROR("CAR DIED!!!");
  }
  else
  {
    cmd_rev_ptr->carAliveTime = ros::Time::now();
  }
  cmd_rev_ptr->carReceivedTime = ros::Time::now();
  last_car_counter = cmd_rev_ptr->car_counter;
}

void ODOM_PUB::set_params(ros::NodeHandle nh)
{
  ROS_INFO("----------- setting params -----------");
  nh.param("Kp_th", Kp_th, double(225));
	ros::param::get("~Kp_th",Kp_th);
  nh.param("Ki_th", Ki_th, double(10));
	ros::param::get("~Ki_th",Ki_th);
  nh.param("Kd_th", Kd_th, double(0));
	ros::param::get("~Kd_th",Kd_th);
  ROS_INFO("[throttle] PID: %.1f, %.1f, %.1f", Kp_th, Ki_th, Kd_th);

  Kp_th_1 = Kp_th+4.5;
  Ki_th_1 = Ki_th+0.5;

  Kp_th_2 = Kp_th+0.42;
  Ki_th_2 = Ki_th+0.05;

  Kp_th_3 = Kp_th+0.38;
  Ki_th_3 = Ki_th+0.06;

  Kp_th_4 = Kp_th-0.7;     //0.7  0.15
  Ki_th_4 = Ki_th+0.15;

  nh.param("Kp_b", Kp_b, double(25));
	ros::param::get("~Kp_b",Kp_b);
  nh.param("Ki_b", Ki_b, double(0));
	ros::param::get("~Ki_b",Ki_b);
  nh.param("Kd_b", Kd_b, double(10));
	ros::param::get("~Kd_b",Kd_b);
  ROS_INFO("[ brake  ] PID: %.1f, %.1f, %.1f", Kp_b, Ki_b, Kd_b);

  Kp_b_1 = Kp_b+3.0;
  Kp_b_2 = Kp_b+1.5;
  Kp_b_3 = Kp_b+0.1;
}

int main(int argc, char **argv)
{
  // parameters
  std::string host;
  std::string frame_id;
  
  ros::init(argc, argv, "vehicle_ctrl");
  ODOM_PUB vel_ctrl;
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  vel_ctrl.set_params(nh);

  ros::Subscriber decision_sub = nh.subscribe<std_msgs::Float64MultiArray>("decision_pub", 10,boost::bind(velcmd_Callback,_1,&vel_ctrl));
  // ros::Subscriber can_sub = nh.subscribe<can_to_eth::can_data>("can2net", 50,boost::bind(CanRec_Callback,_1,&vel_ctrl));
  ros::Subscriber can_sub = nh.subscribe<can_to_eth::can_data>("can2net2", 50,boost::bind(CanRec_Callback,_1,&vel_ctrl));
  ros::Subscriber lidar_heart_sub = nh.subscribe<std_msgs::UInt32>("HeartBeat", 50,boost::bind(lidarHeartCallback,_1,&vel_ctrl));

  vel_ctrl.odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  //  vel_ctrl.cmd_pub = nh.advertise<std_msgs::Float64MultiArray>("hmi_msg", 10);
  vel_ctrl.decision_sub =nh.advertise<std_msgs::Float64MultiArray>("decision_sub", 10);
  vel_ctrl.can_pub = nh.advertise<can_to_eth::can_data>("net2can", 10);
  vel_ctrl.can_pub1 = nh.advertise<can_to_eth::can_data>("net2can1", 10);
  vel_ctrl.can_pub2 = nh.advertise<can_to_eth::can_data>("net2can2", 10); // direct control publisher
  vel_ctrl.steering_pub = nh.advertise<std_msgs::Float64>("c_steering_angle", 10);
  vel_ctrl.target_speed_pub = n.advertise<std_msgs::Float64>("target_speed", 1);
	vel_ctrl.current_speed_pub = n.advertise<std_msgs::Float64>("current_speed", 1);
  std_msgs::Float64 steering_test; 
  std_msgs::Float64 target_speed_test;
  std_msgs::Float64 current_speed_test;


  ROS_WARN("odom_pub node");

  //unsigned char test[13]={5,0,0,6,0x78,1,2,3,4,5,0,0,0};
  CAN_MESSAGE temp_canmessge;

  can_to_eth::can_data temp_rx_canmessge;

  ros::Rate loop_rate(500);
  ros::Time old_time = ros::Time::now();
  vel_ctrl.GPSAliveTime = ros::Time::now();

  while (ros::ok())
  {
      ros::Time now_time = ros::Time::now();

     while(!vel_ctrl.can_Rmessage.empty())
     {
   //     ROS_WARN("receive can");
        temp_canmessge= vel_ctrl.can_Rmessage.front();
        memcpy((unsigned char*)&temp_rx_canmessge,temp_canmessge.can_data,13);
        vel_ctrl.PraseData(temp_canmessge);
        vel_ctrl.can_Rmessage.erase(vel_ctrl.can_Rmessage.begin());
       // can_pub.publish(temp_rx_canmessge);
     }

     if((now_time-old_time).toSec()>=0.05)
     {
       //pub odom
       //vel_ctrl.OdomTran(nh);
      ROS_INFO("---------------------\n");
      carStatusCheck(&vel_ctrl);

      if((vel_ctrl.lidar_status == 0||vel_ctrl.lidar_status == 1)
        &&(((ros::Time::now().toSec()-vel_ctrl.lidarReceivedTime.toSec())>1)
        ||(ros::Time::now().toSec()-vel_ctrl.lidarAliveTime.toSec()>1)))
      {
        // vel_ctrl.targetWorkMode = 1;
        ROS_ERROR("lidar DIED!!!");
      }

      if((ros::Time::now().toSec()-vel_ctrl.cmdReceivedTime.toSec())>1){
          vel_ctrl.targetWorkMode = 0;
          ROS_ERROR("decision DIED!!!");
      }

   		ROS_INFO("now: %.3f, car alive time: %.3f",ros::Time::now().toSec(), vel_ctrl.carAliveTime.toSec());
   		// ROS_INFO("now: %.3f, car Recei time: %.3f",ros::Time::now().toSec(), vel_ctrl.carReceivedTime.toSec());

   		ROS_INFO("now: %.3f, GPS alive time: %.3f",now_time.toSec(), vel_ctrl.GPSAliveTime.toSec());

      if((((ros::Time::now().toSec()-vel_ctrl.carReceivedTime.toSec())>5)
        ||(ros::Time::now().toSec()-vel_ctrl.carAliveTime.toSec()>5)))
      {
        vel_ctrl.targetWorkMode = 0;
        // if(vel_ctrl.car_flag == 0)
        // {
        //   vel_ctrl.car_flag = 1;
        // }
        // else if(vel_ctrl.car_flag == 1)
        // {
          // system("shutdown -h now");
          // ROS_WARN("car shutdown!!!!!");
        // }
        ROS_WARN("car DIED!!!");
        system("shutdown -h now");

      }

      if((now_time.toSec()-vel_ctrl.GPSAliveTime.toSec())>2)
      {
        vel_ctrl.targetWorkMode = 0;
        // if(vel_ctrl.car_flag == 0)
        // {
        //   vel_ctrl.car_flag = 1;
        // }
        // else if(vel_ctrl.car_flag == 1)
        // {
          // system("shutdown -h now");
          // ROS_WARN("car shutdown!!!!!");
        // }
        ROS_ERROR("GPS DIED!!!");
        // system("shutdown -h now");

      }

      vel_ctrl.pub_odom();

      steering_test.data = vel_ctrl.current_steering_angle;
      vel_ctrl.steering_pub.publish(steering_test);
      target_speed_test.data = vel_ctrl.targetSpeed;
      vel_ctrl.target_speed_pub.publish(target_speed_test);
      current_speed_test.data = vel_ctrl.vel_speed*3.6;
      vel_ctrl.current_speed_pub.publish(current_speed_test);

      vel_ctrl.pub_cmd_down();
      vel_ctrl.pub_cmd_down1();
      vel_ctrl.pub_set_up();
      old_time  = now_time;
       
     }

    

     ros::spinOnce();
     loop_rate.sleep();
    //  ros::spinOnce();
 
    }

  return 0;
}
