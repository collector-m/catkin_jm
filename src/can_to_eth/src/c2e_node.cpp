#include <csignal>
#include <cstdio>
#include <can_to_eth/c2e.h>

#include <can_to_eth/can_data.h>
//#include <std_msgs/Float64.h>
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

// parameters
std::string host;
std::string frame_id;
int port1,port2;
bool flag_test;
bool flag_mode;

void can0_t_Callback(const can_to_eth::can_data::ConstPtr& msg, CAN_ETH *c2e)
{
  can_to_eth::can_data temp_tx_canmessge=*msg;
  CAN_MESSAGE temp_canmessge;
  memcpy(temp_canmessge.can_data,(unsigned char*)&temp_tx_canmessge,13);
  c2e->can_Tmessage.push_back(temp_canmessge);
  // ROS_WARN(" write can fifo");
}

bool can_connect(CAN_ETH& c2e, std::string host, int port)
{
  // can1 connect
    ROS_INFO_STREAM("Connecting to c2e at " << host);
    ROS_INFO("Port %d", port);

    c2e.connect(host, port);
    if (!c2e.isConnected())
    {
      ROS_WARN("Unable to connect to c2e, port %d, Retrying...", port);
      ros::Duration(0.5).sleep();
      return 1;
    }
    else
    {
      ROS_WARN(" connection port %d is ok !!!", port);
      return 0;
    }
}

void set_params(ros::NodeHandle& nh)
{
  ROS_INFO("----------- setting params -----------");
  nh.param("host", host, std::string("192.168.0.7"));
	ros::param::get("~host",host);

  nh.param("frame_id", frame_id, std::string("can_trans"));
	ros::param::get("~frame_id",frame_id);

  nh.param("port1", port1, int(4001));
  //  nh.param("port1", port1, int(20001));
	ros::param::get("~port1",port1);

  nh.param("port2", port2, int(4002));
  // nh.param("port2", port2, int(20005));
	ros::param::get("~port2",port2);

  nh.param("flag_test", flag_test, false);
	ros::param::get("~flag_test",flag_test);

  nh.param("flag_mode", flag_mode, false);
	ros::param::get("~flag_mode",flag_mode);
  ROS_INFO("flag_mode:%d",flag_mode);
  // nh.param<std::string>("host", host, "192.168.2.7");
  // nh.param<std::string>("frame_id", frame_id, "can_trans");
  // nh.param<int>("port1", port1, 20001);
  // nh.param<int>("port2", port2, 20005);
  // nh.param<bool>("flag_test", flag_test, 0);
  // nh.param<bool>("flag_mode", flag_mode, 0);
  // ROS_INFO("flag_mode %d",flag_mode);

}

int main(int argc, char **argv)
{
  // laser data
  //scanCfg cfg;
  //scanOutputRange outputRange;
  //scanDataCfg dataCfg;
  //sensor_msgs::LaserScan scan_msg;

  CAN_ETH c2e1; // can1
  CAN_ETH c2e2; // can2
  CAN_ETH c2e; // original can

  int counter1,counter2,counter;
  
  ros::init(argc, argv, "can_to_eth");
  ros::NodeHandle nh;
  // ros::NodeHandle nh("~");

  set_params(nh);

  ros::Publisher can_pub1 = nh.advertise<can_to_eth::can_data>("can2net1", 10);
  ros::Publisher can_pub2 = nh.advertise<can_to_eth::can_data>("can2net2", 10);

  ros::Subscriber can_sub1 = nh.subscribe<can_to_eth::can_data>("net2can1", 10,boost::bind(can0_t_Callback,_1,&c2e1));
  ros::Subscriber can_sub2 = nh.subscribe<can_to_eth::can_data>("net2can2", 10,boost::bind(can0_t_Callback,_1,&c2e2));

  if(flag_mode == 1)
      can_sub2 = nh.subscribe<can_to_eth::can_data>("net2can", 10,boost::bind(can0_t_Callback,_1,&c2e2));

// CAN_MESSAGE temp_canmessge={{1},{ 2,3,4},{ 5,6,7,8,9}};
  unsigned char test[13]={5,0,0,6,0x78,1,2,3,4,5,0,0,0};
  CAN_MESSAGE temp_canmessge;
  can_to_eth::can_data temp_rx_canmessge;

  ros::Rate loop_rate(500);

  ROS_WARN(" node run .");
  while (ros::ok())
  {
    // can connect
    bool flag1,flag2;
    flag1 = can_connect(c2e1,host,port1);
    if(flag1 == 1)
      continue;
    flag2 = can_connect(c2e2,host,port2);
    if(flag2 == 1)
      continue;

    counter = 0;
    counter1 = 0;
    counter2 = 0;
    // can receive and send
    while (ros::ok())
    {
      ros::Time start = ros::Time::now();
      
      // test[9]++;
      // memcpy((unsigned char*)&temp_canmessge, test, 13);
      // c2e1.can_Tmessage.push_back(temp_canmessge);
      // c2e2.can_Tmessage.push_back(temp_canmessge);

      // std::cout << "CANSEND MESSAGE: ";
      // for(int i = 0;i < 13;++i)
      // {
      //   std::cout << static_cast<uint16_t>(c2e1.can_Tmessage[0].can_data[i]) << " ";
      // }
      // std::cout << std::endl;

      // can send
      c2e1.sent_to_can();
      c2e2.sent_to_can();

      // can1 receive
      c2e1.receive_from_can();
      
      while(!c2e1.can_Rmessage.empty())
      {
          temp_canmessge = c2e1.can_Rmessage.front();
          memcpy((unsigned char*)&temp_rx_canmessge,temp_canmessge.can_data,13);
          c2e1.can_Rmessage.erase(c2e1.can_Rmessage.begin());
          can_pub1.publish(temp_rx_canmessge);
          if(flag_test == 1)
          {
            std::cout << "CAN1 MESSAGE: ";
            for(int i = 0;i < 13;++i)
            {
              std::cout << static_cast<uint16_t>(temp_canmessge.can_data[i]) << " ";
            }
            std::cout << " COUNTER1: " << counter1 << " COUNTER: " << counter << std::endl;
            counter1++;
            if(counter1>10000) counter1 = 0;
          }
      }

      // can2 receive
      c2e2.receive_from_can();
      
      while(!c2e2.can_Rmessage.empty())
      {
          temp_canmessge = c2e2.can_Rmessage.front();
          memcpy((unsigned char*)&temp_rx_canmessge,temp_canmessge.can_data,13);
          c2e2.can_Rmessage.erase(c2e2.can_Rmessage.begin());
          can_pub2.publish(temp_rx_canmessge);
          if(flag_test == 1){         
            std::cout << "CAN2 MESSAGE: ";
            for(int i = 0;i < 13;++i)
            {
              std::cout << static_cast<uint16_t>(temp_canmessge.can_data[i]) << " ";
            }
            std::cout << " COUNTER2: " << counter2 << " COUNTER: " << counter << std::endl;
            counter2++;
            if(counter2>10000) counter2 = 0;
          }     
      }

      ros::spinOnce();
      // loop_rate.sleep();
      counter++;
      if(counter>10000) counter = 0;
 
    }

  }

  return 0;
}
