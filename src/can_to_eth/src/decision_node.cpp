#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
 
#include <boost/thread/thread.hpp>
#include <geometry_msgs/Twist.h>

#include <csignal>
#include <cstdio>
#include <can_to_eth/c2e.h>
#include <can_to_eth/can_data.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
 
#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
 
class SmartCarKeyboardTeleopNode
{
    private:
        double walk_vel_;
        double run_vel_;
        double yaw_rate_;
        double yaw_rate_run_;

        int targetWorkMode = 0;
        int tempWorkMode = 0;
        double targetSpeed = 0;
        double targetSteeringAngle = 0;
        double targetSteeringVelocity = 0;
        double brake_pressure = 0;
        
        geometry_msgs::Twist cmdvel_;
        ros::NodeHandle n_;
        ros::Publisher pub_;
        ros::Publisher decision_pub;
        std_msgs::Float64MultiArray decision_array;

 
    public:
        SmartCarKeyboardTeleopNode()
        {
            // pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            decision_pub =n_.advertise<std_msgs::Float64MultiArray>("decision_pub", 1);
            
            ros::NodeHandle n_private("~");
            n_private.param("walk_vel", walk_vel_, 0.5);
            n_private.param("run_vel", run_vel_, 1.0);
            n_private.param("yaw_rate", yaw_rate_, 1.0);
            n_private.param("yaw_rate_run", yaw_rate_run_, 1.5);

            decision_array.data.reserve(8);
	          decision_array.data.clear();
        }
        
        ~SmartCarKeyboardTeleopNode() { }
        void keyboardLoop();
        
        void stopRobot()
        {
            targetWorkMode = 1;
            targetSpeed = 0;
            targetSteeringAngle = 0;
            targetSteeringVelocity = 100;
            brake_pressure = 0;
        }
};
 
// SmartCarKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;
 
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "decision_node", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  ROS_INFO_STREAM("decision_node start ");

  SmartCarKeyboardTeleopNode tbk;
    
  // boost::thread t = boost::thread(boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));

  ros::Rate loop_rate(50);
  ros::Time old_time = ros::Time::now();
  while (ros::ok())
  {
    // ROS_INFO_STREAM("decision_node start ");

    ros::Time now_time = ros::Time::now();

    ros::spinOnce();

    tbk.keyboardLoop();
    
    // t.interrupt();
    // t.join();
    // tbk.stopRobot();
    // tcsetattr(kfd, TCSANOW, &cooked);

    // loop_rate.sleep();
  }
  tbk.stopRobot();
  return 0;
}

void SmartCarKeyboardTeleopNode::keyboardLoop()
{
    char c;
    double max_tv = walk_vel_;
    double max_rv = yaw_rate_;
    bool dirty = false;
    int turn = 0;
    int i=0,j=0;
    kfd = STDIN_FILENO;
    
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    
    // tcgetattr(STDIN_FILENO, &cooked);
    // raw = cooked;
    // raw.c_lflag &= (~ICANON & ~ECHO);
    // tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    
    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");
    
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    
    for(;;)
    {
        // boost::this_thread::interruption_point();
        
        // get the next event from the keyboard
        int num;
        
        // if ((num = poll(&ufd, 1, 250)) < 0)
        // {
        //     perror("poll():");
        //     return;
        // }
        // else if(num > 0)
        // {
        //     if(read(kfd, &c, 1) < 0)
        //     {
        //         perror("read():");
        //         return;
        //     }
        //     printf("NUM: %d \n",num);
        // }
        // else
        // {
        //     if (dirty == true)
        //     {
        //         stopRobot();
        //         dirty = false;
        //     }
            
        //     continue;
        // }

        c = getchar();
        
        switch(c)
        {
            case KEYCODE_W:
                targetSpeed = targetSpeed + 2/3.6;
                dirty = true;
                break;
            case KEYCODE_S:
                targetSpeed = targetSpeed - 2/3.6;
                dirty = true;
                break;
            case KEYCODE_A:
                targetSteeringAngle = targetSteeringAngle + 10;
                dirty = true;
                break;
            case KEYCODE_D:
                targetSteeringAngle = targetSteeringAngle - 10;
                dirty = true;
                break;
                
            case KEYCODE_W_CAP:
                brake_pressure = brake_pressure + 1;
                dirty = true;
                break;
            case KEYCODE_S_CAP:
                brake_pressure = brake_pressure - 1;
                dirty = true;
                break;
            case KEYCODE_A_CAP:
                targetSteeringAngle = targetSteeringAngle + 20;
                dirty = true;
                break;
            case KEYCODE_D_CAP:
                targetSteeringAngle = targetSteeringAngle - 20;
                dirty = true;
                break;
            case 'q':
                tempWorkMode = 1;
                targetWorkMode = 1;
                break;
            case 'e':
                tempWorkMode = 1;
                targetWorkMode = 1;    
                targetSpeed = 0;
                brake_pressure = 20;
                break;
            case 'E':
                tempWorkMode = 1;
                targetWorkMode = 1;    
                targetSpeed = 0;
                brake_pressure = 20; 
                break;                      
            default:
                tempWorkMode = 0;
                targetWorkMode = 0;
                targetSpeed = 0;
                targetSteeringAngle = 0;
                targetSteeringVelocity = 100;
                brake_pressure = 0;
                dirty = false;
        }
        // cmdvel_.linear.x = targetSpeed * max_tv;
        // cmdvel_.angular.z = turn * max_rv;
        // pub_.publish(cmdvel_);

        targetSpeed = (targetSpeed < -100/3.6) ? -100/3.6 : targetSpeed;
        targetSpeed = (targetSpeed > 100/3.6) ? 100/3.6 : targetSpeed;
        targetSteeringAngle = (targetSteeringAngle < -540) ? -540 : targetSteeringAngle;
        targetSteeringAngle = (targetSteeringAngle > 540) ? 540 : targetSteeringAngle;
        brake_pressure = (brake_pressure < 0) ? 0 : brake_pressure;
        brake_pressure = (brake_pressure > 100) ? 100 : brake_pressure;

        decision_array.data.clear();
        decision_array.data.push_back(targetWorkMode);
        decision_array.data.push_back(targetSpeed);
        decision_array.data.push_back(targetSteeringAngle); 
        decision_array.data.push_back(targetSteeringVelocity);
        decision_array.data.push_back(0);
        decision_array.data.push_back(0); 
        decision_array.data.push_back(0);
        decision_array.data.push_back(brake_pressure);
        decision_array.data.push_back(tempWorkMode);
        decision_pub.publish(decision_array);
    }
}
