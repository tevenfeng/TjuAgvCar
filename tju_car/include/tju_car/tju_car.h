#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<sensor_msgs/LaserScan.h>
#include<iostream>
using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)

class TjuCar
{
public:
    TjuCar();

private:
	// file descriptor for opening connection to ttyTHS2
	int fd;
	// error message
	int err;
	// string containing the port "ttyTHS2"
	char* port;
	// tju_car node
    ros::NodeHandle n;
	// callback functions
    void joy_callback(const sensor_msgs::Joy::ConstPtr& Joy);
	void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& Scan);
	// subscriber for receiving joystick information
    ros::Subscriber joySub, lidarSub;
	// max linear velocity and max angular velocity
    double MAX_LINEAR_VEL, MAX_ANGULAR_VEL;

    int axisAngular,axisLinear, brakeButton, startButton;

	int brakeButtonValue, startButtonValue;

	bool isShutdown;
};

