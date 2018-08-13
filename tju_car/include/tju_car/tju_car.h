#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<iostream>
using namespace std;

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
	// callback function for joystick
    void callback(const sensor_msgs::Joy::ConstPtr& Joy);
	// tju_car node
    ros::NodeHandle n;
	// subscriber for receiving joystick information
    ros::Subscriber sub;
	// max linear velocity and max angular velocity
    double MAX_LINEAR_VEL, MAX_ANGULAR_VEL;

    int axisAngular,axisLinear, brakeButton, startButton;

	int brakeButtonValue, startButtonValue;

	bool isShutdown;
};

