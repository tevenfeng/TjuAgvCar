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

	int fd;

	int err;

	char* port;

    void callback(const sensor_msgs::Joy::ConstPtr& Joy);

    ros::NodeHandle n;

    ros::Subscriber sub;

    double MAX_LINEAR_VEL, MAX_ANGULAR_VEL;

    int axis_ang,axis_lin;
};

