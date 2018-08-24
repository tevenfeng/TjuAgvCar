#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<tju_car.h>
#include<iostream>
using namespace std;

int main(int argc,char** argv)
{
    ros::init(argc, argv, "tju_car");
    TjuCar tjuCar; ros::spin();
    return 0;
}
