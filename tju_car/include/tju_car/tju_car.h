#include<sstream>
#include<iostream>
#include<string>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/Image.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<cv_bridge/cv_bridge.h>
#include<pthread.h>
#include"serial_com.h"

using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)

class TjuCar
{
public:
    TjuCar();

private:
    // mutex to avoid timestamp changed when read
    pthread_mutex_t mutex;

    // global variable to store the current velocity
	geometry_msgs::Twist current_v;

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
    void usbcam_callback(const sensor_msgs::Image::ConstPtr& msg);
	void realsense_callback(const sensor_msgs::Image::ConstPtr& msg);
    // subscriber for receiving joystick information
    ros::Subscriber joySub, lidarSub, usbCamSub, realsenseSub;
    // max linear velocity and max angular velocity
    double MAX_LINEAR_VEL, MAX_ANGULAR_VEL;

    int axisAngular,axisLinear, brakeButton, startButton, startRecordButton, stopRecordButton, startRosbagButton, stopRosbagButton;

    int brakeButtonValue, startButtonValue, startRecordButtonValue, stopRecordButtonValue, startRosbagButtonValue, stopRosbagButtonValue;

    bool isShutdown, isRecording, isRecordingRosbag;

    void convert2send(geometry_msgs::Twist v, char send_buf[]);
};

