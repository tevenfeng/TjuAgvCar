#include<tju_car.h>
#include<serial_com.h>

TjuCar::TjuCar()
{
	isShutdown = false;

    // Receive max linear vel and max angular vel
	n.param<double>("max_linear_vel", MAX_LINEAR_VEL, 40);
	n.param<double>("max_angular_vel", MAX_ANGULAR_VEL, 100);

	// Receive serial port name from launch file
	std::string serialPort;
	n.param<std::string>("serial_port", serialPort, "/dev/ttyTHS2");
	port = new char[serialPort.length() + 1];
	strcpy(port, serialPort.c_str());

    // Receive axis_linear and axis_angular 
    // to get the vel of the specified axes
    n.param<int>("axis_linear", axisLinear, 1);
    n.param<int>("axis_angular", axisAngular, 0);

	// Receive brake button
	n.param<int>("brake_button", brakeButton, 7);
	
	// Receive start button
	n.param<int>("start_button", startButton, 6);
	
    joySub = n.subscribe<sensor_msgs::Joy>("joy", 10, &TjuCar::joy_callback, this);
	lidarSub = n.subscribe<sensor_msgs::LaserScan>("scan", 1000, &TjuCar::lidar_callback, this);

    fd = UART0_Open(fd, port);
    do {
        err = UART0_Init(fd, 115200, 0, 8, 1, 'N');
        printf("Set Port Exactly!\n");
    } while (FALSE == err || FALSE == fd);
}

void TjuCar::joy_callback(const sensor_msgs::Joy::ConstPtr& Joy)
{
    char send_buf[10]={0xff,0xfe,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	brakeButtonValue = Joy->buttons[brakeButton];
	if(brakeButtonValue && !isShutdown){
		isShutdown = true;
	}

	startButtonValue = Joy->buttons[startButton];
	if(startButtonValue && isShutdown){
		isShutdown = false;	
	}

	geometry_msgs::Twist v;
	if(!isShutdown){
		v.linear.x =Joy->axes[axisLinear]*MAX_LINEAR_VEL;
		v.angular.z =Joy->axes[axisAngular]*MAX_ANGULAR_VEL;
	}else{
		v.linear.x = 0;
		v.angular.z = 0;	
	}

	// convert to 4 wheel velocity
	double v_a, v_c;
	v_a = v.linear.x - v.angular.z * 0.179f;
	v_c = v.linear.x + v.angular.z * 0.179f;

	int tmp;
	char ch_a, ch_c;
	char flag = 0x00;
	// a,b wheels
	if(v_a >= 0){
	        tmp = (int)(v_a + 0.5);
	        ch_a = (char)tmp;
	}
	else{
	        tmp = (int)-(v_a - 0.5);
	        ch_a = (char)tmp;

	        flag = flag|0x0c;
	}

	// c,d wheels
	if(v_c >= 0){
	        tmp = (int)(v_c + 0.5);
	        ch_c = (char)tmp;
	}
	else{
	        tmp = (int)-(v_c - 0.5);
	        ch_c = (char)tmp;

	        flag = flag|0x03;
	}

	send_buf[3] = ch_a;
	send_buf[4] = ch_a;
	send_buf[5] = ch_c;
	send_buf[6] = ch_c;
	send_buf[9] = flag;
	int len = UART0_Send(fd, send_buf, 10);
	if (len > 0)
	    printf("send data successful\n");
	else
	    printf("send data failed!\n");

	ROS_INFO("linear:%.3lf angular:%.3lf", v.linear.x, v.angular.z);
	ROS_INFO("buttons[7]=%d", brakeButtonValue);
}

void TjuCar::lidar_callback(const sensor_msgs::LaserScan::ConstPtr& Scan)
{
	int count = Scan->scan_time / Scan->time_increment;
    ROS_INFO("I heard a laser scan %s[%d]:", Scan->header.frame_id.c_str(), count);
    ROS_INFO("angle_range, %f, %f", RAD2DEG(Scan->angle_min), RAD2DEG(Scan->angle_max));
	
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(Scan->angle_min + Scan->angle_increment * i);
        ROS_INFO(": [%f, %f]", degree, Scan->ranges[i]);
    }
}
