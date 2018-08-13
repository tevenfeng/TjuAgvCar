#include<tju_car.h>
#include<serial_com.h>

TjuCar::TjuCar()
{
    // Receive max linear vel and max angular vel
	n.param<double>("max_linear_vel", MAX_LINEAR_VEL, 40);
	n.param<double>("max_angular_vel", MAX_ANGULAR_VEL, 100);

	// Receive serial port name from launch file
	std::string serial_port;
	n.param<std::string>("serial_port", serial_port, "/dev/ttyTHS2");
	port = new char[serial_port.length() + 1];
	strcpy(port, serial_port.c_str());

    // Receive axis_linear and axis_angular 
    // to get the vel of the specified axes
    n.param<int>("axis_linear", axis_lin, 1);
    n.param<int>("axis_angular", axis_ang, 0);
    sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &TjuCar::callback, this);
	
    fd = UART0_Open(fd, port);
    do {
        err = UART0_Init(fd, 115200, 0, 8, 1, 'N');
        printf("Set Port Exactly!\n");
    } while (FALSE == err || FALSE == fd);
}

void TjuCar::callback(const sensor_msgs::Joy::ConstPtr& Joy)
{
    char send_buf[10]={0xff,0xfe,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    geometry_msgs::Twist v;
    v.linear.x =Joy->axes[axis_lin]*MAX_LINEAR_VEL;
    v.angular.z =Joy->axes[axis_ang]*MAX_ANGULAR_VEL;

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
    ROS_INFO("a=b= %x, c=d= %x", ch_a, ch_c);
}
