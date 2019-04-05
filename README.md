# Tju Automated Guided Vehicle<!-- omit in toc -->

This is the 'src' directory of the catkin workspace for the Tju Automated Guided Vehicle(TjuAgvCar).

- [Joystick controlling explanation](#joystick-controlling-explanation)
- [Environment](#environment)
	- [Hardware](#hardware)
	- [Software](#software)
- [Building the vehicle](#building-the-vehicle)
	- [FLASHING Jetson TX2 with Jetpack 3.2 or 3.2.1](#flashing-jetson-tx2-with-jetpack-32-or-321)
	- [BUILDING kernel and modules for Jetson TX2](#building-kernel-and-modules-for-jetson-tx2)
	- [INSTALLING ROS for Jetson TX2](#installing-ros-for-jetson-tx2)
	- [INSTALLING driver for sensors(R200, RpLidar A3 and USB camera)](#installing-driver-for-sensorsr200-rplidar-a3-and-usb-camera)
	- [INSTALLING essential libraries and frameworks](#installing-essential-libraries-and-frameworks)


## Joystick controlling explanation

<table border="8">
	<tr>
		<th>Button</th>
		<th>Function explanation</th>
	</tr>
	<tr>
		<th>LT</th>
		<th>Enabling motion</th>
	</tr>
	<tr>
		<th>RT</th>
		<th>Disabling motion(can not move anymore)</th>
	</tr>
	<tr>
		<th>Left joystick</th>
		<th>Controlling speed and direction</th>
	</tr>
	<tr>
		<th>A</th>
		<th>Starting data recorder</th>
	</tr>
	<tr>
		<th>B</th>
		<th>Stopping data recorder</th>
	</tr>
	<tr>
		<th>Start</th>
		<th>Starting Auto-Navigation Mode</th>
	</tr>
	<tr>
		<th>Back</th>
		<th>Stopping Auto-Navigation Mode</th>
	</tr>
</table>

## Environment

### Hardware
- [Nvidia Jetson TX2 Developer Kit](https://developer.nvidia.com/embedded/buy/jetson-tx2-devkit)
- [Slamtec RPLIDAR A3](http://www.slamtec.com/en/Lidar/A3)
- [Intel Realsense R200](https://software.intel.com/en-us/realsense/previous)
- [Logitech C925e web camera](https://www.logitech.com/en-us/product/c925e-webcam)
- [Logitech Gamingpad F710](https://www.logitechg.com/en-us/products/gamepads/f710-wireless-gamepad.html)
- [MiniBalance 4WD chassis](https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-15726392046.74.2a5133049HoKv4&id=549877260447)
- [Gowoops 150W DC-DC 10-32V to 12-35V Step Up Boost Converter Module](https://www.amazon.com/Gowoops-10-32V-Converter-Adjustable-Voltage/dp/B00J1X4XXM/ref=sr_1_5?ie=UTF8&qid=1534161677&sr=8-5&keywords=DC-DC+12-35)
- usb 3.0 hub

### Software
- Host side
	- Ubuntu 16.04.04 x64
	- [Jetpack 3.2 or Jetpack 3.2.1](https://developer.nvidia.com/embedded/jetpack-3_2_1)
- Jetson side
	- Nvidia L4T 28.2(included in Jetpack 3.2 or Jetpack 3.2.1)
	- [ROS Kinetic Kame arm64](http://wiki.ros.org/kinetic/Installation/Ubuntu)
	- [RPLIDAR ROS Package](https://github.com/robopeak/rplidar_ros)
	- [librealsense](https://github.com/jetsonhacks/installLibrealsenseTX2)
	- [Realsense ROS Package](https://github.com/tevenfeng/installRealSenseROSTX1)
	- [ROS Driver for V4L USB Cameras](https://github.com/ros-drivers/usb_cam)
	- Python 2.7.12
	- [TensorFlow Python2 spec for Jetson TX2](https://github.com/jetsonhacks/installTensorFlowJetsonTX2)

- *SPECIAL THANKS to [西安邮电学院XNMS项目组](https://blog.csdn.net/tigerjb) for codes to control MiniBalance 4WD chassis*

## Building the vehicle

### FLASHING Jetson TX2 with Jetpack 3.2 or 3.2.1
In this part, what we're supposed to do is just flashing the Jetson TX2 with Jetpack 3.2 or 3.2.1 under Ubuntu 16.04. Instructions can be found [here](https://developer.download.nvidia.com/embedded/L4T/r28_Release_v2.0/GA/Docs/Jetson_TX1_and_TX2_Developer_Kits_User_Guide.pdf?pca2GDAXIzHkB_ckFujostmR_RYpt36NkYdoCFI9_ecvNhviL94o83LINGmit_IEDtLvE9pgD_l_CVjjIH8NeiMgInlOfUpk2_y-_HNk7aCKqYYQtQMLLiEk5rl3rO-xI2ifhKfHb_ntYKH_HCcZwP8wRptLOrG_0i7WbT3lUw00swhCL7T2DmUtTnle8spyzs656Fw).  

### BUILDING kernel and modules for Jetson TX2
In this part, we are going to build a custom kernel and some essential modules for our Jetson TX2, in case we're going to use the Intel Realsense R200 and RpLidar A1. In order to do so, we follow instructions provided by [JetsonHacks.com](https://www.jetsonhacks.com/2018/07/05/jetson-tx2-build-kernel-for-l4t-28-2-1-updated/), and the github repository is [here](https://github.com/jetsonhacks/buildJetsonTX2Kernel). *Great thanks to JetsonHacks.com!*

With this step finished, our kernel shall be ready for the Intel Realsense R200 and RpLidar A3.

### INSTALLING ROS for Jetson TX2
In this part, we're going to install ROS on our Jetson TX1. With original instructions provided by [JetsonHacks.com](https://github.com/jetsonhacks/installROSTX2) we are able to install ROS outside of China. Due to some well-known reasons, the above instructions are not usable in China. So I change the source mirror of ROS to 'mirror.umd.edu', instructions can be found [here](https://github.com/tevenfeng/installROSTX1).

### INSTALLING driver for sensors(R200, RpLidar A3 and USB camera)


### INSTALLING essential libraries and frameworks
- Numpy  
&emsp;&emsp;&emsp;&emsp;
- TensorFlow python2.7 spec for Jetson TX2  
&emsp;&emsp;&emsp;&emsp;
- OpenCV 3.4, both C++ and python2 binding  
&emsp;&emsp;&emsp;&emsp;We'd like to use OpenCV3.4 in our codes, so we have to install it first for both C++ and python2. Instructions can be found [here](https://github.com/jetsonhacks/buildOpenCVTX2).  
