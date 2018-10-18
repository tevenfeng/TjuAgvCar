# Tju Automated Guided Vehicle<!-- omit in toc -->

This is the 'src' directory of the catkin workspace for the Tju Automated Guided Vehicle(TjuAgvCar).

*Joystick controlling explanation*  
- LT: Enabling motion
- RT: Disabling motion(for emergency brake)
- Left stick: Direction and speed controlling
- A: Starting recording
- B: Stopping recording
    - There may be some delay because of large amount writing/reading of disks. But the timestamp should be 0 if recording is stopped.


- [Environment](#environment)
	- [Hardware](#hardware)
	- [Software](#software)
- [Building the vehicle](#building-the-vehicle)
	- [FLASHING Jetson TX1 with Jetpack 3.2 or 3.2.1](#flashing-jetson-tx1-with-jetpack-32-or-321)
	- [BUILDING kernel and modules for Jetson TX1](#building-kernel-and-modules-for-jetson-tx1)
	- [INSTALLING ROS for Jetson TX1](#installing-ros-for-jetson-tx1)
	- [INSTALLING driver for sensors(R200, RpLidar A1 and USB camera)](#installing-driver-for-sensorsr200-rplidar-a1-and-usb-camera)
	- [INSTALLING essential libraries and frameworks](#installing-essential-libraries-and-frameworks)
	- [BUILDING a node with python3](#building-a-node-with-python3)

## Environment

### Hardware
- [Nvidia Jetson TX1 Developer Kit](https://developer.nvidia.com/embedded/buy/jetson-tx1-devkit)
- [Slamtec RPLIDAR A1](http://www.slamtec.com/en/Lidar/A1)
- [Intel Realsense R200](https://software.intel.com/en-us/realsense/previous)
- [Logitech C925e web camera](https://www.logitech.com/en-us/product/c925e-webcam)
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
	- [librealsense](https://github.com/jetsonhacks/installLibrealsenseTX1)
	- [Realsense ROS Package](https://github.com/tevenfeng/installRealSenseROSTX1)
	- [ROS Driver for V4L USB Cameras](https://github.com/ros-drivers/usb_cam)
	- Python 3.5.2
	- [TensorFlow Python3 spec for Jetson TX1](https://github.com/jetsonhacks/installTensorFlowJetsonTX)

- *SPECIAL THANKS to [西安邮电学院XNMS项目组](https://blog.csdn.net/tigerjb) for codes to control MiniBalance 4WD chassis*

## Building the vehicle

### FLASHING Jetson TX1 with Jetpack 3.2 or 3.2.1
In this part, what we're supposed to do is just flashing the Jetson TX1 with Jetpack 3.2 or 3.2.1 under Ubuntu 16.04. Instructions can be found [here](https://developer.download.nvidia.com/embedded/L4T/r28_Release_v2.0/GA/Docs/Jetson_TX1_and_TX2_Developer_Kits_User_Guide.pdf?pca2GDAXIzHkB_ckFujostmR_RYpt36NkYdoCFI9_ecvNhviL94o83LINGmit_IEDtLvE9pgD_l_CVjjIH8NeiMgInlOfUpk2_y-_HNk7aCKqYYQtQMLLiEk5rl3rO-xI2ifhKfHb_ntYKH_HCcZwP8wRptLOrG_0i7WbT3lUw00swhCL7T2DmUtTnle8spyzs656Fw).  

### BUILDING kernel and modules for Jetson TX1
In this part, we are going to build a custom kernel and some essential modules for our Jetson TX1, in case we're going to use the Intel Realsense R200 and RpLidar A1. In order to do so, we follow instructions provided by [JetsonHacks.com](https://www.jetsonhacks.com/2018/04/21/build-kernel-and-modules-nvidia-jetson-tx1/), and the github repository is [here](https://github.com/jetsonhacks/buildJetsonTX1Kernel). *Great thanks to JetsonHacks.com!*

With this step finished, our kernel shall be ready for the Intel Realsense R200 and RpLidar A1.

### INSTALLING ROS for Jetson TX1
In this part, we're going to install ROS on our Jetson TX1. With original instructions provided by [JetsonHacks.com](https://github.com/jetsonhacks/installROSTX1) we are able to install ROS outside of China. Due to some well-known reason, the above instructions is not usable in China. So I change the source mirror of ROS to 'mirror.umd.edu', instructions can be found [here](https://github.com/tevenfeng/installROSTX1).

### INSTALLING driver for sensors(R200, RpLidar A1 and USB camera)


### INSTALLING essential libraries and frameworks
- Numpy  
&emsp;&emsp;&emsp;&emsp;
- TensorFlow python3 spec for Jetson TX1  
&emsp;&emsp;&emsp;&emsp;
- OpenCV 3.4, both C++ and python3 binding  
&emsp;&emsp;&emsp;&emsp;We'd like to use OpenCV3.4 in our codes, so we have to install it first for both C++ and python3. Instructions can be found [here](https://github.com/jetsonhacks/buildOpenCVTX1).  
&emsp;&emsp;&emsp;&emsp;Actually there may be an error that we still can not import cv2 in python3 codes under ROS, because ROS, in default, will import cv2 for python2.7(all because ROS does not support python3). So we have to remove cv2.so in python2.7's dist-packages using the following command:
```
	sudo mv /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so.bk
```
&emsp;&emsp;&emsp;&emsp;And after that, we should tell ROS to use the 'cv2.so' version of our installed opecv3.4, which is /usr/local/lib/python3.5/dist-packages/cv2.cpython-35m-aarch64-linux-gnu.so. Using the following to make a symbolic link:
```
	cd /usr/local/lib/python3.5
	sudo mkdir site-packages # if site-packages not exists
	sudo ln -sf /usr/local/lib/python3.5/dist-packages/cv2.cpython-35m-aarch64-linux-gnu.so /usr/local/lib/python3.5/site-packages/cv2.so
```
&emsp;&emsp;&emsp;&emsp;Last step, add the following line to your .bashrc or .zshrc file.
```
	export PYTHONPATH = "/usr/local/lib/python3.5/site-packages:$PYTHONPATH"
```
&emsp;&emsp;&emsp;&emsp;With all the above steps we can now import cv2 in our python3 codes.
### BUILDING a node with python3