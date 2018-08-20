# Tju Automated Guided Vehicle<!-- omit in toc -->

This is the 'src' directory of the catkin workspace for the Tju Automated Guided Vehicle(TjuAgvCar).

- [Environment](#environment)
	- [Hardware](#hardware)
	- [Software](#software)
- [Building the vehicle](#building-the-vehicle)
	- [FLASHING Jetson TX1 with Jetpack 3.2 or 3.2.1](#flashing-jetson-tx1-with-jetpack-32-or-321)
	- [](#)

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

### 
