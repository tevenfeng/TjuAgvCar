cmake_minimum_required(VERSION 2.8.3)
project(tju_car)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs sensor_msgs rosconsole)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

set(SERIAL_COM "serial_com")
set(TJU_CAR_INCLUDE "./include/tju_car/")
set(RPLIDAR_SDK_PATH "./rplidar_sdk/")
FILE(GLOB RPLIDAR_SDK_SRC 
	"${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
	"${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
	"${RPLIDAR_SDK_PATH}/src/*.cpp"
)

include_directories(include
	${catkin_INCLUDE_DIRS}
	${TJU_CAR_INCLUDE}
	${RPLIDAR_SDK_PATH}/include
	${RPLIDAR_SDK_PATH}/src
)

## Build the tju_car library
add_library(${PROJECT_NAME} src/tju_car.cpp)
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)

## Build the serial_com library
add_library(${SERIAL_COM} src/serial_com.cpp)
target_link_libraries(${SERIAL_COM}
	${catkin_LIBRARIES}
)

## build tju_car_node to control the car with 
## joystick or commands provided by jetson tx1
add_executable(${PROJECT_NAME}_node nodes/tju_car_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
	${PROJECT_NAME}
	${SERIAL_COM}
	${catkin_LIBRARIES}
)

## build rplidar_node to make the rplidar scan the environment
add_executable(rplidar_node nodes/rplidar_node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(rplidar_node
	${catkin_LIBRARIES}
)

## build rplidar_client_node to receive scanning 
## data from rplidar_node
add_executable(rplidar_client_node nodes/rplidar_client.cpp)
target_link_libraries(rplidar_client_node
	${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
	TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node ${SERIAL_COM} rplidar_node rplidar_client_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch sdk
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	USE_SOURCE_PERMISSIONS
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
