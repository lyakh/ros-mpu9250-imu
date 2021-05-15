cmake_minimum_required(VERSION 2.8.3)
project(mpu9250_imu)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
# add_definitions(-std=c++0x)
# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mpu9250_imu
  CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/mpu9250_9dof_imu_node.cpp)
add_library(mpu9250_imu src/mpu9250_imu.cpp src/AK8963_Magnetometer.cpp src/I2CBus.cpp src/MPU9250_Acc_Gyro.cpp)
target_link_libraries(mpu9250_imu ${catkin_LIBRARIES} i2c pigpio)
#add_dependencies(mpu9250_imu mpu9250_9dof_imu_generate_messages_cpp)

add_executable(mpu9250_imu_test src/mpu9250_main.cpp)
target_link_libraries(mpu9250_imu_test ${catkin_LIBRARIES} mpu9250_imu i2c pigpio)

install(TARGETS mpu9250_imu mpu9250_imu_test
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)
