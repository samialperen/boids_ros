cmake_minimum_required(VERSION 2.8.3)
project(boids_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  message_generation
  nav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  OdometryArray.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ReturnInitials.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Reynolds.cfg
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES boids-ROS
 CATKIN_DEPENDS roscpp rospy dynamic_reconfigure message_runtime
#  DEPENDS system_lib
)

## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


