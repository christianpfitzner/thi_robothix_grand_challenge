cmake_minimum_required(VERSION 3.0.2)
project(tcp_position_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tcp_position_publisher
#  CATKIN_DEPENDS rospy std_srvs tf2_ros
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/tcp_position_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Testing ##
#############

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
