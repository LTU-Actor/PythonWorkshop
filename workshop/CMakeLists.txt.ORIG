cmake_minimum_required(VERSION 3.0.2)
project(workshop)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dbw_polaris_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/AutoDriveDynCfg.cfg
  cfg/CrosswalkDynCfg.cfg
  cfg/BarrelDynCfg.cfg
  cfg/BarrelLidarDynCfg.cfg
  cfg/LineFollowDynCfg.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamic_tutorials
#  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


