cmake_minimum_required(VERSION 2.8.3)
project(kitti_ros)

###########
## Build ##
###########

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

#############
## Package ##
#############

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  roslib
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  # pcl library
  pcl_conversions
)

find_package(PCL    REQUIRED)


############
## Catkin ##
############

catkin_package(
  DEPENDS PCL 

  CATKIN_DEPENDS 
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
)

# include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

###########
## Build ##
###########

################# GPS Odometry #################
# Data Modify
add_executable(${PROJECT_NAME}_main src/main.cpp)
target_link_libraries(${PROJECT_NAME}_main ${catkin_LIBRARIES} ${PCL_LIBRARIES})