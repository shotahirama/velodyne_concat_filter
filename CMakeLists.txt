cmake_minimum_required(VERSION 2.8.3)
project(velodyne_concat_filter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nodelet
  pcl_ros
  pluginlib
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  velodyne_pointcloud
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs message_filters nodelet pcl_ros pluginlib roscpp sensor_msgs tf2 tf2_ros tf2_sensor_msgs velodyne_pointcloud
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/concat_filter.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)