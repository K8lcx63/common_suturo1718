cmake_minimum_required(VERSION 2.8.3)
project(marker_publisher)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	geometry_msgs
	visualization_msgs
	)


catkin_package(
INCLUDE_DIRS include
LIBRARIES marker_publisher
CATKIN_DEPENDS 
	roscpp
	geometry_msgs
	visualization_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(marker_publisher src/marker_publisher)