cmake_minimum_required(VERSION 2.8.3)
project(interactiv_marker_publisher)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    interactive_markers
    )


catkin_package(
INCLUDE_DIRS include
LIBRARIES interactiv_marker_publisher
CATKIN_DEPENDS 
    roscpp
    interactive_markers
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(interactiv_marker_publisher src/interactiv_marker_publisher.cpp include/interactiv_marker_publisher/interactiv_marker_publisher.h)