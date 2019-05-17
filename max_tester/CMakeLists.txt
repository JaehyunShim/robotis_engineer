cmake_minimum_required(VERSION 2.8.3)
project(max_tester)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  dynamixel_sdk
	max_kdl
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    dynamixel_sdk 
    max_kdl 
)

include_directories(
 include 
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/max_tester.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

