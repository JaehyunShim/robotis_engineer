################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(robotis_max_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotis_controller_msgs
  robotis_max_action_module_msgs
  cmake_modules
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_math
  open_cr_module
  robotis_max_arm_module
  robotis_max_head_module
	robotis_max_waist_module
  robotis_max_action_module
	robotis_max_walking_module
	robotis_max_online_walking_module
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
	roscpp 
	robotis_controller_msgs 
	robotis_max_action_module_msgs 
	cmake_modules 
	dynamixel_sdk 
	robotis_framework_common 
	robotis_device 
	robotis_controller 
	robotis_math 
	open_cr_module 
	robotis_max_arm_module 
	robotis_max_head_module 
	robotis_max_waist_module  
	robotis_max_action_module 
	robotis_max_walking_module 
	robotis_max_online_walking_module 
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/robotis_max_manager.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_beta src/robotis_max_manager_beta.cpp)
add_dependencies(${PROJECT_NAME}_beta ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_beta ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
