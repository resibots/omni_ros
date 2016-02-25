cmake_minimum_required(VERSION 2.8.3)
project(omni_driver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamixel_control_hw
  std_msgs
  std_srvs
  trajectory_msgs
  control_msgs
  actionlib_msgs
  actionlib
  tf
  geometry_msgs
  youbot_driver_ros_interface
)

##Needed for ros packages
catkin_package(CATKIN_DEPENDS roscpp dynamixel_control_hw std_msgs std_srvs trajectory_msgs control_msgs actionlib_msgs actionlib tf geometry_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(omni_control src/omni.cpp)
target_link_libraries(omni_control ${catkin_LIBRARIES})
add_dependencies(omni_control ${catkin_EXPORTED_TARGETS})

add_executable(omni_driver src/omni.cpp src/omni_driver.cpp)
target_link_libraries(omni_driver ${catkin_LIBRARIES})
add_dependencies(omni_driver ${catkin_EXPORTED_TARGETS})

install(TARGETS omni_control
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)