cmake_minimum_required(VERSION 2.8.3)
project(omni_controllers)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  std_msgs
  control_toolbox
  controller_interface
  hardware_interface
  realtime_tools
  velocity_controllers
)
find_package(Eigen3 REQUIRED)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    control_msgs
    std_msgs
    control_toolbox
    controller_interface
    hardware_interface
    realtime_tools
    velocity_controllers
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME}
  src/arm_speed_safe_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES omni_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
