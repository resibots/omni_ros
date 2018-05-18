cmake_minimum_required(VERSION 2.8.3)
project(omni_controllers)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_msgs
  std_msgs
  control_toolbox
  controller_interface
  hardware_interface
  realtime_tools
  velocity_controllers
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PublishData.msg
  PublishMatrix.msg
  PolicyParams.msg
  PolicyInit.msg
)

# generate_messages(DEPENDENCIES std_msgs omni_controllers/PublishData)
generate_messages()

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
    message_runtime
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME}
  src/arm_speed_safe_controller.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(TestTopic src/TestTopic.cpp)
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(TestTopic ${catkin_LIBRARIES})

add_dependencies(TestTopic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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
