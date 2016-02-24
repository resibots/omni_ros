cmake_minimum_required(VERSION 2.8.3)
project(omni_bringup)
find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS omni_driver)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})