cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(dysturbance_ros_description VERSION 0.0.1 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED)

catkin_package()

# Installation
install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)