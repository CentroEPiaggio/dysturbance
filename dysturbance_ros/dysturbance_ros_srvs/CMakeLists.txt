cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(dysturbance_ros_srvs VERSION 0.0.1 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    std_srvs
    dysturbance_ros_msgs
    message_generation
)

add_service_files(
  DIRECTORY srv
  FILES

)

generate_messages(
  DEPENDENCIES
    std_srvs
    dysturbance_ros_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    std_srvs
    dysturbance_ros_msgs
    message_runtime
)

# Installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)