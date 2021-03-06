cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(dysturbance_ros_hardware_interface VERSION 0.0.1 LANGUAGES CXX)

set(NIDAQmx_INCLUDE_DIRS $ENV{niextccompilersupp}include)
set(NIDAQmx_LIBRARIES $ENV{niextccompilersupp}lib64/msvc)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    hardware_interface
    ros_opcua_srvs
    dysturbance_ros_msgs
)

include_directories(include
  ${NIDAQmx_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    hardware_interface
    ros_opcua_srvs
    dysturbance_ros_msgs
)

# Exported libraries
add_library(${PROJECT_NAME}
  src/dysturbance_ros_hardware_interface.cpp
)

target_link_directories(${PROJECT_NAME}
  PUBLIC
    ${NIDAQmx_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${catkin_LIBRARIES}
    NIDAQmx
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# C++ Settings
if (CMAKE_VERSION VERSION_LESS "3.1")
  target_compile_options(${PROJECT_NAME}
    PUBLIC
      "-std=c++14"
  )
else ()
  set_property(
    TARGET
      ${PROJECT_NAME}
    PROPERTY CXX_STANDARD 14
  )

  set_property(
    TARGET
      ${PROJECT_NAME}
    PROPERTY CXX_STANDARD_REQUIRED ON
  )
endif ()

# Installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)