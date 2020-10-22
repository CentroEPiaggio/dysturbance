cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(dysturbance_ros_control VERSION 0.0.1 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    controller_manager
    dysturbance_ros_hardware_interface
    dysturbance_ros_msgs
    dysturbance_ros_utils
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    controller_manager
    dysturbance_ros_hardware_interface
    dysturbance_ros_msgs
    dysturbance_ros_utils
)

# Build Settings
add_executable(dysturbance_ros_default
  src/main.cpp
  src/dysturbance_ros_control.cpp
)

target_link_libraries(dysturbance_ros_default
  PUBLIC
    ${catkin_LIBRARIES}
)

add_dependencies(dysturbance_ros_default
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Exported libraries
add_library(${PROJECT_NAME}
  src/dysturbance_ros_control.cpp
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# C++ Settings
if (CMAKE_VERSION VERSION_LESS "3.1")
  target_compile_options(dysturbance_ros_default
    PUBLIC
      "-std=c++14"
  )
  target_compile_options(${PROJECT_NAME}
    PUBLIC
      "-std=c++14"
  )
else ()
  set_property(
    TARGET
      dysturbance_ros_default
      ${PROJECT_NAME}
    PROPERTY CXX_STANDARD 14
  )

  set_property(
    TARGET
      dysturbance_ros_default
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

install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)