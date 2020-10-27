cmake_minimum_required(VERSION 2.8.3)
project(ros_opcua_impl_freeopcua)

set(BUILD_CLIENT ON)
set(BUILD_SERVER ON)
set(BUILD_PYTHON ON)
set(BUILD_TESTING OFF)

set(DCMAKE_BUILD_TYPE=Release)

find_package(catkin REQUIRED
  roscpp
  ros_opcua_msgs
  ros_opcua_srvs
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(freeopcua CONFIG REQUIRED)
set(freeopcua_LIBRARIES opcuacore opcuaclient opcuaserver opcuaprotocol)
set(freeopcua_INCLUDE_DIRS freeopcua/include)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp ros_opcua_msgs ros_opcua_srvs
  DEPENDS Boost freeopcua
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
  include
  ${freeopcua_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(client_node src/opcua_client.cpp)
add_dependencies(client_node ros_opcua_msgs_generate_messages_cpp ros_opcua_srvs_generate_messages_cpp)
target_link_libraries(client_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${freeopcua_LIBRARIES})

#############
## Install ##
#############

install(TARGETS client_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
