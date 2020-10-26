/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2020, qbrobotics®
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 *  following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *    following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DYSTURBANCE_ROS_HARDWARE_INTERFACE_H
#define DYSTURBANCE_ROS_HARDWARE_INTERFACE_H

// Standard libraries
#include <vector>

// ROS libraries
#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <ros_opcua_srvs/Connect.h>
#include <ros_opcua_srvs/Disconnect.h>
#include <ros_opcua_srvs/Read.h>
#include <ros_opcua_srvs/Write.h>

// Internal libraries
#include <NIDAQmx.h>
#include <dysturbance_ros_msgs/dysturbance_ros_msgs.h>

#define NIDAQ_SAMPLING_FREQUENCY 10000
#define STORAGE_FREQUENCY 250
#define NUM_CHANNELS 3
#define NUM_SAMPLES_PER_CHANNEL (NIDAQ_SAMPLING_FREQUENCY/STORAGE_FREQUENCY)

namespace dysturbance_ros_hardware_interface {

class dysturbanceHW : public hardware_interface::RobotHW {
 public:
  dysturbanceHW() = default;
  ~dysturbanceHW() override;

  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;
  void read(const ros::Time &time, const ros::Duration &period) override;
  void write(const ros::Time &time, const ros::Duration &period) override;

  template<typename T>
  void readOPCUA(const std::string &variable_name, T &variable) {
    ros_opcua_srvs::Read srv;
    srv.request.node.nodeId = opcua_node_id_;
    srv.request.node.qualifiedName = variable_name;
    if (!opcua_read_client_.call(srv)) {
      ROS_ERROR_STREAM("OPC-UA read error: failed to read variable.");
      return;
    }
    switch(srv.response.data.type) {
      case "bool":
        variable = srv.response.data.bool_d;
        break;
      case "int32":
        variable = srv.response.data.int32_d;
        break;
      case "double":
        variable = srv.response.data.double_d;
        break;
      case "string":
        variable = srv.response.data.string_d;
        break;
      default:
        ROS_ERROR_STREAM("OPC-UA read error: unrecognized variable type.");
        return;
    }
  }
  template<typename T>
  void writeOPCUA(const std::string &variable_name, const std::string &variable_type, const T &variable) {
    ros_opcua_srvs::Write srv;
    srv.request.node.nodeId = opcua_node_id_;
    srv.request.node.qualifiedName = variable_name;
    srv.request.data.type = variable_type;
    switch(srv.request.data.type) {
      case "bool":
        srv.request.data.bool_d = variable;
        break;
      case "int32":
        srv.request.data.int32_d = variable;
        break;
      case "double":
        srv.request.data.double_d = variable;
        break;
      case "string":
        srv.request.data.string_d = variable;
        break;
      default:
        ROS_ERROR_STREAM("OPC-UA write error: unrecognized variable type.");
        return;
    }
    if (!opcua_read_client_.call(srv)) {
      ROS_ERROR_STREAM("OPC-UA write error: failed to write variable.");
      return;
    }
  }

 private:
  ros::ServiceClient opcua_connect_client_;
  ros::ServiceClient opcua_disconnect_client_;
  ros::ServiceClient opcua_read_client_;
  ros::ServiceClient opcua_write_client_;
  std::string opcua_node_id_;

  ros::Publisher data_publisher_;
  ros::Time last_time_;
  ros::Time init_time_;

  TaskHandle task_handle_ {nullptr};
  std::string channels_;

  static bool errorCodeToString(int error_code, std::string &error_string);
};

typedef std::shared_ptr<dysturbanceHW> dysturbanceHWPtr;
}  // namespace dysturbance_ros_hardware_interface

#endif // DYSTURBANCE_ROS_HARDWARE_INTERFACE_H