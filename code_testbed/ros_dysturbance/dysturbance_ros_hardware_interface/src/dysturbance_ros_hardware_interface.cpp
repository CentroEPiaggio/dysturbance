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

#include <dysturbance_ros_hardware_interface/dysturbance_ros_hardware_interface.h>

using namespace dysturbance_ros_hardware_interface;

dysturbanceHW::~dysturbanceHW() {
  if (task_handle_) {
    std::string error_string;
    if (errorCodeToString(DAQmxClearTask(task_handle_), error_string)) {
      ROS_ERROR_STREAM("NI DAQmxClearTask failed with error : " << error_string << ".");
    }
  }

  ros_opcua_srvs::Disconnect srv;
  if (!opcua_disconnect_client_.call(srv)) {
    ROS_ERROR_STREAM("OPC-UA error: failed to disconnect.");
  }
}

bool dysturbanceHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
  opcua_connect_client_ = robot_hw_nh.serviceClient<ros_opcua_srvs::Connect>("opcua_client/connect");
  opcua_disconnect_client_ = robot_hw_nh.serviceClient<ros_opcua_srvs::Disconnect>("opcua_client/disconnect");
  opcua_read_client_ = robot_hw_nh.serviceClient<ros_opcua_srvs::Read>("opcua_client/read");
  opcua_write_client_ = robot_hw_nh.serviceClient<ros_opcua_srvs::Write>("opcua_client/write");

  ros_opcua_srvs::Connect srv;
  srv.request.endpoint = "opc.tcp://192.168.250.1:4840";
  for (int i=0; i<5; i++) {
    if (opcua_connect_client_.call(srv)) {
      break;
    } else {
      ROS_ERROR_STREAM("OPC-UA error: failed to connect to " << srv.request.endpoint << ".");
      ros::Duration(1.0).sleep();
    }
    if (i == 4) {
      return false;
    }
  }
  opcua_node_id_prefix_ = "ns=4;s=";

  std::string error_string;
  channels_ = "Dev1/ai0, Dev1/ai2, Dev1/ai4";
  if (errorCodeToString(DAQmxCreateTask("", &task_handle_), error_string)) {
    ROS_ERROR_STREAM("NI DAQmxCreateTask failed with error : " << error_string << ".");
    return false;
  }
  if (errorCodeToString(DAQmxCreateAIVoltageChan(task_handle_, channels_.c_str(), "", DAQmx_Val_Diff, -5.0, 5.0, DAQmx_Val_Volts, nullptr), error_string)) {
    ROS_ERROR_STREAM("NI DAQmxCreateAIVoltageChan failed with error : " << error_string << ".");
    return false;
  }
  if (errorCodeToString(DAQmxCfgSampClkTiming(task_handle_, "", NIDAQ_SAMPLING_FREQUENCY, DAQmx_Val_Rising, DAQmx_Val_ContSamps, NUM_SAMPLES_PER_CHANNEL), error_string)) {
    ROS_ERROR_STREAM("NI DAQmxCfgSampClkTiming failed with error : " << error_string << ".");
    return false;
  }
  data_publisher_ = robot_hw_nh.advertise<dysturbance_ros_msgs::StateStamped>("data_acquisition", 1);
  return true;
}

bool dysturbanceHW::startAcquisition() {
  std::string error_string;
  if (errorCodeToString(DAQmxStartTask(task_handle_), error_string)) {
    ROS_ERROR_STREAM("NI DAQmxStartTask failed with error : " << error_string << ".");
    return false;
  }
  elapsed_duration_ = 0.0;
  last_time_ = ros::Time::now();
  return true;
}

bool dysturbanceHW::stopAcquisition() {
  std::string error_string;
  if (errorCodeToString(DAQmxStopTask(task_handle_), error_string)) {
    ROS_ERROR_STREAM("NI DAQmxStopTask failed with error : " << error_string << ".");
    return false;
  }
  return true;
}

void dysturbanceHW::read(const ros::Time &time, const ros::Duration &period) {
  std::string error_string;
  int samples_read = 0;
  std::vector<double> data(NUM_SAMPLES_PER_CHANNEL * NUM_CHANNELS, 0);

  if (errorCodeToString(DAQmxReadAnalogF64(task_handle_, NUM_SAMPLES_PER_CHANNEL, 0.005, DAQmx_Val_GroupByChannel, &data[0], NUM_SAMPLES_PER_CHANNEL * NUM_CHANNELS, &samples_read, nullptr), error_string)) {
    ROS_ERROR_STREAM("NI DAQmxCreateAIVoltageChan failed with error : " << error_string << ".");
    return;
  }

  if (samples_read == 0) {
    ROS_ERROR_STREAM("NI DAQ error: No samples retrieved");
    return;
  }
  if (samples_read < NUM_SAMPLES_PER_CHANNEL) {
    ROS_WARN_STREAM("NI DAQ warning: Fewer samples than expected");
  }

  dysturbance_ros_msgs::StateStamped msg;
  ros::Time current_time = ros::Time::now();
  double since_last_time = (current_time-last_time_).toSec();
  if (since_last_time > 2.0/STORAGE_FREQUENCY) {
    ROS_WARN_STREAM("Data acquisition is slower than expected: " << since_last_time << "s from last data stream (expected " << 2.0/STORAGE_FREQUENCY << "s).");
    elapsed_duration_ += (std::floor(since_last_time / (1.0/STORAGE_FREQUENCY)) - 1) * (1.0/STORAGE_FREQUENCY);
  }
  last_time_ = current_time;

  double time_slot = 1.0/NIDAQ_SAMPLING_FREQUENCY;
  std::generate_n(std::back_inserter(msg.data.times), NUM_SAMPLES_PER_CHANNEL, [&](){ return elapsed_duration_+=time_slot; });
  std::copy_n(data.begin(), NUM_SAMPLES_PER_CHANNEL, std::back_inserter(msg.data.pendulum_positions));
  std::copy_n(data.begin()+NUM_SAMPLES_PER_CHANNEL, NUM_SAMPLES_PER_CHANNEL, std::back_inserter(msg.data.pendulum_torques));
  std::copy_n(data.begin()+2*NUM_SAMPLES_PER_CHANNEL, NUM_SAMPLES_PER_CHANNEL, std::back_inserter(msg.data.contact_forces));

  msg.data.is_reliable = samples_read == NUM_SAMPLES_PER_CHANNEL;
  msg.header.stamp = current_time;
  msg.header.frame_id = channels_;
  data_publisher_.publish(msg);
}

void dysturbanceHW::write(const ros::Time &time, const ros::Duration &period) {
  // sensors do not need to write anything (this is for actuators)
}

void dysturbanceHW::readOPCUABool(const std::string &variable_name, bool &variable) {
  ros_opcua_srvs::Read srv;
  srv.request.node.nodeId = opcua_node_id_prefix_ + variable_name;
  if (!opcua_read_client_.call(srv)) {
    ROS_ERROR_STREAM("OPC-UA read error: failed to read variable.");
    return;
  }
  variable = srv.response.data.bool_d;
}

void dysturbanceHW::readOPCUAUInt16(const std::string &variable_name, int &variable) {
  ros_opcua_srvs::Read srv;
  srv.request.node.nodeId = opcua_node_id_prefix_ + variable_name;
  if (!opcua_read_client_.call(srv)) {
    ROS_ERROR_STREAM("OPC-UA read error: failed to read variable.");
    return;
  }
  variable = srv.response.data.uint16_d;
}

void dysturbanceHW::readOPCUAInt16(const std::string &variable_name, int &variable) {
  ros_opcua_srvs::Read srv;
  srv.request.node.nodeId = opcua_node_id_prefix_ + variable_name;
  if (!opcua_read_client_.call(srv)) {
    ROS_ERROR_STREAM("OPC-UA read error: failed to read variable.");
    return;
  }
  variable = srv.response.data.int16_d;
}

void dysturbanceHW::readOPCUAFloat64(const std::string &variable_name, double &variable) {
  ros_opcua_srvs::Read srv;
  srv.request.node.nodeId = opcua_node_id_prefix_ + variable_name;
  if (!opcua_read_client_.call(srv)) {
    ROS_ERROR_STREAM("OPC-UA read error: failed to read variable.");
    return;
  }
  variable = srv.response.data.double_d;
}

void dysturbanceHW::readOPCUAString(const std::string &variable_name, std::string &variable) {
  ros_opcua_srvs::Read srv;
  srv.request.node.nodeId = opcua_node_id_prefix_ + variable_name;
  if (!opcua_read_client_.call(srv)) {
    ROS_ERROR_STREAM("OPC-UA read error: failed to read variable.");
    return;
  }
  variable = srv.response.data.string_d;
}

void dysturbanceHW::writeOPCUABool(const std::string &variable_name, bool variable) {
  ros_opcua_srvs::Write srv;
  srv.request.node.nodeId = opcua_node_id_prefix_ + variable_name;
  srv.request.data.type = "bool";
  srv.request.data.bool_d = variable;
  if (!opcua_write_client_.call(srv)) {
    ROS_ERROR_STREAM("OPC-UA write error: failed to write variable.");
    return;
  }
}

void dysturbanceHW::writeOPCUAInt16(const std::string &variable_name, int variable) {
  ros_opcua_srvs::Write srv;
  srv.request.node.nodeId = opcua_node_id_prefix_ + variable_name;
  srv.request.data.type = "int16";
  srv.request.data.int16_d = variable;
  if (!opcua_write_client_.call(srv)) {
    ROS_ERROR_STREAM("OPC-UA write error: failed to write variable.");
    return;
  }
}

void dysturbanceHW::writeOPCUAUInt16(const std::string &variable_name, int variable) {
  ros_opcua_srvs::Write srv;
  srv.request.node.nodeId = opcua_node_id_prefix_ + variable_name;
  srv.request.data.type = "uint16";
  srv.request.data.uint16_d = variable;
  if (!opcua_write_client_.call(srv)) {
    ROS_ERROR_STREAM("OPC-UA write error: failed to write variable.");
    return;
  }
}

void dysturbanceHW::writeOPCUAFloat64(const std::string &variable_name, double variable) {
  ros_opcua_srvs::Write srv;
  srv.request.node.nodeId = opcua_node_id_prefix_ + variable_name;
  srv.request.data.type = "float64";
  srv.request.data.double_d = variable;
  if (!opcua_write_client_.call(srv)) {
    ROS_ERROR_STREAM("OPC-UA write error: failed to write variable.");
    return;
  }
}

void dysturbanceHW::writeOPCUAString(const std::string &variable_name, const std::string &variable) {
  ros_opcua_srvs::Write srv;
  srv.request.node.nodeId = opcua_node_id_prefix_ + variable_name;
  srv.request.data.type = "string";
  srv.request.data.string_d = variable;
  if (!opcua_write_client_.call(srv)) {
    ROS_ERROR_STREAM("OPC-UA write error: failed to write variable.");
    return;
  }
}

bool dysturbanceHW::errorCodeToString(int error_code, std::string &error_string) {
  char error_str[1024] = "";
  DAQmxGetErrorString(error_code, error_str, 1024);
  error_string.assign(error_str);
  return !error_string.empty();
}
