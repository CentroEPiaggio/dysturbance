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
  opcua_connect_client_ = robot_hw_nh.serviceClient<ros_opcua_srvs::Connect>("connect");
  opcua_disconnect_client_ = robot_hw_nh.serviceClient<ros_opcua_srvs::Disconnect>("disconnect");
  opcua_read_client_ = robot_hw_nh.serviceClient<ros_opcua_srvs::Read>("read");
  opcua_write_client_ = robot_hw_nh.serviceClient<ros_opcua_srvs::Write>("write");

  ros_opcua_srvs::Connect srv;
  srv.request.endpoint = "opc.tcp://192.168.250.1:4840";
  if (!opcua_connect_client_.call(srv)) {  //TODO: check if it needs to wait for OPC-UA to startup
    ROS_ERROR_STREAM("OPC-UA error: failed to connect to " << srv.request.endpoint << ".");
    return false;
  }
  opcua_node_id_ = "ns=4;i=4001";  //TODO: check if it needs to be a param

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
  if (errorCodeToString(DAQmxStartTask(task_handle_), error_string)) {
    ROS_ERROR_STREAM("NI DAQmxStartTask failed with error : " << error_string << ".");
    return false;
  }
  data_publisher_ = robot_hw_nh.advertise<dysturbance_ros_msgs::StateStamped>("data_acquisition", 1);
  init_time_ = ros::Time::now();
  last_time_ = init_time_;
  return true;
}

void dysturbanceHW::read(const ros::Time &time, const ros::Duration &period) {
  std::string error_string;
  int32 samples_read = 0;
  std::vector<float64> data(NUM_SAMPLES_PER_CHANNEL * NUM_CHANNELS, 0);

  if (errorCodeToString(DAQmxReadAnalogF64(task_handle_, NUM_SAMPLES_PER_CHANNEL, 0.002, DAQmx_Val_GroupByChannel, &data[0], NUM_SAMPLES_PER_CHANNEL * NUM_CHANNELS, &samples_read, nullptr), error_string)) {
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
  float64 time_slot = (current_time-last_time_).toSec()/samples_read;
  for (int i=0; i<samples_read; i++) {
    msg.data.times.push_back((last_time_-init_time_).toSec() + i*time_slot);
    msg.data.pendulum_positions.push_back(data.at(i*NUM_CHANNELS));
    msg.data.pendulum_torques.push_back(data.at(i*NUM_CHANNELS) + 1);
    msg.data.contact_forces.push_back(data.at(i*NUM_CHANNELS) + 2);
  }
  msg.data.is_reliable = samples_read == NUM_SAMPLES_PER_CHANNEL;
  msg.header.stamp = current_time;
  msg.header.frame_id = channels_;
  data_publisher_.publish(msg);

  last_time_ = current_time;
}

void dysturbanceHW::write(const ros::Time &time, const ros::Duration &period) {
  // sensors do not need to write anything (this is for actuators)
}

bool dysturbanceHW::errorCodeToString(int error_code, std::string &error_string) {
  char error_str[1024] = "";
  DAQmxGetErrorString(error_code, error_str, 1024);
  error_string.assign(error_str);
  return !error_string.empty();
}
