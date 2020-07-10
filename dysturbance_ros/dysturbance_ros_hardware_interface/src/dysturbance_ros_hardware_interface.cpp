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

dysturbanceHW::dysturbanceHW() {

}

dysturbanceHW::~dysturbanceHW() {

}

bool dysturbanceHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
  channels_ = "Dev1/ai0, Dev1/ai1, Dev1/ai2, Dev1/ai3, Dev1/ai7, Dev1/ai5, Dev1/ai6";
  DAQmxCreateTask("", &task_handle_);
  DAQmxCreateAIVoltageChan(task_handle_, channels_.c_str(), "", DAQmx_Val_Diff, 0.0, 5.0, DAQmx_Val_Volts, nullptr);
  DAQmxCfgSampClkTiming(task_handle_, "", NIDAQ_SAMPLING_FREQUENCY, DAQmx_Val_Rising, DAQmx_Val_ContSamps, NUM_SAMPLES_PER_CHANNEL);
  DAQmxStartTask(task_handle_);
  //TODO: check returns of all the above DAQ methods
  data_publisher_ = robot_hw_nh.advertise<dysturbance_ros_msgs::StateStamped>("data_acquisition", 1);
  return true;
}

void dysturbanceHW::read(const ros::Time &time, const ros::Duration &period) {
  int32 samples_read = 0;
  std::vector<float64> data(NUM_SAMPLES_PER_CHANNEL * NUM_CHANNELS);
  DAQmxReadAnalogF64(task_handle_, NUM_SAMPLES_PER_CHANNEL, 0.002, DAQmx_Val_GroupByChannel, &data[0], NUM_SAMPLES_PER_CHANNEL * NUM_CHANNELS, &samples_read, nullptr);
  //TODO: check returns of the above DAQ method
  if (samples_read != NUM_SAMPLES_PER_CHANNEL) {
    ROS_WARN_STREAM("Fewer samples than expected");
  }
  data_.push_back(data);
  time_.push_back(ros::Time::now());

  dysturbance_ros_msgs::StateStamped msg;
  msg.data.is_reliable = samples_read == NUM_SAMPLES_PER_CHANNEL;
  msg.data.samples = data_.back();
  msg.header.stamp = time_.back();
  msg.header.frame_id = channels_;
  data_publisher_.publish(msg);
}

void dysturbanceHW::write(const ros::Time &time, const ros::Duration &period) {
  // sensors do not need to write anything (this is for actuators)
}