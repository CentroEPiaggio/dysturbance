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

#ifndef DYSTURBANCE_ROS_CONTROL_H
#define DYSTURBANCE_ROS_CONTROL_H

// Standard libraries
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <mutex>

// ROS libraries
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int32.h>

// Internal libraries
#include <dysturbance_ros_hardware_interface/dysturbance_ros_hardware_interface.h>

namespace dysturbance_ros_control {

class dysturbanceControl {
 public:
  dysturbanceControl();
  virtual ~dysturbanceControl();

 private:
  ros::CallbackQueuePtr callback_queue_;
  ros::AsyncSpinner spinner_;
  ros::NodeHandle node_handle_;
  ros::NodeHandle node_handle_control_;
  ros::Publisher frequency_publisher_;
  ros::Subscriber data_subscriber_;
  ros::WallTimer control_setup_timer_;
  ros::WallTimer control_timer_;
  ros::WallTimer frequency_timer_;
  ros::WallDuration control_duration_;
  std::mutex sync_protector_;
  int counter_;  // control loop counter (just to check the frequency)

  std::string platform_data_file_name_;
  std::ofstream platform_data_file_;
  int system_state_;
  int64_t acquisition_samples_;
  double acquisition_duration_;
  double encoder_offset_;

  // do not change the following variables order
  dysturbance_ros_hardware_interface::dysturbanceHW device_;
  bool init_success_;
  bool setup_success_;
  controller_manager::ControllerManager controller_manager_;

  void controlCallback(const ros::WallTimerEvent &timer_event);
  void controlSetupCallback(const ros::WallTimerEvent &timer_event);
  void dataAcquisitionCallback(const dysturbance_ros_msgs::StateStamped &msg);
  void frequencyCallback(const ros::WallTimerEvent &timer_event);
  bool isUserChoiceValid(std::string &answer) const;
  bool promptUserChoice(const std::string &question) const;
  void update(const ros::WallTime &time, const ros::WallDuration &period);
};

}  // namespace dysturbance_ros_control

#endif // DYSTURBANCE_ROS_CONTROL_H