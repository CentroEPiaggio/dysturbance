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

#include <dysturbance_ros_control/dysturbance_ros_control.h>

using namespace dysturbance_ros_control;

dysturbanceControl::dysturbanceControl()
    : spinner_(1, callback_queue_.get()),  // the dedicated callback queue is needed to avoid deadlocks caused by action client calls (together with controller manager update loop)
      callback_queue_(boost::make_shared<ros::CallbackQueue>()),
      node_handle_(ros::NodeHandle()),
      node_handle_control_(node_handle_, "control"),
      control_duration_(node_handle_.param<double>("control_duration", 0.01)),
      counter_(0),
      init_success_(device_.init(node_handle_, node_handle_)),
      controller_manager_(&device_, node_handle_control_) {
  node_handle_control_.setCallbackQueue(callback_queue_.get());
  spinner_.start();

  if (init_success_) {
    frequency_publisher_ = node_handle_.advertise<std_msgs::Int32>("frequency", 1);
    control_timer_ = node_handle_control_.createWallTimer(control_duration_, &dysturbanceControl::controlCallback, this);
    frequency_timer_ = node_handle_.createWallTimer(ros::WallDuration(1), &dysturbanceControl::frequencyCallback, this);
  }
}

dysturbanceControl::~dysturbanceControl() {
  control_timer_.stop();
  frequency_timer_.stop();
  spinner_.stop();
}

void dysturbanceControl::controlCallback(const ros::WallTimerEvent &timer_event) {
  std::lock_guard<std::mutex> lock(sync_protector_);  // automatically released at the end of the callback
  update(timer_event.current_real, timer_event.current_real - timer_event.last_real);
  counter_++;

  // can serve async pending request when the lock is released
}

void dysturbanceControl::frequencyCallback(const ros::WallTimerEvent &timer_event) {
  ROS_DEBUG_STREAM("Control frequency: " << counter_ << "Hz");
  frequency_publisher_.publish([this](){ std_msgs::Int32 hz_msg; hz_msg.data = counter_; counter_ = 0; return hz_msg; }());  // publish the control loop real Hz
}

void dysturbanceControl::update(const ros::WallTime &time, const ros::WallDuration &period) {
  // read the state from the hardware
  device_.read(ros::Time(time.toSec()), ros::Duration(period.toSec()));

  // update the controllers (set the new control actions)
  controller_manager_.update(ros::Time(time.toSec()), ros::Duration(period.toSec()));  // does it nothing since no controller is loaded

  // write the commands to the hardware
  device_.write(ros::Time(time.toSec()), ros::Duration(period.toSec()));
}