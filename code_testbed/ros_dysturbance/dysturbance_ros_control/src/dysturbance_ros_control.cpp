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
#include <windows.h>
#include <shellapi.h>

using namespace dysturbance_ros_control;

dysturbanceControl::dysturbanceControl()
    : spinner_(1, callback_queue_.get()),  // the dedicated callback queue is needed to avoid deadlocks caused by action client calls (together with controller manager update loop)
      callback_queue_(boost::make_shared<ros::CallbackQueue>()),
      node_handle_(ros::NodeHandle()),
      node_handle_control_(node_handle_, "control"),
      control_duration_(1.0/STORAGE_FREQUENCY),  // depends on NI-DAQ storage frequency
      counter_(0),
      init_success_(device_.init(node_handle_, node_handle_)),
      controller_manager_(&device_, node_handle_control_) {
  node_handle_control_.setCallbackQueue(callback_queue_.get());
  spinner_.start();

  if (init_success_) {
    std::string base_path = "../Desktop/experiments/";
    std::string base_file_name = "subject_" + node_handle_.param<std::string>("subject/id", "0000");
    base_file_name += "_cond_" + node_handle_.param<std::string>("protocol/id", "0") + node_handle_.param<std::string>("protocol/parameters/id", "0000");

    std::string config_file_name = base_path + base_file_name + "_testbed.yaml";
    std::string rosparam_dump_cmd = "/c rosparam dump " + config_file_name + " " + node_handle_.getNamespace();
    ShellExecuteA(nullptr, "open", "cmd.exe", rosparam_dump_cmd.c_str(), nullptr, SW_HIDE);

    for (int i=0; true; i++) {
      std::string data_file_name = base_path + base_file_name + "_run_" + std::to_string(i) + "_platformData.csv";
      std::ifstream existing_data_file(data_file_name);
      if (!existing_data_file.good()) {  // if file does not exist, it is the current run
        platform_data_file_.open(data_file_name, std::ios_base::app);
        platform_data_file_ << "time; pendulum_position; pendulum_torque; contact_force; system_state";
        platform_data_file_ << std::fixed << std::setw(6) << std::setprecision(3) << std::setfill(' ');  //TODO: check values with data
        break;
      }
    }

    frequency_publisher_ = node_handle_.advertise<std_msgs::Int32>("frequency", 1);
    data_subscriber_ = node_handle_.subscribe("data_acquisition", 1, &dysturbanceControl::dataAcquisitionCallback, this);
    control_setup_timer_ = node_handle_.createWallTimer(ros::WallDuration(1.0), &qbDeviceControl::controlSetupCallback, this, true);  // oneshot
  }
}

dysturbanceControl::~dysturbanceControl() {
  platform_data_file_.close();
  control_timer_.stop();
  frequency_timer_.stop();
  spinner_.stop();
}

void dysturbanceControl::controlCallback(const ros::WallTimerEvent &timer_event) {
  std::lock_guard<std::mutex> lock(sync_protector_);  // automatically released at the end of the callback
  update(timer_event.current_real, timer_event.current_real - timer_event.last_real);
  counter_++;

  if (system_state_ == 0) {  // experiment has ended
    platform_data_file_.close();
    control_timer_.stop();
    //TODO: prompt experiment details and exit
  }

  // can serve async pending request when the lock is released
}

void dysturbanceControl::controlSetupCallback(const ros::WallTimerEvent &timer_event) {
  control_setup_timer_.stop();

  std::string protocol_id = node_handle_.param<std::string>("protocol/id", "0");
  switch (std::stoi(protocol_id)) {
    case 1:
      //TODO add protocol initial computations
      break;
    case 2:
      //TODO add protocol initial computations
      break;
    case 3:
      //TODO add protocol initial computations
      break;
    case 4:
      //TODO add protocol initial computations
      break;
    case 5:
      //TODO add protocol initial computations
      break;
    default:  // unexpected protocol number
      return;
  }

  //TODO: add protocol details
  promptUserChoice("Do you want to start the current protocol with the given settings?");  // blocking
  ROS_INFO_STREAM("Starting Protocol " + protocol_id + "...");
  device_.writeOPCUABool("Protocol_" + protocol_id + "_Enable", true);

  //TODO: add experiment details
  promptUserChoice("Do you want to start the current experiment?");  // blocking
  ROS_INFO_STREAM("Starting Experiment...");
  device_.writeOPCUABool("Protocol_" + protocol_id + "_Start_Experiment", true);

  control_timer_ = node_handle_control_.createWallTimer(control_duration_, &dysturbanceControl::controlCallback, this);
  frequency_timer_ = node_handle_.createWallTimer(ros::WallDuration(1), &dysturbanceControl::frequencyCallback, this);
}

void dysturbanceControl::dataAcquisitionCallback(const dysturbance_ros_msgs::StateStamped &msg) {
  device_.readOPCUAInt32("System_State", system_state_);
  for (int i=0; i<msg.data.times.size(); i++) {
    platform_data_file_ << msg.data.times.at(i) << "; ";
    platform_data_file_ << msg.data.pendulum_positions.at(i) << "; ";
    platform_data_file_ << msg.data.pendulum_torques.at(i) << "; ";
    platform_data_file_ << msg.data.contact_forces.at(i) << "; ";
    platform_data_file_ << system_state_ << std::endl;
  }
}

void dysturbanceControl::frequencyCallback(const ros::WallTimerEvent &timer_event) {
  ROS_DEBUG_STREAM("Control node frequency: " << counter_ << "Hz");
  frequency_publisher_.publish([this](){ std_msgs::Int32 hz_msg; hz_msg.data = counter_; counter_ = 0; return hz_msg; }());  // publish the control loop real Hz
}

bool dysturbanceControl::isUserChoiceValid(std::string &answer) {
  std::transform(answer.begin(), answer.end(), answer.begin(), ::tolower);
  return answer == "y" || answer == "n" || answer == "yes" || answer == "no";
}

void dysturbanceControl::promptUserChoice(const std::string &question) {
  std::string answer = "";
  do {
    ROS_INFO_STREAM(question + "[Y/n]");
  } while(std::cin >> answer && !isUserChoiceValid(answer));
  if (!std::cin) {
    throw std::runtime_error("User input read failed...");
  }
}

void dysturbanceControl::update(const ros::WallTime &time, const ros::WallDuration &period) {
  // read the state from the hardware
  device_.read(ros::Time(time.toSec()), ros::Duration(period.toSec()));

  // update the controllers (set the new control actions)
//  controller_manager_.update(ros::Time(time.toSec()), ros::Duration(period.toSec()));  // does it nothing since no controller is loaded

  // write the commands to the hardware
  device_.write(ros::Time(time.toSec()), ros::Duration(period.toSec()));
}
