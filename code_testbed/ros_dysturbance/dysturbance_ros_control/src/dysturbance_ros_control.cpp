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
    : spinner_(10, callback_queue_.get()),
      callback_queue_(boost::make_shared<ros::CallbackQueue>()),
      node_handle_(ros::NodeHandle()),
      control_duration_(1.0/STORAGE_FREQUENCY),  // depends on NI-DAQ storage frequency
      counter_(0),
      acquisition_samples_(0),
      acquisition_duration_(0),
      encoder_offset_(-158),  // WARNING: depends on the encoder magnet mounting
      init_success_(device_.init(node_handle_, node_handle_)),
      setup_success_(false),
      controller_manager_(&device_, node_handle_) {
  node_handle_.setCallbackQueue(callback_queue_.get());
  spinner_.start();

  if (init_success_) {
    if (node_handle_.param<bool>("reset_pendulum", false)) {
      device_.writeOPCUABool("P0_Terminate", true);
      ROS_INFO_STREAM("Reset pendulum position.");
      terminate();
      return;
    }

    std::string subject = "subject_" + std::to_string(node_handle_.param<int>("subject/id", 0));
    std::string base_path = "../Desktop/experiments/" + subject + "/";
    if (!CreateDirectoryA(base_path.c_str(), nullptr) && GetLastError() != ERROR_ALREADY_EXISTS) {
      ROS_ERROR_STREAM("Cannot create the experiment directory.\nTerminating by system...");
      terminate();
      return;
    }

    std::string protocol = "protocol_" + std::to_string(node_handle_.param<int>("protocol/id", 0));
    base_path += protocol + "/";
    if (!CreateDirectoryA(base_path.c_str(), nullptr) && GetLastError() != ERROR_ALREADY_EXISTS) {
      ROS_ERROR_STREAM("Cannot create the experiment directory.\nTerminating by system...");
      terminate();
      return;
    }

    std::string base_file_name = subject + "_cond_";
    base_file_name += std::to_string(node_handle_.param<int>("protocol/id", 0));
    base_file_name += node_handle_.param<std::string>("protocol/parameters/id", "000");
    base_file_name += node_handle_.param<std::string>("pendulum/id", "0000");
    base_file_name += node_handle_.param<std::string>("platform/id", "00");
    base_path += base_file_name + "/";
    if (!CreateDirectoryA(base_path.c_str(), nullptr) && GetLastError() != ERROR_ALREADY_EXISTS) {
      ROS_ERROR_STREAM("Cannot create the experiment directory.\nTerminating by system...");
      terminate();
      return;
    }

    base_path += "raw_data_input/";
    if (!CreateDirectoryA(base_path.c_str(), nullptr) && GetLastError() != ERROR_ALREADY_EXISTS) {
      ROS_ERROR_STREAM("Cannot create the experiment directory.\nTerminating by system...");
      terminate();
      return;
    }

    // The sample frequency is a fixed internal params but it needs to appear in the yaml dump
    node_handle_.setParam("sampling_frequency", NIDAQ_SAMPLING_FREQUENCY);

    std::string config_file_name = base_path + base_file_name + "_testbed.yaml";
    std::string rosparam_dump_cmd = "/c rosparam dump " + config_file_name + " " + node_handle_.getNamespace();
    ShellExecuteA(nullptr, "open", "cmd.exe", rosparam_dump_cmd.c_str(), nullptr, SW_HIDE);

    for (int i=0; true; i++) {
      platform_data_file_name_ = base_path + base_file_name + "_run_" + std::to_string(i) + "_platformData.csv";
      std::ifstream existing_data_file(platform_data_file_name_);
      if (!existing_data_file.good()) {  // if file does not exist, it is the current run
        if (i >= node_handle_.param<int>("protocol/repetitions", 0)) {
          if (!promptUserChoice("You have already performed the expected number of runs, do you want to continue anyway?")) {  // blocking
            ROS_INFO_STREAM("Terminating by user...");
            terminate();
            return;
          }
        }

        platform_data_file_.open(platform_data_file_name_, std::ios_base::app);
        platform_data_file_ << "time [s]; pendulum_position [deg]; pendulum_torque [Nm]; contact_force [N]; UTC_time[YY-MM-DD HH:MM:SS +-OFF]; fallen [bool]" << std::endl;
        platform_data_file_ << std::fixed << std::setprecision(6) << std::setfill(' ');
        break;
      }
    }

    setup_success_ =  true;

    frequency_publisher_ = node_handle_.advertise<std_msgs::Int32>("frequency", 1);
    data_subscriber_ = node_handle_.subscribe("data_acquisition", 1, &dysturbanceControl::dataAcquisitionCallback, this);
    control_setup_timer_ = node_handle_.createWallTimer(ros::WallDuration(1.0), &dysturbanceControl::controlSetupCallback, this, true);  // oneshot
    frequency_timer_ = node_handle_.createWallTimer(ros::WallDuration(1.0), &dysturbanceControl::frequencyCallback, this);
  } else {
    ROS_ERROR_STREAM("Cannot initialize the hardware interface.\nTerminating by system...");
    terminate();
    return;
  }
}

dysturbanceControl::~dysturbanceControl() {
  spinner_.stop();
}

void dysturbanceControl::controlCallback(const ros::WallTimerEvent &timer_event) {
  std::lock_guard<std::mutex> lock(sync_protector_);  // automatically released at the end of the callback
  update(timer_event.current_real, timer_event.current_real - timer_event.last_real);
  counter_++;

  if (system_state_ < 100 && !node_handle_.param<bool>("debug_acquisition", false)) {  // experiment has ended
    ROS_INFO_STREAM("The experiment is end in " << acquisition_duration_ << "s (" << acquisition_samples_ << " samples acquired)");
    bool fallen = promptUserChoice("Is the subject fallen during the experiment?");  // blocking
    platform_data_file_ << ";;;;; " << std::boolalpha << fallen << std::endl;  // only for the last row

    terminate();
  }

  // can serve async pending request when the lock is released
}

void dysturbanceControl::controlSetupCallback(const ros::WallTimerEvent &timer_event) {
  control_setup_timer_.stop();

  if (node_handle_.param<bool>("debug_acquisition", false)) {
    if (!device_.startAcquisition()) {
      ROS_ERROR_STREAM("Terminating by system...");
      terminate();
      return;
    }
    ROS_INFO_STREAM("Debugging...");
  } else {
    int protocol_id = node_handle_.param<int>("protocol/id", 0);
    ROS_INFO_STREAM(" ---------------------------------------------------------------------- ");
    ROS_INFO_STREAM("  Protocol ID : " << protocol_id);
    ROS_INFO_STREAM("  Protocol Name : " << node_handle_.param<std::string>("protocol/name", "undefined"));
    ROS_INFO_STREAM("  Protocol Parameters :");
    ROS_INFO_STREAM("   * ID : " << node_handle_.param<std::string>("protocol/parameters/id", "undefined"));
    switch (protocol_id) {
      case 1:
        {
          double initial_upper_position = std::abs(node_handle_.param<float>("protocol/parameters/initial_upper_position", 0.0));
          double pendulum_added_mass = node_handle_.param<float>("pendulum/added_mass", 0.0);
          double pendulum_length = node_handle_.param<float>("pendulum/length", 0.0);
          double deg_to_rad = std::acos(-1)/180;
          double gravity = 9.81;
          double impact_duration = 0.01;
          double linear_density = 4.13;

          double initial_energy = (pendulum_added_mass + 0.5*linear_density*pendulum_length)*gravity*pendulum_length*(1 - std::cos(deg_to_rad*initial_upper_position));
          double impact_force = std::sqrt(2*(pendulum_added_mass*std::pow(pendulum_length,2) + linear_density*std::pow(pendulum_length,2)/3)*initial_energy);

          device_.writeOPCUAFloat64("P1_Upper_Position", -initial_upper_position);
          ROS_INFO_STREAM("   * Initial Upper Position : " << -initial_upper_position << " [deg]");
          ROS_INFO_STREAM("   * Expected Initial Energy : " << initial_energy << " [J]");
          ROS_INFO_STREAM("   * Expected Impact Force : " << impact_force << " [N]");
        }
        break;
      case 2:
        device_.writeOPCUAFloat64("P2_Displacement_Amplitude", node_handle_.param<float>("protocol/parameters/displacement_amplitude", 0.0));
        device_.writeOPCUAFloat64("P2_Frequency", node_handle_.param<float>("protocol/parameters/frequency", 0.0));
        device_.writeOPCUAUInt16("P2_Cycles", node_handle_.param<int>("protocol/parameters/cycles_number", 0));
        ROS_INFO_STREAM("   * Displacement Amplitude : " << node_handle_.param<float>("protocol/parameters/displacement_amplitude", 0.0) << " [deg]");
        ROS_INFO_STREAM("   * Sinusoid Frequency : " << node_handle_.param<float>("protocol/parameters/frequency", 0.0) << " [Hz]");
        ROS_INFO_STREAM("   * Cycles Number : " << node_handle_.param<int>("protocol/parameters/cycles_number", 0) << " [#]");
        break;
      case 3:
        device_.writeOPCUAFloat64("P3_Torque_Amplitude", node_handle_.param<float>("pendulum/length", 0.0)*node_handle_.param<float>("protocol/parameters/force_amplitude", 0.0));
        device_.writeOPCUAFloat64("P3_Frequency", node_handle_.param<float>("protocol/parameters/frequency", 0.0));
        device_.writeOPCUAUInt16("P3_Cycles", node_handle_.param<int>("protocol/parameters/cycles_number", 0));
        ROS_INFO_STREAM("   * Force Amplitude : " << node_handle_.param<float>("protocol/parameters/force_amplitude", 0.0) << " [N]");
        ROS_INFO_STREAM("   * Sinusoid Frequency : " << node_handle_.param<float>("protocol/parameters/frequency", 0.0) << " [Hz]");
        ROS_INFO_STREAM("   * Cycles Number : " << node_handle_.param<int>("protocol/parameters/cycles_number", 0) << " [#]");
        break;
      case 4:
        device_.writeOPCUAFloat64("P4_Displacement_Ramp_Slope", node_handle_.param<float>("protocol/parameters/displacement_ramp_slope", 0.0));
        ROS_INFO_STREAM("   * Displacement Ramp Slope : " << node_handle_.param<float>("protocol/parameters/displacement_ramp_slope", 0.0) << " [deg/s]");
        break;
      case 5:
        device_.writeOPCUAFloat64("P5_Torque_Ramp_Slope", node_handle_.param<float>("pendulum/length", 0.0)*node_handle_.param<float>("protocol/parameters/force_ramp_slope", 0.0));
        ROS_INFO_STREAM("   * Force Ramp Slope : " << node_handle_.param<float>("protocol/parameters/force_ramp_slope", 0.0) << " [N/s]");
        break;
      default:  // unexpected protocol number
        return;
    }
    ROS_INFO_STREAM(" ---------------------------------------------------------------------- ");
    ROS_INFO_STREAM("  Pendulum ID : " << node_handle_.param<std::string>("pendulum/id", "undefined"));
    ROS_INFO_STREAM("  Pendulum Parameters :");
    device_.writeOPCUAFloat64("P0_Pendulum_Length", node_handle_.param<float>("pendulum/length", 0.0));
    ROS_INFO_STREAM("   * Length : " << node_handle_.param<float>("pendulum/length", 0.0) << " [m]");
    ROS_INFO_STREAM("   * Axis Height : " << node_handle_.param<float>("pendulum/axis_height", 0.0) << " [m]");
    ROS_INFO_STREAM("   * Added Mass : " << node_handle_.param<float>("pendulum/added_mass", 0.0) << " [kg]");
    ROS_INFO_STREAM("   * Tip Type : " << node_handle_.param<std::string>("pendulum/tip_type", "undefined"));
    ROS_INFO_STREAM(" ---------------------------------------------------------------------- ");
    ROS_INFO_STREAM("  Platform ID : " << node_handle_.param<std::string>("platform/id", "undefined"));
    ROS_INFO_STREAM("  Platform Parameters :");
    ROS_INFO_STREAM("   * Ground Inclination : " << node_handle_.param<float>("platform/ground_inclination", 0.0) << " [deg]");
    ROS_INFO_STREAM("   * Ground Type : " << node_handle_.param<std::string>("platform/ground_type", "undefined"));
    ROS_INFO_STREAM(" ---------------------------------------------------------------------- ");
    ROS_INFO_STREAM("  Subject ID : " << node_handle_.param<int>("subject/id", 0));
    ROS_INFO_STREAM("  Subject Name : " << node_handle_.param<std::string>("subject/name", "undefined"));
    ROS_INFO_STREAM("  Subject Parameters :");
    ROS_INFO_STREAM("   * Mass : " << node_handle_.param<float>("subject/mass", 0.0) << " [kg]");
    ROS_INFO_STREAM("   * Height : " << node_handle_.param<float>("subject/height", 0.0) << " [m]");
    ROS_INFO_STREAM("   * CoM Height : " << node_handle_.param<float>("subject/com_height", 0.0) << " [m]");
    ROS_INFO_STREAM("   * Base Depth : " << node_handle_.param<float>("subject/base_depth", 0.0) << " [m]");
    ROS_INFO_STREAM("   * Base Width : " << node_handle_.param<float>("subject/base_width", 0.0) << " [m]");
    ROS_INFO_STREAM("   * Orientation : " << node_handle_.param<float>("subject/orientation", 0.0) << " [deg]");
    ROS_INFO_STREAM(" ---------------------------------------------------------------------- ");

    device_.readOPCUAUInt16("P0_System_State", system_state_);  // only to keep alive OPC-UA ROS node
    do {
      if (system_state_ == 30) {
        ROS_WARN_STREAM("Please close the gates and release the emergency button before starting the experiment...");
      }
      if (!promptUserChoice("Do you want to start the current protocol with the given settings?")) {  // blocking
        ROS_INFO_STREAM("Terminating by user...");
        terminate();
        return;
      }
      device_.readOPCUAUInt16("P0_System_State", system_state_);  // only to keep alive OPC-UA ROS node
    } while (system_state_ == 30);

    device_.writeOPCUABool("P" + std::to_string(protocol_id) + "_Enable", true);
    ROS_INFO_STREAM("Starting Protocol " << protocol_id << "...");

    if (!promptUserChoice("Do you want to start the current experiment?")) {  // blocking
      ROS_INFO_STREAM("Terminating by user...");
      terminate();
      return;
    }
    device_.readOPCUAUInt16("P0_System_State", system_state_);
    device_.writeOPCUABool("P" + std::to_string(protocol_id) + "_Start_Experiment", true);
    ROS_INFO_STREAM("Starting Experiment...");

    char utc_time[32] = "";
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::strftime(utc_time, sizeof(utc_time), "%F %T %z", std::localtime(&now));
    platform_data_file_ << ";;;; " << utc_time << ";" << std::endl;  // only for the first row

    if (!device_.startAcquisition()) {
      ROS_ERROR_STREAM("Terminating by system...");
      terminate();
      return;
    }
  }

  control_timer_ = node_handle_.createWallTimer(control_duration_, &dysturbanceControl::controlCallback, this);
}

void dysturbanceControl::dataAcquisitionCallback(const dysturbance_ros_msgs::StateStamped &msg) {
  for (int i=0; i<msg.data.times.size(); i++) {
    platform_data_file_ << std::setw(12) << msg.data.times.at(i) << "; ";
    platform_data_file_ << std::setw(12) << -(msg.data.pendulum_positions.at(i)*105.7 + encoder_offset_) << "; ";  // 360deg @3.47VDC
    platform_data_file_ << std::setw(12) << msg.data.pendulum_torques.at(i)*100.0 << "; ";  // 500Nm @5VDC
    platform_data_file_ << std::setw(12) << msg.data.contact_forces.at(i)*444.8 << "; ";  // 2224N @5VDC
    platform_data_file_ << "; " << std::endl;  // exclude UTC time and fallen fields
  }
  acquisition_samples_ += msg.data.times.size();
  acquisition_duration_ = msg.data.times.back();
}

void dysturbanceControl::frequencyCallback(const ros::WallTimerEvent &timer_event) {
  device_.readOPCUAUInt16("P0_System_State", system_state_);  // only to keep alive OPC-UA ROS node

  ROS_DEBUG_STREAM("Control node frequency: " << counter_ << "Hz");
  frequency_publisher_.publish([this](){ std_msgs::Int32 hz_msg; hz_msg.data = counter_; counter_ = 0; return hz_msg; }());  // publish the control loop real Hz
}

bool dysturbanceControl::isUserChoiceValid(std::string &answer) const {
  std::transform(answer.begin(), answer.end(), answer.begin(), ::tolower);
  return answer == "y" || answer == "n" || answer == "yes" || answer == "no";
}

bool dysturbanceControl::promptUserChoice(const std::string &question) const {
  std::string answer = "";
  do {
    ROS_INFO_STREAM(question + "[Y/n]");
  } while(std::cin >> answer && !isUserChoiceValid(answer));
  if (!std::cin) {
    throw std::runtime_error("User input read failed...");
  }
  return answer == "y" || answer == "yes";
}

void dysturbanceControl::update(const ros::WallTime &time, const ros::WallDuration &period) {
  // read the state from the hardware
  device_.read(ros::Time(time.toSec()), ros::Duration(period.toSec()));

  // update the controllers (set the new control actions)
//  controller_manager_.update(ros::Time(time.toSec()), ros::Duration(period.toSec()));  // does it nothing since no controller is loaded

  // write the commands to the hardware
  device_.write(ros::Time(time.toSec()), ros::Duration(period.toSec()));
}

void dysturbanceControl::terminate() {
  if (setup_success_) {
    device_.stopAcquisition();
    platform_data_file_.close();
    control_timer_.stop();
    frequency_timer_.stop();
    if (!acquisition_samples_) {
      if (!DeleteFileA(platform_data_file_name_.c_str())) {
        ROS_ERROR_STREAM("Cannot delete the empty data file...");
      }
    }
  }

  if (!node_handle_.param<bool>("reset_pendulum", false) && !node_handle_.param<bool>("debug_acquisition", false) && promptUserChoice("Do you want to bring the pendulum to the initial position?")) {  // blocking
    device_.writeOPCUABool("P0_Terminate", true);
  }

  ROS_INFO_STREAM("Exiting...");
  ros::Duration(1.0).sleep();
  GenerateConsoleCtrlEvent(0, 0);
}
