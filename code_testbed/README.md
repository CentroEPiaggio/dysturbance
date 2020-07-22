# ROS packages for the EUROBENCH DYSTURBANCE platform
These ROS packages are meant to be run only on the hardware of the EUROBENCH DYSTURBANCE platform.
Try to execute the code on a different environment will probably end in a failure.

## Purposes
This code is aimed to control the pendulum of the EUROBENCH DYSTURBANCE platform to perform several type of experiments on a given target, and gather the sensor measurements that will be used to estimate the stability under certain perturbations.

Analysis and post-processing computations are addressed in [`code_pi`](../code_pi/).

## Requirements
* Windows 10 64-bit IoT Enterprise
* ROS Melodic or Noetic: http://wiki.ros.org/Installation/Windows
* National Instruments NI-DAQmx drivers: https://www.ni.com/it-it/support/downloads/drivers/download.ni-daqmx.html
* Hardware described in the platform details: [dysturbance_testbed_description.pdf](../docs/dysturbance_testbed_description.pdf)

## Installation
As for many ROS packages, `code_testbed` directory has to be copied in the catkin workspace and then `catkin build` (or `catkin_make` if you are not using the [Catkin Command Line Tools](https://catkin-tools.readthedocs.io)) must be executed within the workspace to compile the whole code.

## Usage
>This code is not yet complete due to the COVID-19 delays and the final usage documentation will probably differ from the one proposed.

Each protocol has a distinct launch file that contains all the parameters required for that specific experiment.

It is worth noticing that some of the parameters cannot be measured by the sensors equipped on the EUROBENCH DYSTURBANCE platform, therefore the user must be careful during the setup and the launch procedure (e.g. the pendulum length is adjusted manually by the user during the setup and that value has to be specified correctly during launch).

An example of protocol execution is as follows:
```
roslaunch dysturbance_ros_control protocol_tmp.launch arg1:= arg2:= ...
```

During the experiments the following ROS Topic is advertised and can be inspected by other ROS Nodes:
* `data_acquisition`: each message contains the samples acquired by the NI board in given time slot. The message type is [`dysturbance_ros_msgs::StateStamped`](dysturbance_ros_msgs/msg/StateStamped.msg).

All the data published in this topic is also stored in a `.csv` file for post-processing (one file per experiment), together with a `.yaml` file containing the current setup of the testbed.