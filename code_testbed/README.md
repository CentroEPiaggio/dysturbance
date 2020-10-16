# ROS packages for the EUROBENCH DYSTURBANCE platform
These ROS packages are meant to be run only on the hardware of the EUROBENCH DYSTURBANCE platform (described in the platform details: [dysturbance_testbed_description.pdf](../docs/dysturbance_testbed_description.pdf)).

Try to execute the code on a different environment will probably end in a failure.

## Purposes
This code is aimed to control the pendulum of the EUROBENCH DYSTURBANCE platform to perform several type of experiments on a given target, and gather the sensor measurements that will be used to estimate the stability under certain perturbations.

Analysis and post-processing computations are addressed in [`code_pi`](../code_pi/).

## Requirements and Installation
Download and install the following software on the IPC running Windows 10 64-bit IoT Enterprise:
 - OMRON Sysmac Studio 1.30: https://automation.omron.com/en/us/products/family/sysstdio
 - ROS Melodic (or Noetic): http://wiki.ros.org/Installation/Windows
 - NI-DAQmx 20.1 C libraries: https://www.ni.com/it-it/support/downloads/drivers/download.ni-daqmx.html

Helpful official OMRON documentation for Sysmac Studio:
 - https://assets.omron.eu/downloads/manual/en/v13/w504_sysmac_studio_operation_manual_en.pdf
 - https://assets.omron.eu/downloads/manual/en/v1/i823_1s-series_setup_manual_en.pdf

When everything is installed, install the free OPC-UA library for Windows (which does not compile directly from the official [ROS repository](https://github.com/iirob/ros_opcua_communication)).

To do so execute the following command from a terminal (ConEmu is suggested):
```
vcpkg install freeopcua:x64-windows
```
Note: `vcpkg` is already installed with ROS in `C:\opt\` (but you might need to add it to the system path depending on the terminal you are using).

Then you need to:
 1. Create the Catkin workspace where you prefer, e.g. your Desktop:
    ```
    cd C:\Users\<your_username>\Desktop
    mkdir catkin_ws
    cd catkin_ws
    mkdir src
    catkin_make
    ```
    Note: to call `catkin_make` you need to use the ROS terminal created during the ROS installation.

 2. Update ROS terminal shortcut with the proper workspace source (from now on you will need this one until you change the Catkin workspace directory):
    ```
    C:\Users\<your_username>\Desktop\catkin_ws\devel\setup.bat
    ```
    Use the above string instead of the one pointed to the ROS installation directory.

 3. Clone the repository under `catkin_ws\src\`:
    ```
    cd C:\Users\<your_username>\Desktop
    git clone https://github.com/CentroEPiaggio/dysturbance.git
    cp dysturbance\code_testbed\* catkin_ws\src\
    ```

 4. Compile everything with the following command:
    ```
    cd C:\Users\<your_username>\Desktop\catkin_ws
    catkin_make -DCMAKE_BUILD_TYPE=Release install
    ```

## Usage
>This code is not yet complete due to the COVID-19 delays and the final usage documentation will probably differ from the one proposed.

Each protocol has a distinct launch file that contains all the parameters required for that specific experiment.

It is worth noticing that some of the parameters cannot be measured by the sensors equipped on the EUROBENCH DYSTURBANCE platform, therefore the user must be careful during the setup and the launch procedure (e.g. the pendulum length is adjusted manually by the user during the setup and that value has to be specified correctly during launch).

An example of protocol execution is as follows:
```
roslaunch dysturbance_ros_control protocol_tmp.launch arg1:= arg2:= ...
```

During the experiments the following ROS Topic is advertised and can be inspected by other ROS Nodes:
 - `data_acquisition`: each message contains the samples acquired by the NI board in given time slot. The message type is [`dysturbance_ros_msgs::StateStamped`](dysturbance_ros_msgs/msg/StateStamped.msg).

All the data published in this topic is also stored in a `.csv` file for post-processing (one file per experiment), together with a `.yaml` file containing the current setup of the testbed.