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

Each protocol has distinct launch and config files that contains all the parameters required for that specific experiment.

It is worth noticing that some of the parameters cannot be measured by the sensors equipped on the EUROBENCH DYSTURBANCE platform, therefore the user must be careful during the setup and the launch procedure (e.g. the pendulum length is adjusted manually by the user during the setup and that value has to be specified correctly during launch).

An example of protocol execution is as follows:
```
roslaunch dysturbance_ros_control generic_protocol_1.launch
```

During the experiments the following ROS Topic is advertised and can be inspected by other ROS Nodes:
 - `/dysturbance/data_acquisition`: each message contains the samples acquired by the NI board in given time slot. The message type is [`dysturbance_ros_msgs::StateStamped`](ros_dysturbance/dysturbance_ros_msgs/msg/StateStamped.msg).

All the data published on this topic is also stored in a `.csv` file for post-processing (one file per experiment), together with a `.yaml` file containing the current setup of the testbed.

### Protocols
The detailed description of each protocol is documented under [`docs`](../docs/) and is not replicated in this paragraph.
What it is added here is all the parameter configuration involving the given protocols.

#### P0: Common
In the following, only the parameters in bold are directly used in variables and settings computations for a given experiment, while the others are not required to execute a run. However, all the parameters are demanded for a proper post-processing and PI algorithm evaluation. 
 - [`config_pendulum.yaml`](ros_dysturbance/dysturbance_ros_control/config/config_pendulum.yaml): contains all the parameters of the pendulum, which needs to be set up manually on the testbed and carefully reported in this configuration file.
   <pre>
   pendulum:
     id: "0000"  # The integer unique identifier used for data storage
     <b>length</b>: 1.0  # The pendulum length [m]
     axis_height: 1.0  # The pendulum axis height [m]
     <b>added_mass</b>: 0.0  # The pendulum added mass (not including the pendulum mass itself) [kg]
     tip_type: "default"  # The pendulum tip type
   </pre>
 - [`config_platform.yaml`](ros_dysturbance/dysturbance_ros_control/config/config_platform.yaml): contains all the parameters of the platform ground, which needs to be set up manually on the testbed and carefully reported in this configuration file.
   ```
   platform:
     id: "00"  # The integer unique identifier used for data storage
     ground_inclination: 0.0  # The ground inclination [deg]
     ground_type: "default"  # The ground material type
   ```
 - [`config_subject.yaml`](ros_dysturbance/dysturbance_ros_control/config/config_subject.yaml): contains all the parameters of the subject.
   This fields are not directly used by the experimental protocols but which are highly required for post-processing and PI algorithm evaluation.
   ```
   subject:
     id: 0  # The integer unique identifier used for data storage
     name: ""  # The identifier (e.g. name) of the subject performing the experiment
     mass: 0.0  # The subject mass [kg]
     height: 0.0  # The subject height [m]
     com_height: 0.0  # The subject Center of Mass height [m]
     base_depth: 0.0  # The subject base depth [m]
     base_width: 0.0  # The subject base width [m]
     orientation: 0.0  # The subject orientation w.r.t. the pendulum [deg]
   ```

#### P1: Impulsive Disturbance
 - [`config_protocol_1.yaml`](ros_dysturbance/dysturbance_ros_control/config/config_protocol_1.yaml): contains all the specific parameters of the current protocol.
   <pre>
   protocol:
     id: 1  # The integer unique identifier used for data storage
     name: "impulsive_disturbance"  # The identifier of the protocol to be used in the current experiment
     notes: ""  # Some relevant notes regarding the experiment setup
     repetitions: 3  # The number of repetitions of the experiment
     parameters:
       id: "000"  # The integer unique identifier used for data storage
       <b>initial_energy</b>: 5.0  # The initial energy which determines the initial pendulum position [J]
       <b>impact_force</b>: 400.0 # The desired impact force [N]
   </pre>
 - Automatic recalling of these specific and all common parameters through:
   ```
   roslaunch dysturbance_ros_control generic_protocol_1.launch
   ```

#### P2: Sinusoidal Displacement Disturbance
 - [`config_protocol_2.yaml`](ros_dysturbance/dysturbance_ros_control/config/config_protocol_2.yaml): contains all the specific parameters of the current protocol.
   <pre>
   protocol:
     id: 2  # The integer unique identifier used for data storage
     name: "sinusoidal_displacement_disturbance"  # The identifier of the protocol to be used in the current experiment
     notes: ""  # Some relevant notes regarding the experiment setup
     repetitions: 3  # The number of repetitions of the experiment
     parameters:
       id: "000"  # The integer unique identifier used for data storage
       <b>displacement_amplitude</b>: 5.0  # The pendulum sinusoidal displacement amplitude [deg]
       <b>frequency</b>: 0.1  # The pendulum oscillation frequency [Hz]
       <b>cycles_number</b>: 5  # The number of sinusoidal periods
   </pre>
 - Automatic recalling of these specific and all common parameters through:
   ```
   roslaunch dysturbance_ros_control generic_protocol_2.launch
   ```

#### P3: Sinusoidal Force Disturbance
 - [`config_protocol_3.yaml`](ros_dysturbance/dysturbance_ros_control/config/config_protocol_3.yaml): contains all the specific parameters of the current protocol.
   <pre>
   protocol:
     id: 3  # The integer unique identifier used for data storage
     name: "sinusoidal_force_disturbance"  # The identifier of the protocol to be used in the current experiment
     notes: ""  # Some relevant notes regarding the experiment setup
     repetitions: 3  # The number of repetitions of the experiment
     parameters:
       id: "001"  # The integer unique identifier used for data storage
       <b>force_amplitude</b>: 60.0  # The pendulum sinusoidal force amplitude [N]
       <b>frequency</b>: 0.1  # The pendulum oscillation frequency [Hz]
       <b>cycles_number</b>: 5  # The number of sinusoidal periods
   </pre>
 - Automatic recalling of these specific and all common parameters through:
   ```
   roslaunch dysturbance_ros_control generic_protocol_3.launch
   ```

#### P4: Quasi-Static Displacement Disturbance
 - [`config_protocol_4.yaml`](ros_dysturbance/dysturbance_ros_control/config/config_protocol_4.yaml): contains all the specific parameters of the current protocol.
   <pre>
   protocol:
     id: 4  # The integer unique identifier used for data storage
     name: "quasi_static_displacement_disturbance"  # The identifier of the protocol to be used in the current experiment
     notes: ""  # Some relevant notes regarding the experiment setup
     repetitions: 3  # The number of repetitions of the experiment
     parameters:
       id: "000"  # The integer unique identifier used for data storage
       <b>displacement_ramp_slope</b>: 1.0  # The pendulum displacement ramp slope [deg/s]
   </pre>
 - Automatic recalling of these specific and all common parameters through:
   ```
   roslaunch dysturbance_ros_control generic_protocol_4.launch
   ```

#### P5: Quasi-Static Force Disturbance
 - [`config_protocol_5.yaml`](ros_dysturbance/dysturbance_ros_control/config/config_protocol_5.yaml): contains all the specific parameters of the current protocol.
   <pre>
   protocol:
     id: 5  # The integer unique identifier used for data storage
     name: "quasi_static_force_disturbance"  # The identifier of the protocol to be used in the current experiment
     notes: ""  # Some relevant notes regarding the experiment setup
     repetitions: 3  # The number of repetitions of the experiment
     parameters:
       id: "000"  # The integer unique identifier used for data storage
       <b>force_ramp_slope</b>: 1.0  # The pendulum force ramp slope [N/s]
   </pre>
 - Automatic recalling of these specific and all common parameters through:
   ```
   roslaunch dysturbance_ros_control generic_protocol_5.launch
   ```

#### Custom Files
The files described in the above paragraphs are the default and common settings, but it is up to the user to create some custom files which have the same structure of the above and which contain specific information for a given setup or subject. Be aware that in this case the launch files need to recall these custom files and not the default ones (custom launch files to speed up this process are always welcome).

### Data Storage and File Naming Conventions
The `.csv` data stream files are stored in a path relative to the Catkin workspace, to avoid possible error due to absolute paths.
The directory `experiments` is indeed created next to `catkin_ws` (or whatever name had been chosen during installation) at the first experiment run, if it is not already there.
Inside `experiments`, files are organized in directories named by subject and protocols ids.

This could be a hierarchical structure example:
```
- catkin_ws
  - build
  - devel
  - install
  - src
- experiments
  - subject_13
    - protocol_1
      - subject_13_cond_1000000000
        - raw_data_input
          - subject_13_cond_1000000000_run_0_platformData.csv
          - subject_13_cond_1000000000_run_1_platormData.csv
            ...
          - subject_13_cond_1000000000_testbed.yaml
      - subject_13_cond_1000000001
        ...
    - protocol_2
      ...
  - subject_1
    ...
``` 

As shown in the example, the file names are built to be able to get a quick glance to the specific settings of a given experiment or run.
In detail, the file name format exploits the IDs described in the above paragraphs (in bold) and it is as follows:
<pre>
subject_<b>[subject/id]</b>_cond_<b>[protocol/id]</b><b>[protocol/parameters/id]</b><b>[pendulum/id]</b><b>[platform/id]</b>_run_<b>[run_incremental_counter]</b>_platformData.csv
subject_<b>[subject/id]</b>_cond_<b>[protocol/id]</b><b>[protocol/parameters/id]</b><b>[pendulum/id]</b><b>[platform/id]</b>_testbed.yaml
</pre>

It is worth noticing that there always should be a single `*_testbed.yaml` per a series of same config (i.e. same IDs) experiment.

### Debug Data Acquisition
Execute the following command to make a simple check of data acquisition (without even powering up the servo motor):
```
roslaunch dysturbance_ros_control data_acquisition.launch
```

This launch file initialize everything needed to start the NI sensors data stream, but it won't enable any protocol.