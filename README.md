# abb_wrapper

[ROS-Industrial] packages intended to ease interaction between ABB robot OmniCore controllers and ROS-based systems, by providing ready-to-run ROS nodes.

## Important Notes
Tested on Ubuntu 18.04 with ROS Melodic 

## Overview

[ROS-Industrial][] packages intended to ease interaction between ABB robot OmniCore controllers and ROS-based systems, by providing ready-to-run ROS nodes.

The included (*principal*) packages are briefly described in the following table:

| Package | Description |
| --- | --- |
| [abb_librws](abb_librws) | (A modified version of https://github.com/ros-industrial/abb_librws) Provides a ROS node that communicate with the controller using Robot Web Services 2.0  |
| [abb_libegm](abb_libegm) | (A modified version of https://github.com/ros-industrial/abb_libegm) Provides a ROS node that exposes hardware interface, for *direct motion control* of ABB robots (via the *Externally Guided Motion* (`EGM`) interface). |
| [abb_driver](abb_driver) | Provides ROS nodes for the main interface with the controller. It combine the RWS and EGM node and use the parameters in the yaml file. |
| [abb_controllers](abb_controllers) | Provides ROS nodes for kinematic calculation using the URDF model of the robot. |
| [gofa_description](gofa_description) | Provides ROS nodes for kinematic calculation using the URDF model of the robot. |
| [yumi_description](yumi_description) | Provides ROS nodes for kinematic calculation using the URDF model of the robot. |
| [degub_rviz_tool](abb_description) | Tools used for tests and debug |

Please see each package for more details (*e.g. additional requirements, limitations and troubleshooting*).

## Build Instructions

It is assumed that [ROS Melodic has been installed](http://wiki.ros.org/melodic/Installation/Ubuntu) on the system in question.

### Set up ROS

The following instructions assume that a [Catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) has been created at `$HOME/catkin_ws` and that the *source space* is at `$HOME/catkin_ws/src`. Update paths appropriately if they are different on the build machine.

The following instructions should build the main branches of all required repositories on a ROS Melodic system:

```bash
$ source /opt/ros/melodic/setup.bash

$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make

$ source devel/setup.bash
```

If no errors were reported as part of the `catkin_make` command, the build has succeeded and the driver should now be usable.

### Install POCO

Start a terminal session (launch terminal) by <kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd>

Install essential dependencies and git, execute the following commands one by one:

```bash
$ sudo apt update
$ sudo apt upgrade
$ sudo apt install build-essential gdb cmake git
$ sudo apt-get install openssl libssl-dev
$ sudo apt-get install libiodbc2 libiodbc2-dev
$ sudo apt-get install libmysqlclient-dev
```

Get root access:
```bash
$ sudo -i
```

Navigate to /tmp/ directory (or any other directory to store temporary files).
```bash
$ cd /tmp/
```

Clone the Poco git repo:
```bash
$ git clone -b master https://github.com/pocoproject/poco.git
```

Compile the libraries:
```bash
$ cd poco
$ mkdir cmake-build
$ cd cmake-build
$ cmake ..
$ cmake --build . --config Release
```

**Note**: If you get a library not found error, just install that library via apt.

Install the libraries to include in C++ code:
```bash
$ sudo cmake --build . --target install
```

Copy all the poco file from /usr/local/lib/ to /usr/lib

### Install Boost C++

[Boost C++](https://www.boost.org)
```bash
$ sudo apt-get install libboost-all-dev

```

### Set up the interface

Copy **abb_wrapper** folder to **src** folder on catkin workspace (`~/catkin_ws/src`).

Source the workspace
```bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

Move to the workspace folder
```bash
$ cd catkin_ws/
```

Compile the workspace
```bash
$ catkin_make
```

If there are no errors you are ready to use the interface.



## Robot Set up

### Requirements

* RobotWare version `7.2` or higher (lower versions are incompatible due to changes in the EGM communication protocol).
* A license for the RobotWare option *Externally Guided Motion* (`3124-1`).
* StateMachine 2.0 RobotWare Add-In

After the creation of the system just configure robot to accept external communication both for EGM and Web Services.

### Setup the IP address for the WAN port
With this configuration, we will set up the IP address of the WAN port where the computer running ROS will be connected.

* On the Controller tab, in the Configuration group, Click Properties and then click `Network settings`.
  The Network settings dialog opens.
* Select `Use the following IP address` and then enter the required IP address and Subnet mask boxes to manually set the IP address of the controller

### Setup the UDP device
Configure the IP address and the port to use for the UDP protocol.
This IP address must be the same of the PC running ROS.

On the Controller tab, in the Configuration group, Click Configuration and then click `Communication`.
An example of UDP device is show in the folloging table.

| Name | Type | Remote Address | Remote Port Number | Local Port Number |
| --- | --- | --- | --- | --- |
| ROB_1 | UDPUC | 92.168.100.100 | 6511 | 0 |



### Set up Config File and launch your abb robot (e.g. Gofa) 
Navigate to abb_driver/config/gofa_cfg.yaml
Modify the parameters based on your robot configuration (e.g. ip_robot, name_robot,task_robot, etc.)
Finally 
```bash
$ roslaunch abb_driver interface_gofa.launch
```
