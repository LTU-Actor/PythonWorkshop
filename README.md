# PythonWorkshop
Polaris GEM Dataspeed Drive by Wire Python Workshop 

## Overview
This repository contains three ROS packages to support the Dataspeed DBW workshop using Python
1. workshop: Python code for the workshop
2. simple_sim: Supporting simulation package
3. simple_sim_atrium: Set up files for the atrium simulation

## Installation
These instructions assume a fresh install of Unbuntu 20.04.
- Install Git  
`sudo apt install git`

- Install ROS and SDK
https://bitbucket.org/DataspeedInc/dbw_polaris_ros/src/e15411f22faed2750b2f8066bbc103c39b6c9a03/ROS_SETUP.md  
Place .bash contents into file install_ros_sdk  
`chmod +x install_ros_sdk`  
`./install_ros_sdk | tee install_ros_sdk.log`  

- Check log file for warnings or errors (For example, Failed to detected ROS version)  
If GPG NO_PUBKEY error: https://itsfoss.com/solve-gpg-error-signatures-verified-ubuntu/  
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys <KEY>  
If KEYEXPIRED ERROR, https://futurestud.io/tutorials/fix-ubuntu-debian-apt-get-keyexpired-the-following-signatures-were-invalidd~

- Remove autostart DBW joystick example launch  
`rm $HOME/.config/autostart/joystick_demo.desktop`

- Set up workspace  
`mkdir -p ~/dbw_ws/src && cd ~/dbw_ws`  
`cd ~/dbw_ws/src`

- Clone Dataspeed Polaris ROS ADAS Code  
https://bitbucket.org/DataspeedInc/dbw_polaris_ros/src/master/

- Clone Dataspeed ULC ROS Code  
https://bitbucket.org/DataspeedInc/dataspeed_ulc_ros/src/master/

- Download simple_camera_publisher  
https://github.com/ltu-ros/simple_camera_publisher

- Install rosdep  
`sudo apt install python3-rosdep2`  
`sudo rosdep init`  
`rosdep update`

- Install C++ compiler and supporting apps  
`sudo apt-get install build-essential`  

- Update workspace and resolve dependencies  
`source /opt/ros/noetic/setup.bash`  
`cd ~/dbw_ws`  
`rosdep update && rosdep install --from-paths src --ignore-src`

- Build the workspace  
`catkin_make`  

- Install extra tools  
`sudo apt install ros-noetic-image-view`  
`sudo apt install ros-noetic-dynamic-reconfigure`  
`sudo apt install ros-noetic-rqt-reconfigure`  
`sudo apt install ros-noetic-rosbash`  
`sudo apt install ros-noetic-rqt-graph`
`sudo apt install xterm`  
`sudo apt install jstest-gtk`


## Execution
To run an example of the python code to drive the vehicle, use the following commands:  
`cd ~/dbw_ws`  
`source devel/setup.bash`
`roslaunch workshop duration_drive.launch sys:=true`






