# Pioneer 3DX and Cyton Gamma 1500 Mobile Manipulator

![Screenshot from 2017-05-06 20-47-35.png](https://bitbucket.org/repo/5qxggj8/images/3122682148-Screenshot%20from%202017-05-06%2020-47-35.png)

## Introduction

This is the code for Marquette University's 2017 E53: Development of Simulation Tools for Mobile Robotic Manipulation Senior Design Team

## Summary
The following project is a ROS-Gazebo implementation of a mobile manipulator. In particular, this project details the fusion of the [Cyton Gamma 1500 robotic arm](https://github.com/SteveMacenski/cyton_gamma_300-1500_operation_and_simulation) and the Pioneer 3DX robotic base (Example ROS package: multi_robot_scenario). The result is a combined robot ready for simulation testing via python script. This project was created under Ubuntu 16.04 with ROS Kinetic and Gazebo 7.0. This document covers a setup guide from scratch and running a basic example with python script. As much as possible, this document tries to reference relevant materials, useful debug tools, and existing problems with the robot and how one might go about and fix them.

## Basic Setup Guide
The following terminal commands for dependencies installation and setup are provided.

#### Gazebo Installation
The basic program is not only required, but also the developer package.
```
#!terminal
sudo apt-get update
sudo apt-get install gazebo7 libgazebo7-dev
```

#### ROS Installation and Setup
ROS provides an easy-to-follow [installation and setup guide](http://wiki.ros.org/kinetic/Installation/Ubuntu).

Install ROS
```
#!terminal
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
```

Setup ROS
```
#!terminal
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/kinetic/setup.bash
```

Install ROS related packages with terminal commands.
```
#!terminal
sudo apt-get install python python-rosinstall pyassimp ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control ros-kinetic-moveit-visual-tools ros-kinetic-moveit
```

Setup and install ROS catkin workspace via the following terminal commands.
```
#!terminal
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

#### Add Source Repositories
The existing [repository](https://github.com/SteveMacenski/cyton_gamma_300-1500_operation_and_simulation) is used. Notice that several folders are removed for incompatibility with ROS Kinetic.
```
#!terminal
cd ~/catkin_ws/src
git clone https://github.com/SteveMacenski/cyton_gamma_300-1500_operation_and_simulation.git
rm -r cyton_gamma_300-1500_operation_and_simulation/moveit_plugins-indigo-devel/
rm -r cyton_gamma_300-1500_operation_and_simulation/industrial_core-indigo-devel/
cd ~/catkin_ws
catkin_make
```
To test if that package works, run the example program. Notice that a Gazebo and RVIZ Window should pop up. A Gazebo and RVIZ Window should pop up and both the arm should be fully controllable. To test, on the RVIZ window, open the Planning Tab. Under “Workspace:”, set the Center XYZ values to anything nonzero. Click the Update Button, click the Plan and Execute Button, and then watch the arm move in both Gazebo and RVIZ.
```
#!terminal
roslaunch cyton_gamma_pkg simulation_gamma_1500.launch
```

Now, add this repository to the catkin_workspace.
```
#!terminal
cd ~/catkin_ws/src
git clone https://Walden95@bitbucket.org/Walden95/pioneer-3dx-and-cyton-gamma-1500-mobile-manipulator.git
cd ~/catkin_ws
catkin_make
```

Notice that while this project does make use of the example ROS package multi_robot_scenario found at */opt/ros/kinetic/share/gazebo_plugins/test*, a modified version to fix problems with the latest version of ROS is included. See this [blog post](https://afsyaw.wordpress.com/2017/01/12/running-the-pioneer-3dx-in-gazebo-and-ros-kinetic/) on why this is necessary.

## Basic Run Guide

#### Standalone Robot
To test if this repository works, run the standalone example program. A Gazebo and RVIZ Window should pop up and both the differential drive and arm should be fully controllable.
```
#!terminal
roslaunch p3dx_cyton1500_fusion fusion_gazebo.launch 
```

#### Python and World Example
The basic python control script is made for the demo world launch file. This program is horribly hard coded and calls the terminal interface for the differential drive. Notice that the world makes use of Gazebo example models not included on installation. To do so, open up Gazebo, click the insert tab, and under http://gazebosim.org/models/, select Cafe and Coke Can. The files should be downloaded and placed in the appropriate folder automatically.
```
#!terminal
roslaunch p3dx_cyton1500_fusion world_demo.launch 
```
Run the python code in separate terminal.
```
#!terminal
cd ~/catkin_ws/src/pioneer-3dx-and-cyton-gamma-1500-mobile-manipulator/base-arm_fusion/p3dx_cyton1500_fusion/python
python world_demo.py
```

For feedback, contact ryan.w.walsh@marquette.edu