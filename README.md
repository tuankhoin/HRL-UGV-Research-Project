# HRL-UGV-Research-Project
This project is developed by Human Robotics Lab at the University of Melbourne based on **[Ubuntu](https://releases.ubuntu.com/)** and **[ROS](http://wiki.ros.org)**. If you have any questions, feel free to contact me: yangmengfeix@student.unimelb.edu.au.

# Introduction:
The framework of this project consists three parts: communication layer, function layer and decision layer.

The communication layer is in charge of intra-robot communication between the on-board computer and the Real Time low-level robot controller. Currently, it supports the ACCR-UTGV platform and will support DJI AI Robot in short future. These are two main UGV platforms we have currently in the lab.

The function layer contains the most of the functions including perception, navigation, and some other small functions.

The decision layer currently can demonstrate a certain level of intelligence by using differnt algorithms. In future, the decision layer will support multi-agent decision-making framework.

Aside from these three layers, a simulator based on the Gazebo is included to provide a virtual platform for testing purpose. It is expected to be replaced by the Unity engine in the next step.

# Instruction:
## 1. ROS Installation
This project has been tested on both ROS Melodic (with Ubuntu 18.04) and ROS Noetic (with Ubuntu 20.04). The main development is based on ROS Noetic and Ubuntu 20.04.
ROS installation instructions can be found via ROS website. Download [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) or [Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) based on the Ubuntu version. The *desktop-full* version is recommended for this project.

## *Optional* (Recommanded)
After finishing the installation of ROS, you can use the following code to make your ROS environment working **globally**.

    $ echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

In this case, in the following steps, you can always **ignore** the following code.

    $ source /opt/ros/$ROS_DISTRO/setup.bash

This makes your life easier, as you don't need to include this line every time when you use a ROS command in a new terminal.

If you want to **remove** this global setting from your device, simply go to `Home`, and then press `Ctrl` + `H` to show all hidden files, then open the `.bashrc` file and delete the following line:

    source /opt/ros/$ROS_DISTRO/setup.bash
    
## 2. Packages Installation
Install necessary dependency packages:

    $ sudo apt clean && sudo apt update
    $ sudo apt install libgoogle-glog-dev 
    $ source /opt/ros/$ROS_DISTRO/setup.bash
    $ sudo apt-get install -y ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-navigation ros-$ROS_DISTRO-tf2-sensor-msgs ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-teb-local-planner ros-$ROS_DISTRO-realsense2-camera
    $ sudo apt-get update && sudo apt-get upgrade    

## 3. Complie
As long as the package is under some child directory of src, catkin can detect it automatically. The next step is then to build the system.

    $ source /opt/ros/$ROS_DISTRO/setup.bash
    $ cd <yourpath>/HRL-UGV-Research-Project
    $ catkin_make
    
## 4. Test your project
Several different launch files will be provided after finish testing.

# TODO
At this beginning level, we still have a lot of works that need to be done. Here are some short-term goals:

* Communication layer:
  * Update the ACCR-UTGV protocol (currently under testing).
  * Add support for DJI AI robot. :heavy_check_mark:
  * Add MQTT protocol to allow data communication between agents or a monitor device.
  
* Function layer:
  * Add computer vision function.
  * Add navigation function. 
  * Add Livox 3D LiDAR support.
  * Add RPLiDAR support.
  
* Decision layer:
  * Add Finite State Machine support, and examples.
  * Add Decision Tree support, and examples.
  * Add multi-agent support in far future.
