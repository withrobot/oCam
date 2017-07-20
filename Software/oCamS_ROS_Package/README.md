# oCam-Stereo ROS Package
oCam control program

## Release Note
20170523
	Support for the oCamS-1CGN-U.

## Requirements
- libv4l  (video for linux Two)
- libudev (udev, the device manager for the Linux kernel)

## ROS & oCam ROS Package installation
***1. ROS install*** (If ROS is already installed on your PC, just skip it.)</br>
* refer to the following tutorial.
* We recommend you to install ROS kinetic version
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

***2. oCamS ROS Package download and install***

**2.1 Download and install required library from linux package manager(e.g. apt)**
```
$ sudo apt-get install libv4l-dev libudev-dev
```
**2.2 Download source tree from github**
* using SVN checkout
```
$ cd YOUR_WORKING_DIRECTORY (ex. $ cd ~/catkin_ws/src/)
$ svn export https://github.com/withrobot/oCam/trunk/Software/oCamS_ROS_Package/ocams
```
**2.3 Build Source**
```
$ cd YOUR_CATKIN_WORKSPACE
(ex. $ cd ~/catkin_ws/)
$ catkin_make
$ source devel/setup.bash
```
