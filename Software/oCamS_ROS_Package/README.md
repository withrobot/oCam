# oCam-Stereo ROS Package
oCamS ROS control program

## Release Note
***2017.07.19***
* Support oCamS-1CGN-U IMU.
* Auto-Exposure.
* Added 640x360 resolution and more frame rates.
* Camera controller.</br></br>

***2017.05.23***
* Support for the oCamS-1CGN-U.

## Requirements
- libv4l  (video for linux Two)
- libudev (udev, the device manager for the Linux kernel)

## ROS & oCam ROS Package installation
***1. ROS install*** (If ROS is already installed on your PC, just skip it.)</br>
* Refer to the following tutorial.
* We recommend you to install ROS kinetic version.</br>
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
***3. Run***</br>
oCamS-1CGN-U sends IMU data through Virtual COM port.</br>
So, user needs to write following rules into udev rule file like below.
```
$ sudo vi /etc/udev/rules.d/99-ttyacms.rules
ATTRS{idVendor}=="04b4" ATTRS{idProduct}=="00f9", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
ATTRS{idVendor}=="04b4" ATTRS{idProduct}=="00f8", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
$ sudo udevadm control –reload-rules
```
And, run...
```
$ roslaunch ocams disparity.launch
```
