# oCam-Stereo ROS Package
oCamS ROS control program

## Release Note
***2017.07.19***
* Support for the oCamS-1CGN-U_R1707
* Auto-Exposure
* Added 640x360 resolution and more frame rates
* Camera controller

***2017.05.23***
* Support for the oCamS-1CGN-U


## Requirements
- ROS
- Linux OS
- libv4l  (Video for Linux Two)
- libudev (udev, the device manager for the Linux kernel)


## ROS & oCam ROS Package installation
**1. ROS install** (If ROS is already installed on your PC, just skip it.)</br>
* Refer to the following tutorial.</br>
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
* We recommend you to install ROS kinetic version.


**2. oCamS ROS Package download and install**

  ##### 2.1 Download and install required library from linux package manager(e.g. apt)
  ```
  $ sudo apt-get install libv4l-dev libudev-dev
  ```
  ##### 2.2 Download source tree from github
  * using SVN checkout
  ```
  $ cd YOUR_WORKING_DIRECTORY (ex. $ cd ~/catkin_ws/src/)
  $ svn export https://github.com/withrobot/oCam/trunk/Software/oCamS_ROS_Package/ocams
  ```
  ##### 2.3 Build
  ```
  $ cd YOUR_CATKIN_WORKSPACE (ex. $ cd ~/catkin_ws/)
  $ catkin_make
  $ source devel/setup.bash
  ```

**3. Run**</br>
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
$ roslaunch ocams ocams_ros.launch
```

## Update Firmware
**1. Firmware update on Windows OS**</br>
- You can find the latest firmware at https://github.com/withrobot/oCam/blob/master/Firmware</br>
  (The latest firmware version is [oCamS-1CGN-U_R1707_170719](https://github.com/withrobot/oCam/raw/master/Firmware/oCamS-1CGN-U_R1707_170719.img))
  - Correct file for the oCamS-1CGN-U should be selected for the firmware update
- You can find the firmware writing program at https://github.com/withrobot/oCam/tree/master/Firmware/Update_FW</br>
  - Uncompress the downloaded "UpdateFW.zip"</br>
  - Follow the instruction described at https://github.com/withrobot/oCam/tree/master/Firmware
