# oCam ROS Package
oCam-1xGN-U and oCam-1xGN-U-T ROS control program

## Release Note
***2020.05.08***
* oCam-1xGN-U and oCam-1xGN-U-T get frame ROS example
* Added Auto-Exposure on/off using the launch file


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


**2. oCam ROS Package download and install**

  ##### 2.1 Download and install required library from linux package manager(e.g. apt)
  ```
  $ sudo apt-get install libv4l-dev libudev-dev ros-kinetic-rtabmap*
  ```
  ##### 2.2 Download source tree from github
  * using SVN checkout
  ```
  $ cd YOUR_WORKING_DIRECTORY (ex. $ cd ~/catkin_ws/src/)
  $ svn export https://github.com/withrobot/oCamS/trunk/Software/oCamS_ROS_Package/ocams_1cgn
  ```
  ##### 2.3 Build
  ```
  $ cd YOUR_CATKIN_WORKSPACE (ex. $ cd ~/catkin_ws/)
  $ catkin_make
  $ source devel/setup.bash
  ```

**3. Run**</br>
```
$ roslaunch ocam ocam_ros.launch
```

