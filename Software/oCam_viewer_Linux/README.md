# oCam-viewer 
oCam control program

## Release Note
* 20191007</br>
	Support for the oCamS-1XGN-U.

* 20170925</br>
	Add: Auto-Exposure Control menu.

* 20170420</br>
	Support for the oCam-1CGN-U.

## How to build on linux
Requirements
- qt4
- libv4l  (video for linux Two)
- libudev (udev, the device manager for the Linux kernel)

1.Download source three from github.
- using SVN checkout
```
$ cd YOUR_WORKING_DIRECTORY
$ svn export https://github.com/withrobot/oCam/trunk/Software/oCam_viewer_Linux
```

2.Download and install from linux package manager(e.g. apt).
```
$ sudo apt-get install qt4-default libv4l-dev libudev-dev
```

3.Build.
```
$ cd ./oCam_viewer_Linux
$ mkdir build
$ cd ./build
$ qmake ..
$ make release
```

## How to run
```
$ cd OCAM_VIEWER_BUILD_DIRECTORY
$ ./oCam-viewer
```
