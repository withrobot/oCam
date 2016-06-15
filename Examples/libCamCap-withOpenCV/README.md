# The example of the oCam's windows C/C++ API "libCamCap"
This example shows how to use the "libCamCap" with the OpenCV APIs.


## Windows
### How to build on Windows 7
Requirements
- OpenCV 2.4.11 (`http://opencv.org/downloads.html`)
- Microsoft Visual C++ 2010 (or latest version, Tested on Microsoft Visual C++ 2010)
- Microsoft Windows 7 (or latest version, Tested on Microsoft Windows 7)

1.Download and install OpenCV 2.4.11
- Refer to "opencv.org > Documentation > quick start > Installation in Windows"
    Link: `http://docs.opencv.org/3.0-last-rst/doc/tutorials/introduction/windows_install/windows_install.html#windowssetpathandenviromentvariable`

2.Download oCam master branch to zip
3.Unzip oCam-master.zip in YOUR_WORKING_DIRECTORY
4.Open the MS Visual Studio solution file(libCamCap_withOpenCV.sln) in the following folder.
```
YOUR_WORKING_DIRECTORY\oCam-master\Examples\libCamCap-withOpenCV
```

5.Set up the project in MS Visual C++ 2010
5.1.Open the project property
`"Project > Properties"`

5.2.Change the build configuration to "All Configurations"
`"Configuration > All Configurations"`

5.3.Change the C/C++ additional include directories.
`"Configuration Properties > C/C++ > General > Additional Include Directories > <Edit...>"`
- Change "C:\dev\opencv\2.4.11\build\include" to your opencv include directory.
- Press Ok and Apply.

5.4.Change the Linker additional library directories.
`"Configuration Properties > Linker > General > Additional Library Directories > <Edit...>"`
- Change "C:\dev\opencv\2.4.11\build\x86\vc10\lib" to your opencv library directory.
- Press Ok and Apply.

5.5.Change Debugging Environments
`"Configuration Properties > Debugging > Environments > <Edit...>"`
- Change "PATH=%PATH%;C:\dev\opencv\2.4.11\build\x86\vc10\bin" to "PATH=%PATH%;<YOUR_OPENCV_BINARY_PATH>"
- Press Ok and Apply.

6.Build Solution</br>

7.Start Debugging or Start Without Debugging

