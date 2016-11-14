#Release Note
##Ver. 1611(November 2016)
oCam firmware is updated (161109_oCam-1MGN-U_R1611.img) to support Win 10 as well as Win 7 and Win 8.</br>
It can be used with the Windows libraries of libCamCap.</br>
(Please refer “Software/oCam-viewer_Win” and “Example/libCamCap-withOpenCV” for how to use the libCamCap)</br>


##Ver. 1604(April 2016)
oCam firmware is updated to support more frame rates up to 120 fps.</br>
New frame rates for 640 (H) x  480 (V) pixels and 320 (H) x  240 (V) pixels are;

###USB 3.0
**YUV format**</br>
 640 (H) x  480 (V) pixels   120, 90, 60 fps</br>
 320 (H) x  240 (V) pixels   120, 90, 60 fps</br>

###USB 2.0
**YUV format**</br>
 640 (H) x  480 (V) pixels   60 fps </br>
 320 (H) x  240 (V) pixels   120, 90, 60 fps</br>



##Ver. 1602(February 2016)
oCam is updated to support more resolutions and frame rates
Currently supported resolutions and frame rates are;

###USB 3.0
**YUV format**</br>
2592 (H) x 1944 (V) pixels   7.5, 3.75 fps</br>
1920 (H) x 1080 (V) pixels   15, 7.5 fps</br>
1280 (H) x  960 (V) pixels   30, 15 fps</br>
1280 (H) x  720 (V) pixels   30, 15 fps</br>
 640 (H) x  480 (V) pixels   30, 15 fps</br>
 320 (H) x  240 (V) pixels   30, 15 fps</br>
 
**MJPEG format**</br>
1920 (H) x 1080 (V) pixels   30, 15 fps </br>
1280 (H) x  720 (V) pixels   45, 30, 15 fps </br>
 640 (H) x  480 (V) pixels   30, 15 fps </br>

###USB 2.0
**YUV format**</br>
2592 (H) x 1944 (V) pixels   3.75 fps </br>
1920 (H) x 1080 (V) pixels   7.5 fps </br>
1280 (H) x  960 (V) pixels   15 fps </br>
1280 (H) x  720 (V) pixels   15 fps </br>
 640 (H) x  480 (V) pixels   30 fps </br>
 320 (H) x  240 (V) pixels   30 fps</br>

With PC, oCam can handle all above resolutions and frame rates while the image is being displayed. </br>
However, with ODROID-XU4, the frame rates will be reduced because of the processing load to display the image. Without displaying the image on screen, all the frame rates are supported.</br>

The currently supported resolutions and frame rates on ODROID-XU4 with oCam-viewer while displaying the image are;
###USB 3.0
**YUV format**</br>
2592 (H) x 1944 (V) pixels   3 fps </br>
1920 (H) x 1080 (V) pixels   7 fps </br>
1280 (H) x  960 (V) pixels   11 fps </br>
1280 (H) x  720 (V) pixels   15 fps </br>
 640 (H) x  480 (V) pixels   30, 15 fps </br>
 320 (H) x  240 (V) pixels   30, 15 fps</br>
 
**MJPEG format**</br>
1920 (H) x 1080 (V) pixels   5 fps </br>
1280 (H) x  720 (V) pixels   11 fps </br>
 640 (H) x  480 (V) pixels   30, 15 fps </br>

###USB 2.0
**YUV format**</br>
2592 (H) x 1944 (V) pixels   3 fps </br>
1920 (H) x 1080 (V) pixels   7 fps </br>
1280 (H) x  960 (V) pixels   11 fps </br>
1280 (H) x  720 (V) pixels   15 fps </br>
 640 (H) x  480 (V) pixels   30 fps </br>
 320 (H) x  240 (V) pixels   30 fps</br>

