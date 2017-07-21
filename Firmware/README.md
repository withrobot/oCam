# Release Note

## oCamS-1CGN-U_R1707(Jul 2017)
oCamS firmware(oCamS-1CGN-U_R1707_170719.img) supports resolutions and frame rates as follows.</br>
### USB 3.0
**Bayer GRBG format**</br>
1280 (H) x  960 (V) pixels   45, 30, 25, 20, 15, 10  fps </br>
1280 (H) x  720 (V) pixels   60, 50, 45, 30, 25, 20, 15, 10  fps</br>
 640 (H) x  480 (V) pixels   45, 30, 25, 20, 15, 10  fps (binning)</br>
 640 (H) x  360 (V) pixels   60, 50, 45, 30, 25, 20, 15, 10  fps (binning)</br>

### USB 2.0
**Bayer GRBG format**</br>
 640 (H) x  480 (V) pixels   45, 30, 25, 20, 15, 10  fps (binning)</br>
 640 (H) x  360 (V) pixels   60, 50, 45, 30, 25, 20, 15, 10  fps (binning)</br>
 
- IMU data output
- Auto-Exposure

## oCam-5CRO-U_R1707(Jul 2017)
Bug Fixed; oCam sometimes stops on ODROID board.</br>
Color uniformity improved.

## oCam-1MGN-U_R1707(Jul 2017)
Bug Fixed; oCam sometimes stops on ODROID board.

## oCam-1CGN-U_R1707(Jul 2017)
Bug Fixed; oCam sometimes stops on ODROID board.

## oCamS-1CGN-U_R1705(May 2017)
oCam firmware(oCamS-1CGN-U_R1705_170523.img) supports resolutions and frame rates as follows
### USB 3.0
**Bayer GRBG format**</br>
1280 (H) x 960 (V) pixels   45  fps</br>
1280 (H) x 720 (V) pixels   60  fps</br>
 640 (H) x 480 (V) pixels   45  fps (binning)</br>

### USB 2.0
**Bayer GRBG format**</br>
 640 (H) x 480 (V) pixels   45  fps (binning)</br>

## oCam-1CGN-U_R1705(May 2017)
oCam firmware(oCam-1CGN-U_R1705_170428.img) supports resolutions and frame rates as follows

### USB 3.0
**Bayer GRBG format**</br>
1280 (H) x  960 (V) pixels   45  fps</br>
1280 (H) x  720 (V) pixels   60  fps</br>
 640 (H) x  480 (V) pixels   80  fps (binning)</br>
 320 (H) x  240 (V) pixels   160 fps (cropping)</br>

### USB 2.0
**Bayer GRBG format**</br>
1280 (H) x  960 (V) pixels   22.5 fps</br>
1280 (H) x  720 (V) pixels   30  fps</br>
 640 (H) x  480 (V) pixels   80  fps (binning)</br>
 320 (H) x  240 (V) pixels   160 fps (cropping)</br>

## oCam-1MGN-U_R1705(May2017)
Bug Fixed; When first start the oCam-viewer does not display images.</br>
Changed from cropping to binning for 640x480(80fps) resolution.

## oCam-1MGN-U_R1703(March 2017)
Bug Fixed; When first start the oCam-viewer does not display images.</br>

## oCam-1MGN-U_R1611(November 2016)
oCam firmware(oCam-1MGN-U_R1611_161109.img) is updated to support Win 10 as well as Win 7 and Win 8.</br>
It can be used with the Windows libraries of libCamCap.</br>
(Please refer “Software/oCam-viewer_Win” and “Example/libCamCap-withOpenCV” for how to use the libCamCap)</br>

## oCam-1MGN-U_R1607(July 2016)
oCam firmware(oCam-1MGN-U_R1607_160722.img) supports resolutions and frame rates as follows

### USB 3.0
**Y800 format**</br>
1280 (H) x  960 (V) pixels   45 fps</br>
1280 (H) x  720 (V) pixels   60 fps</br>
 640 (H) x  480 (V) pixels   80 fps</br>
 320 (H) x  240 (V) pixels   160 fps</br>

### USB 2.0
**Y800 format**</br>
1280 (H) x  960 (V) pixels   22.5 fps</br>
1280 (H) x  720 (V) pixels   30 fps</br>
 640 (H) x  480 (V) pixels   80 fps</br>
 320 (H) x  240 (V) pixels   160 fps</br>

## oCam-5CRO-U_R1607(December 2016)
The firmware for the oCam-5CRO series cameras is updated to be compatible with the latest oCam-viewer and the Windows libraries of libCamCap released on November to support Win 10.</br>

## oCam-5CRO-U_R1604(April 2016)
oCam firmware(oCam-5CRO-U_R1604_160419.img) is updated to support more frame rates up to 120 fps.</br>
New frame rates for 640 (H) x  480 (V) pixels and 320 (H) x  240 (V) pixels are;

### USB 3.0
**YUV format**</br>
 640 (H) x  480 (V) pixels   120, 90, 60 fps</br>
 320 (H) x  240 (V) pixels   120, 90, 60 fps</br>

### USB 2.0
**YUV format**</br>
 640 (H) x  480 (V) pixels   60 fps </br>
 320 (H) x  240 (V) pixels   120, 90, 60 fps</br> 



## oCam-5CRO-U_R1602(February 2016)
oCam firmware(oCam-5CRO-U_R1602_160218.img)is updated to support more resolutions and frame rates
Currently supported resolutions and frame rates are;

### USB 3.0
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

### USB 2.0
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
### USB 3.0
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

### USB 2.0
**YUV format**</br>
2592 (H) x 1944 (V) pixels   3 fps </br>
1920 (H) x 1080 (V) pixels   7 fps </br>
1280 (H) x  960 (V) pixels   11 fps </br>
1280 (H) x  720 (V) pixels   15 fps </br>
 640 (H) x  480 (V) pixels   30 fps </br>
 320 (H) x  240 (V) pixels   30 fps</br>

