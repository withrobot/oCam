#include <Windows.h>
#include <iostream>

#include "opencv2/opencv.hpp"
#include "libCamCap.h"

/*
 * Withrobot oCam-5CR-U3 (Ver. 1604) supported resolutions and frame rates.
 *
 *  [USB 3.0 - YUV format]
 *  2592 x 1944   @ 3.75 fps, 7.50 fps
 *  1920 x 1080   @ 7.50 fps, 15 fps
 *  1280 x 960    @ 15 fps, 30 fps
 *  1280 x 720    @ 15 fps, 30 fps
 *   640 x 480    @ 30 fps, 60 fps, 90 fps, 120 fps
 *   320 x 240    @ 30 fps, 60 fps, 90 fps, 120 fps
 *
 *  [USB 2.0 - YUV format]
 *  2592 x 1944   @ 3.75 fps
 *  1920 x 1080   @ 7.50 fps
 *  1280 x 960    @ 15 fps
 *  1289 x 720    @ 15 fps
 *   640 x 480    @ 30 fps, 60 fps
 *   320 x 240    @ 30 fps, 60 fps, 90 fps, 120 fps
 */

static const int IMAGE_WIDTH = 640;     // pixel
static const int IMAGE_HEIGHT = 480;    // pixel
static const double IMAGE_FPS = 30.0;   // frame per second

/*
 *  BGR Image
 */
static cv::Mat* camImageBGR = NULL;

/*
 *  Callback function for CamOpen argument;
 */
void imageReadCB(void* data)
{
    cv::Mat camImageYUV = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC2, static_cast<unsigned char*>(data));
    
    // convert color space for YUYV to BGR.
    cv::cvtColor(camImageYUV, *camImageBGR, CV_YUV2BGR_YUYV);
}

/*
 *  Main Function
 */
int main(int argc, char* argv[])
{
    /* Create the BGR image; Create the cv::Mat instance for the BGR image. */
    if (camImageBGR == NULL) {
        camImageBGR = new cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    }

    /* Open the first connected (0) oCam-OCRO-U. */
    CAMPTR ptrCam0 = CamOpen(0, IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_FPS, imageReadCB);;

    /* Start the camera 0 */
    CamStart(ptrCam0);

    /* Create the named window for imshow. */
    const char* windowName = "oCam-5CRO-U 0";
    cv::namedWindow(windowName);

    while (true) {
        /* Show the BGR image */
        cv::imshow(windowName, *camImageBGR);

        /* When pressed 'q' key then exit the loop. */
        if (cv::waitKey(5) == 'q') {
            break;
        }
    }

    /* Stop the streamming */
    CamStop(ptrCam0);
    /* Close the camera 0 */
    CamClose(ptrCam0);

    /* Delete the BGR image */
    delete camImageBGR;

    return 0;
}