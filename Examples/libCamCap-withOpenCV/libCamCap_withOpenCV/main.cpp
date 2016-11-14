#include <Windows.h>
#include <iostream>

#include "opencv2/opencv.hpp"
#include "libCamCap.h"

using namespace cv;

#define		MODEL_5CRO	0
#define		MODEL_1MGN	1

/*
 *  oCam-5CR-U (Ver. 1604) supported resolutions and frame rates.
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
 *
 *
 *  oCam-1MGN-U (Ver. 1611) supported resolutions and frame rates.
 *
 *  [USB 3.0 - GREY format]
 *  1280 x 960    @ 45 fps
 *  1280 x 720    @ 60 fps
 *   640 x 480    @ 80 fps
 *   320 x 240    @ 160 fps
 *
 *  [USB 2.0 - GREY format]
 *  1280 x 960    @ 22.5 fps
 *  1280 x 720    @ 30 fps
 *   640 x 480    @ 80 fps
 *   320 x 240    @ 160 fps
 */

static const int IMAGE_WIDTH = 1280;     // pixel
static const int IMAGE_HEIGHT = 720;    // pixel
static const double IMAGE_FPS = 30.0;   // frame per second

/*
 *  Main Function
 */
int main(int argc, char* argv[])
{
	int camNum = GetConnectedCamNumber();
	if (camNum==0)
		printf("Can not find oCam\n");

	int model = 0;
	char *camModel = CamGetDeviceInfo(0, INFO_MODEL_NAME);
	if (strcmp(camModel, "oCam-5CRO-U")==0)
		model = MODEL_5CRO;
	else
		model = MODEL_1MGN;

	Mat image, image_rgb;
	if (model == MODEL_5CRO)
	{
		image = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC2);
		image_rgb = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
	}
	else
	{
		image = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
	}

    /* Create the named window for imshow. */
    const char* windowName = "oCam";
    cv::namedWindow(windowName);

    /* Open oCam */
    CAMPTR ptrCam = CamOpen(0, IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_FPS, NULL, NULL);;

    /* Start the camera */
    CamStart(ptrCam);

    while (true) 
	{
        /* Get the image */
		if (CamGetImage(ptrCam, image.data))
		{
	        /* Show the image */
			if (model == MODEL_5CRO)
			{
				cvtColor(image, image_rgb, CV_YUV2BGR_YUYV);
				imshow(windowName, image_rgb);
			}
			else
			{
			    imshow(windowName, image);
			}
		}

        /* When pressed 'q' key then exit the loop. */
        if (waitKey(5) == 'q')
            break;
    }

    /* Stop the streamming */
    CamStop(ptrCam);

    /* Close the oCam */
    CamClose(ptrCam);

    return 0;
}

#if 0
	disp_3d_data = Mat(480, 640, CV_8UC3);
	Mat disp_3d_data = Mat(480, 640, CV_8UC3);
	if (model == MODEL_5CRO)
	{
		Mat disp_3d_data = Mat(480, 640, CV_8UC3);
	    cv::Mat ImageYUV = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC2);
	    cv::Mat ImageYUV = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC2);
	}
	else if (strcmp(model, "oCam-1MGN-U")==0)
	{
	}

    /* Create the BGR image; Create the cv::Mat instance for the BGR image. */
    if (camImageBGR == NULL) {
        camImageBGR = new cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    }

	char *model = CamGetDeviceInfo(0, INFO_MODEL_NAME);
	if (strcmp(model, "oCam-5CRO-U")==0)
	{
		Mat disp_3d_data = Mat(480, 640, CV_8UC3);
	    cv::Mat ImageYUV = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC2);
	    cv::Mat ImageYUV = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC2);
	}
	else if (strcmp(model, "oCam-1MGN-U")==0)
	{
	}
///*
// *  BGR Image
// */
//static cv::Mat* camImageBGR = NULL;
//
///*
// *  Callback function for CamOpen argument;
// */
//void imageReadCB(void* para, void *data)
//{
//    cv::Mat camImageYUV = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC2, static_cast<unsigned char*>(data));
//    
//    // convert color space for YUYV to BGR.
//    cv::cvtColor(camImageYUV, *camImageBGR, CV_YUV2BGR_YUYV);
//}

#endif