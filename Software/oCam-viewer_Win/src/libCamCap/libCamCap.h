/* libCamCap.h */

#ifndef LIBCAMCAP_H_
#define LIBCAMCAP_H_

#define INFO_USB_TYPE		1
#define INFO_SERIAL_NUM		2
#define INFO_MODEL_NAME		3
#define INFO_DATE_TIME		4

#define CTRL_BRIGHTNESS		1
#define CTRL_CONTRAST		2
#define CTRL_HUE			3
#define CTRL_SATURATION		4
#define CTRL_EXPOSURE		5
#define CTRL_GAIN			6
#define CTRL_WHITEBALANCE_COMPONENT_BLUE	7
#define CTRL_WHITEBALANCE_COMPONENT_RED		8

#ifdef LIBCAMCAP_EXPORTS
#define LIBCAMCAP_API   __declspec(dllexport)
#else
#define LIBCAMCAP_API   __declspec(dllimport)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  CAMPTR : Pointer(Handle) of the opened camera
 */
typedef void* CAMPTR;

/**
 *  CamOpen
 *      Open the connected oCam. When the function is failure, than return NULL.
 *
 *      - Input:
 *          const int ID        : Camera number (e.g. 0, 1, 2 ...)
 *          const int Width     : Width of the output image (e.g. 640)
 *          const int Height    : Height of the output image (e.g. 480)
 *          const double Fps    : Frame per second (e.g. 30.0)
 *          void(*FtnCB)(void*) : Pointer of the callback function
 *          void* Para          : Parameter of the callback function
 *
 *      - Output:
 *          CAMPTR              : Pointer(Handle) of the opened camera
 */
LIBCAMCAP_API CAMPTR CamOpen(const int ID, const int Width, const int Height, const double FPS, void(*FtnCB)(void *Para, void *Data), void *Para);

/**
 *  CamStart
 *      Start the video streaming.
 *      
 *      - Input:
 *          CAMPTR ptrCam       : Handle of the opened camera
 *
 *      - Output:
 *          int                 : Failed(0) or Success(1)
 */
LIBCAMCAP_API int CamStart(CAMPTR ptrCam);

/**
 *  CamStop
 *      Stop the video streaming.
 *      
 *      - Input:
 *          CAMPTR ptrCam       : Handle of the opened camera
 *
 *      - Output:
 *          int                 : Failed(0) or Success(1)
 */
LIBCAMCAP_API int CamStop(CAMPTR ptrCam);

/**
 *  CamClose
 *      Close the opened camera; Release the camera handle.
 *      
 *      - Input:
 *          CAMPTR ptrCam       : Handle of the opened camera
 */
LIBCAMCAP_API void CamClose(CAMPTR& ptrCam);

/**
 *  CamGetImage
 *      Get camera image
 *      
 *      - Input:
 *          CAMPTR ptrCam       : Handle of the opened camera
 *          BYTE* pImage        : Pointer of image
 *
 *      - Output:
 *          int                 : Failed(0) or Success(1)
 */
LIBCAMCAP_API int CamGetImage(CAMPTR ptrCam, BYTE* pImage);

/**
 *  CamSetCtrl
 *      Set camera control parameter
 *      
 *      - Input:
 *          CAMPTR ptrCam       : Handle of the opened camera
 *          int CtrlPara        : Control parameter
 *          int Value           : Control value
 *
 *      - Output:
 *          int                 : Failed(0) or Success(1)
 */
LIBCAMCAP_API int CamSetCtrl(CAMPTR ptrCam, int CtrlPara, long Value);

/**
 *  CamGetCtrl
 *      Get camera control parameter
 *      
 *      - Input:
 *          CAMPTR ptrCam       : Handle of the opened camera
 *          int CtrlPara        : Control parameter
 *          int* Value          : Control value
 *
 *      - Output:
 *          int                 : Failed(0) or Success(1)
 */
LIBCAMCAP_API int CamGetCtrl(CAMPTR ptrCam, int CtrlPara, long* Value);

/**
 *  CamGetCtrlRange
 *      Get camera parameter ranges
 *      
 *      - Input:
 *          CAMPTR ptrCam       : Handle of the opened camera
 *          int CtrlPara        : Control parameter
 *          int* MinValue       : Control min value
 *          int* MaxValue       : Control max value
 *
 *      - Output:
 *          int                 : Failed(0) or Success(1)
 */
LIBCAMCAP_API int CamGetCtrlRange(CAMPTR ptrCam, int CtrlPara, long* MinValue, long* MaxValue);

/**
 *  GetConnectedCamNumber
 *      Search the connected oCam-5CRO-U and return the number of the connected camera.
 *      
 *      - Output:
 *          int                 : Failed(-1, 0) or the number of connected camera
 */
LIBCAMCAP_API int GetConnectedCamNumber();

/**
 *  CamGetDeviceInfo
 *      Get device information.
 *      
 *      - Input:
 *          CAMPTR ptrCam       : Handle of the opened camera
 *          int    Info         : Type of information
 *
 *      - Output:
 *          char*               : Information string of Camera
 */
LIBCAMCAP_API char* CamGetDeviceInfo(const int ID, int Info);

#ifdef __cplusplus
}
#endif

#endif  /* LIBCAMCAP_H_ */