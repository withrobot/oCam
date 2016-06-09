/* libCamCap.h */

#ifndef LIBCAMCAP_H_
#define LIBCAMCAP_H_

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
 *      Open the connected oCam-5CRO-U. When the function is failure, than return NULL.
 *
 *      - Input:
 *          const int id        : Camera number (e.g. 0, 1, 2 ...)
 *          const int Width     : Width of the output image (e.g. 640)
 *          const int Height    : Height of the output image (e.g. 480)
 *          const double Fps    : Frame per second (e.g. 30.0)
 *          void(*FtnCB)(void*) : Pointer of the callback function
 *
 *      - Output:
 *          CAMPTR              : Pointer(Handle) of the opened camera
 */
LIBCAMCAP_API CAMPTR CamOpen(const int id, const int Width, const int Height, const double Fps, void(*FtnCB)(void*));

/**
 *  CamStart
 *      Start the video streaming.
 *      
 *      - Input:
 *          CAMPTR ptrCam       : Handle of the opend camera
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
 *          CAMPTR ptrCam       : Handle of the opend camera
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
 *          CAMPTR ptrCam       : Handle of the opend camera
 */
LIBCAMCAP_API void CamClose(CAMPTR ptrCam);

/**
 *  GetConnectedCamNumber
 *      Search the connected oCam-5CRO-U and return the number of the connected camera.
 *      
 *      - Output:
 *          int                 : Failed(-1, 0) or the number of connected camera
 */
LIBCAMCAP_API int GetConnectedCamNumber();


#ifdef __cplusplus
}
#endif

#endif  /* LIBCAMCAP_H_ */