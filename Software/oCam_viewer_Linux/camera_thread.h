#ifndef CAMERA_THREAD_H
#define CAMERA_THREAD_H
#include <QThread>
#include "withrobot_camera.hpp"

//#define test // camera thread test

class CameraThread : public QThread
{
    Q_OBJECT

public:
    CameraThread(Withrobot::Camera* ocam);
    void run();
    Withrobot::Camera* ocam;
    bool getStart_signal;
    Withrobot::camera_format format;
    int frame_size;
    unsigned char* frame_buffer;
    int g_cnt;
signals:
    //void getframe(const char* buffer);
    void getframe(unsigned char* output);

public slots:
    void get_signal(int size);
    void get_timer();
};
#endif // CAMERATHREAD_H
