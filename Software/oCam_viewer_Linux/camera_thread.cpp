#include "camera_thread.h"
#include "time.h"

CameraThread::CameraThread(Withrobot::Camera* _ocam)
{
    ocam = _ocam;
}

void CameraThread::run()
{
    g_cnt = 0;
    while(1)
    {
#ifdef test
        if(frame_size == -1)
        {
            break;
        }
        else
        {
            int rtn = ocam->get_frame(frame_buffer, frame_size, 1);
            if (rtn != -1)
            {
                emit getframe(frame_buffer); // notify for get frame
                g_cnt++;
            }
        }

#endif
        usleep(100*1000);
    }
}

void CameraThread::get_signal(int size)
{
    frame_size = size;
    //std::cout << frame_size << std::endl;
    if(frame_size != -1)
    {
        frame_buffer = new unsigned char[frame_size*2];
    }

}
void CameraThread::get_timer()
{
    //std::cout << g_cnt << std::endl;
    g_cnt = 0;
}
