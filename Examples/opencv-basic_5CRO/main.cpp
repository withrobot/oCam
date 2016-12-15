/*
 * main.cpp
 *
 *  Withrobot oCam OpenCV example
 *
 *  Created on: Apr 27, 2016
 *      Author: Withrobot
 */


#include <iostream>
#include <string>
#include <queue>

#include "withrobot_platform.hpp"
#include "opencv2/opencv.hpp"


/**
 * Example of the OpenCV VideoCapture.
 *  - Using the independent thread for the high frame rate streaming(>= 60fps.)
 *  - Requirements;
 *      withrobot_platform.hpp(Thread, Mutex)
 */
class Camera
{
    std::queue<cv::Mat> image_buffer;
    std::size_t max_buffer_size;
    double calc_fps;
    bool running;

    int cam_id;
    int cam_width;
    int cam_height;

    Withrobot::Thread rcv_thread;
    Withrobot::Mutex mutex;

    cv::VideoCapture capture;

public:
    /*
     * Camera constructor
     * @input
     *  id : camera id (system dependent value)
     *  w  : streaming image width
     *  h  : streaming image height
     *  fps: streaming image rate (frame per second)
     *  buffer_size: This class internal buffer size(No camera internal buffer size.)
     */
    Camera(const int id, const int w=640, const int h=480, const double fps=30, const int buffer_size=10)
        : cam_id(id), cam_width(w), cam_height(h), max_buffer_size(buffer_size), calc_fps(0), running(false) {
        capture.open(id);
        if (capture.isOpened()) {
            capture.set(CV_CAP_PROP_FRAME_WIDTH, static_cast<double>(w));
            capture.set(CV_CAP_PROP_FRAME_HEIGHT, static_cast<double>(h));
            capture.set(CV_CAP_PROP_FPS, fps);
            std::cout << "Cam #" << id << " is opend!\n";
        }
        else {
            std::cout << "Cam #" << id << " cannot opened!\n";
        }
    }

    ~Camera() {
        stop();
        capture.release();
    }

    /*
     * Start the streaming.
     */
    bool start() {
        running = false;
        if (capture.isOpened()) {
            running = rcv_thread.start(thread_process, this);
        }
        return running;
    }

    /*
     * Stop the streaming.
     */
    void stop() {
        running = false;
        rcv_thread.join();
    }

    /*
     * Get a streaming image.
     */
    bool get_image(cv::Mat& img) {
        LockGuard l(mutex);
        bool retval = false;

        if (!image_buffer.empty()) {
            //img = cv::Mat(cv::Size(cam_width, cam_height), CV_8UC3, image_buffer.front().data, cv::Mat::AUTO_STEP);
            image_buffer.front().copyTo(img);
            image_buffer.pop();
            retval = true;
        }

        return retval;
    }

    /*
     * Get the current frame rate.
     */
    inline double get_fps() {
        LockGuard l(mutex);
        return calc_fps;
    }

private:
    static void* thread_process(void* arg) {
        static_cast<Camera*>(arg)->read_image();
        return 0;
    }

    void read_image() {
        int read_cnt = 0;
        int elap_t = 0;
        Withrobot::Timer timer;

        cv::Mat image;

        while (running) {
            if (capture.read(image)) { // check if we succeeded
                /*
                 * push image to buffer
                 */
                if (image.data != 0) {
                    LockGuard l(mutex);

                    image_buffer.push(image);
                    if (image_buffer.size() > max_buffer_size) {
                        image_buffer.pop();
                    }

                    /*
                     * calcualte fps
                     */
                    read_cnt++;
                    elap_t = timer.now();
                    if (elap_t >= 1) {
                        calc_fps = static_cast<double>(read_cnt) / static_cast<double>(elap_t);
                        read_cnt = 0;
                    }
                }
            }
        }

        std::cout << __FUNCTION__ << " is ternimated." << std::endl;
    }

    class LockGuard {
        Withrobot::Mutex& mutex;
    public:
        LockGuard(Withrobot::Mutex& m): mutex(m) {
            mutex.lock();
        }

        ~LockGuard() {
            mutex.unlock();
        }
    };
};


/**
 * OpenCV putText
 */
class Text
{
    int fontFace;
    double fontScale;
    cv::Scalar color;
    int thickness;
    int lineType;
    bool bottomLeftOrigin;

public:
    Text(int _fontFace, double _fontScale, cv::Scalar _color, int _thickness=1, int _lineType=8, bool _bottomLeftOrigin=false)
        : fontFace(_fontFace), fontScale(_fontScale), color(_color), thickness(_thickness), lineType(_lineType), bottomLeftOrigin(_bottomLeftOrigin) {
    }

    void draw(cv::Mat& image, const cv::Point position, const std::string str) {
        cv::putText(image, str, position, fontFace, fontScale, color, thickness, lineType, bottomLeftOrigin);
    }
};


/**
 * main
 */
int main(int argc, char* argv[])
{
    /*
     * Define Camera settings
     *
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
    const int default_cam_number = 0;
    const int default_cam_width = 640;
    const int default_cam_height = 480;
    const double default_cam_fps = 30;

    /*
     * Create the Camera instance and start streaming;
     */
    Camera ocam(default_cam_number, default_cam_width, default_cam_height, default_cam_fps);
    ocam.start();

    /*
     * Create the Text instance.
     */
    Text text(CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);

    /*
     * Create the named window.
     */
    const std::string window_title = "Hello, oCam!";
    cv::namedWindow(window_title, CV_WINDOW_NORMAL);

    /*
     * Main loop
     *  - put text (image size, streaming fps)
     *  - Show the streaming image
     *  - Exit the main loop; Hit the 'q' key.
     */
    cv::Mat image;
    char kb_input = 0;
    const int keywait_ms = 10;    // ms

    while (kb_input != 'q') {
        if (ocam.get_image(image)) {
            text.draw(image, cv::Point(50, 30), Withrobot::to_string<int>(image.size().width) + "x" + Withrobot::to_string<int>(image.size().height));
            text.draw(image, cv::Point(50, 80), Withrobot::to_string<double>(ocam.get_fps()) + " fps.");
            cv::imshow(window_title, image);
        }

        kb_input = cv::waitKey(keywait_ms);
    }

    return 0;
}


