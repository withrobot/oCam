/*******************************************************************************#
#                                                                               #
# Withrobot Camera API                                                          #
#                                                                               #
# Copyright (C) 2016 Withrobot. Inc.                                            #
#                                                                               #
# This program is free software: you can redistribute it and/or modify          #
# it under the terms of the GNU General Public License as published by          #
# the Free Software Foundation, either version 3 of the License, or             #
# (at your option) any later version.                                           #
#                                                                               #
# This program is distributed in the hope that it will be useful,               #
# but WITHOUT ANY WARRANTY; without even the implied warranty of                #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                 #
# GNU General Public License for more details.                                  #
# You should have received a copy of the GNU General Public License             #
# along with this program.  If not, see <http://www.gnu.org/licenses/>          #
#                                                                               #
********************************************************************************/

#ifndef WITHROBOT_CAMERA_HPP_
#define WITHROBOT_CAMERA_HPP_

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <libv4l2.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <memory.h>
#include <vector>
#include <string>
#include <cstring>
#include <map>
#include <sstream>
#include <stdio.h>

#include "withrobot_utility.hpp"
#include <ros/ros.h>


#define WITHROBOT_CAMERA_DEFAULT_WIDTH     640
#define WITHROBOT_CAMERA_DEFAULT_HEIGHT    480
#define WITHROBOT_CAMERA_DEFAULT_FIXFORMAT V4L2_PIX_FMT_YUYV

/*
 * DBG print
 */
//#define PRINT_DEBUG_MSG	/* When you want to show the debug messages remove the comment here. */

#ifdef PRINT_DEBUG_MSG

#define DBG_PRINTF(...) {\
    ROS_INFO(__VA_ARGS__);\
}
//#define DBG_PRINTF(...) {\
//    fprintf(stdout, "DBG: [%s, %d, %s] ", __FILE__, __LINE__, __FUNCTION__); \
//    fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n"); fflush(stdout); \
//}

#define DBG_PRINTF_MSG(...) { fprintf(stdout, __VA_ARGS__); fflush(stdout); }
#define DBG_PERROR(...) { fflush(stdout); printf("DBG_ERR: [%s, %d, %s] ", __FILE__, __LINE__, __func__); fflush(stdout); perror(__VA_ARGS__); fflush(stdout); }

#else
#define DBG_PRINTF(...)
#define DBG_PRINTF_MSG(...)
#define DBG_PERROR(...)

#endif /* PRINT_DEBUG_MSG */



/**
 * @defgroup Withrobot Camera API
 * @brief Withrobot inc. UVC type camera control API
 *
 * Withrobot UVC type camera control API
 *
 * @{
 */

namespace Withrobot {
    /**
     * FOURCC 문자를 pixel format 코드로 변환하는 함수
     *
     * e.g. fourcc_to_pixformat('Y', 'U', 'Y', 'V')
     *
     * @param a [입력] first character
     * @param b [입력] second character
     * @param c [입력] third character
     * @param d [입력] forth character
     * @return pixel_format 코드
     */
    inline static unsigned int fourcc_to_pixformat(const char a, const char b, const char c, const char d) {
        return (((unsigned int)a << 0)|((unsigned int)b << 8)|((unsigned int) c << 16)|((unsigned int)d << 24));
    }

    /**
     * Camera control types
     */
    enum camera_control_type {
        CAM_CTRL_TYPE_INTEGER      = 1,//!< 정수
        CAM_CTRL_TYPE_BOOLEAN      = 2,//!< 불리언
        CAM_CTRL_TYPE_MENU         = 3,//!< 하위 메뉴
        CAM_CTRL_TYPE_BUTTON       = 4,//!< 버튼
        CAM_CTRL_TYPE_INTEGER64    = 5,//!< 64비트 정수
        CAM_CTRL_TYPE_CTRL_CLASS   = 6,//!< 컨트롤 클래스
        CAM_CTRL_TYPE_STRING       = 7,//!< 문자열
        CAM_CTRL_TYPE_BITMASK      = 8,//!< 비트 마스크
        CAM_CTRL_TYPE_INTEGER_MENU = 9 //!< 정부 메뉴
    };

    /**
     * Camera control flags
     */
    enum camera_control_flags {
        CAM_CTRL_FLAG_DISABLED      = 1, //!< Disabled
        CAM_CTRL_FLAG_GRABBED       = 2, //!< Grabbed
        CAM_CTRL_FLAG_READ_ONLY     = 4, //!< Read only
        CAM_CTRL_FLAG_UPDATE        = 8, //!< Update
        CAM_CTRL_FLAG_INACTIVE      = 16,//!< Inactive
        CAM_CTRL_FLAG_SLIDER        = 32,//!< Slider
        CAM_CTRL_FLAG_WRITE_ONLY    = 64 //!< Wirte only
    };

    /**
     * Camera image format structure
     */
    struct camera_format {
        unsigned int pixformat;
        unsigned int width;
        unsigned int height;
        unsigned int image_size;

        unsigned int rate_numerator;
        unsigned int rate_denominator;

        double frame_rate;

        camera_format() { clear(); }
        void clear() { memset(this, 0, sizeof(*this)); }

        void print() {
            printf("Pixel format: %c, %c, %c, %c\n", (pixformat >> 0) & 0xFF, (pixformat >> 8) & 0xFF, (pixformat >> 16) & 0xff, (pixformat >> 24) & 0xFF);
            printf("Width: %d, Height: %d, Image size: %d\n", width, height, image_size);
            printf("Frame Rate : %d / %d (%.2f fps)\n", rate_numerator, rate_denominator, frame_rate);
        }

        void dbg_print() {
            DBG_PRINTF("Pixel format: %c, %c, %c, %c", (pixformat >> 0) & 0xFF, (pixformat >> 8) & 0xFF, (pixformat >> 16) & 0xff, (pixformat >> 24) & 0xFF);
            DBG_PRINTF("Width: %d, Height: %d, Image size: %d", width, height, image_size);
            DBG_PRINTF("Frame Rate : %d / %d (%.2f fps)", rate_numerator, rate_denominator, frame_rate);
        }
    };

    /**
     * Camera control menu structure
     */
    struct camera_control_menu {
        unsigned int index;
        char name[256];
        int value;

        camera_control_menu() { clear(); }
        void clear() { memset(this, 0, sizeof(*this)); }

        void dbg_print() {
            DBG_PRINTF("Index: %d, Name: %s, Value: %d", index, name, value);
        }
    };

    /**
     * Camera control structure
     */
    struct camera_control {
        unsigned int id;
        //char name[256];
        std::string name;
        int value;
        int default_value;
        unsigned int type;
        unsigned int flags;

        int minimum;
        int maximum;
        int step;

        std::vector<camera_control_menu> menu_list;

        camera_control() { clear(); }
        void clear() { memset(this, 0, sizeof(*this)); }

        void dbg_print() {
            DBG_PRINTF("Id: %u, Name: %s, Value(default [min, step, max]): %d ( %d [%d, %d, %d] ), Type: %d, Flag: %d",
                       id, name.c_str(), value, default_value, minimum, step, maximum, type, flags);
            for (unsigned int i=0; i < menu_list.size(); i++) {
                DBG_PRINTF("Menu %d", i);
                menu_list[i].dbg_print();
            }
        }
    };


    /**
     * Camera Class
     */
    class Camera
    {
    public:
        /*
         * oCam-1MGN
         *
         * [ supported image formats ]
         *
         * USB 3.0
         * 	[1] "8-bit Greyscale 1280 x 720 60 fps"
    	 *	[2] "8-bit Greyscale 1280 x 960 45 fps"
    	 *	[3] "8-bit Greyscale 320 x 240 160 fps"
    	 * 	[4] "8-bit Greyscale 640 x 480 80 fps"
    	 *
    	 * USB 2.0
         * 	[1] "8-bit Greyscale 1280 x 720 30 fps"
    	 *	[2] "8-bit Greyscale 1280 x 960 22.5 fps"
    	 *	[3] "8-bit Greyscale 320 x 240 160 fps"
    	 * 	[4] "8-bit Greyscale 640 x 480 80 fps"
    	 *
    	 *
		 * [ supported camera controls; The double quotes are the 'get_control' and the 'set_control' function string argument values. ]
		 *
		 *  [1] "Exposure (Absolute)", Value(default [min, step, max]): 39 ( 39 [1, 1, 625] )
		 *  [2] "Brightness",          Value(default [min, step, max]): 64 ( 64 [0, 1, 127] )  // gain
		 *
         */
        Camera(const char* dev_name, struct camera_format* conf=0, const char* format_string=0, const unsigned char disable_libv4l2=0);
        ~Camera();

        bool start();
        bool stop();

        void get_configurations(std::vector<std::string>& formats, std::vector<std::string>& controls);
        bool get_current_format(camera_format& fmt);

        int get_frame(unsigned char* out_buffer, const unsigned int size, unsigned int timeout_sec=1);

        bool set_format(const char* format_description);
        bool set_format(unsigned int width, unsigned int height, unsigned int pixelformat, unsigned int rate_numerator=0, unsigned int rate_denomonator=0);

        bool get_control(camera_control& ctrl);
        int get_control(const char* name);

        bool set_control(const char* name, const int value);

        int valid_controls(std::vector<std::pair<const char*, unsigned int> >& list);
        int get_valid_image_format_list(std::vector<const char*>& list);
        int get_valid_resolution_list(const char* format_description, std::vector<const char*>& list);
        int get_valid_ratio_list(const char* resolution_description, std::vector<const char*>& list);

        /**
         * 동작중(streaming) 여부 확인 함수
         * @return streaming status
         */
        inline bool is_running() { return streaming; }

        /**
         * 장치 이름 제공 함수
         * @return device name
         */
        inline std::string get_dev_name() { return std::string((const char*) v4l2_s.capability.card); }
        std::string get_serial_number();

    private:
        struct _buffer {
            unsigned char* buffer;
            unsigned int length;
        };


        struct _v4l2 {
            enum v4l2_buf_type buf_type;

            struct v4l2_capability capability;
            struct v4l2_format format;
            struct v4l2_requestbuffers requestbuffers;
            struct v4l2_buffer buffer;
            struct v4l2_queryctrl queryctrl;
            struct v4l2_querymenu querymenu;

            struct v4l2_fmtdesc fmtdesc;
            struct v4l2_frmsizeenum frmsizeenum;
            struct v4l2_frmivalenum frmivalenum;

            struct v4l2_streamparm streamparm;
        };

    private:
        std::string dev_name;

        int fd;

        _v4l2 v4l2_s;
        _buffer* buffers;

        unsigned int buffer_count;
        unsigned char disable_libv4l2; /* set to 1 to disable libv4l2 calls */

        std::map<std::string, v4l2_queryctrl> valid_control_list;
        std::map<std::string, v4l2_fmtdesc> valid_format_list;
        std::map<std::string, v4l2_frmsizeenum> valid_resolution_list;
        std::map<std::string, v4l2_frmivalenum> valid_ratio_list;

        std::vector<usb_device_info> usb_device_list;
        struct camera_format config;

        bool streaming;

        Mutex mutex;

    private:
        bool get_capability();
        bool get_current_format();

        bool set_buffer();
        void check_essential_capability();

        int enumerate_controls();
        int enumerate_control_menu();

        bool enumerate_image_formats(const enum v4l2_buf_type type);
        bool enumerate_frame_sizes(const unsigned int pixelformat, std::string& description);
        bool enumerate_frame_intervals(const unsigned int pixelformat, const unsigned int width, const unsigned int height, std::string& description);

        int get_buffer(unsigned char* yuy2_buffer, const unsigned int size);

        bool remove_buffers();

        int xioctl(int IOCTL_X, void *arg);
        int query_ioctl(int current_ctrl, struct v4l2_queryctrl* ctrl);
    };
}  /* namespace Withrobot */

/** @} */ // end of Withrobot

#endif /* WITHROBOT_CAMERA_HPP_ */
