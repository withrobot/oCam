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

/*
 * withrobot_camera.cpp
 *
 *  Created on: Oct 6, 2015
 *      Author: gnohead
 */

#include <linux/kernel.h>
#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <libv4l2.h>

#include "withrobot_camera.hpp"


using namespace Withrobot;

#define WITHROBOT_CAMERA_IOCTL_RETRY     5
#define WITHROBOT_CAMERA_REQUEST_BUFFER_COUNT   1

/**
 * Cam class constructor
 * @param dev_name          [입력] 장치 이름
 * @param conf              [출력] 동작 중인 포멧
 * @param format_string     [입력] 사용하고자 하는 포멧
 * @param disable_libv4l2   [입력, 삭제 예정] v4l2_ioctl 사용 여부 결정
 */
Camera::Camera(const char* dev_name, struct camera_format* conf, const char* format_string, const unsigned char disable_libv4l2) :
    dev_name(dev_name), disable_libv4l2(disable_libv4l2)
{
    /* open device */
    fd = open(dev_name, O_RDWR | O_NONBLOCK, 0);
    if (fd == -1) {
        DBG_PERROR("Opening video device");
        exit(EXIT_FAILURE);
    }
    else {
        DBG_PRINTF("Device \"%s\" is opened.", dev_name);
    }

    /*
     * Initialize
     */
    streaming = false;
    buffer_count = 0;
    buffers = 0;

    memset(&v4l2_s, 0, sizeof(v4l2_s));

    /* capability */
    get_capability();
    check_essential_capability();

    /*
     * Enumerate
     */
    /* format */
    enumerate_image_formats(V4L2_BUF_TYPE_VIDEO_CAPTURE);
    /* control */
    enumerate_controls();

    get_current_format(config);

    /* format */
    if (format_string != 0) {
        set_format(format_string);
    }
    else {
        set_format(config.width, config.height, config.pixformat, config.rate_numerator, config.rate_denominator);
    }

    get_current_format(config);
    if (conf != 0) {
    	get_current_format(*conf);
    }
}


/**
 * Cam class destructor
 */
Camera::~Camera()
{
    /* free memory */
    remove_buffers();

    /* close device */
    if (fd > 0) {
        close(fd);
        DBG_PRINTF("Device \"%s\" is closed.", dev_name.c_str());
    }
}

/**
 * 연결한 장치의 시리얼번호를 읽어온다.
 */
std::string Camera::get_serial_number() {
	std::vector<usb_device_info> info_list;
	int num = get_usb_device_info_list(info_list);
	for (int i=0; i < num; i++) {
		if (strcmp(info_list[i].dev_node.c_str(), dev_name.c_str()) == 0) {
			return info_list[i].serial;
		}
	}
	return "";
}


/**
 * 연결한 카메라가 지원하는 모든 포멧과 컨트롤 이름을 읽어온다.
 */
void Camera::get_configurations(std::vector<std::string>& formats, std::vector<std::string>& controls)
{
	formats.clear();
	DBG_PRINTF("Supported formats:");
    for (auto it = valid_ratio_list.begin(); it != valid_ratio_list.end(); ++it) {
		formats.push_back(it->first);
		DBG_PRINTF("\"%s\"", it->first.c_str());
	}

	controls.clear();
	DBG_PRINTF("Supported controls:");
    for (auto it = valid_control_list.begin(); it != valid_control_list.end(); ++it) {
		controls.push_back(it->first);
		DBG_PRINTF("\"%s\"", it->first.c_str());
	}
}


/**
 * PC 메모리에 마련된 버퍼를 해제
 * @return
 */
bool Camera::remove_buffers()
{
    /* free memory */
    for (unsigned int i=0; i < buffer_count; i++) {
        munmap(buffers[i].buffer, buffers[i].length);
    }
    buffer_count = 0;

    if (buffers) {
        delete[] buffers;
        buffers = 0;
    }

    return true;
}

/**
 * 장치 특성을 가져옴
 * @return
 */
bool Camera::get_capability()
{
    memset(&v4l2_s.capability, 0, sizeof(v4l2_s.capability));
    if (xioctl(VIDIOC_QUERYCAP, &v4l2_s.capability) == -1) {
        DBG_PERROR("VIDIOC_QUERYCAP");
        return false;
    }

    DBG_PRINTF("---- Device capabilities ----");
    DBG_PRINTF("Name: %s", (char*)v4l2_s.capability.card);
    DBG_PRINTF("kernel version: %u.%u.%u", (v4l2_s.capability.version >> 16) & 0xFF, (v4l2_s.capability.version >> 8) & 0xFF, v4l2_s.capability.version & 0xFF);
    DBG_PRINTF("Driver: %s", (char*)v4l2_s.capability.driver);
    DBG_PRINTF("Bus: %s", (char*)v4l2_s.capability.bus_info);
    DBG_PRINTF("V4L2_CAP_STREAMING : %s", (v4l2_s.capability.capabilities & V4L2_CAP_STREAMING) ? "True" : "False");
    DBG_PRINTF("V4L2_CAP_VIDEO_CAPTURE : %s", (v4l2_s.capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) ? "True" : "False");

    return true;
}

/**
 * 스트리밍에 반드시 필요한 기능을 확인함
 */
void Camera::check_essential_capability()
{
    if (!(v4l2_s.capability.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        DBG_PERROR("V4L2_CAP_VIDEO_CAPTURE");
        exit(EXIT_FAILURE);
    }

    if (!(v4l2_s.capability.capabilities & V4L2_CAP_STREAMING)) {
        DBG_PERROR("V4L2_CAP_STREAMING");
        exit(EXIT_FAILURE);
    }
}

/**
 * PC 메모리에 버퍼 공간을 마련함
 */
bool Camera::set_buffer()
{
    /* request buffers */
    memset(&v4l2_s.requestbuffers, 0, sizeof(v4l2_s.requestbuffers));
    v4l2_s.requestbuffers.count = WITHROBOT_CAMERA_REQUEST_BUFFER_COUNT;
    v4l2_s.requestbuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_s.requestbuffers.memory = V4L2_MEMORY_MMAP;
    if (xioctl(VIDIOC_REQBUFS, &v4l2_s.requestbuffers) == -1) {
        DBG_PERROR("request buffer");
        return false;
    }

    buffer_count = v4l2_s.requestbuffers.count;
    buffers = new _buffer[v4l2_s.requestbuffers.count];

    /* memory */
    unsigned int buffer_max = 0;
    for (unsigned int i = 0; i < buffer_count; i++) {
        memset(&v4l2_s.buffer, 0, sizeof(v4l2_s.buffer));

        v4l2_s.buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_s.buffer.memory = V4L2_MEMORY_MMAP;
        v4l2_s.buffer.index = i;

        if (xioctl(VIDIOC_QUERYBUF, &v4l2_s.buffer) == -1) {
            DBG_PERROR("query buffer");
            return false;
        }

        if (v4l2_s.buffer.length > buffer_max) {
            buffer_max = v4l2_s.buffer.length;
        }

        buffers[i].length = v4l2_s.buffer.length;
        buffers[i].buffer = (unsigned char*)mmap(NULL, v4l2_s.buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, v4l2_s.buffer.m.offset);

        if (buffers[i].buffer == MAP_FAILED) {
            DBG_PERROR("memory mmap");
            return false;
        }
    }

    return true;
}

/**
 * 장치가 제공하는 이미지 포멧을 목록화 함
 * @param buf_type  [입력] 버퍼 종류
 * @return
 */
bool Camera::enumerate_image_formats(const enum v4l2_buf_type buf_type)
{
    memset(&v4l2_s.fmtdesc, 0, sizeof(v4l2_s.fmtdesc));
    v4l2_s.fmtdesc.type = buf_type;
    v4l2_s.fmtdesc.index = 0;

    std::string desc;

    DBG_PRINTF("---- Valid image formats ----");
    while (xioctl(VIDIOC_ENUM_FMT, &v4l2_s.fmtdesc) != -1) {
        DBG_PRINTF("Index: %d", v4l2_s.fmtdesc.index);
        DBG_PRINTF("Format description: %s", v4l2_s.fmtdesc.description);
        DBG_PRINTF("Format pixel format: %c, %c, %c, %c", (v4l2_s.fmtdesc.pixelformat >> 0) & 0xFF, (v4l2_s.fmtdesc.pixelformat >> 8) & 0xFF, (v4l2_s.fmtdesc.pixelformat >> 16) & 0xff, (v4l2_s.fmtdesc.pixelformat >> 24) & 0xFF);

        desc.clear();
        desc = std::string((const char*)v4l2_s.fmtdesc.description);
        valid_format_list[desc] = v4l2_s.fmtdesc;

        enumerate_frame_sizes(v4l2_s.fmtdesc.pixelformat, desc);

        v4l2_s.fmtdesc.index++;
    }

    return true;
}

/**
 * 장치에서 제공하는 화상 크기를 목록화 함
 * @param pixelformat   [입력] 목록화 하고자하는 포멧
 * @param description   [출력] 인덱스 문자열
 * @return
 */
bool Camera::enumerate_frame_sizes(const unsigned int pixelformat, std::string& description)
{
    memset(&v4l2_s.frmsizeenum, 0, sizeof(v4l2_s.frmsizeenum));
    v4l2_s.frmsizeenum.index = 0;
    v4l2_s.frmsizeenum.pixel_format = pixelformat;

    std::string desc;

    DBG_PRINTF("---- Valid image sizes ----");
    while (xioctl(VIDIOC_ENUM_FRAMESIZES, &v4l2_s.frmsizeenum) != -1) {
        DBG_PRINTF("Index: %d", v4l2_s.frmsizeenum.index);

        switch (v4l2_s.frmsizeenum.type) {
        case V4L2_FRMSIZE_TYPE_DISCRETE:
            DBG_PRINTF("Discrete :: Width: %d, Height: %d", v4l2_s.frmsizeenum.discrete.width, v4l2_s.frmsizeenum.discrete.height);

            desc.clear();
            desc = description + (" " + Withrobot::to_string<int>(v4l2_s.frmsizeenum.discrete.width) + " x " + Withrobot::to_string<int>(v4l2_s.frmsizeenum.discrete.height));

            valid_resolution_list[desc] = v4l2_s.frmsizeenum;

            enumerate_frame_intervals(v4l2_s.frmsizeenum.pixel_format, v4l2_s.frmsizeenum.discrete.width, v4l2_s.frmsizeenum.discrete.height, desc);

            break;

        case V4L2_FRMSIZE_TYPE_STEPWISE:
            DBG_PRINTF("Stepwise :: Width step(max/min): %d (%d / %d), Height step(max / min): %d (%d / %d)",
                    v4l2_s.frmsizeenum.stepwise.step_width, v4l2_s.frmsizeenum.stepwise.max_width, v4l2_s.frmsizeenum.stepwise.min_width,
                    v4l2_s.frmsizeenum.stepwise.step_height, v4l2_s.frmsizeenum.stepwise.max_height, v4l2_s.frmsizeenum.stepwise.min_height);
            break;

        case V4L2_FRMSIZE_TYPE_CONTINUOUS:
            DBG_PRINTF("Continuous")
            break;

        default:
            DBG_PRINTF("ERROR: Invalid VIDIOC_ENUM_FRAMESIZES");
            exit(EXIT_FAILURE);
        }

        v4l2_s.frmsizeenum.index++;
    }

    return true;
}

/**
 * 프레임 간격(초당 프레임 수)를 목록화 함
 * @param pixelformat   [입력] 목록화 하고자 하는 포멧
 * @param width         [입력] 이미지 너비
 * @param height        [입력] 이미지 높이
 * @param description   [출력] 인덱스 문자열
 * @return
 */
bool Camera::enumerate_frame_intervals(const unsigned int pixelformat, const unsigned int width, const unsigned int height, std::string& description)
{
    DBG_PRINTF("---- Valid frame intervals ----");
    memset(&v4l2_s.frmivalenum, 0, sizeof(v4l2_s.frmivalenum));

    v4l2_s.frmivalenum.index = 0;
    v4l2_s.frmivalenum.pixel_format = pixelformat;
    v4l2_s.frmivalenum.width = width;
    v4l2_s.frmivalenum.height = height;

    std::string desc;

    double fps = 0;

    while (xioctl(VIDIOC_ENUM_FRAMEINTERVALS, &v4l2_s.frmivalenum) != -1) {
        switch(v4l2_s.frmivalenum.type) {
        case V4L2_FRMIVAL_TYPE_DISCRETE:

            fps = 1.0 / ((double)v4l2_s.frmivalenum.discrete.numerator / (double)v4l2_s.frmivalenum.discrete.denominator);

            desc.clear();
            desc = description + (" " + Withrobot::to_string<double>(fps) + " fps");
            valid_ratio_list[desc] = v4l2_s.frmivalenum;

            DBG_PRINTF("Discrete: %.2f fps", fps);
            break;

        case V4L2_FRMIVAL_TYPE_CONTINUOUS:
            DBG_PRINTF("Continuous");
            break;

        case V4L2_FRMIVAL_TYPE_STEPWISE:
            DBG_PRINTF("Stepwise step (max/min): %.2f (%.2f / %.2f) fps",
                    1.0 / ((double)v4l2_s.frmivalenum.stepwise.step.numerator / (double)v4l2_s.frmivalenum.stepwise.step.denominator),
                    1.0 / ((double)v4l2_s.frmivalenum.stepwise.max.numerator / (double)v4l2_s.frmivalenum.stepwise.max.denominator),
                    1.0 / ((double)v4l2_s.frmivalenum.stepwise.min.numerator / (double)v4l2_s.frmivalenum.stepwise.min.denominator));
            break;

        default:
            DBG_PRINTF("ERROR: Invalid VIDIOC_ENUM_FRAMEINTERVALS");
            exit(EXIT_FAILURE);
        }

        v4l2_s.frmivalenum.index++;
    }

    return true;
}

/**
 * 장치에서 지원하는 제어 항목을 목록화 함
 * @return
 */
int Camera::enumerate_controls()
{
    DBG_PRINTF("---- Valid control lists ----");
    memset(&v4l2_s.queryctrl, 0, sizeof(v4l2_s.queryctrl));
    v4l2_s.queryctrl.id = V4L2_CTRL_CLASS_USER | V4L2_CTRL_FLAG_NEXT_CTRL;

    while (query_ioctl(VIDIOC_QUERYCTRL, &v4l2_s.queryctrl) == 0) {
        if (v4l2_s.queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
            continue;

        DBG_PRINTF ("id: 0x%X, %s, flags: %d", v4l2_s.queryctrl.id, v4l2_s.queryctrl.name, v4l2_s.queryctrl.flags);
        valid_control_list[(const char*)v4l2_s.queryctrl.name] = v4l2_s.queryctrl;

        if (v4l2_s.queryctrl.type == V4L2_CTRL_TYPE_MENU) {
            enumerate_control_menu();
        }

        v4l2_s.queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }

    if (errno != EINVAL) {
        DBG_PERROR("VIDIOC_QUERYCTRL");
        return -1;
    }

    return 0;
}

/**
 * 메뉴 타입 제어 항목의 하위 메뉴를 목록화 함
 * @return
 */
int Camera::enumerate_control_menu()
{
    DBG_PRINTF("Menu items:");

    memset(&v4l2_s.querymenu, 0, sizeof(v4l2_s.querymenu));
    v4l2_s.querymenu.id = v4l2_s.queryctrl.id;

    for (v4l2_s.querymenu.index = v4l2_s.queryctrl.minimum; (int)v4l2_s.querymenu.index <= v4l2_s.queryctrl.maximum; v4l2_s.querymenu.index++) {
        if (xioctl(VIDIOC_QUERYMENU, &v4l2_s.querymenu) == 0) {
            DBG_PRINTF ("id: %s, 0x%X, %s", v4l2_s.queryctrl.name, v4l2_s.querymenu.index, v4l2_s.querymenu.name);
        } else {
            DBG_PERROR ("VIDIOC_QUERYMENU");
        }
    }

    return 0;
}

/**
 * Start streaming
 */
bool Camera::start()
{
    if (streaming) {
        return false;
    }

    /* buffer */
    if (!set_buffer()) {
        return false;
    }

    streaming = true;

    for (unsigned int i = 0; i < buffer_count; i++) {
        memset(&v4l2_s.buffer, 0, sizeof(v4l2_s.buffer));
        v4l2_s.buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_s.buffer.memory = V4L2_MEMORY_MMAP;
        v4l2_s.buffer.index = i;
        if (xioctl(VIDIOC_QBUF, &v4l2_s.buffer) == -1) {
            DBG_PERROR("VIDIOC_QBUF");
            return false;
        }
    }

    v4l2_s.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(VIDIOC_STREAMON, &v4l2_s.buf_type) == -1) {
        DBG_PERROR("VIDIOC_STREAMON");
        return false;
    }

    return true;
}


/**
 * Stop streaming
 */
bool Camera::stop()
{
    if (!streaming) {
        return false;
    }

    streaming = false;

    remove_buffers();

    v4l2_s.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(VIDIOC_STREAMOFF, &v4l2_s.buf_type) == -1) {
        DBG_PERROR("VIDIOC_STREAMOFF");
        return false;
    }

    return true;
}

/**
 * 장치 드라이버에서 한 프레임을 제공하는 함수
 * @param out_buffer    [출력] 출력 메모리의 포인터
 * @param size          [입력] 출력 메모리의 크기
 * @param timeout_sec   [입력] 타임아웃 시간(초)
 * @return 버퍼에 담은 데이터의 크기
 */
int Camera::get_frame(unsigned char* out_buffer, const unsigned int size, unsigned int timeout_sec)
{
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    struct timeval timeout;
    timeout.tv_sec = timeout_sec;
    timeout.tv_usec = 0;
    int r = select(fd+1, &fds, NULL, NULL, &timeout);

    if (r == -1) {
        DBG_PERROR("error");
        exit(EXIT_FAILURE);
    }

    if (r == 0) {
        return -1;
    }

    return get_buffer(out_buffer, size);
}

/**
 * 장치에서 지원 가능한 image format 목록을 제공하는 함수
 * @param list  [출력] 이미지 포멧 목록
 * @return 목록의 크기
 */
int Camera::get_valid_image_format_list(std::vector<const char*>& list)
{
    list.clear();

    std::map<std::string, v4l2_fmtdesc>::iterator it;
    for (it = valid_format_list.begin(); it != valid_format_list.end(); ++it) {
        list.push_back(it->first.c_str());
    }

    return (int)list.size();
}

/**
 * 장치에서 지원 가능한 해상도 목록을 제공하는 함수
 * @param format_description    [입력] 포멧 이름 문자열
 * @param list                  [출력] 해상도 목록
 * @return 목록의 크기
 */
int Camera::get_valid_resolution_list(const char* format_description, std::vector<const char*>& list)
{
    list.clear();

    std::map<std::string, v4l2_frmsizeenum>::iterator it;
    for (it = valid_resolution_list.begin(); it != valid_resolution_list.end(); ++it) {
        if ((it->first.find(format_description) != std::string::npos)) {
            list.push_back(it->first.c_str());
        }
    }

    return (int)list.size();
}

/**
 * 장치에서 지원 가능한 초당 프레임 수 목록을 제공하는 함수
 * @param resolution_description    [입력] 해상도 문자열
 * @param list                      [출력] 초당 프레임 수 목록
 * @return 목록의 크기
 */
int Camera::get_valid_ratio_list(const char* resolution_description, std::vector<const char*>& list)
{
    list.clear();

    std::map<std::string, v4l2_frmivalenum>::iterator it;

    for (it = valid_ratio_list.begin(); it != valid_ratio_list.end(); ++it) {
        if ((it->first.find(resolution_description) != std::string::npos)) {
            list.push_back(it->first.c_str());
        }
    }

    return (int)list.size();
}

/**
 * 장치에서 지원 가능한 제어 목록을 제공하는 함수
 * @param list  [출력] 제어 목록
 * @return 목록의 크기
 */
int Camera::valid_controls(std::vector<std::pair<const char*, unsigned int> >& list)
{
    list.clear();

    std::map<std::string, v4l2_queryctrl>::iterator it;

    try {
        for (it = valid_control_list.begin(); it != valid_control_list.end(); ++it) {
            list.push_back(std::pair<const char*, unsigned int>(it->first.c_str(), it->second.type));
        }
    }
    catch (...) {
        DBG_PERROR("--error-- valid_control_list");
        return -1;
    }

    return (int)list.size();
}

/**
 * 장치의 현재 제어 상태를 가져오는 함수
 *
 * 가져오고자 하는 name을 입력하면, 장치의 해당 name의 상태를 출력함
 * @param ctrl  [입력, 출력] 제어 상태 구조체
 * @return
 */
bool Camera::get_control(camera_control& ctrl)
{
    memset(&v4l2_s.queryctrl, 0, sizeof(v4l2_s.queryctrl));

    if (ctrl.name.empty()) {
        return false;
    }

    std::map<std::string, v4l2_queryctrl>::iterator it = valid_control_list.find(ctrl.name);
    if (it == valid_control_list.end()) {
        return false;
    }

    v4l2_s.queryctrl.id = valid_control_list[ctrl.name].id;

    if (xioctl(VIDIOC_QUERYCTRL, &v4l2_s.queryctrl) == -1) {
        DBG_PERROR("VIDIOC_QUREYCTRL");
        return false;
    }

    struct v4l2_control control;
    memset(&control, 0, sizeof(control));

    control.id = v4l2_s.queryctrl.id;

    if (xioctl(VIDIOC_G_CTRL, &control) == -1) {
        DBG_PERROR("VIDIOC_G_CTRL");
        return false;
    }

    memset(&ctrl, 0, sizeof(camera_control));

    //strcpy(ctrl.name, (const char*)v4l2_s.queryctrl.name);
    ctrl.name 			= (const char*)v4l2_s.queryctrl.name;
    ctrl.id             = v4l2_s.queryctrl.id;
    ctrl.step           = v4l2_s.queryctrl.step;
    ctrl.type           = v4l2_s.queryctrl.type;
    ctrl.flags          = v4l2_s.queryctrl.flags;
    ctrl.minimum        = v4l2_s.queryctrl.minimum;
    ctrl.maximum        = v4l2_s.queryctrl.maximum;
    ctrl.default_value  = v4l2_s.queryctrl.default_value;
    ctrl.value          = control.value;

    ctrl.dbg_print();

    /* control type: menu */
    if (v4l2_s.queryctrl.type == CAM_CTRL_TYPE_MENU) {
        camera_control_menu m;
        for (v4l2_s.querymenu.index = v4l2_s.queryctrl.minimum; (int)v4l2_s.querymenu.index <= v4l2_s.queryctrl.maximum; v4l2_s.querymenu.index++) {
            if (xioctl(VIDIOC_QUERYMENU, &v4l2_s.querymenu) == 0) {
                strcpy(m.name, (const char*)v4l2_s.querymenu.name);
                m.index = v4l2_s.querymenu.index;
                m.value = v4l2_s.querymenu.value;
                ctrl.menu_list.push_back(m);
            } else {
                DBG_PERROR ("VIDIOC_QUERYMENU");
            }
        }
    }

    return true;
}

/**
 * 장치의 현재 제어값을 가져오는 함수
 * @param name		[입력] 제어 이름 문자열
 * @return			[출력] 현재 값 (정수), 실패 또는 오류시 -1
 */
int Camera::get_control(const char* name)
{
	camera_control ctrl;
	ctrl.name = name;
	bool res = get_control(ctrl);
	if (res) {
		return ctrl.value;
	}
	else {
		printf("Invalid control name: %s\n", name);
		return -1;
	}
}

/**
 * 장치에 제어 상태를 설정하는 함수
 * @param name      [입력] 설정하고자 하는 제어 이름 문자열
 * @param value     [입력] 제어 입력 값
 * @return
 *
 * ToDo: 정수형이 아닌 제어값에 대응할 수 있도록 수정할 것
 */
bool Camera::set_control(const char* name, const int value)
{
    std::map<std::string, v4l2_queryctrl>::iterator it = valid_control_list.find(name);
    if (it == valid_control_list.end()) {
        return false;
    }

    struct v4l2_control control;
    memset(&control, 0, sizeof(control));

    control.id = it->second.id;
    control.value = value;

    if (xioctl(VIDIOC_S_CTRL, &control) == -1 && errno != ERANGE) {
        DBG_PERROR("VIDIOC_S_CTRL");
        return false;
    }

    return true;
}

/**
 * 장치에서 현재 포멧 상태를 가져오는 함수
 * @return
 */
bool Camera::get_current_format()
{
    memset(&v4l2_s.format, 0, sizeof(v4l2_s.format));
    v4l2_s.format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(VIDIOC_G_FMT, &v4l2_s.format.type) == -1) {
        DBG_PERROR("VIDIOC_G_FMT");
        return false;
    }

    memset(&v4l2_s.streamparm, 0, sizeof(v4l2_s.streamparm));
    v4l2_s.streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(VIDIOC_G_PARM, &v4l2_s.streamparm) == -1) {
        DBG_PERROR("VIDIOC_G_PARM");
        return false;
    }

    DBG_PRINTF("V4L2_CAP_TIMEPERFRAME : %s", (v4l2_s.streamparm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) ? "True" : "False");

    return true;
}

/**
 * 장치에서 현재 포멧 상태를 가져오는 함수
 * @param fmt   [출력] 현재 포멧 상태
 * @return
 */
bool Camera::get_current_format(camera_format& fmt)
{
    bool res = get_current_format();

    fmt.pixformat = v4l2_s.format.fmt.pix.pixelformat;
    fmt.width = v4l2_s.format.fmt.pix.width;
    fmt.height = v4l2_s.format.fmt.pix.height;
    fmt.image_size = v4l2_s.format.fmt.pix.sizeimage;
    fmt.rate_numerator = v4l2_s.streamparm.parm.capture.timeperframe.numerator;
    fmt.rate_denominator = v4l2_s.streamparm.parm.capture.timeperframe.denominator;
    fmt.frame_rate = ((double)fmt.rate_denominator) / ((double)fmt.rate_numerator);

    fmt.dbg_print();

    return res;
}

/**
 * 장치에 포멧 설정
 * @param width             [입력] 이미지 너비
 * @param height            [입력] 이미지 높
 * @param pixelformat       [입력] 포멧 코드
 * @param rate_numerator    [입력] 초당 프레임 수의 분자
 * @param rate_denomonator  [입력] 초당 프레임 수의 분모
 * @return
 */
bool Camera::set_format(unsigned int width, unsigned int height, unsigned int pixelformat, unsigned int rate_numerator, unsigned int rate_denomonator)
{
    get_current_format();

    v4l2_s.format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_s.format.fmt.pix.width = width;
    v4l2_s.format.fmt.pix.height = height;
    v4l2_s.format.fmt.pix.pixelformat = pixelformat;
    v4l2_s.format.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(VIDIOC_S_FMT, &v4l2_s.format) == -1) {
        DBG_PERROR("VIDIOC_S_FMT");
        return false;
    }

    if (!rate_numerator || !rate_denomonator) {
        return true;
    }

    v4l2_s.streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_s.streamparm.parm.capture.capability |= V4L2_CAP_TIMEPERFRAME;
    v4l2_s.streamparm.parm.capture.timeperframe.numerator = rate_numerator;
    v4l2_s.streamparm.parm.capture.timeperframe.denominator = rate_denomonator;
    if (xioctl(VIDIOC_S_PARM, &v4l2_s.streamparm) == -1) {
        DBG_PERROR("VIDIOC_S_PARM");
        return false;
    }

    return true;
}

/**
 * 장치에 포멧 설정
 * @param format_description    [입력] 포멧 문자열
 * @return
 */
bool Camera::set_format(const char* format_description)
{
    if (valid_ratio_list.find(format_description) == valid_ratio_list.end()) {
        return false;
    }

    memset(&v4l2_s.frmivalenum, 0, sizeof(v4l2_s.frmivalenum));
    v4l2_s.frmivalenum = valid_ratio_list[format_description];

    set_format(v4l2_s.frmivalenum.width, v4l2_s.frmivalenum.height, v4l2_s.frmivalenum.pixel_format, v4l2_s.frmivalenum.discrete.numerator, v4l2_s.frmivalenum.discrete.denominator);

    return true;
}

/**
 * 장치에서 이미지 데이터를 가져와서 PC 메모리 버퍼에 담는 함수
 * @param buffer    [출력] 메모리 버퍼의 포인터
 * @param size      [입력] 메모리 버퍼의 크기
 * @return
 */
int Camera::get_buffer(unsigned char* buffer, const unsigned int size)
{
    int retval = -1;

//    memset(&v4l2_s.buffer, 0, sizeof(v4l2_s.buffer));

    v4l2_s.buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_s.buffer.memory = V4L2_MEMORY_MMAP;
    if (xioctl(VIDIOC_DQBUF, &v4l2_s.buffer) == -1) {
        DBG_PERROR("VIDIOC_DQBUF");
        return retval;
    }

    if (v4l2_s.buffer.bytesused == size || v4l2_s.format.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG) {
        retval = v4l2_s.buffer.bytesused;
        memcpy(buffer, buffers[v4l2_s.buffer.index].buffer, v4l2_s.buffer.bytesused);
    }
    else {
        DBG_PRINTF("Warning :: Different buffer size. v4l2: %d, User: %d", v4l2_s.buffer.bytesused, size);
    }

    // time stamp
//    DBG_PRINTF("time stamp : 0x%x, 0x%x, 0x%x, 0x%x", buffer[0],buffer[1],buffer[2],buffer[3]);

    if (xioctl(VIDIOC_QBUF, &v4l2_s.buffer) == -1) {
        DBG_PERROR("VIDIOC_QBUF");
        return retval;
    }

    return retval;
}

/**
 * ioctl with a number of retries in the case of I/O failure
 *
 * @param IOCTL_X   [입력] ioctl reference
 * @param arg       [입력, 출력] pointer to ioctl data
 * @return
 */
int Camera::xioctl(int IOCTL_X, void *arg)
{
    LockGuard l(mutex);

    int ret = 0;
    int tries= WITHROBOT_CAMERA_IOCTL_RETRY;
    do
    {
        if(!disable_libv4l2)
            ret = v4l2_ioctl(fd, IOCTL_X, arg);
        else
            ret = ioctl(fd, IOCTL_X, arg);
    }
    while (ret && tries-- && ((errno == EINTR) || (errno == EAGAIN) || (errno == ETIMEDOUT)));

    if (ret && (tries <= 0)) {
        DBG_PRINTF("V4L2_CORE: ioctl (%i) retried %i times - giving up: %s)", IOCTL_X, WITHROBOT_CAMERA_IOCTL_RETRY, strerror(errno));
    }

    return (ret);
}

/**
 * don't use xioctl for control query when using V4L2_CTRL_FLAG_NEXT_CTRL
 * @param current_ctrl  [입력] current control id
 * @param ctrl          [입력, 출력] pointer to v4l2_queryctrl data
 * @return error code
 */
int Camera::query_ioctl(int current_ctrl, struct v4l2_queryctrl* ctrl)
{
    LockGuard l(mutex);

    /*assertions*/
    assert(fd > 0);
    assert(ctrl != NULL);

    /**
     * @todo v4l2_ioctl 로 하는 경우 실패시 무한반복되는 상황이 발생하는것 해결할 것
     */
    int ret = 0;
    int tries = 4;

    do
    {
#if 0
        ret = ioctl(fd, VIDIOC_QUERYCTRL, ctrl);
#else

        if(ret) {
            ctrl->id = current_ctrl | V4L2_CTRL_FLAG_NEXT_CTRL;
        }
        ret = v4l2_ioctl(fd, VIDIOC_QUERYCTRL, ctrl);
#endif
    }
    while (ret && tries-- && (errno == EIO || errno == EPIPE || errno == ETIMEDOUT));

    return(ret);
}

