/*******************************************************************************#
#                                                                               #
# Withrobot Utilities                                                           #
#                                                                               #
# Copyright (C) 2015 Withrobot. Inc.                                            #
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
 * withrobot_utility.hpp
 *
 *  Created on: Oct 7, 2015
 *      Author: gnohead
 */

#ifndef WITHROBOT_UTILITY_HPP_
#define WITHROBOT_UTILITY_HPP_

#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include <cstring>
#include <string>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <libudev.h>
#include <sstream>

/**
 * @defgroup Withrobot Utilities
 */
namespace Withrobot {

    /**
     * Usb device Infomation structure(class)
     */
    struct usb_device_info {
        std::string dev_node;       //!< device node.       (e.g. /dev/video0)
        std::string id_vendor;      //!< vendor id, VID.    (e.g. 04B4)
        std::string id_product;     //!< product id, PID.   (e.g. 00F9)
        std::string manufacturer;   //!< manufacturer.      (e.g. withrobot)
        std::string product;        //!< product name.      (e.g. oCam-5CR-U3)
        std::string serial;         //!< serial string.     (e.g. SN000000000)
        std::string busnum;         //!< bus number.        (e.g. 4)
        std::string devnum;         //!< device number.     (e.g. 3)

        /**
         * default constructor
         */
        usb_device_info() {
            clear();
        }

        /**
         * clear members
         */
        void clear() {
            dev_node.clear();
            id_vendor.clear();
            id_product.clear();
            manufacturer.clear();
            product.clear();
            serial.clear();
            busnum.clear();
            devnum.clear();
        }

        /**
         * 장치 정보 출력 (to standard output)
         */
        void print() {
            printf("Device Node Path      : %s\n", dev_node.c_str());
            printf("VID/PID               : %s / %s\n", id_vendor.c_str(), id_product.c_str());
            printf("Product [menufacturer]: %s [ %s ]\n", product.c_str(), manufacturer.c_str());
            printf("serial                : %s\n", serial.c_str());
            printf("busnum                : %s\n", busnum.c_str());
            printf("devnum                : %s\n", devnum.c_str());
        }
    };

    /**
     * linux 장치관리자(udev)에서 video4linux 관련 장치 목록을 가져옴 (libudev-dev 필요)
     * @param info_list [출력] 연결 장치 목록
     * @return number of video4linux devices
     */
    int get_usb_device_info_list(std::vector<usb_device_info>& info_list);

    /**
     * Exception
     */
    class WithRobotException
    {
    public:
        std::string err;
        WithRobotException(std::string e=""): err(e){}
        virtual ~WithRobotException() {}

        const char* what() const throw() {
            return err.c_str();
        }
    };


    /**
     * Thread
     */
    class Thread
    {
        int id;
        pthread_t thread;

    public:
        Thread();
        ~Thread();
        bool start(void*(*thread_proc)(void*), void* arg);
        void join();
    };

    /**
     * Mutex
     */
    class Mutex
    {
        pthread_mutex_t mutex;
        pthread_mutexattr_t attr;

    public:
        Mutex();
        ~Mutex();
        inline void lock() { pthread_mutex_lock(&mutex); }
        inline void unlock() { pthread_mutex_unlock(&mutex); }
    };

    /**
     * LockGuard
     */
    class LockGuard {
        Mutex& mutex;

    public:
        LockGuard(Mutex& m) : mutex(m) { mutex.lock(); }
        ~LockGuard() { mutex.unlock(); }
    };

    /**
     * 현재 호출 위치에서 밀리초 단위로 일시 정지 한다.
     * @param msec  일시 정지 시간 [milliseconds]
     */
    static inline void msleep(unsigned int msec) { usleep(msec*1000); }

    /**
     * Timer
     */
    class Timer
    {
        timeval start_timeval;
        timeval end_timeval;
        bool running;

        double start_sec;
        double end_sec;

        std::string name;

        unsigned int cnt;
        unsigned int max_cnt;

        double elapsed_sum;
        double elapsed_avg;

    public:
        Timer(std::string name=NULL, unsigned int max_cnt=10);
        ~Timer();

        void start();
        void stop();
        double restart();

        double get();

        void print();

    private:
        void init();
    };

    /**
     * Input value to string
     *
     * e.g. "3.141592" = to_string<double>(3.141592);
     *
     * @param value [입력] string으로 변경할 값
     * @return 값의 숫자 문자열
     */
    template <typename T>
    std::string to_string(T value)
    {
        std::ostringstream ss;
        ss << value;
        return ss.str();
    }

} /* namespace Withrobot */


#endif /* WITHROBOT_UTILITY_HPP_ */
