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
 * withrobot_utility.cpp
 *
 *  Created on: Oct 7, 2015
 *      Author: gnohead
 */

#include "withrobot_utility.hpp"

using namespace Withrobot;

/*
 * References: https://www.kernel.org/pub/linux/utils/kernel/hotplug/libudev/ch01.html, http://www.signal11.us/oss/udev/
 */
int Withrobot::get_usb_device_info_list(std::vector<usb_device_info>& info_list)
{
    info_list.clear();

    struct udev* my_device;
    my_device = udev_new();
    if (!my_device) {
        printf("Can't create udev\n");
        return false;
    }

    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices;
    struct udev_list_entry *dev_list_entry;

    struct usb_device_info dev_info;
    int num_dev = 0;

    /* Create a list of the devices in the 'v4l2' subsystem. */
    enumerate = udev_enumerate_new(my_device);
    udev_enumerate_add_match_subsystem(enumerate, "video4linux");
    udev_enumerate_scan_devices(enumerate);
    devices = udev_enumerate_get_list_entry(enumerate);

    /*
     * For each item enumerated, print out its information.
     * udev_list_entry_foreach is a macro which expands to
     * a loop. The loop will be executed for each member in
     * devices, setting dev_list_entry to a list entry
     * which contains the device's path in /sys.
     */
    udev_list_entry_foreach(dev_list_entry, devices)
    {
        const char *path;

        /*
         * Get the filename of the /sys entry for the device
         * and create a udev_device object (dev) representing it
         */
        path = udev_list_entry_get_name(dev_list_entry);
        struct udev_device *dev = udev_device_new_from_syspath(my_device, path);

        /* usb_device_get_devnode() returns the path to the device node
            itself in /dev. */
        const char* dev_node = udev_device_get_devnode(dev);
        dev_info.dev_node = dev_node;

        /* The device pointed to by dev contains information about
            the v4l2 device. In order to get information about the
            USB device, get the parent device with the
            subsystem/devtype pair of "usb"/"usb_device". This will
            be several levels up the tree, but the function will find
            it.*/
        dev = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
        if (!dev)
        {
            fprintf(stderr, "V4L2_CORE: Unable to find parent usb device [ %s ].\n", dev_node);
            continue;
        }

        num_dev++;

        /* From here, we can call get_sysattr_value() for each file
            in the device's /sys entry. The strings passed into these
            functions (idProduct, idVendor, serial, etc.) correspond
            directly to the files in the directory which represents
            the USB device. Note that USB strings are Unicode, UCS2
            encoded, but the strings returned from
            udev_device_get_sysattr_value() are UTF-8 encoded. */
        const char* id_vendor = udev_device_get_sysattr_value(dev, "idVendor");
        if (id_vendor) {
            dev_info.id_vendor = id_vendor;
        }

        const char* id_product = udev_device_get_sysattr_value(dev, "idProduct");
        if (id_product) {
            dev_info.id_product = id_product;
        }

        const char* manufacturer = udev_device_get_sysattr_value(dev, "manufacturer");
        if (manufacturer) {
            dev_info.manufacturer = manufacturer;
        }

        const char* product = udev_device_get_sysattr_value(dev, "product");
        if (product) {
            dev_info.product = product;
        }

        const char* serial = udev_device_get_sysattr_value(dev, "serial");
        if (serial) {
            dev_info.serial = serial;
        }

        const char* busnum = udev_device_get_sysattr_value(dev, "busnum");
        if (busnum) {
            dev_info.busnum = busnum;
        }

        const char* devnum = udev_device_get_sysattr_value(dev, "devnum");
        if (devnum) {
            dev_info.devnum = devnum;
        }

        info_list.push_back(dev_info);
        dev_info.clear();

        udev_device_unref(dev);
    }
    /* Free the enumerator object */
    udev_enumerate_unref(enumerate);

    return num_dev;
}


/*
 * pthread wrapper
 */
Thread::Thread() : id(0), thread(0)
{
}

Thread::~Thread()
{
    join();
}

bool Thread::start(void*(*thread_proc)(void*), void* arg)
{
    id = pthread_create(&thread, NULL, thread_proc, (void*) arg);
    if (id != 0) {
        thread = 0;
    }
    return (id == 0);
}

void Thread::join()
{
    if (thread != 0) {
        pthread_join(thread, NULL);
    }
    thread = 0;
}

/*
 * pthread mutex wrapper
 */
Mutex::Mutex()
{
//	mutex = new pthread_mutex_t PTHREAD_MUTEX_INITIALIZER;
    char str_res[16];

    int res = pthread_mutexattr_init(&attr);
    if(res != 0) {
        sprintf(str_res, "%d", res);
        throw (WithRobotException("pthread_mutexattr_init returns " + std::string(str_res)));
    }

    res = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    if(res != 0) {
        sprintf(str_res, "%d", res);
        throw (WithRobotException("pthread_mutexattr_settype returns " + std::string(str_res)));
    }

    res = pthread_mutex_init (&mutex, &attr);
    if(res != 0) {
        sprintf(str_res, "%d", res);
        throw (WithRobotException("pthread_mutex_init returns " + std::string(str_res)));
    }

    res = pthread_mutexattr_destroy(&attr);
    if(res != 0) {
        sprintf(str_res, "%d", res);
        throw (WithRobotException("pthread_mutexattr_destroy returns " + std::string(str_res)));
    }
}

Mutex::~Mutex()
{
    pthread_mutex_destroy(&mutex);
}


/*
 * Timer
 */

Timer::Timer(std::string name, unsigned int max_cnt) : name(name), cnt(0), max_cnt(max_cnt), elapsed_sum(0), elapsed_avg(0)
{
    if (max_cnt < 1) {
        max_cnt = 1;
    }

    init();
}

Timer::~Timer()
{
    stop();
}

void Timer::start()
{
    init();

    running = true;
    gettimeofday(&start_timeval, NULL);
}

void Timer::stop()
{
    running = false;
    gettimeofday(&end_timeval, NULL);

    start_sec = start_timeval.tv_sec + (start_timeval.tv_usec / 1000000.0);
    end_sec = end_timeval.tv_sec + (end_timeval.tv_usec / 1000000.0);

    elapsed_sum += (end_sec - start_sec);
    cnt++;

    if (cnt == max_cnt) {
        elapsed_avg = elapsed_sum / (double)cnt;
        if (!elapsed_avg) {
            elapsed_avg = (end_sec - start_sec);
        }
        cnt = 0;
        elapsed_sum = 0;
    }
}

double Timer::restart()
{
    double res = get();
    start();
    return res;
}

double Timer::get()
{
    if (running) {
        stop();
    }

    return elapsed_avg;
}

void Timer::print()
{
    double elps = get();
    printf("[ %s ] ElapsedTime: %f sec (%.2f fps)\n", name.c_str(), elps, 1.0/elps); fflush(stdout);
}

void Timer::init()
{
    running = false;
    start_sec = 0;
    end_sec = 0;

    memset(&start_timeval, 0, sizeof(start_timeval));
    memset(&end_timeval, 0, sizeof(end_timeval));
}


