/*******************************************************************************#
#                                                                               #
# Withrobot Camera GUI for oCam                                                 #
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

#include "oCam_viewer.h"
#include "ui_oCam_viewer.h"

#include "withrobot_debug_print.h"

#include <QGraphicsPixmapItem>
#include <QMessageBox>

#define PIX_FORMAT_MJPEG    0x47504A4D
#define PIX_FORMAT_YUYV     0x56595559
#define PIX_FORMAT_GREY     0x59455247
#define PIX_FORMAT_UNKNOWN  0x00000000

oCam::oCam(QWidget *parent) :
    QMainWindow(parent), ui(new Ui::oCam), ui_default_contents_margin(3,3,3,3), frame_interval("Frame Interval", 5)
{   
    ui->setupUi(this);
    ui->centralWidget->setContentsMargins(ui_default_contents_margin);
    ui->frmImage->setContentsMargins(ui_default_contents_margin);

    fmt_name.clear();
    title.clear();

    ocam = 0;
    qtmr_timer = 0;

    frame_buffer = 0;
    rgb_buffer = 0;
    format_converter = 0;

    /* initialize */
    format.width = DEFAULT_IMAGE_WIDTH;
    format.height = DEFAULT_IMAGE_HEIGHT;
    format.pixformat = DEFAULT_PIXFORMAT;

    enum_dev_list();
}

oCam::~oCam()
{
    stop();

    delete ui;
}

/**
 * QTimer 주기에 따라 이미지 출력 영역과 컨트롤 값을 갱신한다
 */
void oCam::update_gui()
{
    if (!ocam->is_running()) {
        return;
    }

    /* foramt */
    if (format_tree->is_changed()) {
        fmt_name = format_tree->get_format_name();
        if (!fmt_name.empty()) {
            restart();
            return;
        }
    }

    /* control */
    Withrobot::camera_control ctrl;

    for (unsigned int i=0; i < integer_control_form_list.size(); i++) {
        strcpy(ctrl.name, integer_control_form_list[i]->get_name());
        //ocam->get_control(ctrl);
        integer_control_form_list[i]->set_enabled(ctrl.flags != Withrobot::CAM_CTRL_FLAG_INACTIVE);

        if (!integer_control_form_list[i]->value_changed()) {
            continue;
        }

        ocam->set_control(integer_control_form_list[i]->get_name(), integer_control_form_list[i]->get_value());
    }

    for (unsigned int i=0; i < menu_control_form_list.size(); i++) {
        if(!menu_control_form_list[i]->value_changed()) {
            continue;
        }

        ocam->set_control(menu_control_form_list[i]->get_name(), menu_control_form_list[i]->get_value());
    }

    for (unsigned int i=0; i < boolean_control_form_list.size(); i++) {
        if(!boolean_control_form_list[i]->value_changed()) {
            continue;
        }

        ocam->set_control(boolean_control_form_list[i]->get_name(), boolean_control_form_list[i]->get_value());
    }

    /* get frame */
    int frame_size = ocam->get_frame(frame_buffer, format.image_size, 1);
    if (frame_size != -1) {
#ifndef NO_DRAW
        /* convert format */
        switch (format.pixformat) {
        case PIX_FORMAT_YUYV:
            /* convert yuyv to rgb */
            format_converter->yuyv_to_rgb(rgb_buffer, frame_buffer);
            break;

        case PIX_FORMAT_MJPEG:
            /* convert mjpeg to rgb */
            format_converter->jpeg_to_rgb(rgb_buffer, frame_buffer, frame_size);
            break;

        case PIX_FORMAT_GREY:
            /* convert grey to rgb */
            format_converter->grey_to_rgb(rgb_buffer, frame_buffer);
            break;

        default:
            break;
        }

        /* show image */
        ui->lblImage->setPixmap(QPixmap::fromImage(QImage(rgb_buffer, format.width, format.height, QImage::Format_RGB888)));
#endif
        ui->dockControls->setWindowTitle(("   " + title + " " + Withrobot::to_string<double>(round(1.0 / frame_interval.restart())) + " fps").c_str());
    }
    else {
        if (!QDir().exists(dev_node.c_str())) {
            ui->lblImage->clear();

            stop();
            ui->cbbDeviceList->setDisabled(false);
            ui->btnStart->setText("Connect");
            ui->tbtnDevRefresh->setDisabled(false);

            QMessageBox::warning(this, tr("Deivce warning"), tr("No device : ") + ui->cbbDeviceList->currentText());

            enum_dev_list();
        }
    }
}

/**
 * 시작 버튼: 클릭되면 프로그램 시작/종료
 */
void oCam::on_btnStart_clicked()
{
    if (ocam) {
        if (stop()) {
            enum_dev_list();

            ui->cbbDeviceList->setDisabled(false);
            ui->btnStart->setText("Connect");
            ui->tbtnDevRefresh->setDisabled(false);
        }
    } else {
        if (start()) {
            ui->cbbDeviceList->setDisabled(true);
            ui->btnStart->setText("Disconnect");
            ui->tbtnDevRefresh->setDisabled(true);
        }
    }
}

/**
 * 장치 연결 시작
 */
bool oCam::start()
{
    if (ocam) {
        return false;
    }
    DBG_PRINTF(fmt_name.c_str());

    /* Camera instance */
    dev_node = dev_list[ui->cbbDeviceList->currentIndex()].dev_node;

    if (!QDir().exists(dev_node.c_str())) {
        enum_dev_list();
        return false;
    }

    ocam = new Withrobot::Camera(dev_node.c_str(), &format, fmt_name.c_str());

    /* start streaming */
    if (!ocam->start()) {
        delete ocam;
        ocam = 0;

        ui->lblImage->clear();
        QMessageBox::warning(this, tr("Deivce warning"), tr("Device busy : ") + ui->cbbDeviceList->currentText());

        return false;
    }

    title = ocam->get_dev_name() + "  " +
            Withrobot::to_string<unsigned int>(format.width) + " x " + Withrobot::to_string<unsigned int>(format.height) + " (" +
            Withrobot::to_string<unsigned char>((format.pixformat >> 0) & 0xFF) +
            Withrobot::to_string<unsigned char>((format.pixformat >> 8) & 0xFF) +
            Withrobot::to_string<unsigned char>((format.pixformat >> 16) & 0xFF) +
            Withrobot::to_string<unsigned char>((format.pixformat >> 24) & 0xFF) + ") ";

    /* change window geometry */
    QRect geo = geometry();

#if 0
    unsigned int width = format.width + ui->twController->maximumWidth() + 100;
    unsigned int heigth = format.height + 200;

    if (width > MAX_WINDOW_WIDTH) {
        width = MAX_WINDOW_WIDTH;
    }

    if (heigth > MAX_WINDOW_HEIGHT) {
        heigth = MAX_WINDOW_HEIGHT;
    }

#else
    unsigned int width = DEFAULT_WINDOW_GEO_WIDTH;
    unsigned int heigth = DEFAULT_WINDOW_GEO_HEIGHT;
#endif

    geo.setWidth(width);
    geo.setHeight(heigth);
    setGeometry(geo);

    /* Enumerate avaliable formats */
    std::vector<const char*> format_list;
    std::vector<const char*> rate_list;

    format_tree = new FormatTreeForm(title.c_str());
    ocam->get_valid_image_format_list(format_list);
    for (unsigned int i=0; i < format_list.size(); i++) {
        ocam->get_valid_ratio_list(format_list[i], rate_list);
        format_tree->add_format_list(format_list[i], rate_list);
    }
    ui->frmTabFormat->layout()->addWidget(format_tree);

    /* Enumerate avaliable controls */
    std::vector<std::pair<const char*, unsigned int> > valid_control_list;

    void* control_form = 0;
    ocam->valid_controls(valid_control_list);
    for (unsigned int i=0; i < valid_control_list.size(); i++) {
        control.clear();
        strcpy(control.name, valid_control_list[i].first);

        switch (valid_control_list[i].second) {
        case Withrobot::CAM_CTRL_TYPE_BOOLEAN:
            ocam->get_control(control);

            control_form = new ControlFormBoolean(control.name, control.flags!=Withrobot::CAM_CTRL_FLAG_INACTIVE);
            ((ControlFormBoolean*)control_form)->set_value(control.value);

            ui->vLayoutControls->addWidget((ControlFormBoolean*)control_form);
            boolean_control_form_list.push_back((ControlFormBoolean*)control_form);
            control_form = 0;

            break;

        case Withrobot::CAM_CTRL_TYPE_MENU:
            ocam->get_control(control);

            control_form = new ControlFormMenu(control.name, control.menu_list, control.flags!=Withrobot::CAM_CTRL_FLAG_INACTIVE);
            ((ControlFormMenu*)control_form)->set_value(control.value);

            ui->vLayoutControls->addWidget((ControlFormMenu*)control_form);
            menu_control_form_list.push_back((ControlFormMenu*)control_form);
            control_form = 0;

            break;

        case Withrobot::CAM_CTRL_TYPE_INTEGER:
            ocam->get_control(control);

            control_form = new ControlFormInteger(control.name, control.minimum, control.maximum, control.step, control.flags!=Withrobot::CAM_CTRL_FLAG_INACTIVE);
            ((ControlFormInteger*)control_form)->set_value(control.value);

            ui->vLayoutControls->addWidget((ControlFormInteger*)control_form);
            integer_control_form_list.push_back((ControlFormInteger*)control_form);
            control_form = 0;

            break;

        default:
            break;
        }
    }

    /* image buffer */
    frame_buffer = new unsigned char[format.height*format.width*2];
    rgb_buffer = new unsigned char[format.height*format.width*3];

    /* format converter initialize */
    format_converter = new GuvcviewFormatConverter(format.width, format.height);

    /* image refresh thread */
    qtmr_timer = new QTimer(this);
    connect(qtmr_timer, SIGNAL(timeout()), this, SLOT(update_gui()));

#ifdef STATIC_QTIMER_RATE
    qtmr_timer->start(STATIC_QTIMER_RATE);
#else
    int timer_rate = (int)(1.0 / format.frame_rate * 1000.0);
    DBG_PRINTF("qTimer Rate: %d", timer_rate);
    qtmr_timer->start(timer_rate);
#endif

    return true;
}

/**
 * 장치 연결 중단
 */
bool oCam::stop()
{
    if (!ocam) {
        return false;
    }

    /* clear formats */
    delete format_tree;

    /* clear controls */
    for (unsigned int i=0; i < integer_control_form_list.size(); i++) {
        if (integer_control_form_list[i]) {
            delete integer_control_form_list[i];
        }
    }
    integer_control_form_list.clear();

    for (unsigned int i=0; i < menu_control_form_list.size(); i++) {
        if (menu_control_form_list[i]) {
            delete menu_control_form_list[i];
        }
    }
    menu_control_form_list.clear();

    for (unsigned int i=0; i < boolean_control_form_list.size(); i++) {
        if (boolean_control_form_list[i]) {
            delete boolean_control_form_list[i];
        }
    }
    boolean_control_form_list.clear();


    /* stop */
    ocam->stop();
    qtmr_timer->stop();

    delete ocam;
    delete qtmr_timer;

    delete[] frame_buffer;
    delete[] rgb_buffer;

    delete format_converter;

    ocam = 0;
    qtmr_timer = 0;

    frame_buffer = 0;
    rgb_buffer = 0;

    format_converter = 0;

    return true;
}

/**
 * 장치 재시작
 */
void oCam::restart()
{
    stop();
    start();
}

/**
 * DEFAULT_DEVICE_FOLDER안의 video장치를 목록화 한다
 */
void oCam::enum_dev_list()
{
    /* enumerate device(UVC compatible devices) list */
    std::vector<Withrobot::usb_device_info> dev_list_new;
    int dev_num = Withrobot::get_usb_device_info_list(dev_list_new);

    if (dev_num < 1) {
        ui->cbbDeviceList->clear();
        dev_list.clear();
        ui->btnStart->setDisabled(true);
        return;
    }
    else {
        ui->btnStart->setDisabled(false);
    }

    bool matched = dev_list_new.size() == dev_list.size();
    if (matched) {
        for (unsigned int i=0; i < dev_list_new.size(); i++) {
            matched = matched && (dev_list_new[i].dev_node == dev_list[i].dev_node);
            matched = matched && (dev_list_new[i].id_vendor == dev_list[i].id_vendor);
            matched = matched && (dev_list_new[i].id_product == dev_list[i].id_product);
            matched = matched && (dev_list_new[i].manufacturer == dev_list[i].manufacturer);
            matched = matched && (dev_list_new[i].product == dev_list[i].product);
            matched = matched && (dev_list_new[i].serial == dev_list[i].serial);
            matched = matched && (dev_list_new[i].busnum == dev_list[i].busnum);
            matched = matched && (dev_list_new[i].devnum == dev_list[i].devnum);

            if (!matched) {
                break;
            }
        }
    }

    if (!matched) {
        dev_list = dev_list_new;
        ui->cbbDeviceList->clear();

        for (unsigned int i=0; i < dev_list.size(); i++) {
            dev_list[i].print();
            if (dev_list[i].serial.empty()) {
                std::string dev_brief = dev_list[i].dev_node + " [ " + dev_list[i].product + " ]";
                ui->cbbDeviceList->addItem(tr(dev_brief.c_str()));
            }
            else {
                std::string dev_brief = dev_list[i].dev_node + " [ " + dev_list[i].product + " ( " + dev_list[i].serial + " )" + " ]";
                ui->cbbDeviceList->addItem(tr(dev_brief.c_str()));
            }
        }
    }
}

void oCam::on_tbtnDevRefresh_clicked()
{
    enum_dev_list();
}
