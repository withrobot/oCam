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

/**
 * 201702
 *  - 1CGN 옵션 및 기능 추가 (bayer2Rgb, color balance)
 */

#include "time.h"
#include "oCam_viewer.h"
#include "ui_oCam_viewer.h"
#include "format_converter/ConvertColor.h"
#include "image_funcs.hpp"

#include "withrobot_debug_print.h"

#include <QGraphicsPixmapItem>
#include <QMessageBox>
#include <QThread>

#define PIX_FORMAT_MJPEG        0x47504A4D
#define PIX_FORMAT_YUYV         0x56595559
#define PIX_FORMAT_GREY         0x59455247
#define PIX_FORMAT_BAYER_GBRG   0x47524247
#define PIX_FORMAT_BAYER_GRBG   0x47425247
#define PIX_FORMAT_UNKNOWN      0x00000000

#define OCAM_BAYER_RGB_NAME "oCam-1CGN"

oCam::oCam(QWidget *parent) :
    QMainWindow(parent), ui(new Ui::oCam), ui_default_contents_margin(3,3,3,3), frame_interval("Frame Interval", 5), showConvertImage(false)
{   
    ui->setupUi(this);
    ui->centralWidget->setContentsMargins(ui_default_contents_margin);
    ui->frmImage->setContentsMargins(ui_default_contents_margin);

    ui->ckbToggleConvert->setDisabled(true);

    fmt_name.clear();
    title.clear();

    ocam = 0;
    qtmr_timer = 0;

    frame_buffer = 0;
    rgb_buffer = 0;
    ir_buffer = 0;
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

    /* format */
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
        if (!integer_control_form_list[i]->is_enabled()) {
            strcpy(ctrl.name, integer_control_form_list[i]->get_name());
            ocam->get_control(ctrl);
            integer_control_form_list[i]->set_enabled(ctrl.flags != Withrobot::CAM_CTRL_FLAG_INACTIVE);
        }

        if (!integer_control_form_list[i]->value_changed()) {
            continue;
        }

        ocam->set_control(integer_control_form_list[i]->get_name(), integer_control_form_list[i]->get_value());
    }

    for (unsigned int i=0; i < menu_control_form_list.size(); i++) {
        if (!menu_control_form_list[i]->is_enabled()) {
            strcpy(ctrl.name, menu_control_form_list[i]->get_name());
            ocam->get_control(ctrl);
            menu_control_form_list[i]->set_enabled(ctrl.flags != Withrobot::CAM_CTRL_FLAG_INACTIVE);
        }

        if(!menu_control_form_list[i]->value_changed()) {
            continue;
        }

        ocam->set_control(menu_control_form_list[i]->get_name(), menu_control_form_list[i]->get_value());
    }

    for (unsigned int i=0; i < boolean_control_form_list.size(); i++) {
        if (!boolean_control_form_list[i]->is_enabled()) {
            strcpy(ctrl.name, boolean_control_form_list[i]->get_name());
            ocam->get_control(ctrl);
            boolean_control_form_list[i]->set_enabled(ctrl.flags != Withrobot::CAM_CTRL_FLAG_INACTIVE);
        }

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
        switch (format.pixformat)
        {
        case PIX_FORMAT_YUYV:
            /* convert yuyv to rgb */
            if(StereoImage){
                /*Split StereoImage funtion*/
                Split_Stereo_image(frame_buffer,stereo_buffer,format.width, format.height);
                Bayer2BGR(stereo_buffer, rgb_buffer, format.width, format.height, BayerGB2RGB);
            }
            else{
                format_converter->yuyv_to_rgb(rgb_buffer, frame_buffer);
            }
            
            break;

        case PIX_FORMAT_MJPEG:
            /* convert mjpeg to rgb */
            //format_converter->jpeg_to_rgb(rgb_buffer, frame_buffer, frame_size);
            break;

        case PIX_FORMAT_GREY:
            /* convert grey to rgb */
            format_converter->grey_to_rgb(rgb_buffer, frame_buffer);
            break;

        case PIX_FORMAT_BAYER_GBRG:
            if (showConvertImage) {
                /* convert bayerRGB to RGB */
                Bayer2BGR(frame_buffer, rgb_buffer, format.width, format.height, BayerGR2RGB);
            }
            else {
                /* convert grey to rgb */
                format_converter->grey_to_rgb(rgb_buffer, frame_buffer);
            }
            break;
        case PIX_FORMAT_BAYER_GRBG:
            if(IRImage)
            {
                if (showConvertImage) {
                    /* collect IR Pixel and fill empty pixel */
                    generate_IR_image(frame_buffer, rgb_buffer);
                    Bayer2BGR(frame_buffer, rgb_buffer, format.width, format.height, BayerGB2RGB);
                    //format_converter->grey_to_rgb(rgb_buffer, frame_buffer);
                }

                else {
                    /* convert grey to rgb */
                    generate_RGB_image(frame_buffer, rgb_buffer);
                    //green-light control
                    for (unsigned int i = 1; i < format.width*format.height * 3; i+=3) {
                        rgb_buffer[i] = (unsigned char)rgb_buffer[i] * 0.7;
                    }
                }
            }
            else
            {
                if (showConvertImage) {
                    /* convert bayerRGB to RGB */
                    Bayer2BGR(frame_buffer, rgb_buffer, format.width, format.height, BayerGB2RGB);
                }
                else {
                    /* convert grey to rgb */
                    format_converter->grey_to_rgb(rgb_buffer, frame_buffer);
                }
            }

            break;
        default:
            break;
        }
        /* show image */
        
        ui->lblImage->setPixmap(QPixmap::fromImage(QImage(rgb_buffer, format.width, format.height, QImage::Format_RGB888)));
        ui->dockControls->setWindowTitle(("   " + title + " " + Withrobot::to_string<double>(round(1.0 / frame_interval.restart())) + " fps").c_str());
#endif
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
 * 시작 버튼: 클릭하면 프로그램 시작/종료
 */
void oCam::on_btnStart_clicked()
{
    if (ocam)
    {
        if (stop())
        {
            enum_dev_list();
            ui->cbbDeviceList->setDisabled(false);
            ui->btnStart->setText("Connect");
            ui->tbtnDevRefresh->setDisabled(false);
        }
    }
    else
    {
        if (start())
        {
            ui->cbbDeviceList->setDisabled(true);
            ui->btnStart->setText("Disconnect");
            ui->tbtnDevRefresh->setDisabled(true);
        }
        //usleep(1000);
        //format_size = format.image_size;
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

    DBG_PRINTF("title: %s\n", title.c_str());
    
    /* check bayer rgb sensor camera */
    if (format.pixformat == PIX_FORMAT_BAYER_GRBG) {
        
        if( ocam->get_dev_name() == "oCam-4IRO-U" )
        {
            StereoImage = false;
            IRImage = true;
            ui->ckbToggleConvert->setEnabled(true);
            if(showConvertImage)
            {
                ui->ckbToggleConvert->setChecked(true);
            }
            ui->ckbToggleConvert->setText("Show IR Image (oCam-4IRO-U Only)");
            defaultPushBtn = new ControlFormPushBtn("Set Default", "Do it.");
            ui->vLayoutMisc->addWidget(defaultPushBtn);
            defaultPushBtn->setEnabled(true);

            connect(defaultPushBtn, SIGNAL(btnClicked(bool)), this, SLOT(set_default_color_correction()));
            misc_control_form_pushBtn.push_back(defaultPushBtn);
        }
        else
        {
            StereoImage = false;
            IRImage = false;
            ui->ckbToggleConvert->setEnabled(true);
            ui->ckbToggleConvert->setChecked(true);
            ui->ckbToggleConvert->setText("Show RGB Image (oCam-1CGN-U Only)");

            // add miscellaneous controls
            /* Default Push Button */
            defaultPushBtn = new ControlFormPushBtn("Set Default", "Do it.");
            ui->vLayoutMisc->addWidget(defaultPushBtn);
            defaultPushBtn->setEnabled(true);

            connect(defaultPushBtn, SIGNAL(btnClicked(bool)), this, SLOT(set_default_color_correction()));
            misc_control_form_pushBtn.push_back(defaultPushBtn);

            /* Reset Push Button */
            resetPushBtn = new ControlFormPushBtn("Reset Color correction", "Do it.");
            ui->vLayoutMisc->addWidget(resetPushBtn);
            resetPushBtn->setEnabled(true);

            connect(resetPushBtn, SIGNAL(btnClicked(bool)), this, SLOT(reset_color_correction()));
            misc_control_form_pushBtn.push_back(resetPushBtn);

            /* Color Correction Push Button */
            correctionPushBtn = new ControlFormPushBtn("Color correction", "Do it.");
            ui->vLayoutMisc->addWidget(correctionPushBtn);
            correctionPushBtn->setEnabled(false);

            connect(correctionPushBtn, SIGNAL(btnClicked(bool)), this, SLOT(calculate_color_correction()));
            misc_control_form_pushBtn.push_back(correctionPushBtn);
        }

    }

    if (format.pixformat == PIX_FORMAT_BAYER_GBRG) {
        if( ocam->get_dev_name() == "oCam-18CRN-U" )
        {
            ui->ckbToggleConvert->setEnabled(true);
            ui->ckbToggleConvert->setChecked(true);
            ui->ckbToggleConvert->setText("Show RGB Image (oCam-18CRN-U Only)");

            // add miscellaneous controls
            /* Default Push Button */
            defaultPushBtn = new ControlFormPushBtn("Set Default", "Do it.");
            ui->vLayoutMisc->addWidget(defaultPushBtn);
            defaultPushBtn->setEnabled(true);

            connect(defaultPushBtn, SIGNAL(btnClicked(bool)), this, SLOT(set_default_color_correction()));
            misc_control_form_pushBtn.push_back(defaultPushBtn);

            /* Reset Push Button */
            resetPushBtn = new ControlFormPushBtn("Reset Color correction", "Do it.");
            ui->vLayoutMisc->addWidget(resetPushBtn);
            resetPushBtn->setEnabled(true);

            connect(resetPushBtn, SIGNAL(btnClicked(bool)), this, SLOT(reset_color_correction()));
            misc_control_form_pushBtn.push_back(resetPushBtn);

            /* Color Correction Push Button */
            correctionPushBtn = new ControlFormPushBtn("Color correction", "Do it.");
            ui->vLayoutMisc->addWidget(correctionPushBtn);
            correctionPushBtn->setEnabled(false);

            connect(correctionPushBtn, SIGNAL(btnClicked(bool)), this, SLOT(calculate_color_correction()));
            misc_control_form_pushBtn.push_back(correctionPushBtn);
        }

    }
    else{
        if(ocam->get_dev_name() =="oCamS-1CGN-U"){
            format.width = format.width*2;
            StereoImage = true;
        }
        else if(ocam->get_dev_name() =="oCamS-1MGN-U"){
            format.width = format.width*2;
            StereoImage = true;
        }
    }
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
    if(StereoImage){
        rgb_buffer = new unsigned char[format.height*format.width*6];
    }
    else{
        rgb_buffer = new unsigned char[format.height*format.width*3];
    }
    ir_buffer = new unsigned char[format.height*format.width/4];
    stereo_buffer = new unsigned char[format.height*format.width*2];
    /* format converter initialize */
    format_converter = new GuvcviewFormatConverter(format.width, format.height);

    //receive signal when get frame
    qtmr_timer = new QTimer(this);
    connect(qtmr_timer, SIGNAL(timeout()), this, SLOT(update_gui()));

#ifdef STATIC_QTIMER_RATE
    qtmr_timer->start(STATIC_QTIMER_RATE);
#else
    int timer_rate = (int)(1.0 / format.frame_rate * 1000.0);
    std::cout << "Frame Rate : " << format.frame_rate << "Timer rate : " << timer_rate << std::endl;
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

    for(std::size_t i=0; i < misc_control_form_pushBtn.size(); i++) {
        if (misc_control_form_pushBtn[i]) {
            delete misc_control_form_pushBtn[i];
        }
    }
    misc_control_form_pushBtn.clear();


    ui->ckbToggleConvert->setDisabled(true);
    /* stop */
    ocam->stop();
    delete ocam;
    delete[] frame_buffer;
    delete[] rgb_buffer;
    delete[] ir_buffer;
    delete[] stereo_buffer;

    delete format_converter;

    ocam = 0;
    qtmr_timer->stop();
    delete qtmr_timer;
    qtmr_timer = 0;
    frame_buffer = 0;
    rgb_buffer = 0;
    ir_buffer = 0;

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

void oCam::on_ckbToggleConvert_toggled(bool checked)
{
    if (checked) {
        DBG_PRINTF("Show convert image");
        showConvertImage = true;
    }
    else {
        DBG_PRINTF("Show raw image");
        showConvertImage = false;
    }
}


/*
 *  Color correction for 1CGN(Bayer RGB pattern)
 */
//#define FLOAT_ORDER     2   // 소수점 두 째 자리까지만 고려한다.
#define FLOAT_SCALER    100 // 소수점을 UVC 인터페이스에 설정할 수 없으므로, 소수점 두째자리까지 정수로 변환하기 위한 값
#define SAVE_SCALE      0x73617665  // scale 저장명령(ASCII:save)
#define ERASE_SCALE     0x78787878  // scale 지우기명령
#define LOAD_SCALE      0x89898989  // scale 불러오기명령
#define ERASE_FIRMWARE  0x30303030  // 펌웨어 지우기명령

static const int DEFAULT_EXPOSURE = 128;
static const int DEFAULT_GAIN = 64;
static const int DEFAULT_WB_COMP = 100;

void oCam::calculate_color_correction()
{
    correctionPushBtn->setEnabled(false);

    DBG_PRINTF("calculate_color_correction called!");

    /*
     *  calculate the white balance
     */
    Bayer2BGR(frame_buffer, rgb_buffer, format.width, format.height, BayerGB2RGB);

    double normList[3];
    calNormOfImage(normList, rgb_buffer, format.width, format.height);

    double scaleRed = normList[1] / normList[2];
    double scaleBlue = normList[1] / normList[0];

    int settingValueRed = static_cast<int>(round(scaleRed * FLOAT_SCALER));
    int settingValueBlue = static_cast<int>(round(scaleBlue * FLOAT_SCALER));

    DBG_PRINTF("scaleRed: %f(%d), scaleBlue: %f(%d)\n", scaleRed, settingValueRed, scaleBlue, settingValueBlue);

    /* White Balance Red/Blue setting */
    ocam->set_control("White Balance Blue Component", settingValueBlue);
    ocam->set_control("White Balance Red Component", settingValueRed);

    /* White Balance Red/Blue setting & slider update*/
    for (unsigned int i=0; i < integer_control_form_list.size(); i++) {
        if (integer_control_form_list[i]->is_enabled()) {
            if(!strcmp("White Balance Blue Component", integer_control_form_list[i]->get_name()))
                integer_control_form_list[i]->set_value(settingValueBlue);

            if(!strcmp("White Balance Red Component", integer_control_form_list[i]->get_name()))
                integer_control_form_list[i]->set_value(settingValueRed);
        }
    }

    /*
     * save trigger
     */
    control_command(SAVE_SCALE);

    // 껏다키는방법
    ocam->stop();
    ocam->start();

    // save trigger 이후, Gain 값이 변했으므로 껐다가 켰을때 64로 시작 할 수 있도록
    ocam->set_control("Gain", DEFAULT_GAIN);
}


void oCam::reset_color_correction()
{
    //    correctionPushBtn->setEnabled(false);

    DBG_PRINTF("reset_color_correction called!");


    set_default_color_correction();
    /*
     * erase scale trigger
     */
    control_command(ERASE_SCALE);

    /*
     * load scale trigger
     */
    control_command(LOAD_SCALE);

    // 껏다키는방법
    ocam->stop();
    ocam->start();

    // save trigger 이후, Gain 값이 변했으므로 껐다가 켰을때 64로 시작 할 수 있도록
    ocam->set_control("Gain", DEFAULT_GAIN);

    correctionPushBtn->setEnabled(true);

}


void oCam::set_default_color_correction()
{
    /* White Balance Red/Blue setting & slider update*/
    for (unsigned int i=0; i < integer_control_form_list.size(); i++) {
        if (integer_control_form_list[i]->is_enabled()) {
            if(!strcmp("Exposure (Absolute)", integer_control_form_list[i]->get_name()))
                integer_control_form_list[i]->set_value(DEFAULT_EXPOSURE);

            if(!strcmp("Gain", integer_control_form_list[i]->get_name()))
                integer_control_form_list[i]->set_value(DEFAULT_GAIN);

            if(!strcmp("White Balance Blue Component", integer_control_form_list[i]->get_name()))
                integer_control_form_list[i]->set_value(DEFAULT_WB_COMP);

            if(!strcmp("White Balance Red Component", integer_control_form_list[i]->get_name()))
                integer_control_form_list[i]->set_value(DEFAULT_WB_COMP);
        }
    }

}

void oCam::control_command(uint32_t value)
{
    uint8_t cmd[8];

    cmd[0] = (value >> 28) & 0xf;
    cmd[1] = (value >> 24) & 0xf;
    cmd[2] = (value >> 20) & 0xf;
    cmd[3] = (value >> 16) & 0xf;
    cmd[4] = (value >> 12) & 0xf;
    cmd[5] = (value >>  8) & 0xf;
    cmd[6] = (value >>  4) & 0xf;
    cmd[7] = (value >>  0) & 0xf;

    printf("ControlCommand=0x%X%X%X%X%X%X%X%X\n",cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5],cmd[6],cmd[7]);

    ocam->set_control("Gain", cmd[0]);
    ocam->set_control("Gain", cmd[1]);
    ocam->set_control("Gain", cmd[2]);
    ocam->set_control("Gain", cmd[3]);
    ocam->set_control("Gain", cmd[4]);
    ocam->set_control("Gain", cmd[5]);
    ocam->set_control("Gain", cmd[6]);
    ocam->set_control("Gain", cmd[7]);

}

/**
 * RGB-IR 이미지에서 IR 추출
 */
void oCam::generate_IR_image(unsigned char* frame_buffer, unsigned char* ir_buffer)
{
    unsigned char *calIR_space = (unsigned char*)frame_buffer;
    int Width = format.width;
    int Height = format.height;
    int Height_last_line = Height - 1;
    int Width_last_line = Width - 1;
    memset(ir_buffer, 0, (Width)*(Height));

    //step 1~3
    int i = 0, j = 0;
    for (i = 0; i < Height_last_line-1; i += 2) {
        for (j = 0; j <Width_last_line; j += 8) {
            calIR_space[i*Width + (j+1)] = (calIR_space[i*Width + j] + calIR_space[i*Width + (j + 2)]) >> 1;
            calIR_space[(i+1)*Width + j] = (calIR_space[i*Width + j] + calIR_space[(i + 2)*Width + j]) >> 1;
            calIR_space[(i+1)*Width + (j+2)] = (calIR_space[i *Width + j] + calIR_space[(i + 2)*Width + j]) >> 1;
            calIR_space[(i+1)*Width + (j+1)] = (calIR_space[(i + 1)*Width + j] + calIR_space[(i + 1)*Width + (j + 2)]) >> 1;
            calIR_space[i*Width + (j+3)] = (calIR_space[i*Width + (j + 2)] + calIR_space[i*Width + (j + 4)]) >> 1;
            calIR_space[(i+1)*Width + (j+4)] = (calIR_space[i*Width + (j + 4)] + calIR_space[(i + 2)*Width + (j + 4)]) >> 1;
            calIR_space[(i+1)*Width + (j+3)] = (calIR_space[(i + 1)*Width + (j + 2)] + calIR_space[(i + 1)*Width + (j + 4)]) >> 1;
            calIR_space[i*Width + (j+5)] = (calIR_space[i*Width + (j + 4)] + calIR_space[i*Width + (j + 6)]) >> 1;
            calIR_space[(i+1)*Width + (j+6)] = (calIR_space[i*Width + (j + 6)] + calIR_space[(i + 2)*Width + (j + 6)]) >> 1;
            calIR_space[(i+1)*Width + (j+5)] = (calIR_space[(i + 1)*Width + (j + 4)] + calIR_space[(i + 1)*Width + (j + 6)]) >> 1;
            calIR_space[i*Width + (j+7)] = (calIR_space[i*Width + (j + 6)] + calIR_space[i*Width + (j + 8)]) >> 1;
            calIR_space[(i+1)*Width + (j+7)] = (calIR_space[i*Width + (j + 6)] + calIR_space[i*Width + (j + 8)] + calIR_space[(i + 2)*Width + (j + 6)] + calIR_space[(i + 2)*Width + (j + 8)]) >> 2;
        }

        calIR_space[i*Width + (j + 1)] = (calIR_space[i*Width + j] + calIR_space[i*Width + (j + 2)]) >> 1;
        calIR_space[(i + 1)*Width + j] = (calIR_space[i*Width + j] + calIR_space[(i + 2)*Width + j]) >> 1;
        calIR_space[(i + 1)*Width + (j + 2)] = (calIR_space[i *Width + j] + calIR_space[(i + 2)*Width + j]) >> 1;
        calIR_space[(i + 1)*Width + (j + 1)] = (calIR_space[(i + 1)*Width + j] + calIR_space[(i + 1)*Width + (j + 2)]) >> 1;
        calIR_space[i*Width + (j + 3)] = (calIR_space[i*Width + (j + 2)] + calIR_space[i*Width + (j + 4)]) >> 1;
        calIR_space[(i + 1)*Width + (j + 4)] = (calIR_space[i*Width + (j + 4)] + calIR_space[(i + 2)*Width + (j + 4)]) >> 1;
        calIR_space[(i + 1)*Width + (j + 3)] = (calIR_space[(i + 1)*Width + (j + 2)] + calIR_space[(i + 1)*Width + (j + 4)]) >> 1;
        calIR_space[i*Width + (j + 5)] = (calIR_space[i*Width + (j + 4)] + calIR_space[i*Width + (j + 6)]) >> 1;
        calIR_space[(i + 1)*Width + (j + 6)] = (calIR_space[i*Width + (j + 6)] + calIR_space[(i + 2)*Width + (j + 6)]) >> 1;
        calIR_space[(i + 1)*Width + (j + 5)] = (calIR_space[(i + 1)*Width + (j + 4)] + calIR_space[(i + 1)*Width + (j + 6)]) >> 1;
        calIR_space[i*Width + (j + 7)] = calIR_space[i*Width + (j + 6)];
        calIR_space[(i + 1)*Width + (j + 7)] = calIR_space[(i + 1)*Width + (j + 6)];
    }

    for (j = 0; j < Width_last_line; j += 8) {
        calIR_space[i*Width + (j + 1)] = (calIR_space[i*Width + j] + calIR_space[i*Width + (j + 2)]) >> 1;
        calIR_space[(i + 1)*Width + j] = calIR_space[i*Width + j];
        calIR_space[(i + 1)*Width + (j + 2)] = calIR_space[i*Width + (j + 2)];
        calIR_space[(i + 1)*Width + (j + 1)] = calIR_space[i*Width + (j + 1)];
        calIR_space[i*Width + (j + 3)] = (calIR_space[i*Width + (j + 2)] + calIR_space[i*Width + (j + 4)]) >> 1;
        calIR_space[(i + 1)*Width + (j + 4)] = calIR_space[i*Width + (j + 4)];
        calIR_space[(i + 1)*Width + (j + 3)] = calIR_space[i*Width + (j + 3)];
        calIR_space[i*Width + (j + 5)] = (calIR_space[i*Width + (j + 4)] + calIR_space[i*Width + (j + 6)]) >> 1;
        calIR_space[(i + 1)*Width + (j + 6)] = calIR_space[i*Width + (j + 6)];
        calIR_space[(i + 1)*Width + (j + 5)] = calIR_space[i*Width + (j + 5)];
        calIR_space[i*Width + (j + 7)] = (calIR_space[i*Width + (j + 6)] + calIR_space[i*Width + (j + 8)]) >> 1;
        calIR_space[(i + 1)*Width + (j + 7)] = calIR_space[i*Width + (j + 7)];
    }

    calIR_space[i*Width + (j + 1)] = (calIR_space[i*Width + j] + calIR_space[i*Width + (j + 2)]) >> 1;
    calIR_space[(i + 1)*Width + j] = calIR_space[i*Width + j];
    calIR_space[(i + 1)*Width + (j + 2)] = calIR_space[i*Width + (j + 2)];
    calIR_space[(i + 1)*Width + (j + 1)] = calIR_space[i*Width + (j + 1)];
    calIR_space[i*Width + (j + 3)] = (calIR_space[i*Width + (j + 2)] + calIR_space[i*Width + (j + 4)]) >> 1;
    calIR_space[(i + 1)*Width + (j + 4)] = calIR_space[i*Width + (j + 4)];
    calIR_space[(i + 1)*Width + (j + 3)] = calIR_space[i*Width + (j + 3)];
    calIR_space[i*Width + (j + 5)] = (calIR_space[i*Width + (j + 4)] + calIR_space[i*Width + (j + 6)]) >> 1;
    calIR_space[(i + 1)*Width + (j + 6)] = calIR_space[i*Width + (j + 6)];
    calIR_space[(i + 1)*Width + (j + 5)] = calIR_space[i*Width + (j + 5)];
    calIR_space[i*Width + (j + 7)] = calIR_space[i*Width + (j + 6)];
    calIR_space[(i + 1)*Width + (j + 7)] = calIR_space[i*Width + (j + 7)];

    memcpy(ir_buffer, calIR_space, (Width)*(Height));
}

/**
 * RGB-IR 이미지에서 RGB 추출
 */
void oCam::generate_RGB_image(unsigned char* frame_buffer, unsigned char* rgb_buffer)
{
    unsigned char *bayer = (unsigned char*)frame_buffer;
    int Width = format.width;
    int Height = format.height;

    for (int i = 0; i < Height; i+=2) {
        for (int j = 0; j < Width; j+=2) {
            if (i == 0 && j == 0) {
                bayer[0] = bayer[1 * Width + 1];
            }
            else if (i == 0 && j != 0) {
                bayer[(i)*Width + (j)] = (bayer[(i + 1)*Width + (j - 1)] + bayer[(i + 1)*Width + (j + 1)]) / 2;
            }
            else if (i != 0 && j == 0) {
                bayer[(i)*Width + (j)] = (bayer[(i - 1)*Width + (j + 1)] + bayer[(i + 1)*Width + (j + 1)]) / 2;
            }
            else {
                bayer[(i)*Width + (j)] = (bayer[(i - 1)*Width + (j - 1)]  + bayer[(i - 1)*Width + (j + 1)] + bayer[(i + 1)*Width + (j - 1)] + bayer[(i + 1)*Width + (j + 1)]) / 4;
            }
        }
    }
    Bayer2BGR(bayer, rgb_buffer, format.width, format.height, BayerGB2RGB);
}
void oCam::Split_Stereo_image(unsigned char* Src, unsigned char* Dst, int Width, int Height)
{
	unsigned char* Srcimg = (unsigned char*)Src;
	unsigned char* Dstimg = (unsigned char*)Dst;
	memset(Dst, 0, Width * Height * sizeof(Dst[0]));
	unsigned char temp = 0;
	int k = 0;
	Width = Width/2;
	for (int i = 0; i < Height; i++) {
		k = 0;
		for (int j = 0; j < Width*2; j+=2) {
			Dstimg[(i * Width*2) + (k + Width)] = Srcimg[i*Width * 2 + j];
			Dstimg[(i * Width*2) + (k)] = Srcimg[i*Width * 2 + (j + 1)];
			k++;
		}
	}
}