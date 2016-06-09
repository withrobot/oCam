#-------------------------------------------------
#
# Project created by QtCreator 2015-10-12T18:56:22
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = oCam-viewer
TEMPLATE = app

CONFIG += debug_and_release

SOURCES += main.cpp \
    controlform_boolean.cpp \
    controlform_integer.cpp \
    controlform_menu.cpp \
    format_tree_form.cpp \
    format_converter/colorspaces.c \
    format_converter/jpeg_decoder.c \
    oCam_viewer.cpp \
    withrobot_utility.cpp \
    withrobot_camera.cpp

HEADERS += \
    controlform_boolean.h \
    controlform_integer.h \
    controlform_menu.h \
    format_tree_form.h \
    format_converter/colorspaces.h \
    format_converter/format_converter.hpp \
    format_converter/jpeg_decoder.h \
    oCam_viewer.h \
    withrobot_utility.hpp \
    withrobot_debug_print.h \
    withrobot_camera.hpp

FORMS    += \
    controlform_boolean.ui \
    controlform_integer.ui \
    controlform_menu.ui \
    format_tree_form.ui \
    oCam_viewer.ui

INCLUDEPATH += -I/usr/local/include

LIBS += -L/usr/local/lib
LIBS += -lv4l2
LIBS += -ludev

RESOURCES += \
    oCamResources.qrc


