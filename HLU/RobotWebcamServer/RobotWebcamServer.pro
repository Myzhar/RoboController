#-------------------------------------------------
#
# Project created by QtCreator 2014-04-29T12:05:17
#
#-------------------------------------------------

QT       += testlib

QT       -= gui

TARGET = RobotWebcamServer
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

ROBOCONTROLLERSDKPATH = ../RoboControllerSDK

DEFINES += ROBOCONTROLLERSDK_LIBRARY

include($$ROBOCONTROLLERSDKPATH/mod_CORE/RoboControllerSDK_CORE.pri)        # CORE module
include($$ROBOCONTROLLERSDKPATH/mod_VISION/RoboControllerSDK_VISION.pri)    # VISION module
include($$ROBOCONTROLLERSDKPATH/mod_VIDEOSTREAM/RoboControllerSDK_VIDEOSTREAM.pri)    # VIDEOSTREAM module
include($$ROBOCONTROLLERSDKPATH/mod_EXTERN/RoboControllerSDK_EXTERN.pri)    # EXTERN module

INCLUDEPATH += \
               include \

SOURCES += \
           main.cpp



