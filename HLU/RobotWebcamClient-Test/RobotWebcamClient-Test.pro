#-------------------------------------------------
#
# Project created by QtCreator 2014-07-24T19:04:01
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RobotWebcamClient-Test
TEMPLATE = app

ROBOCONTROLLERSDKPATH = ../RoboControllerSDK

DEFINES += ROBOCONTROLLERSDK_LIBRARY

include($$ROBOCONTROLLERSDKPATH/mod_CORE/RoboControllerSDK_CORE.pri) # Core module
include($$ROBOCONTROLLERSDKPATH/mod_VISION/RoboControllerSDK_VISION.pri) # Vision module: to be included always before GUI module
include($$ROBOCONTROLLERSDKPATH/mod_VIDEOSTREAM/RoboControllerSDK_VIDEOSTREAM.pri)    # VIDEOSTREAM module
include($$ROBOCONTROLLERSDKPATH/mod_GUI/RoboControllerSDK_GUI.pri) # GUI module
include($$ROBOCONTROLLERSDKPATH/mod_EXTERN/RoboControllerSDK_EXTERN.pri) # EXTERN module: to be included always as last module

SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

ANDROID_PACKAGE_SOURCE_DIR = $$PWD/android

OTHER_FILES += \
    android/AndroidManifest.xml \
    android/src/org/qtproject/qt5/android/bindings/QtActivity.java
