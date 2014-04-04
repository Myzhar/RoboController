#-------------------------------------------------
#
# Project created by QtCreator 2012-10-23T11:37:07
#
#-------------------------------------------------

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RobotGUI
TEMPLATE = app

ROBOCONTROLLERSDKPATH = ../RoboControllerSDK

DEFINES += ROBOCONTROLLERSDK_LIBRARY

include($$ROBOCONTROLLERSDKPATH/mod_CORE/RoboControllerSDK_CORE.pri) # Core module
include($$ROBOCONTROLLERSDKPATH/mod_VISION/RoboControllerSDK_VISION.pri) # Vision module: to be included always before GUI module
include($$ROBOCONTROLLERSDKPATH/mod_VIDEOSTREAM/RoboControllerSDK_VIDEOSTREAM.pri)    # VIDEOSTREAM module
include($$ROBOCONTROLLERSDKPATH/mod_GUI/RoboControllerSDK_GUI.pri) # GUI module
#include($$ROBOCONTROLLERSDKPATH/mod_SERVER/RoboControllerSDK_SERVER.pri) # SERVER module
include($$ROBOCONTROLLERSDKPATH/mod_EXTERN/RoboControllerSDK_EXTERN.pri) # EXTERN module: to be included always as last module

INCLUDEPATH += \
    ../common/include\

SOURCES += \
    main.cpp\
    cmainwindow.cpp\    
    qcommon.cpp

HEADERS  += \
    cmainwindow.h \
    qcommon.h

FORMS    += \
    cmainwindow.ui

RESOURCES += \
    resources.qrc

android {
    message(Building for Android)

    DEFINES += QT_NO_OPENGL

    ANDROID_PACKAGE_SOURCE_DIR = $$PWD/android

    OTHER_FILES += \
        android/AndroidManifest.xml
}


