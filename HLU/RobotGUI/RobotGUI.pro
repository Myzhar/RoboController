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
include($$ROBOCONTROLLERSDKPATH/mod_GUI/RoboControllerSDK_GUI.pri) # GUI module

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

ANDROID_PACKAGE_SOURCE_DIR = $$PWD/android

OTHER_FILES += \
    android/AndroidManifest.xml


