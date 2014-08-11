#-------------------------------------------------
#
# Project created by QtCreator 2014-08-07T15:51:56
#
#-------------------------------------------------

TARGET = Test-AsusXTionProLive-OpenNI2
TEMPLATE = app

OPENNI_PATH = "C:/Program Files (x86)/OpenNI2"

ROBOCONTROLLERSDKPATH = ../RoboControllerSDK

DEFINES += ROBOCONTROLLERSDK_LIBRARY

include($$ROBOCONTROLLERSDKPATH/mod_CORE/RoboControllerSDK_CORE.pri) # Core module
include($$ROBOCONTROLLERSDKPATH/mod_VISION/RoboControllerSDK_VISION.pri) # Vision module: to be included always before GUI module
include($$ROBOCONTROLLERSDKPATH/mod_GUI/RoboControllerSDK_GUI.pri) # GUI module
include($$ROBOCONTROLLERSDKPATH/mod_EXTERN/RoboControllerSDK_EXTERN.pri) # EXTERN module: to be included always as last module

INCLUDEPATH += $$OPENNI_PATH/Include

LIBS += $$OPENNI_PATH/Lib/OpenNI2.lib

SOURCES +=  main.cpp\
            mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui
