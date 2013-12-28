#-------------------------------------------------
#
# Project created by QtCreator 2012-10-23T11:42:48
#
#-------------------------------------------------

QT       += core

TARGET = RoboControllerSDK
TEMPLATE = lib

DEFINES += ROBOCONTROLLERSDK_LIBRARY

ROBOCONTROLLERSDKPATH = ./

INCLUDEPATH += \
        ../common/3rdparty/qcustomplot/include/ \
        ../common/include/

include($$ROBOCONTROLLERSDKPATH/mod_CORE/RoboControllerSDK_CORE.pri) # Core module
include($$ROBOCONTROLLERSDKPATH/mod_GUI/RoboControllerSDK_GUI.pri) # GUI module

