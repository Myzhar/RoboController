#-------------------------------------------------
#
# Project created by QtCreator 2012-10-23T11:42:48
#
#-------------------------------------------------

QT       += core

TARGET = RoboControllerSDK
TEMPLATE = lib

DEFINES += ROBOCONTROLLERSDK_LIBRARY

ROBOCONTROLLERSDKPATH = $$PWD

message( ROBOCONTROLLERSDKPATH: $$ROBOCONTROLLERSDKPATH)

INCLUDEPATH += \
        ../common/3rdparty/qcustomplot/include/ \
        ../common/include/

include($$ROBOCONTROLLERSDKPATH/mod_CORE/RoboControllerSDK_CORE.pri) # Core module
include($$ROBOCONTROLLERSDKPATH/mod_VISION/RoboControllerSDK_VISION.pri) # Vision module: to be included always before GUI module
include($$ROBOCONTROLLERSDKPATH/mod_VIDEOSTREAM/RoboControllerSDK_VIDEOSTREAM.pri)    # VIDEOSTREAM module
include($$ROBOCONTROLLERSDKPATH/mod_GUI/RoboControllerSDK_GUI.pri) # GUI module
include($$ROBOCONTROLLERSDKPATH/mod_SERVER/RoboControllerSDK_SERVER.pri) # SERVER module
include($$ROBOCONTROLLERSDKPATH/mod_EXTERN/RoboControllerSDK_EXTERN.pri) # EXTERN module: always the last module included

