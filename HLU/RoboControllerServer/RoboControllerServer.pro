#-------------------------------------------------
#
# Project created by QtCreator 2012-04-14T13:59:47
#
#-------------------------------------------------

QT       += testlib

QT       -= gui

TARGET = RobotTcpServer
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

ROBOCONTROLLERSDKPATH = ../RoboControllerSDK

DEFINES += ROBOCONTROLLERSDK_LIBRARY

include($$ROBOCONTROLLERSDKPATH/mod_CORE/RoboControllerSDK_CORE.pri) # Core module
include($$ROBOCONTROLLERSDKPATH/mod_SERVER/RoboControllerSDK_SERVER.pri) # SERVER module

INCLUDEPATH += \
               include \
               ../common/include\
               ../common/3rdparty/libmodbus-3.0.1 \
               ../common/3rdparty/libmodbus-3.0.1/src

SOURCES +=  \
            main.cpp \
            ../common/3rdparty/libmodbus-3.0.1/src/modbus.c \
            ../common/3rdparty/libmodbus-3.0.1/src/modbus-tcp.c \
            ../common/3rdparty/libmodbus-3.0.1/src/modbus-rtu.c \
            ../common/3rdparty/libmodbus-3.0.1/src/modbus-data.c \
            ../common/src/loghandler.cpp \

HEADERS +=  \
            ../common/3rdparty/libmodbus-3.0.1/src/modbus.h \
            ../common/include/loghandler.h \
            ../common/include/network_msg.h \

unix {
    DEFINES += _TTY_POSIX_
}

win32 {
    DEFINES += _TTY_WIN_  WINVER=0x0501 __WIN32__ _WIN32_WINNT=0x0501
    LIBS += -lsetupapi \
            -lwsock32 \
            -lws2_32
}
