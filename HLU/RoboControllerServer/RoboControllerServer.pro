#-------------------------------------------------
#
# Project created by QtCreator 2012-04-14T13:59:47
#
#-------------------------------------------------

QT       += core network testlib serialport

QT       -= gui

TARGET = RobotTcpServer
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

DEFINES += ROBOCONTROLLERSDK_LIBRARY

INCLUDEPATH += \
               include \
               ../common/include\
               ../RoboControllerSDK/include \
               ../common/3rdparty/libmodbus-3.0.1 \
               ../common/3rdparty/libmodbus-3.0.1/src

SOURCES +=  \
            main.cpp \
            ../common/3rdparty/libmodbus-3.0.1/src/modbus.c \
            ../common/3rdparty/libmodbus-3.0.1/src/modbus-tcp.c \
            ../common/3rdparty/libmodbus-3.0.1/src/modbus-rtu.c \
            ../common/3rdparty/libmodbus-3.0.1/src/modbus-data.c \
            ../common/src/loghandler.cpp \            
            ../RoboControllerSDK/src/qrobottcpserver.cpp \
            ../RoboControllerSDK/src/exception.cpp

HEADERS +=  \
            ../common/3rdparty/libmodbus-3.0.1/src/modbus.h \
            ../common/include/loghandler.h \
            ../common/include/network_msg.h \            
            ../RoboControllerSDK/include/qrobottcpserver.h \
            ../RoboControllerSDK/include/exception.h

unix {
    DEFINES += _TTY_POSIX_
}

win32 {
    DEFINES += _TTY_WIN_  WINVER=0x0501 __WIN32__ _WIN32_WINNT=0x0501
    LIBS += -lsetupapi \
            -lwsock32 \
            -lws2_32
}
