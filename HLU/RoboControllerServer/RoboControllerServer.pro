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

SOURCES +=  \
            main.cpp \
            ../common/src/loghandler.cpp \

HEADERS +=  \
            ../common/include/loghandler.h

