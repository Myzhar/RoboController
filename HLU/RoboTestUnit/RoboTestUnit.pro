#-------------------------------------------------
#
# Project created by QtCreator 2014-03-29T13:35:09
#
#-------------------------------------------------

QT       += network testlib

QT       -= gui

TARGET = tst_robotestunittest
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

INCLUDEPATH += .

ROBOCONTROLLERSDKPATH = ../RoboControllerSDK

DEFINES += ROBOCONTROLLERSDK_LIBRARY

include($$ROBOCONTROLLERSDKPATH/mod_CORE/RoboControllerSDK_CORE.pri)        # CORE module

SOURCES += tst_robotestunittest.cpp
DEFINES += SRCDIR=\\\"$$PWD/\\\"

HEADERS +=


