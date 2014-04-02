#-------------------------------------------------
#
# Project created by QtCreator 2013-04-20T11:36:43
#
#-------------------------------------------------

QT       += core gui printsupport network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RoboController-PID_Tuner
TEMPLATE = app

ROBOCONTROLLERSDKPATH = ../RoboControllerSDK

DEFINES += ROBOCONTROLLERSDK_LIBRARY

include($$ROBOCONTROLLERSDKPATH/mod_CORE/RoboControllerSDK_CORE.pri) # Core module
include($$ROBOCONTROLLERSDKPATH/mod_GUI/RoboControllerSDK_GUI.pri) # GUI module
include($$ROBOCONTROLLERSDKPATH/mod_EXTERN/RoboControllerSDK_EXTERN.pri) # EXTERN module

INCLUDEPATH += \
        ../common/3rdparty/qcustomplot/include/ \
        ../common/include/

SOURCES += \
        main.cpp\
        cmainwindow.cpp \        
        cselectipdlg.cpp \
        cconfiginidialog.cpp

HEADERS  += \
    cmainwindow.h \    
    macros.h \
    cselectipdlg.h \
    cconfiginidialog.h

FORMS    += \
    cmainwindow.ui \
    cselectipdlg.ui \
    cconfiginidialog.ui
