#-------------------------------------------------
#
# Project created by QtCreator 2013-04-20T11:36:43
#
#-------------------------------------------------

QT       += core gui printsupport network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RoboController-PID_Regulator
TEMPLATE = app

DEFINES += ROBOCONTROLLERSDK_LIBRARY

INCLUDEPATH += \
        ../common/3rdparty/qcustomplot/include/ \
        ../common/include/ \
        ../RoboControllerSDK/include

SOURCES += \
        main.cpp\
        cmainwindow.cpp \        
        cselectipdlg.cpp \
        ../RoboControllerSDK/src/robocontrollersdk.cpp \
        ../common/3rdparty/qcustomplot/src/qcustomplot.cpp \
    cconfiginidialog.cpp

HEADERS  += \
    cmainwindow.h \    
    macros.h \
    cselectipdlg.h \
    ../RoboControllerSDK/include/robocontrollersdk.h \
    ../common/3rdparty/qcustomplot/include/qcustomplot.h \
    cconfiginidialog.h

FORMS    += \
    cmainwindow.ui \
    cselectipdlg.ui \
    cconfiginidialog.ui
