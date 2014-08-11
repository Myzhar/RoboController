message(Added CORE module)

QT += core network

CONFIG += CORE

SOURCES += \
    $$ROBOCONTROLLERSDKPATH/mod_CORE/src/robocontrollersdk.cpp \
    $$ROBOCONTROLLERSDKPATH/mod_CORE/src/rcexception.cpp \
    $$ROBOCONTROLLERSDKPATH/mod_CORE/src/loghandler.cpp

INCLUDEPATH += $$ROBOCONTROLLERSDKPATH/mod_CORE/include/

HEADERS += \
        $$ROBOCONTROLLERSDKPATH/mod_CORE/include/robocontrollersdk.h\
        $$ROBOCONTROLLERSDKPATH/mod_CORE/include/RoboControllerSDK_global.h \
        $$ROBOCONTROLLERSDKPATH/mod_CORE/include/rcexception.h \
        $$ROBOCONTROLLERSDKPATH/mod_CORE/include/network_msg.h \
        $$ROBOCONTROLLERSDKPATH/mod_CORE/include/modbus_registers.h \
        $$ROBOCONTROLLERSDKPATH/mod_CORE/include/loghandler.h \

win32 {
#to avoid error with qdatetime.h
    DEFINES += NOMINMAX

    DEFINES += _CRT_SECURE_NO_WARNINGS
}
