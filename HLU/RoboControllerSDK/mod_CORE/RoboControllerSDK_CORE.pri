QT += network serialport

SOURCES += \
    $$ROBOCONTROLLERSDKPATH/mod_CORE/src/robocontrollersdk.cpp \
    $$ROBOCONTROLLERSDKPATH/mod_CORE/src/exception.cpp \
    $$ROBOCONTROLLERSDKPATH/mod_CORE/src/qrobottcpserver.cpp

INCLUDEPATH += $$ROBOCONTROLLERSDKPATH/mod_CORE/include/

HEADERS += \
        $$ROBOCONTROLLERSDKPATH/mod_CORE/include/robocontrollersdk.h\
        $$ROBOCONTROLLERSDKPATH/mod_CORE/include/RoboControllerSDK_global.h \
        $$ROBOCONTROLLERSDKPATH/mod_CORE/include/exception.h \
        $$ROBOCONTROLLERSDKPATH/mod_CORE/include/qrobottcpserver.h

