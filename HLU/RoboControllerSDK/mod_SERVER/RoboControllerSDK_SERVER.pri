QT += network serialport

SOURCES += \
    $$ROBOCONTROLLERSDKPATH/mod_SERVER/src/qrobottcpserver.cpp

INCLUDEPATH += $$ROBOCONTROLLERSDKPATH/mod_SERVER/include/

HEADERS += \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/include/qrobottcpserver.h

