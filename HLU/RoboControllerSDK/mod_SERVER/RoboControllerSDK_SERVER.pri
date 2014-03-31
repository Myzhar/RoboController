message(Added SERVER module)

QT += network serialport

CONFIG += server

SOURCES += \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/src/qrobotserver.cpp

INCLUDEPATH += \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/include/

HEADERS += \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/include/qrobotserver.h
