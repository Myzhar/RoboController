message(Added SERVER module)

QT += network serialport

CONFIG += rcserver

SOURCES += \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/src/qrobotserver.cpp

INCLUDEPATH += \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/include/

HEADERS += \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/include/qrobotserver.h

CONFIG(opencv) {
    HEADERS += \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/include/qwebcamserver.h \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/include/qwebcamclient.h

    SOURCES += \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/src/qwebcamserver.cpp \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/src/qwebcamclient.cpp
}
