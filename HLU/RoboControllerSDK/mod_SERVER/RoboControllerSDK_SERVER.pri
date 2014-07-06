message(Added SERVER module)

QT += network serialport

CONFIG += server

SOURCES += \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/src/qrobotserver.cpp \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/src/qrobotctrlserver.cpp

INCLUDEPATH += \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/include/

HEADERS += \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/include/qrobotserver.h \
        $$ROBOCONTROLLERSDKPATH/mod_SERVER/include/qrobotctrlserver.h
