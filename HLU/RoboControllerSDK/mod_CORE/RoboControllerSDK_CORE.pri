message(Added CORE module)

SOURCES += \
    $$ROBOCONTROLLERSDKPATH/mod_CORE/src/robocontrollersdk.cpp \
    $$ROBOCONTROLLERSDKPATH/mod_CORE/src/exception.cpp \
    $$ROBOCONTROLLERSDKPATH/mod_CORE/src/qwebcamclient.cpp

INCLUDEPATH += $$ROBOCONTROLLERSDKPATH/mod_CORE/include/

HEADERS += \
        $$ROBOCONTROLLERSDKPATH/mod_CORE/include/robocontrollersdk.h\
        $$ROBOCONTROLLERSDKPATH/mod_CORE/include/RoboControllerSDK_global.h \
        $$ROBOCONTROLLERSDKPATH/mod_CORE/include/exception.h \
        $$ROBOCONTROLLERSDKPATH/mod_CORE/include/network_msg.h \
        $$ROBOCONTROLLERSDKPATH/mod_CORE/include/qwebcamclient.h

win32 {
#to avoid error with qdatetime.h
    DEFINES += NOMINMAX
}

