QT += network

CONFIG += videostream

CONFIG(opencv) {
    message(Added VIDEOSTREAM module)
    
    INCLUDEPATH += \
        $$ROBOCONTROLLERSDKPATH/mod_VIDEOSTREAM/include/

    HEADERS += \
        $$ROBOCONTROLLERSDKPATH/mod_VIDEOSTREAM/include/qwebcamserver.h \
        $$ROBOCONTROLLERSDKPATH/mod_VIDEOSTREAM/include/qwebcamclient.h

    SOURCES += \
        $$ROBOCONTROLLERSDKPATH/mod_VIDEOSTREAM/src/qwebcamserver.cpp \
        $$ROBOCONTROLLERSDKPATH/mod_VIDEOSTREAM/src/qwebcamclient.cpp
} else {
  message(ERROR: The Module VIDEOSTREAM requires VISION module to be include before it in PRO file)
}

!android {
    CONFIG(openni2) {
        HEADERS += \
                ../RoboControllerSDK/mod_VIDEOSTREAM/include/qrgbdserver.h

        SOURCES += \
                ../RoboControllerSDK/mod_VIDEOSTREAM/src/qrgbdserver.cpp
    } else {
        message(Warning: The Module VIDEOSTREAM requires RGBD module to be include before it in PRO file to use OpenNI2 Grabber)
    }
}

RESOURCES += \
    ../RoboControllerSDK/mod_VIDEOSTREAM/resources/resources.qrc


