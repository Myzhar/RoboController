!android {
    message(Added RGBD module)
    
    CONFIG += openni2


    SOURCES += \
                $$ROBOCONTROLLERSDKPATH/mod_VISION/src/qopenni2grabber.cpp

    HEADERS += \
                $$ROBOCONTROLLERSDKPATH/mod_VISION/include/qopenni2grabber.h
} else {
    message(RGBD module is not supported under Android)
}

