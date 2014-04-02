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
  message(The Module VIDEOSTREAM requires VISION module to be include before in PRO file)
}

