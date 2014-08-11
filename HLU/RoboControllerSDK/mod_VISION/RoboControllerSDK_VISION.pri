message(Added VISION module)

CONFIG += opencv

DEFINES += USE_OPENCV #useful in coding phase to verify if OpenCV is available

SOURCES += \
            $$ROBOCONTROLLERSDKPATH/mod_VISION/src/opencvtools.cpp \
            $$ROBOCONTROLLERSDKPATH/mod_VISION/src/qopenni2grabber.cpp

INCLUDEPATH += \
            $$ROBOCONTROLLERSDKPATH/mod_VISION/include/

HEADERS += \
            $$ROBOCONTROLLERSDKPATH/mod_VISION/include/opencvtools.h \
            $$ROBOCONTROLLERSDKPATH/mod_VISION/include/qopenni2grabber.h

