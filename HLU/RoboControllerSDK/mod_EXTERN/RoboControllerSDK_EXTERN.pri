message(Added EXTERN module)

#######################################################################################
# LIBMODBUS
CONFIG(server) {
    message(Building with LibModBus support)

    CONFIG += libmodbus

    SOURCES += \
                $$ROBOCONTROLLERSDKPATH/mod_EXTERN/libmodbus-3.0.1/src/modbus.c \
                $$ROBOCONTROLLERSDKPATH/mod_EXTERN/libmodbus-3.0.1/src/modbus-tcp.c \
                $$ROBOCONTROLLERSDKPATH/mod_EXTERN/libmodbus-3.0.1/src/modbus-rtu.c \
                $$ROBOCONTROLLERSDKPATH/mod_EXTERN/libmodbus-3.0.1/src/modbus-data.c


    INCLUDEPATH += \
                $$ROBOCONTROLLERSDKPATH/mod_EXTERN/libmodbus-3.0.1 \
                $$ROBOCONTROLLERSDKPATH/mod_EXTERN/libmodbus-3.0.1/src

    HEADERS += \
                $$ROBOCONTROLLERSDKPATH/mod_EXTERN/libmodbus-3.0.1/src/modbus.h

    unix {
        DEFINES += _TTY_POSIX_
    }

    win32 {
        DEFINES += _TTY_WIN_  WINVER=0x0501 __WIN32__ _WIN32_WINNT=0x0501
        LIBS += -lsetupapi \
                -lwsock32 \
                -lws2_32
    }
}
#######################################################################################

#######################################################################################
# LIB_OPENCV
CONFIG(opencv)
{
    message(Building with OpenCV support)

    QT       += printsupport

    INCLUDEPATH += \
                $$ROBOCONTROLLERSDKPATH/mod_EXTERN/opencv-2.4.9/include

    win32 {
        message(Using OpenCV for Windows)
        OPENCV_LIB_PATH = $$ROBOCONTROLLERSDKPATH/mod_EXTERN/opencv-2.4.9/bin/vc10/lib

        #LIBS += \
        #        $$OPENCV_LIB_PATH/opencv_core249d.lib \
        #        $$OPENCV_LIB_PATH/opencv_highgui249d.lib

        LIBS += \
            $$OPENCV_LIB_PATH/opencv_core249.lib \
            $$OPENCV_LIB_PATH/opencv_highgui249.lib
    }

    android {
        message(Used OpenCV for Android)
        OPENCV_LIB_PATH = $$ROBOCONTROLLERSDKPATH/mod_EXTERN/opencv-2.4.9/bin/android-armeabi-v7a

        LIBS += \
            $$OPENCV_LIB_PATH/libopencv_contrib.a \
            $$OPENCV_LIB_PATH/libopencv_video.a \
            $$OPENCV_LIB_PATH/libopencv_highgui.a \
            $$OPENCV_LIB_PATH/libopencv_androidcamera.a \
            $$OPENCV_LIB_PATH/libopencv_core.a \
            $$OPENCV_LIB_PATH/libopencv_imgproc.a \
            $$OPENCV_LIB_PATH/libIlmImf.a \
            $$OPENCV_LIB_PATH/liblibjpeg.a \
            $$OPENCV_LIB_PATH/liblibpng.a \
            $$OPENCV_LIB_PATH/liblibtiff.a \
            $$OPENCV_LIB_PATH/liblibjasper.a \
            $$OPENCV_LIB_PATH/libtbb.a

        CONFIG+=link_pkgconfig PKGCONFIG+=opencv
    }

    linux {
        !android {
            message(Used OpenCV for Unix)
            OPENCV_LIB_PATH = /usr/lib
        
            LIBS += \
                -lopencv_core \
                -lopencv_highgui
        }
            
    }
}
#######################################################################################

#######################################################################################
# LIB_QCUSTOMPLOT
CONFIG(gui) {
    message(Building with QCustomPlot support)

    CONFIG += qcustomplot

    INCLUDEPATH += \
                $$ROBOCONTROLLERSDKPATH/mod_EXTERN/qcustomplot-1.2.0/include

    HEADERS += \
                $$ROBOCONTROLLERSDKPATH/mod_EXTERN/qcustomplot-1.2.0/include/qcustomplot.h

    SOURCES += \
                $$ROBOCONTROLLERSDKPATH/mod_EXTERN/qcustomplot-1.2.0/src/qcustomplot.cpp
}

message($$CONFIG)
