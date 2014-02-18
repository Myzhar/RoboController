#######################################################################################
# LIBMODBUS

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
#######################################################################################
