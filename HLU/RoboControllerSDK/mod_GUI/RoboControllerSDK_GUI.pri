QT += gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

SOURCES += \
     $$ROBOCONTROLLERSDKPATH/mod_GUI/Joypad/src/cqtjoypad.cpp

INCLUDEPATH += \
    $$ROBOCONTROLLERSDKPATH/mod_GUI/Joypad/include/

HEADERS += \
        $$ROBOCONTROLLERSDKPATH/mod_GUI/Joypad/include/cqtjoypad.h
