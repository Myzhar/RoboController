message(Added GUI module)

QT += gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

DEFINES += SDK_PATH=\\\"$$ROBOCONTROLLERSDKPATH\\\"
DEFINES += RES_PATH=\\\"/mod_GUI/res/gui-resources.rcc\\\"

SOURCES += \
            $$ROBOCONTROLLERSDKPATH/mod_GUI/src/qjoypad.cpp \
            $$ROBOCONTROLLERSDKPATH/mod_GUI/src/qrobotconfigdialog.cpp \
            $$ROBOCONTROLLERSDKPATH/mod_GUI/src/qbatterycalibdialog.cpp \
            $$ROBOCONTROLLERSDKPATH/mod_GUI/src/qscreentools.cpp

INCLUDEPATH += \
            $$ROBOCONTROLLERSDKPATH/mod_GUI/include/ \

HEADERS += \
            $$ROBOCONTROLLERSDKPATH/mod_GUI/include/qjoypad.h \
            $$ROBOCONTROLLERSDKPATH/mod_GUI/include/qrobotconfigdialog.h \
            $$ROBOCONTROLLERSDKPATH/mod_GUI/include/qbatterycalibdialog.h \
            $$ROBOCONTROLLERSDKPATH/mod_GUI/include/qscreentools.h

FORMS    += \
            $$ROBOCONTROLLERSDKPATH/mod_GUI/ui/qrobotconfigdialog.ui \
            $$ROBOCONTROLLERSDKPATH/mod_GUI/ui/qbatterycalibdialog.ui
            
RESOURCES += \
            $$ROBOCONTROLLERSDKPATH/mod_GUI/res/joypad.qrc

CONFIG(opencv) {
    #TODO: ADD OpenGL Widget for Qt
}
