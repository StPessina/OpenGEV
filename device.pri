@INCLUDE += device

INCLUDEPATH += Device
DEPENDPATH += Device

HEADERS += \
    $$PWD/Device/gvdevice.h \
    $$PWD/Device/deviceregisters.h \
    $$PWD/Device/networkinterfaceregisters.h \
    $$PWD/Device/streamchanneltransmitter.h

SOURCES += \
    $$PWD/Device/gvdevice.cpp \
    $$PWD/Device/deviceregisters.cpp \
    $$PWD/Device/networkinterfaceregisters.cpp \
    $$PWD/Device/streamchanneltransmitter.cpp

@INCLUDE += DeviceCommandHandler

INCLUDEPATH += DeviceCommandHandler
DEPENDPATH += DeviceCommandHandler

HEADERS += \
    $$PWD/DeviceCommandHandler/deviceackcode.h \
    $$PWD/DeviceCommandHandler/cmdnotsupportedmh.h \
    $$PWD/DeviceCommandHandler/devicecommandhandlerfactory.h \
    $$PWD/DeviceCommandHandler/discoverycommandhandler.h \
    $$PWD/DeviceCommandHandler/readregistercommandhandler.h \
    $$PWD/DeviceCommandHandler/writeregistercommandhandler.h \
    $$PWD/DeviceCommandHandler/packetresendcommandhandler.h

SOURCES += \
    $$PWD/DeviceCommandHandler/cmdnotsupportedmh.cpp \
    $$PWD/DeviceCommandHandler/devicecommandhandlerfactory.cpp \
    $$PWD/DeviceCommandHandler/discoverycommandhandler.cpp \
    $$PWD/DeviceCommandHandler/readregistercommandhandler.cpp \
    $$PWD/DeviceCommandHandler/writeregistercommandhandler.cpp \
    $$PWD/DeviceCommandHandler/packetresendcommandhandler.cpp

INCLUDEPATH += OpenNICamera
DEPENDPATH += OpenNICamera

HEADERS += \
    $$PWD/OpenNICamera/opennicamera.h

SOURCES += \
    $$PWD/OpenNICamera/opennicamera.cpp

INCLUDEPATH += Kinect
DEPENDPATH += Kinect

HEADERS += \
    $$PWD/Kinect/kinectv2camera.h

SOURCES += \
    $$PWD/Kinect/kinectv2camera.cpp
