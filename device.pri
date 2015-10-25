@INCLUDE += device

INCLUDEPATH += Device
DEPENDPATH += Device

HEADERS += \
    $$PWD/Device/gvdevice.h \
    $$PWD/Device/deviceregisters.h \
    $$PWD/Device/networkinterfaceregisters.h

SOURCES += \
    $$PWD/Device/gvdevice.cpp \
    $$PWD/Device/deviceregisters.cpp \
    $$PWD/Device/networkinterfaceregisters.cpp

@INCLUDE += DeviceMessageHandler

INCLUDEPATH += DeviceMessageHandler
DEPENDPATH += DeviceMessageHandler

HEADERS += \
    $$PWD/DeviceMessageHandler/discoverymessagehandler.h \
    $$PWD/DeviceMessageHandler/devicemessagehandlerfactory.h \
    $$PWD/DeviceMessageHandler/deviceackcode.h \
    $$PWD/DeviceMessageHandler/cmdnotsupportedmh.h \
    $$PWD/DeviceMessageHandler/deviceackstatus.h

SOURCES += \
    $$PWD/DeviceMessageHandler/discoverymessagehandler.cpp \
    $$PWD/DeviceMessageHandler/cmdnotsupportedmh.cpp \
    $$PWD/DeviceMessageHandler/devicemessagehandlerfactory.cpp
