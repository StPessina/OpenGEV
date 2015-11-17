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

@INCLUDE += DeviceMessageHandler

INCLUDEPATH += DeviceMessageHandler
DEPENDPATH += DeviceMessageHandler

HEADERS += \
    $$PWD/DeviceMessageHandler/discoverymessagehandler.h \
    $$PWD/DeviceMessageHandler/devicemessagehandlerfactory.h \
    $$PWD/DeviceMessageHandler/deviceackcode.h \
    $$PWD/DeviceMessageHandler/cmdnotsupportedmh.h \
    $$PWD/DeviceMessageHandler/deviceackstatus.h \
    $$PWD/DeviceMessageHandler/readregistermessagehandler.h \
    $$PWD/DeviceMessageHandler/writeregistermessagehandler.h

SOURCES += \
    $$PWD/DeviceMessageHandler/discoverymessagehandler.cpp \
    $$PWD/DeviceMessageHandler/cmdnotsupportedmh.cpp \
    $$PWD/DeviceMessageHandler/devicemessagehandlerfactory.cpp \
    $$PWD/DeviceMessageHandler/readregistermessagehandler.cpp \
    $$PWD/DeviceMessageHandler/writeregistermessagehandler.cpp
