@INCLUDE += CommonBootstrapRegister

INCLUDEPATH += CommonBootstrapRegister

HEADERS += \
    $$PWD/CommonBootstrapRegister/bootstrapregister.h \
    $$PWD/CommonBootstrapRegister/registeraccess.h \
    $$PWD/CommonBootstrapRegister/bootstrapregistertype.h

SOURCES += \
    $$PWD/CommonBootstrapRegister/bootstrapregister.cpp

@INCLUDE += CommonComponent

INCLUDEPATH += CommonComponent
DEPENDPATH += CommonComponent

HEADERS += \
    $$PWD/CommonComponent/gvcomponent.h

SOURCES += \
    $$PWD/CommonComponent/gvcomponent.cpp

@INCLUDE += CommonUdpChannel

INCLUDEPATH += CommonUdpChannel
DEPENDPATH += CommonUpdChannel

HEADERS += \
    $$PWD/CommonUdpChannel/controlchannelprivilege.h \
    $$PWD/CommonUdpChannel/privilege.h \
    $$PWD/CommonUdpChannel/udpchannel.h \
    $$PWD/CommonUdpChannel/udpchanneltransmitter.h \
    $$PWD/CommonUdpChannel/udpchannelreceiver.h

SOURCES += \
    $$PWD/CommonUdpChannel/udpchannel.cpp \
    $$PWD/CommonUdpChannel/udpchanneltransmitter.cpp \
    $$PWD/CommonUdpChannel/udpchannelreceiver.cpp

@INCLUDE += CommonPacket

INCLUDEPATH += CommonPacket
DEPENDPATH += CommonPacket

HEADERS += \
    $$PWD/CommonPacket/abstractpacket.h \
    $$PWD/CommonPacket/abstractpackethandler.h \
    $$PWD/CommonPacket/abstractpackethandlerfactory.h \
    $$PWD/CommonPacket/conversionutils.h

SOURCES += \
    $$PWD/CommonPacket/abstractpacket.cpp \
    $$PWD/CommonPacket/abstractpackethandler.cpp \
    $$PWD/CommonPacket/abstractpackethandlerfactory.cpp \
    $$PWD/CommonPacket/conversionutils.cpp

@INCLUDE += CommonCommand

INCLUDEPATH += CommonCommand
DEPENDPATH += CommonCommand

HEADERS += \
    $$PWD/CommonCommand/abstractcommand.h \
    $$PWD/CommonCommand/abstractcommandhandler.h \
    $$PWD/CommonCommand/abstractcommandhandlerfactory.h

SOURCES += \
    $$PWD/CommonCommand/abstractcommand.cpp \
    $$PWD/CommonCommand/abstractcommandhandler.cpp \
    $$PWD/CommonCommand/abstractcommandhandlerfactory.cpp

@INCLUDE += CommonStream

INCLUDEPATH += CommonStream
DEPENDPATH += CommonStream

HEADERS += \
    $$PWD/CommonStream/abstractstreamdata.h

SOURCES += \
    $$PWD/CommonStream/abstractstreamdata.cpp
