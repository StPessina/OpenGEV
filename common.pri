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
    $$PWD/CommonUdpChannel/udpchannel.h \
    $$PWD/CommonUdpChannel/udpchanneltransmitter.h \
    $$PWD/CommonUdpChannel/udpchannelreceiver.h

SOURCES += \
    $$PWD/CommonUdpChannel/udpchannel.cpp \
    $$PWD/CommonUdpChannel/udpchanneltransmitter.cpp \
    $$PWD/CommonUdpChannel/udpchannelreceiver.cpp

@INCLUDE += CommonMessages

INCLUDEPATH += CommonMessages
DEPENDPATH += CommonMessages

HEADERS += \
    $$PWD/CommonMessages/privilege.h \
    $$PWD/CommonMessages/abstractcommand.h \
    $$PWD/CommonMessages/abstractmessagehandler.h \
    $$PWD/CommonMessages/abstractmessagehandlerfactory.h \
    $$PWD/CommonMessages/conversionutils.h

SOURCES += \
    $$PWD/CommonMessages/abstractcommand.cpp \
    $$PWD/CommonMessages/abstractmessagehandler.cpp \
    $$PWD/CommonMessages/abstractmessagehandlerfactory.cpp \
    $$PWD/CommonMessages/conversionutils.cpp
