@INCLUDE += CommonBootstrapRegister

INCLUDEPATH += CommonBootstrapRegister

HEADERS += \
    $$PWD/CommonBootstrapRegister/bootstrapregister.h \
    $$PWD/CommonBootstrapRegister/registeraccess.h \
    $$PWD/CommonBootstrapRegister/bootstrapregistertype.h \
    $$PWD/CommonControlChannel/udpchannel.h

SOURCES += \
    $$PWD/CommonBootstrapRegister/bootstrapregister.cpp \
    $$PWD/CommonControlChannel/udpchannel.cpp

@INCLUDE += CommonComponent

INCLUDEPATH += CommonComponent
DEPENDPATH += CommonComponent

HEADERS += \
    $$PWD/CommonComponent/gvcomponent.h

SOURCES += \
    $$PWD/CommonComponent/gvcomponent.cpp

@INCLUDE += CommonCtrlChannel

INCLUDEPATH += CommonControlChannel
DEPENDPATH += CommonControlChannel

HEADERS += \
    $$PWD/CommonControlChannel/controlchannelprivilege.h \
    $$PWD/CommonControlChannel/controlchannelmaster.h \
    $$PWD/CommonControlChannel/controlchannelslave.h

SOURCES += \
    $$PWD/CommonControlChannel/controlchannelmaster.cpp \
    $$PWD/CommonControlChannel/controlchannelslave.cpp

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
