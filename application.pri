@INCLUDE += application

INCLUDEPATH += Application
DEPENDPATH += Application

HEADERS += \
    $$PWD/Application/gvapplication.h \
    $$PWD/Application/partnerdevice.h \
    $$PWD/Application/streamdatareceiver.h

SOURCES += \
    $$PWD/Application/gvapplication.cpp \
    $$PWD/Application/partnerdevice.cpp \
    $$PWD/Application/streamdatareceiver.cpp

INCLUDEPATH += ApplicationCommand
DEPENDPATH += ApplicationCommand

HEADERS += \
    $$PWD/ApplicationCommand/discoverycommand.h \
    $$PWD/ApplicationCommand/applicationcommandcode.h \
    $$PWD/ApplicationCommand/readregistercommand.h \
    $$PWD/ApplicationCommand/writeregistercommand.h \
    $$PWD/ApplicationCommand/packetresendcommand.h

SOURCES += \
    $$PWD/ApplicationCommand/discoverycommand.cpp \
    $$PWD/ApplicationCommand/readregistercommand.cpp \
    $$PWD/ApplicationCommand/writeregistercommand.cpp \
    $$PWD/ApplicationCommand/packetresendcommand.cpp

INCLUDEPATH += ApplicationCommandHandler
DEPENDPATH += ApplicationCommandHandler

HEADERS +=  \
    $$PWD/ApplicationCommandHandler/applicationcommandhandlerfactory.h

SOURCES += \
    $$PWD/ApplicationCommandHandler/applicationcommandhandlerfactory.cpp

INCLUDEPATH += ApplicationStreamDataHandler
DEPENDPATH += ApplicationStreamDataHandler

HEADERS +=  \
    $$PWD/ApplicationStreamDataHandler/streamimagedatahandlerfactory.h \
    $$PWD/ApplicationStreamDataHandler/streamimagedataleaderhandler.h \
    $$PWD/ApplicationStreamDataHandler/streamimagedatapayloadhandler.h \
    $$PWD/ApplicationStreamDataHandler/streamimagedatatrailerhandler.h \
    $$PWD/ApplicationStreamDataHandler/streamimagedataallinhandler.h

SOURCES += \
    $$PWD/ApplicationStreamDataHandler/streamimagedatahandlerfactory.cpp \
    $$PWD/ApplicationStreamDataHandler/streamimagedataleaderhandler.cpp \
    $$PWD/ApplicationStreamDataHandler/streamimagedatapayloadhandler.cpp \
    $$PWD/ApplicationStreamDataHandler/streamimagedatatrailerhandler.cpp \
    $$PWD/ApplicationStreamDataHandler/streamimagedataallinhandler.cpp
