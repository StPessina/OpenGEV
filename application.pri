@INCLUDE += application

INCLUDEPATH += Application
DEPENDPATH += Application

HEADERS += \
    $$PWD/Application/gvapplication.h \
    $$PWD/Application/partnerdevice.h

SOURCES += \
    $$PWD/Application/gvapplication.cpp \
    $$PWD/Application/partnerdevice.cpp

INCLUDEPATH += ApplicationCommand
DEPENDPATH += ApplicationCommand

HEADERS += \
    $$PWD/ApplicationCommand/discoverycommand.h \
    $$PWD/ApplicationCommand/applicationcommandcode.h \
    $$PWD/ApplicationCommand/readregistercommand.h \
    $$PWD/ApplicationCommand/writeregistercommand.h

SOURCES += \
    $$PWD/ApplicationCommand/discoverycommand.cpp \
    $$PWD/ApplicationCommand/readregistercommand.cpp \
    $$PWD/ApplicationCommand/writeregistercommand.cpp

INCLUDEPATH += ApplicationCommandHandler
DEPENDPATH += ApplicationCommandHandler

HEADERS +=  \
    $$PWD/ApplicationCommandHandler/applicationcommandhandlerfactory.h

SOURCES += \
    $$PWD/ApplicationCommandHandler/applicationcommandhandlerfactory.cpp
