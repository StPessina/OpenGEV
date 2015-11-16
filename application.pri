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

INCLUDEPATH += ApplicationMessageHandler
DEPENDPATH += ApplicationMessageHandler

HEADERS += \
    $$PWD/ApplicationMessageHandler/applicationmessagehandlerfactory.h

SOURCES += \
    $$PWD/ApplicationMessageHandler/applicationmessagehandlerfactory.cpp
