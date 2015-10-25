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
    $$PWD/ApplicationCommand/applicationcommandcode.h

SOURCES += \
    $$PWD/ApplicationCommand/discoverycommand.cpp

INCLUDEPATH += ApplicationMessageHandler
DEPENDPATH += ApplicationMessageHandler

HEADERS += \
    $$PWD/ApplicationMessageHandler/applicationmessagehandlerfactory.h

SOURCES += \
    $$PWD/ApplicationMessageHandler/applicationmessagehandlerfactory.cpp
