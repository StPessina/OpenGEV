#-------------------------------------------------
#
# Project created by QtCreator 2015-10-14T15:51:20
#
#-------------------------------------------------

QT       += network

QT       -= gui

TARGET = OpenGV
TEMPLATE = lib

DEFINES += OPENGV_LIBRARY

SOURCES += opengv.cpp

HEADERS += opengv.h\
        opengv_global.h

include(commonComponent.pri)
include(commonCtrlChannel.pri)
include(commonCtrlMsg.pri)
include(commonUdpProtocol.pri)
include(device.pri)

unix {
    target.path = /usr/lib
    INSTALLS += target

    QMAKE_CXXFLAGS+= -std=c++11
}


