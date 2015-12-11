#-------------------------------------------------
#
# Project created by QtCreator 2015-10-14T15:51:20
#
#-------------------------------------------------

QT       += network

QT       -= gui

TARGET = OpenGV
TEMPLATE = lib

DEFINES += OPENGV_LIBRARY \
            CORE_LIBRARY

SOURCES += opengv.cpp

HEADERS += opengv.h\
        opengv_global.h

include(common.pri)
include(device.pri)
include(application.pri)

unix {
    target.path = /usr/lib
    INSTALLS += target

    QMAKE_CXXFLAGS+= -std=c++11

    INCLUDEPATH += /usr/local/include/log4cpp \
                   /usr/include/boost \
                   /usr/include/openni2

    LIBS += "-L/usr/lib" \
             -lboost_system

    LIBS += -L/usr/local/lib

    LIBS += -llog4cpp

    LIBS += -lOpenNI2
}

OTHER_FILES += \
    opengv_log4cpp.properties

