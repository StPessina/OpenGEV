#-------------------------------------------------
#
# Project created by QtCreator 2015-10-14T15:51:20
#
#-------------------------------------------------

QT       += network

QT       -= gui

TARGET = OpenGEV
TEMPLATE = lib

DEFINES += OPENGEV_LIBRARY \
            CORE_LIBRARY

SOURCES += opengev.cpp

HEADERS += opengev.h\
        opengev_global.h

include(common.pri)
include(device.pri)
include(application.pri)

unix {
    target.path = /usr/lib
    INSTALLS += target

    QMAKE_CXXFLAGS+= -std=c++11

    INCLUDEPATH += /usr/local/include/log4cpp \
                   /usr/include/boost \
                   /usr/include/pcl-1.7 \
                   /usr/include/eigen3 \
                   /usr/include/openni2 \
                   /usr/local/include/libfreenect2

    LIBS += "-L/usr/lib"

    LIBS += -lboost_system

    LIBS += -lpcl_common

    LIBS += -lOpenNI2

    LIBS += "-L/usr/local/lib"

    LIBS += -llog4cpp

    LIBS += -lfreenect2
}

OTHER_FILES += \
    opengev_log4cpp.properties

